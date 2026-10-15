# ======================================================================
# OO_TestMain.py  --  tests the features we have added
#
# Run it like OO_MainV2.py. It builds the same workcell, then checks:
#
#   FOR EVERY ROBOT (automatic, prints PASS or FAIL):
#     1. solve_ik gives the SAME joint angles every time, and they reach the target
#     2. move_to_pose reaches its target
#     3. move_rmrc follows a straight line and reaches its target
#     4. move_rmrc fails safely on an unreachable target
#     5. jog moves the hand by the right amount, and refuses a jog out of reach
#     6. the e-stop stops a move, blocks movement, and only recovers in the
#        right order: e-stop -> reset -> restart  (tested for BOTH move types)
#   FOR THE WORKCELL AND GUI (automatic):
#     7. input devices are polled on every step
#     8. the robot dropdown, joint sliders, queue and beacon obey the safety state
#   WATCH IT (look at the Swift window):
#     9. camera tour through every view in OO_CameraAngles.py
#   WITH THE CONTROLLER (needs you to press buttons):
#    10. X = e-stop, B = back (reset), A = start (restart)
#
# Change the switches below to choose what runs. Don't touch the Xbox
# controller during the automatic tests, or its e-stop will stop them.
#
# NOTE: there is no collision detection yet, so the test moves can pass
# through scene objects. They are small moves, but keep the area clear.
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
import numpy as np                                   # arrays and maths
from math import pi                                  # the constant pi
from spatialmath import SE3                          # poses
from spatialmath.base import transl                  # positions (used to place the robots)
from OO_MovementCommands import Workcell             # the shared scene
from OO_GUI import RobotGUI                          # the Swift GUI
from OO_Controller import XboxController             # Xbox controller e-stop


# ---------------------------- SWITCHES --------------------------------
RUN_AUTOMATED_TESTS = True       # tests 1 to 8
RUN_CAMERA_TOUR = True           # test 9: cycles through every camera view
RUN_CONTROLLER_TEST = False      # test 10: you press X, then B, then A on the controller
START_GUI_AFTER = True           # open the normal GUI when the tests finish
ADD_SCENE = True                 # add the bar scene (tables, bottles) like OO_MainV2.py
ROBOTS_TO_TEST = None            # None = every robot, or a list like ["UR3", "Jack's Robot"]


# ---------------------------- TEST REPORT -------------------------------
results = []                                         # (name, passed) for every check

# check: records and prints one PASS or FAIL line.
def check(name, passed, detail=""):
    results.append((name, bool(passed)))             # remember the result
    print("   [%s] %s %s" % ("PASS" if passed else "FAIL", name, ("- " + detail) if detail else ""))


# ---------------------------- TEST HELPERS ------------------------------

# A fake input device. Workcell.step() calls poll() on every device in
# workcell.inputs, so this lets a test see the polling happen, and trigger
# an e-stop part way through a move.
class TestDevice:
    def __init__(self, workcell, stop_after=None):
        self.workcell = workcell                     # the workcell to stop
        self.stop_after = stop_after                 # press e-stop after this many polls (None = never)
        self.polls = 0                               # how many times poll() has been called

    def poll(self):
        self.polls += 1                              # count this poll
        if self.stop_after is not None and self.polls == self.stop_after:
            self.workcell.emergency_stop()           # "press" the e-stop


# Records the hand position every step, so a test can check the path afterwards.
class PathRecorder:
    def __init__(self, mover):
        self.mover = mover                           # the robot to watch
        self.points = []                             # hand positions, one per step

    def poll(self):
        robot = self.mover.robot
        self.points.append(robot.fkine(robot.q).t.copy())   # where the hand is right now


# find_test_pose: picks a joint pose near home that is NOT near a singularity.
# It tries 300 repeatable random poses and keeps the best-conditioned one.
def find_test_pose(mover):
    robot = mover.robot                              # short name for the robot
    rng = np.random.default_rng(1)                   # fixed seed so every run picks the same pose
    home = np.array(mover.home_q, dtype=float)       # start from the home pose
    best_q, best_score = home, -1.0
    for _ in range(300):                             # try 300 candidates
        q = np.clip(home + rng.uniform(-0.6, 0.6, robot.n), robot.qlim[0, :], robot.qlim[1, :])
        score = np.linalg.svd(robot.jacob0(q), compute_uv=False)[-1]   # big = far from a singularity
        if score > best_score:                       # best so far?
            best_q, best_score = q, score
    return best_q


# nearby_pose: a second pose a short joint-move away from q (so it is reachable).
def nearby_pose(mover, q, size=0.3):
    robot = mover.robot
    signs = np.array([1, -1, 1, -1, 1, -1, 1][:robot.n], dtype=float)   # alternate directions
    return np.clip(q + size * signs, robot.qlim[0, :], robot.qlim[1, :])


# put_at: instantly places a robot at joint angles q (test setup, not a movement).
def put_at(mover, q):
    mover.robot.q = np.array(q, dtype=float)         # set the robot
    mover.update_meshes(mover.robot.q)               # move its meshes to match
    mover.workcell.step(0.01)                        # redraw once


# hand_position: where the hand is in the world right now.
def hand_position(mover):
    return mover.robot.fkine(mover.robot.q).t.copy()


# line_deviation: the furthest any recorded point strays from the straight line a -> b.
def line_deviation(points, a, b):
    ab = b - a                                       # the line
    worst = 0.0
    for p in points:                                 # check every recorded point
        t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1)   # nearest spot on the line
        worst = max(worst, np.linalg.norm(p - (a + t * ab)))
    return worst


# ---------------------------- ROBOT TESTS -------------------------------

# Tests 1 and 2: IK is consistent, and move_to_pose reaches the target.
def test_ik(mover, q_a, q_b):
    robot = mover.robot
    T_goal = robot.fkine(q_b)                        # a pose we know is reachable
    put_at(mover, q_a)

    answers = [mover.solve_ik(T_goal) for _ in range(5)]          # solve the same target 5 times
    same = all(np.allclose(answers[0], a) for a in answers)       # are all 5 identical?
    check("1a. solve_ik gives identical joint angles on 5 repeats", same)

    reached = np.linalg.norm(robot.fkine(answers[0]).t - T_goal.t)
    check("1b. solve_ik solution reaches the target", reached < 0.001, "%.3f mm off" % (reached * 1000))

    plain = {tuple(np.round(robot.ikine_LM(T_goal).q, 2)) for _ in range(5)}   # the OLD way, for comparison
    print("      (for comparison, plain ikine_LM gave %d different answers in 5 tries)" % len(plain))

    put_at(mover, q_a)
    ok = mover.move_to_pose(T_goal, steps=30)        # joint-space move to the pose
    error = np.linalg.norm(hand_position(mover) - T_goal.t)
    check("2.  move_to_pose reaches its target", ok and error < 0.002, "%.3f mm off" % (error * 1000))


# Tests 3 and 4: RMRC follows a straight line, and fails safely when it can't reach.
def test_rmrc(mover, q_a, q_b):
    robot = mover.robot
    workcell = mover.workcell
    T_goal = robot.fkine(q_b)

    put_at(mover, q_a)
    start = hand_position(mover)
    recorder = PathRecorder(mover)
    workcell.add_input(recorder)                     # record the hand path during the move
    ok = mover.move_rmrc(T_goal, duration=2.0, steps=40)
    workcell.inputs.remove(recorder)
    error = np.linalg.norm(hand_position(mover) - T_goal.t)
    deviation = line_deviation(recorder.points, start, T_goal.t)
    check("3a. move_rmrc reaches its target (within 5 mm)", ok and error < 0.005, "%.3f mm off" % (error * 1000))
    # The lab's damping (0.05) trades a little accuracy for safety near singularities,
    # so "straight" here means within 10 mm. Pass max_damping=0.01 to move_rmrc for about 2 mm.
    check("3b. move_rmrc stays on the straight line (within 10 mm)", deviation < 0.010, "max %.2f mm from the line" % (deviation * 1000))

    put_at(mover, q_a)
    far = SE3(0, 0, 3.0) * robot.fkine(robot.q)      # 3 m straight up: impossible
    before = np.array(robot.q)
    ok = mover.move_rmrc(far, duration=1.0, steps=20)
    safe = np.all(np.isfinite(robot.q)) and np.all(robot.q >= robot.qlim[0, :] - 1e-9) and np.all(robot.q <= robot.qlim[1, :] + 1e-9)
    check("4.  move_rmrc returns False and stays inside joint limits when out of reach", (not ok) and safe)


# Test 5: jog moves the right amount, and refuses out-of-reach jogs.
def test_jog(mover, q_a):
    put_at(mover, q_a)
    start = hand_position(mover)

    ok = mover.jog(0.02, 0, 0)                       # 20 mm along world x
    moved = hand_position(mover) - start
    check("5a. jog +20 mm in x moves the hand by about 20 mm in x",
          ok and np.linalg.norm(moved - np.array([0.02, 0, 0])) < 0.002, "moved %s mm" % np.round(moved * 1000, 1))

    mover.jog(-0.02, 0, 0)                           # and back again
    back = np.linalg.norm(hand_position(mover) - start)
    check("5b. jogging back returns to the start", back < 0.003, "%.2f mm from start" % (back * 1000))

    before = hand_position(mover)
    ok = mover.jog(5.0, 0, 0)                        # 5 m: out of reach
    unchanged = np.linalg.norm(hand_position(mover) - before) < 1e-6
    check("5c. jog out of reach is refused and nothing moves", (not ok) and unchanged)


# Test 6: the e-stop stops a move and recovery only works in the right order.
def test_estop(mover, q_a, q_b):
    workcell = mover.workcell
    robot = mover.robot
    T_goal = robot.fkine(q_b)

    for kind in ("joint move", "RMRC move"):
        print("   -- e-stop during a %s" % kind)
        workcell.state = "RUNNING"                   # make sure we start clean
        put_at(mover, q_a)
        start = np.array(robot.q)

        device = TestDevice(workcell, stop_after=8)  # presses e-stop on the 8th step
        workcell.add_input(device)
        if kind == "joint move":
            ok = mover.move_to_q(q_b, steps=40)
        else:
            ok = mover.move_rmrc(T_goal, duration=2.0, steps=40)
        workcell.inputs.remove(device)

        stopped_midway = (not np.allclose(robot.q, start, atol=1e-6)) and (not np.allclose(robot.q, q_b, atol=1e-3))
        check("6a. e-stop aborts the move part way (%s)" % kind, (not ok) and workcell.state == "ESTOPPED" and stopped_midway,
              "state %s" % workcell.state)

        stopped_at = np.array(robot.q)
        blocked = mover.move_to_q(q_b, steps=10)
        check("6b. movement is blocked while e-stopped (%s)" % kind, (not blocked) and np.allclose(robot.q, stopped_at))

        workcell.restart()                           # restart BEFORE reset: must be ignored
        check("6c. restart before reset is ignored (%s)" % kind, workcell.state == "ESTOPPED")

        workcell.reset()
        check("6d. reset moves to RESET but still blocks motion (%s)" % kind,
              workcell.state == "RESET" and not mover.move_to_q(q_b, steps=10))

        workcell.restart()
        recovered = mover.move_to_q(q_b, steps=30)   # carries on from where it stopped
        check("6e. after reset then restart it moves again, from where it stopped (%s)" % kind,
              workcell.state == "RUNNING" and recovered and np.allclose(robot.q, q_b, atol=1e-3))


# home_pose_report: prints (not a PASS/FAIL) whether the home pose is close to a singularity.
# RMRC starting from a singular pose can stall, so move to a "ready" pose first with move_to_q.
def home_pose_report(mover):
    robot = mover.robot
    J = robot.jacob0(np.array(mover.home_q, dtype=float))
    m = np.sqrt(max(np.linalg.det(J @ J.T), 0.0))             # manipulability: 0 = singular
    try:
        epsilon = 0.1 * (robot.reach / 1.706) ** 3             # same threshold move_rmrc uses
    except Exception:
        epsilon = 0.1
    note = "OK"
    if m < 0.05 * epsilon:
        note = "WARNING: home pose is (almost) singular - move_rmrc from home may stall. Use move_to_q to a ready pose first."
    print("   (home pose manipulability %.5f, damping starts below %.4f) %s" % (m, epsilon, note))


# run_robot_tests: runs tests 1 to 6 on one robot.
def run_robot_tests(name, mover):
    print("\n=== %s ===" % name)
    home_pose_report(mover)
    q_a = find_test_pose(mover)                      # a good starting pose
    q_b = nearby_pose(mover, q_a)                    # a reachable pose nearby
    test_ik(mover, q_a, q_b)
    test_rmrc(mover, q_a, q_b)
    test_jog(mover, q_a)
    test_estop(mover, q_a, q_b)
    mover.workcell.state = "RUNNING"                 # leave the workcell running
    put_at(mover, mover.home_q)                      # send the robot home


# ---------------------------- WORKCELL AND GUI TESTS ------------------------

# Test 7: Workcell.step() polls every input device.
def test_inputs(workcell):
    print("\n=== Workcell inputs ===")
    device = TestDevice(workcell)
    workcell.add_input(device)
    for _ in range(3):
        workcell.step(0.01)                          # three steps
    workcell.inputs.remove(device)
    check("7.  Workcell.step() polls an input device once per step", device.polls == 3, "%d polls" % device.polls)


# Test 8: the GUI callbacks obey the safety state.
def test_gui(gui):
    workcell = gui.workcell
    print("\n=== GUI ===")
    workcell.state = "RUNNING"
    gui.show_state()

    # robot dropdown
    chosen = []
    for index, name in enumerate(gui.names):
        gui.on_select_robot(index)                   # same call the dropdown makes
        chosen.append(gui.selected == name)
    check("8a. the robot dropdown selects every robot by index", all(chosen), "%d robots" % len(gui.names))

    # joint slider moves the selected robot
    mover = gui.current()
    start_q = np.array(mover.robot.q)
    target = np.degrees(start_q[0]) + 5              # 5 degrees from where joint 1 is now
    gui.on_joint_slider(0, target)
    check("8b. joint slider moves joint 1 of the selected robot",
          abs(mover.robot.q[0] - np.radians(target)) < 1e-6 or abs(mover.robot.q[0] - mover.robot.qlim[1, 0]) < 1e-6)

    # beacon follows the state
    def beacon_in_view(state):
        z = gui.beacons[state].T[2, 3]
        return abs(z - gui.beacon_position[2]) < 1e-6
    check("8c. beacon is green while RUNNING", beacon_in_view("RUNNING"))
    gui.on_estop()
    check("8d. beacon turns red on e-stop", workcell.state == "ESTOPPED" and beacon_in_view("ESTOPPED"))

    # everything is ignored while stopped
    q_before = np.array(mover.robot.q)
    gui.on_joint_slider(0, target + 10)
    gui.queue("home")
    check("8e. slider and movement buttons are ignored while e-stopped",
          np.allclose(mover.robot.q, q_before) and gui.pending is None)

    gui.on_restart()
    check("8f. restart before reset is ignored", workcell.state == "ESTOPPED")
    gui.on_reset()
    check("8g. beacon turns amber on reset", workcell.state == "RESET" and beacon_in_view("RESET"))
    gui.on_restart()
    check("8h. beacon turns green on restart", workcell.state == "RUNNING" and beacon_in_view("RUNNING"))

    put_at(mover, start_q)                           # undo the slider test move


# ---------------------------- WATCH-IT TESTS ----------------------------------

# Test 9: shows every camera view for about a second each. Look at the window.
def camera_tour(gui):
    print("\n=== Camera tour (look at the Swift window) ===")
    for name in gui.camera_names:
        gui.set_camera(name)                         # same call the dropdown makes
        for _ in range(20):
            gui.workcell.step(0.05)                  # hold this view for about a second
    check("9.  every camera view applied without an error", True, "%d views" % len(gui.camera_names))
    gui.set_camera(gui.camera_names[0])              # back to the first view


# wait_for_state: steps the simulation until the state changes, or gives up after about 15 seconds.
def wait_for_state(workcell, wanted):
    for _ in range(300):                             # 300 steps of 0.05 s
        workcell.step(0.05)
        if workcell.state == wanted:
            return True
    return False


# Test 10: you press the controller buttons.
def controller_test(workcell, controller):
    print("\n=== Xbox controller ===")
    workcell.step(0.05)                              # let it look for the controller
    check("10a. controller is connected", controller.connected)
    if not controller.connected:
        print("      (plug in or switch on the controller, then run the test again)")
        return
    workcell.state = "RUNNING"
    print("   Press X (emergency stop) within 15 seconds ...")
    check("10b. X triggers the e-stop", wait_for_state(workcell, "ESTOPPED"))
    print("   Press A (start) NOW - it should be ignored because you haven't pressed B ...")
    for _ in range(60):
        workcell.step(0.05)                          # about 3 seconds
    check("10c. A before B is ignored", workcell.state == "ESTOPPED")
    print("   Press B (back) within 15 seconds ...")
    check("10d. B resets the e-stop", wait_for_state(workcell, "RESET"))
    print("   Press A (start) within 15 seconds ...")
    check("10e. A restarts after the reset", wait_for_state(workcell, "RUNNING"))


# ---------------------------- BUILD THE WORKCELL --------------------------------

# build_workcell: the same robots and positions as OO_MainV2.py.
# The imports are inside the function so you can comment out a robot (and its
# import) here if one of them won't load.
def build_workcell():
    from OO_WillRobotTest import KawasakiRS007N      # Will's robot
    from OO_JamesRobotTest import ABBIRB140          # James' robot
    from OO_JackRobotTest import StaubliTX60         # Jack's robot
    from OO_UR3 import UR3Robot                      # the UR3 on its platform

    workcell = Workcell()
    workcell.add_robot("James' Robot", ABBIRB140(base=transl(-1, 0.75, 0.25)))
    workcell.add_robot("Jack's Robot", StaubliTX60(base=transl(0, 0.75, 0.25)))
    workcell.add_robot("UR3", UR3Robot(base=transl(0.08, 0.1, 0.25)))
    workcell.add_robot("Will's Robot", KawasakiRS007N(base=transl(1, 0.75, 0.25)))
    workcell.launch()                                # open Swift and show every robot
    if ADD_SCENE:
        from OO_Scene_Objects import BarScene        # tables, rail, bottles
        BarScene(workcell.env)                       # has to be after launch
    return workcell


# ---------------------------- MAIN PROGRAM --------------------------------------
def main():
    workcell = build_workcell()

    T_target = SE3(-1, 0.75, 0.25) * SE3(-0.2, 0.1, 0.5) * SE3.Rx(pi)   # demo pose, as in OO_MainV2.py
    gui = RobotGUI(workcell, demo_targets={"James' Robot": T_target})
    gui.build()                                      # the GUI is needed by tests 8, 9 and 10

    if RUN_AUTOMATED_TESTS:
        test_inputs(workcell)
        for name, mover in workcell.movers.items():  # every robot in the scene
            if ROBOTS_TO_TEST is None or name in ROBOTS_TO_TEST:
                run_robot_tests(name, mover)
        test_gui(gui)

    if RUN_CAMERA_TOUR:
        camera_tour(gui)

    controller = XboxController(on_estop=gui.on_estop, on_back=gui.on_reset, on_start=gui.on_restart)
    workcell.add_input(controller)                   # from here on the controller can stop the robots
    if RUN_CONTROLLER_TEST:
        controller_test(workcell, controller)

    # ---- summary ----
    passed = sum(1 for _, ok in results if ok)
    print("\n==================== SUMMARY ====================")
    print("%d of %d checks passed" % (passed, len(results)))
    for name, ok in results:
        if not ok:
            print("   FAILED:", name)

    if START_GUI_AFTER:
        print("\nGUI is running. Manual checks: camera dropdown, robot dropdown, joint sliders,")
        print("jog buttons, Go Home / Run Demo Move, the e-stop buttons and the beacon colour.")
        print("Press Ctrl+C to quit.")
        gui.run()


if __name__ == "__main__":
    main()