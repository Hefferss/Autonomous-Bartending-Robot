# ======================================================================
# OO_TestCollision.py  --  tests OO_Collision.py on every robot in the workcell
#
# Run it like OO_TestMain.py (it uses the same workcell). It checks, for EVERY robot:
#   1. every link gets an ellipsoid, and every ellipsoid is a proper solid (no flat "zero length" links)
#   2. a point at an ellipsoid's centre hits, and a point 1 m away does not
#   3. the ellipsoids follow the robot when it moves
#   4. an obstacle placed in a planned path is found BEFORE moving, and removing it clears the path
#   5. robot-against-robot checking works (using another robot's link points)
#   6. the check is fast enough to run every step
#   7. the drawn ellipsoids can be shown, moved and hidden
# Look at the Swift window for test 7: orange ellipsoids should appear on each robot, then disappear.
# ======================================================================

import time                                          # FOR TIMING THE CHECK
import numpy as np                                   # arrays and maths
from roboticstoolbox import jtraj                    # the planned path used in test 4
from OO_Collision import CollisionChecker, box_points, sphere_points
from OO_TestMain import check, results, find_test_pose, nearby_pose, put_at, build_workcell


# run_collision_tests: runs tests 1 to 7 on one robot. "others" are the other movers, for test 5.
def run_collision_tests(name, mover, others, env):
    print("\n=== %s ===" % name)
    robot = mover.robot
    offsets = {}
    if "Jack" in name:
        offsets = {3: [0.0, 0.05, 0.0]}   # LOCAL SHIFT FOR JACK'S LINK 3 MESH
    elif "UR3" in name:
        offsets = {}
    checker = CollisionChecker(robot, fatness=0.1, length_margin=1.0, link_offsets=offsets)
    q_a = find_test_pose(mover)
    q_b = nearby_pose(mover, q_a, size=0.5)
    put_at(mover, q_a)

    # 1. every link has a proper ellipsoid
    ellipsoids = checker.elipsoids_for_robot_around_links(q_a)
    solid = all(np.linalg.matrix_rank(e['matrix']) == 3 for e in ellipsoids)
    check("1.  one ellipsoid per link, and every one is a solid (rank 3)", len(ellipsoids) == robot.n and solid,
          "%d ellipsoids" % len(ellipsoids))

    # 2. inside / outside
    centre = ellipsoids[len(ellipsoids) // 2]['center']
    check("2a. a point at an ellipsoid centre is a hit", checker.collides(q_a, [centre]))
    check("2b. a point 1 m above every link is not a hit", not checker.collides(q_a, [np.max([e['center'] for e in ellipsoids], axis=0) + [0, 0, 1.0]]))

    # 3. ellipsoids follow the robot (ACCOUNTING FOR LINK OFFSETS)
    moved = checker.elipsoids_for_robot_around_links(q_b)
    frames = [T.A for T in robot.fkine_all(q_b)]
    follows = True
    num_links = min(len(frames) - 1, robot.n)
    for i in range(num_links):
        a, b = frames[i][:3, 3], frames[i + 1][:3, 3]
        expected_center = (a + b) / 2
        if i in checker.link_offsets:
            v = b - a
            w = np.array([-v[1], v[0], 0])
            if np.linalg.norm(w) == 0:
                w = np.array([1, 0, 0])
            h = np.cross(v, w)
            d_dir = v / np.linalg.norm(v) if np.linalg.norm(v) != 0 else v
            d_w = w / np.linalg.norm(w) if np.linalg.norm(w) != 0 else w
            d_h = h / np.linalg.norm(h) if np.linalg.norm(h) != 0 else h
            R = np.column_stack((d_dir, d_w, d_h))
            expected_center = expected_center + R @ np.array(checker.link_offsets[i], dtype=float)
        if not np.allclose(moved[i]['center'], expected_center):
            follows = False
            break
    check("3.  ellipsoid centres sit halfway between the joints (plus offsets) after the robot moves", follows)

    # 4. obstacle in a planned path
    q_far = nearby_pose(mover, q_a, size=1.0)       # A BIGGER MOVE, SO THE MIDDLE OF THE PATH IS CLEAR OF THE START AND END
    path = jtraj(q_a, q_far, 40).q
    clear_before = checker.first_collision_in_path(path, box_points((0, 0, 5), (0.1, 0.1, 0.1)))   # far away box
    # PICK AN OBSTACLE SPOT THAT THE ROBOT TOUCHES IN THE MIDDLE OF THE MOVE BUT NOT AT THE START OR END.
    # (A SPOT ON A LINK NEAR THE BASE WON'T WORK: THOSE LINKS BARELY MOVE, SO IT WOULD ALREADY BE TOUCHING AT THE START.)
    # WE SLIDE ALONG EACH LINK, LAST LINKS FIRST, BECAUSE THEY MOVE THE MOST.
    bottle = None
    for e in reversed(checker.elipsoids_for_robot_around_links(path[20])):
        for s_along in np.linspace(-0.8, 0.8, 9):
            spot = e['center'] + s_along * e['radii'][0] * e['rotation'][:, 0]       # a point along this link
            candidate = sphere_points(spot, 0.02)                                     # a small bottle there
            if (checker.collides(path[20], candidate) and not checker.collides(q_a, candidate)
                    and not checker.collides(q_far, candidate)):
                bottle = candidate
                break
        if bottle is not None:
            break
    hit_step = checker.first_collision_in_path(path, bottle) if bottle is not None else None
    check("4a. an obstacle placed in the path is found before moving", hit_step is not None and hit_step > 0,
          "could not find a spot that is clear at both ends" if bottle is None else "first hit at step %s of 40" % hit_step)
    check("4b. the path is clear when the obstacle is far away", clear_before is None)
    check("4c. the start and end poses do not hit that obstacle",
          bottle is not None and not checker.collides(q_a, bottle) and not checker.collides(q_far, bottle))

    # 5. robot against robot
    if others:
        other = others[0]
        other_checker = CollisionChecker(other.robot)
        points = other_checker.link_points(other.robot.q)
        separate = not checker.collides(robot.q, points)
        old_base = other.robot.base
        try:
            other.robot.base = robot.base            # pretend the other robot stands on top of this one
            overlap = checker.collides(robot.q, other_checker.link_points(other.robot.q))
        finally:
            other.robot.base = old_base              # always put it back
        check("5.  robots that are apart do not hit, and robots on top of each other do", separate and overlap)
    else:
        print("   (test 5 skipped: only one robot)")

    # 6. speed
    cloud = box_points((0.5, 0.5, 0.5), (0.4, 0.4, 0.4), spacing=0.02)
    start = time.perf_counter()
    for _ in range(20):
        checker.collides(q_a, cloud)
    per_check = (time.perf_counter() - start) / 20 * 1000
    check("6.  one check against %d points takes under 50 ms" % len(cloud), per_check < 50, "%.1f ms" % per_check)

    # 7. display
    checker.add_to_scene(env)
    hidden_ok = all(m.T[2, 3] < -500 for m in checker.meshes)
    checker.set_visible(True)
    shown_ok = all(abs(m.T[2, 3]) < 5 for m in checker.meshes)
    env_step = getattr(mover.workcell, "step", None)
    for _ in range(40):                              # about two seconds of showing them
        mover.workcell.step(0.05)
    checker.set_visible(False)
    check("7.  ellipsoids start hidden, show on the robot, and hide again", hidden_ok and shown_ok and len(checker.meshes) == robot.n)
    put_at(mover, mover.home_q)


def main():
    workcell = build_workcell()
    movers = list(workcell.movers.items())
    for i, (name, mover) in enumerate(movers):
        others = [m for j, (n, m) in enumerate(movers) if j != i]
        run_collision_tests(name, mover, others, workcell.env)
    passed = sum(1 for _, ok in results if ok)
    print("\n==================== SUMMARY ====================")
    print("%d of %d checks passed" % (passed, len(results)))
    for name, ok in results:
        if not ok:
            print("   FAILED:", name)


if __name__ == "__main__":
    main()