# ======================================================================
# OO_MovementCommands.py  --  GENERIC code (works with any robots)
#
# Two classes:
#   Workcell    : the ONE shared scene. Owns the Swift window, the e-stop
#                 state and every robot. One e-stop press stops all robots.
#   RobotMover  : moves ONE robot inside the workcell.
#
# A "robot setup" object (like KawasakiRS007N) must have:
#     .robot  .meshes  .offsets  .home_q
#
# Safety state is one of:
#     "RUNNING"  -> normal, movement allowed
#     "ESTOPPED" -> emergency stop pressed, all movement blocked
#     "RESET"    -> e-stop reset, but still blocked until RESTART
# The only allowed order is:  RUNNING -> ESTOPPED -> RESET -> RUNNING
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
import swift                      # the 3D simulator window
from roboticstoolbox import jtraj # makes a smooth joint-space path
from spatialmath import SE3       # poses (used for the jog movement)


# --------------------------- THE WORKCELL -----------------------------
class Workcell:

    # ---- Constructor: makes the shared window and safety state ---------
    def __init__(self):
        self.env = swift.Swift()              # the ONE simulator window (not opened yet)
        self.state = "RUNNING"                # the ONE safety state shared by all robots
        self.movers = {}                      # name -> RobotMover, for every robot in the scene

    # ---- Add robot: registers a robot in the scene ----------------------
    # Overview: wraps the robot setup in a RobotMover that knows about
    # this workcell, and stores it under a name (used by the GUI dropdown).
    def add_robot(self, name, robot_setup):
        mover = RobotMover(robot_setup, self) # give the mover the setup AND this workcell
        self.movers[name] = mover             # remember it under its name
        return mover                          # hand it back in case you want it

    # ---- Launch: opens Swift and puts every robot in the scene ----------
    def launch(self):
        self.env.launch(realtime=True)        # open the simulator window
        for mover in self.movers.values():    # go through every robot
            mover.add_to_scene()              # add it to the window at its home pose
        self.env.step()                       # refresh the window

    # ---- Safety state functions: used by the GUI buttons ----------------
    # Overview: three separate steps, each only allowed from the right
    # state, so the robots can never restart by accident.
    def emergency_stop(self):
        self.state = "ESTOPPED"               # allowed from ANY state: block everything now
        print("STATE: ESTOPPED - all motion stopped")

    def reset(self):
        if self.state != "ESTOPPED":          # reset only makes sense after an e-stop
            print("Reset ignored - e-stop is not active")
            return
        self.state = "RESET"                  # e-stop cleared, but robots are still blocked
        print("STATE: RESET - press RESTART when the area is clear")

    def restart(self):
        if self.state != "RESET":             # restart only allowed after a reset
            print("Restart ignored - reset the e-stop first")
            return
        self.state = "RUNNING"                # movement allowed again
        print("STATE: RUNNING")


# ------------------------- THE MOVER CLASS ----------------------------
class RobotMover:

    # ---- Constructor ----------------------------------------------------
    # Overview: stores one robot's parts and a link back to the shared workcell.
    def __init__(self, robot_setup, workcell):
        self.workcell = workcell              # the shared scene (window + e-stop state)
        self.robot = robot_setup.robot        # keep the DHRobot
        self.meshes = robot_setup.meshes      # keep the list of STL meshes
        self.offsets = robot_setup.offsets    # keep the mesh alignment matrices
        self.home_q = robot_setup.home_q      # keep the starting pose

    # ---- Update meshes: moves every STL to match the robot's joints ----
    def update_meshes(self, q):
        link_check = self.robot.fkine_all(q)  # list of the position of every link
        for i in range(len(self.meshes)):     # go through each mesh one by one
            # link i position  @  alignment offset  ->  mesh position
            self.meshes[i].T = link_check[i].A @ self.offsets[i]

    # ---- Add to scene: puts this robot in the shared window -------------
    def add_to_scene(self):
        env = self.workcell.env               # the shared window
        env.add(self.robot)                   # add the robot (cylinders)
        self.robot.q = self.home_q            # put it in its home pose
        env.step()                            # refresh the window
        self.update_meshes(self.home_q)       # place each mesh on its link
        for mesh in self.meshes:              # go through every mesh
            env.add(mesh)                     # add it to the simulator

    # ---- Move to joint angles: smooth movement ------------------------
    # Overview: plays a smooth path step by step. Before EVERY step it
    # checks the shared state; if the e-stop was pressed it abandons the
    # rest of the path, so the robot stops where it is.
    def move_to_q(self, q_goal, steps=50):
        traj = jtraj(self.robot.q, q_goal, steps) # smooth path: current pose -> goal

        for q_step in traj.q:                     # go through each step of the path
            if self.workcell.state != "RUNNING":  # e-stop (or reset) active?
                print("Motion aborted")           # tell the terminal
                return False                      # stop immediately, drop the rest of the path
            self.robot.q = q_step                 # set the robot to this step
            self.update_meshes(q_step)            # move the meshes to match
            self.workcell.env.step()              # redraw the window (also reads GUI clicks)
        return True                               # reached the end of the path

    # ---- Move to pose: inverse kinematics + smooth movement -----------
    def move_to_pose(self, T_target, steps=50):
        ik_sol = self.robot.ikine_LM(T_target)    # solve inverse kinematics
        if not ik_sol.success:                    # if no solution was found...
            raise ValueError("Inverse kinematics failed to find a valid solution for the target pose!")
        print("IK Solution found (radians):", ik_sol.q)  # show it in the terminal
        return self.move_to_q(ik_sol.q, steps)    # play the movement (it checks the e-stop itself)

    # ---- Jog: nudge the end effector by dx, dy, dz metres --------------
    def jog(self, dx, dy, dz, steps=10):
        T_now = self.robot.fkine(self.robot.q)                # current end effector pose
        T_new = SE3(dx, dy, dz) * T_now                       # same pose, shifted in the world frame
        ik_sol = self.robot.ikine_LM(T_new, q0=self.robot.q)  # IK, starting from where we are
        if not ik_sol.success:                                # if it can't reach there...
            print("Jog out of reach - move cancelled")        # ...say so and do nothing
            return False
        return self.move_to_q(ik_sol.q, steps)                # otherwise do the small move