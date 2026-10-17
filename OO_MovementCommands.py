# Comment Author: Will
# Workcell = swift, robots, inputs and the e-stop states (RUNNING -> ESTOPPED -> RESET -> RUNNING). RobotMover = the shared movement commands (needs .robot .meshes .offsets .home_q)

import swift                      
import numpy as np
from roboticstoolbox import jtraj, trapezoidal
from spatialmath import SE3       
from OO_Collision import CollisionChecker

TABLE_HEIGHT = 0.25 # top of every table. no robot joint is allowed below this


# The workcell class sets up swift and adds the robot, input and safety states to the 3D environment
class Workcell:

    # Constructor: swift window, safety state, robots, input devices
    def __init__(self):
        self.env = swift.Swift()
        self.state = "RUNNING"
        self.movers = {}
        self.inputs = []
        self.collision_on = False


    # Add Robot Function: stores the robot under its name (the name is used in the GUI drop down box)
    def add_robot(self, name, robot_setup):
        print("Added robot:", name)
        mover = RobotMover(robot_setup, self)
        self.movers[name] = mover
        return mover

    # Add Input Function: the device needs a poll() method, called on every step() below
    def add_input(self, device):
        self.inputs.append(device)


    # Step function: checks the inputs (e.g. xbox controller) then updates swift. replaces env.step
    def step(self, *args):
        for device in self.inputs:
            device.poll()
        self.env.step(*args)

    # Launch Function: opens swift, adds robots
    def launch(self):
        self.env.launch(realtime=True)
        for mover in self.movers.values():
            mover.add_to_scene()
        self.env.step()

    # Safety State Functions: each step only works from the right state, so the robots can never restart by accident
    def emergency_stop(self):
        self.state = "ESTOPPED"
        print("STATE: ESTOPPED - all motion stopped")

    def reset(self):
        if self.state != "ESTOPPED":
            print("Reset ignored - e-stop is not active")
            return
        self.state = "RESET"
        print("STATE: RESET - press RESTART when the area is clear")

    def restart(self):
        if self.state != "RESET":
            print("Restart ignored - reset the e-stop first")
            return
        self.state = "RUNNING"
        print("STATE: RUNNING")


# Robot Mover Class
class RobotMover:

    # Constructor: the robot, its meshes + offsets, home pose, held items and collision ellipsoids
    def __init__(self, robot_setup, workcell):
        self.workcell = workcell
        self.robot = robot_setup.robot
        self.meshes = robot_setup.meshes
        self.offsets = robot_setup.offsets
        self.home_q = robot_setup.home_q
        self.held = []
        self.checker = CollisionChecker(self.robot)

    # Grab (ATTACHMENT): attaches an item to the end effector, same idea as the lab 4 pen
    def grab(self, item):
        T_ee = self.robot.fkine(self.robot.q).A
        self.held.append((item, np.linalg.inv(T_ee) @ np.array(item.T)))
        print("Grabbed item")

    # Release (ATTACHMENT): lets go of everything
    def release(self):
        self.held = []
        print("Released item")

    # Update Mesh Function: moves every mesh (and any held item) to match the DH robot. same function for all robots
    def update_meshes(self, q):
        link_check = self.robot.fkine_all(q)
        for i in range(len(self.meshes)):
            self.meshes[i].T = link_check[i].A @ self.offsets[i]
        if self.held:
            T_ee = self.robot.fkine(q).A
            for item, offset in self.held:
                item.T = T_ee @ offset

    # Adds the Robot to the Scene at its home pose
    def add_to_scene(self):
        env = self.workcell.env
        env.add(self.robot)
        self.robot.q = self.home_q
        env.step()
        self.update_meshes(self.home_q)
        for mesh in self.meshes:
            env.add(mesh)                     

    # Move Robot via giving Joint angles. checks the e-stop and collisions (lab 5) before every step. if it starts touching something it's let move so it can get out
    def move_to_q(self, q_goal, steps=50):
        traj = jtraj(self.robot.q, q_goal, steps)
        touching = self.check_collision(self.robot.q)

        for q_step in traj.q:
            if self.workcell.state != "RUNNING":
                print("Motion aborted")
                return False
            if not touching and self.check_collision(q_step):
                print("UNSAFE: Robot stopped, collision ahead")
                return False
            self.robot.q = q_step
            self.update_meshes(q_step)
            self.workcell.step()
        return True

    # Check Collision: lab 5 check_collision. hits if a joint goes under the table, or the other robots' links (as points) are inside my lab 6 ellipsoids
    def check_collision(self, q):
        if not self.workcell.collision_on:
            return False
        if not self.above_table(q):
            return True
        for other in self.workcell.movers.values():
            if other is self:
                continue
            points = other.checker.link_points(other.robot.q)
            if self.checker.collides(q, points):
                return True
        return False

    # Above Table: true if every joint is above the table top (links are straight, so checking the joints is enough)
    def above_table(self, q):
        return all(T.t[2] > TABLE_HEIGHT - 0.01 for T in self.robot.fkine_all(q)[1:])

    # SOLVE IK: tries from the current pose, then from home, with a fixed seed. only keeps answers above the table. raises ValueError if none work
    def solve_ik(self, T_target, q0=None):
        if q0 is None:
            q0 = self.robot.q
        for start in [q0, self.home_q]:
            ik_sol = self.robot.ikine_LM(T_target, q0=start, joint_limits=True, seed=0)
            if ik_sol.success and self.above_table(ik_sol.q):
                return ik_sol.q
        raise ValueError("Inverse kinematics failed to find a valid solution for the target pose!")

    # Move to Pose: IK then move_to_q. if a collision stops it, re-plan like the A1 lift step: up, across above the target, down (tries 0.2 then 0.4 m)
    def move_to_pose(self, T_target, steps=50):
        q_goal = self.solve_ik(T_target)
        if self.move_to_q(q_goal, steps):
            return True
        if self.workcell.state != "RUNNING":
            return False

        for lift in [0.2, 0.4]:
            print("Re-planning: going over the top at", lift, "m")
            try:
                q_up_here = self.solve_ik(SE3(0, 0, lift) * self.robot.fkine(self.robot.q))
                q_up_there = self.solve_ik(SE3(0, 0, lift) * T_target, q0=q_up_here)
                q_down = self.solve_ik(T_target, q0=q_up_there)
            except ValueError:
                continue
            if self.move_to_q(q_up_here, steps // 2) and self.move_to_q(q_up_there, steps) and self.move_to_q(q_down, steps // 2):
                return True
            if self.workcell.state != "RUNNING":
                return False
        print("No clear path - robot staying put")
        return False

    # PICK DROP: ABOVE, DOWN, GRAB (IF GIVEN AN ITEM) OR LET GO, BACK UP. STOPS ON E-STOP
    def pick_drop(self, xyz, item=None, lift=0.1, steps=50):
        T_spot = SE3(*xyz) * SE3.Rx(np.pi)
        T_above = SE3(0, 0, lift) * T_spot
        if not self.move_to_pose(T_above, steps):
            return False
        if not self.move_to_pose(T_spot, steps // 2):
            return False
        if item is not None:
            self.grab(item)
        else:
            self.release()
        return self.move_to_pose(T_above, steps // 2)

    # TUCK: PULLS THE EE BACK OVER ITS OWN BASE, OUT OF THE WAY (X FOLLOWS THE BASE ON THE RAIL)
    def tuck(self, y=0.0, height=0.7, steps=50):
        return self.move_to_pose(SE3(self.robot.base.t[0], y, height) * SE3.Rx(np.pi), steps)

    # MOVE RMRC (NEW)
    # RESOLVED MOTION RATE CONTROL, FOLLOWING THE LAB 9 SOLUTION (QUESTION 1). THE STEP NUMBERS BELOW MATCH THE LAB.
    # MOVES THE HAND IN A STRAIGHT LINE FROM WHERE IT IS NOW TO T_TARGET.
    # EVERY SMALL STEP IT:
    #   - WORKS OUT THE VELOCITY NEEDED TO REACH THE NEXT POINT ON THE LINE (FEEDBACK, SO DRIFT IS CORRECTED)
    #   - WEIGHTS THAT VELOCITY WITH W SO POSITION MATTERS MORE THAN ORIENTATION
    #   - USES THE JACOBIAN TO TURN IT INTO JOINT VELOCITIES (DAMPED LEAST SQUARES NEAR A SINGULARITY)
    #   - STOPS ANY JOINT THAT WOULD GO PAST ITS LIMIT, THEN MOVES THE JOINTS A SMALL STEP
    # WHAT IS DIFFERENT FROM THE LAB, AND WHY:
    #   - THE LAB COMPUTES EVERY STEP FIRST AND PLAYS IT AFTERWARDS. THIS PLAYS EACH STEP AS IT IS CALCULATED
    #     (SAME MATHS) SO THE E-STOP CAN INTERRUPT IT.
    #   - EPSILON IS SCALED BY THE ROBOT'S SIZE. THE LAB USES 0.1, WHICH SUITS THE PUMA 560. THE MANIPULABILITY
    #     VALUE GETS SMALLER FOR SMALLER ROBOTS, SO THE SAME 0.1 WOULD DAMP A UR3 ALL THE TIME.
    #   - THE LAB'S ORIENTATION STAYS FIXED. HERE THE ORIENTATION TURNS SMOOTHLY FROM THE START POSE TO THE TARGET POSE.
    #   - IT CHECKS THE HAND ARRIVED AND RETURNS TRUE OR FALSE (THE LAB JUST PLOTS).
    #   - NO PLOTTING: THE LAB'S RESULT ARRAYS ARE NOT SAVED ANY MORE, ONLY THE ARRAYS THE MOVEMENT NEEDS ARE KEPT.
    # ARGUMENTS:
    #   DURATION: SECONDS THE MOVE TAKES IN THE MATHS (THE LAB'S t)
    #   STEPS: HOW MANY STEPS THE MOVE IS SPLIT INTO
    #   TOLERANCE: HOW CLOSE (METRES) THE HAND MUST FINISH TO COUNT AS SUCCESS
    #   EPSILON: MANIPULABILITY BELOW THIS MEANS "NEAR A SINGULARITY". None = THE LAB'S 0.1 SCALED TO THIS ROBOT
    #   ORIENTATION_WEIGHT: THE 0.1 IN THE LAB'S W MATRIX. RAISE IT TO 1 IF THE FINAL ORIENTATION MATTERS MORE
    #   MAX_JOINT_STEP: OPTIONAL SAFE-SPEED LIMIT (RADIANS PER STEP). None = OFF, AS IN THE LAB
    #   MAX_DAMPING: THE 0.05 IN THE LAB'S DAMPING FORMULA. LOWER = MORE ACCURATE TRACKING, BUT LESS PROTECTION NEAR A SINGULARITY
    # RETURNS TRUE IF IT REACHED THE TARGET. RETURNS FALSE IF THE E-STOP STOPPED IT, OR IF IT FINISHED TOO FAR AWAY
    # (TARGET OUT OF REACH, OR BLOCKED BY A SINGULARITY OR JOINT LIMIT).
    def move_rmrc(self, T_target, duration=3.0, steps=60, tolerance=0.005,
                  epsilon=None, orientation_weight=0.1, max_joint_step=None, max_damping=0.05):
        robot = self.robot # SHORT NAME FOR THE ROBOT
        n = robot.n # NUMBER OF JOINTS

        # 1.1) SET PARAMETERS FOR THE SIMULATION
        delta_t = duration / (steps - 1) # DISCRETE TIME STEP (S)
        if epsilon is None: # NO EPSILON GIVEN?
            PUMA_REACH = 1.706 # THE REACH (AS ROBOTICS TOOLBOX MEASURES IT) OF THE LAB'S PUMA 560
            try:
                size = (robot.reach / PUMA_REACH) ** 3 # MANIPULABILITY GROWS WITH SIZE CUBED
            except Exception: # THIS ROBOT HAS NO REACH VALUE
                size = 1.0 # SO USE THE LAB'S VALUE AS IT IS
            epsilon = 0.1 * size # THE LAB'S 0.1, SCALED TO THIS ROBOT
        W = np.diag([1, 1, 1, orientation_weight, orientation_weight, orientation_weight]) # WEIGHTING MATRIX FOR THE VELOCITY VECTOR

        # 1.2) ALLOCATE ARRAY DATA
        q_matrix = np.zeros([steps, n]) # JOINT ANGLES
        qdot = np.zeros([steps, n]) # JOINT VELOCITIES

        # 1.3) SET UP TRAJECTORY, INITIAL POSE
        T_start = robot.fkine(robot.q) # WHERE THE HAND IS NOW (WORLD FRAME)
        s = trapezoidal(0, 1, steps).q # TRAPEZOIDAL TRAJECTORY SCALAR
        x = np.zeros([3, steps]) # POINTS ON THE STRAIGHT LINE
        R_want = [] # THE ORIENTATION WANTED AT EACH POINT
        for i in range(steps):
            x[:, i] = (1 - s[i]) * T_start.t + s[i] * T_target.t # STRAIGHT LINE FROM START TO TARGET
            R_want.append(T_start.interp(T_target, s[i]).R) # ORIENTATION TURNS SMOOTHLY (THE LAB KEEPS IT FIXED)
        q_matrix[0, :] = robot.q # START FROM WHERE THE ROBOT IS (THE LAB USES IK HERE)
        qlim = np.transpose(robot.qlim) # JOINT LIMITS, ONE ROW PER JOINT

        # 1.4) TRACK THE TRAJECTORY WITH RMRC
        for i in range(steps - 1):
            if self.workcell.state != "RUNNING": # SAME E-STOP CHECK AS MOVE_TO_Q
                print("RMRC motion aborted") # PRINT
                return False # STOP IMMEDIATELY, DROP THE REST OF THE LINE
            # (A COLLISION CHECK WOULD GO HERE, BEFORE THE ROBOT MOVES)

            T = robot.fkine(q_matrix[i, :]).A # FORWARD TRANSFORMATION AT THE CURRENT JOINT STATE
            delta_x = x[:, i + 1] - T[:3, 3] # POSITION ERROR FROM THE NEXT WAYPOINT
            Rd = R_want[i + 1] # THE NEXT ORIENTATION WANTED
            Ra = T[:3, :3] # CURRENT END-EFFECTOR ROTATION MATRIX
            Rdot = (1 / delta_t) * (Rd - Ra) # ROTATION MATRIX ERROR
            S = Rdot @ Ra.T # SKEW-SYMMETRIC MATRIX
            linear_velocity = (1 / delta_t) * delta_x
            angular_velocity = np.array([S[2, 1], S[0, 2], S[1, 0]]) # PICK THE ANGULAR VELOCITY OUT OF THE SKEW MATRIX
            xdot = W @ np.vstack((linear_velocity.reshape(3, 1),
                                  angular_velocity.reshape(3, 1))) # END-EFFECTOR VELOCITY TO REACH THE NEXT WAYPOINT
            J = robot.jacob0(q_matrix[i, :]) # JACOBIAN AT THE CURRENT JOINT STATE
            m = np.sqrt(np.linalg.det(J @ J.T)) # MANIPULABILITY: CLOSE TO 0 MEANS NEAR A SINGULARITY
            if m < epsilon: # IF MANIPULABILITY IS LESS THAN THE THRESHOLD
                m_lambda = (1 - m / epsilon) * max_damping # DAMPING GROWS AS WE GET CLOSER TO THE SINGULARITY (LAB: 0.05)
            else:
                m_lambda = 0
            inv_j = np.linalg.inv(J.T @ J + m_lambda * np.eye(n)) @ J.T # DAMPED LEAST SQUARES INVERSE
            qdot[i, :] = (inv_j @ xdot).T # SOLVE THE RMRC EQUATION
            if max_joint_step is not None: # OPTIONAL SAFE-SPEED LIMIT (OFF BY DEFAULT, NOT IN THE LAB)
                biggest = np.max(np.abs(qdot[i, :])) * delta_t
                if biggest > max_joint_step:
                    qdot[i, :] = qdot[i, :] * (max_joint_step / biggest)
            for j in range(n): # LOOP THROUGH THE JOINTS
                if q_matrix[i, j] + delta_t * qdot[i, j] < qlim[j, 0]: # NEXT ANGLE BELOW THE LIMIT...
                    qdot[i, j] = 0 # STOP THE MOTOR
                elif q_matrix[i, j] + delta_t * qdot[i, j] > qlim[j, 1]: # NEXT ANGLE ABOVE THE LIMIT...
                    qdot[i, j] = 0 # STOP THE MOTOR
            q_matrix[i + 1, :] = q_matrix[i, :] + delta_t * qdot[i, :] # NEXT JOINT STATE FROM THE JOINT VELOCITIES

            robot.q = q_matrix[i + 1, :] # THE LAB PLAYS THESE AT THE END. HERE EACH STEP IS PLAYED AS IT IS CALCULATED
            self.update_meshes(robot.q) # MOVE THE MESHES TO MATCH
            self.workcell.step() # CHECK INPUTS, UPDATE SWIFT

        # DID THE HAND ACTUALLY ARRIVE? (NOT IN THE LAB)
        error = np.linalg.norm(robot.fkine(q_matrix[-1, :]).t - T_target.t) # DISTANCE LEFT TO THE TARGET (METRES)
        if error > tolerance: # FINISHED TOO FAR AWAY?
            print("RMRC finished %.1f mm from the target (out of reach, near a singularity or at a joint limit)" % (error * 1000))
            return False
        return True # REACHED THE TARGET

    # Jog: nudges the end effector in x, y, z (GUI jog buttons)
    def jog(self, dx, dy, dz, steps=10):
        T_now = self.robot.fkine(self.robot.q)
        T_new = SE3(dx, dy, dz) * T_now
        try:
            q_goal = self.solve_ik(T_new)
        except ValueError:
            print("Jog out of reach - move cancelled")
            return False
        return self.move_to_q(q_goal, steps)