# Comment Author: Will
# Workcell = swift, robots, inputs and the e-stop states (RUNNING -> ESTOPPED -> RESET -> RUNNING). RobotMover = the shared movement commands (needs .robot .meshes .offsets .home_q)

import swift                      
import numpy as np
from roboticstoolbox import jtraj, trapezoidal
from spatialmath import SE3       
from OO_Collision import CollisionChecker
from math import pi 

TABLE_HEIGHT = 0.25 # top of every table. no robot joint is allowed below this
TABLE_CLEARANCE = 0.03 # links have thickness, so every joint away from the gripper stays this far above the table


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
        if self.checker.self_hit(q): # NEW: A LINK GOING THROUGH ITS OWN ROBOT
            return True
        for other in self.workcell.movers.values():
            if other is self:
                continue
            points = other.checker.link_points(other.robot.q)
            if self.checker.collides(q, points):
                return True
        return False

    # Above Table: the gripper end (within 10 cm of the tip) can go down to the table top so it can pick things up. every other joint stays TABLE_CLEARANCE above it
    def above_table(self, q):
        frames = self.robot.fkine_all(q)
        tip = self.robot.fkine(q).t
        for T in frames[1:]:
            if np.allclose(T.t, frames[0].t):
                continue
            near_tip = np.linalg.norm(T.t - tip) < 0.1
            if T.t[2] < TABLE_HEIGHT - 0.005 or (not near_tip and T.t[2] < TABLE_HEIGHT + TABLE_CLEARANCE):
                return False
        return True

    # SOLVE IK: tries from the current pose, then from home, with a fixed seed. only keeps answers above the table. raises ValueError if none work
    # SOLVE IK: TRIES SEVERAL STARTS, KEEPS ONLY ANSWERS INSIDE THE JOINT LIMITS AND ABOVE THE TABLE,
    # THEN PICKS THE ONE CLOSEST TO THE CURRENT JOINT ANGLES (AVOIDS WRIST FLIPS AND ODD PATHS)
    def solve_ik(self, T_target, q0=None):
        if q0 is None:
            q0 = self.robot.q
        q_now = np.array(self.robot.q, dtype=float)
        best_q = None
        best_dist = None
        for start in [q0, self.home_q]:
            ik_sol = self.robot.ikine_LM(T_target, q0=start, joint_limits=True, seed=0)
            if not ik_sol.success:
                continue
            q = np.array(ik_sol.q, dtype=float)
            q = self.closest_turn(q, q_now) # SAME POSE, BUT NO EXTRA FULL SPINS OF THE WRIST
            if np.any(q < self.robot.qlim[0]) or np.any(q > self.robot.qlim[1]):   # EXPLICIT LIMIT CHECK
                continue
            if not self.above_table(q):
                continue
            dist = np.linalg.norm(q - q_now)               # HOW FAR THE JOINTS HAVE TO TRAVEL
            if best_dist is None or dist < best_dist:
                best_q, best_dist = q, dist
        if best_q is None:
            raise ValueError("Inverse kinematics failed to find a valid solution for the target pose!")
        return best_q

    # CLOSEST TURN: q AND q +- 360 DEG IS THE SAME POSE. USE WHICHEVER IS CLOSEST TO q_now IF IT'S INSIDE THE LIMITS
    def closest_turn(self, q, q_now):
        q_close = q_now + (q - q_now + np.pi) % (2 * np.pi) - np.pi
        inside = (q_close >= self.robot.qlim[0]) & (q_close <= self.robot.qlim[1])
        return np.where(inside, q_close, q)

    # Move to Pose: IK then move_to_q. if a collision stops it, re-plan like the A1 lift step: up, across above the target, down (tries 0.2 then 0.4 m)
    def move_to_pose(self, T_target, steps=50):
        if self.workcell.state != "RUNNING":              # NEW: AFTER AN E-STOP, DON'T EVEN TRY THE IK
            return False
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
    def pick_drop(self, xyz, item=None, lift=0.1, steps=50, tool=SE3.Rx(np.pi)):
        T_spot = SE3(*xyz) * tool
        T_above = SE3(0, 0, lift) * T_spot
        if not self.move_to_pose(T_above, steps):
            return False
        if not self.move_to_pose(T_spot, steps // 2):
            return False
        if item is not None:
            for it in (item if isinstance(item, (list, tuple)) else [item]): # ONE ITEM OR A LIST OF THEM (E.G. GLASS + UMBRELLA)
                self.grab(it)
        else:
            self.release()
        return self.move_to_pose(T_above, steps // 2)

    # TUCK: PULLS THE EE BACK OVER ITS OWN BASE, OUT OF THE WAY (X FOLLOWS THE BASE ON THE RAIL)
    def tuck(self, y=0.0, height=0.7, steps=50):
        tuck_pos = [0, -pi/2, pi/2, 0, 0, 0]
        return self.move_to_q(tuck_pos)
        #return self.move_to_pose(SE3(self.robot.base.t[0], y, height) * SE3.Rx(np.pi), steps)

    # RMRC TO: STRAIGHT LINE WITH RMRC, NORMAL MOVE IF RMRC FAILS (SINGULARITY, JOINT LIMIT)
    def rmrc_to(self, T_target, orientation_weight=0.1, duration=1.0, steps=20):
        if self.move_rmrc(T_target, duration=duration, steps=steps, orientation_weight=orientation_weight):
            return True
        return self.workcell.state == "RUNNING" and self.move_to_pose(T_target)

    # SIDE GRIP: HAND ON THE SIDE OF A BOTTLE, GRIPPER POINTING ALONG direction (DEFAULT: STRAIGHT OUT FROM THE ROBOT)
    def side_grip(self, centre, radius=0.0, direction=None):
        d = np.array(direction if direction is not None else np.array(centre[:2]) - self.robot.base.t[:2], dtype=float)
        d = np.append(d[:2] / np.linalg.norm(d[:2]), 0.0) # GRIPPER POINTS THIS WAY, LEVEL
        x = np.array([0.0, 0.0, -1.0]) # TOOL X STRAIGHT DOWN
        return SE3.Rt(np.column_stack([x, np.cross(d, x), d]), np.array(centre) - d * radius), d

    # SIDE PICK DROP: COMES IN SIDEWAYS, GRABS THE BOTTLE LOW DOWN, LIFTS. NO ITEM = PUTS IT DOWN THERE AND BACKS OFF
    def side_pick_drop(self, centre, radius, item=None, back=0.1, lift=0.1):
        T_grip, d = self.side_grip(centre, radius)
        T_back = SE3(*(-d * back)) * T_grip # HAND PULLED BACK FROM THE BOTTLE
        T_up = SE3(0, 0, lift) * T_grip # HAND STRAIGHT ABOVE THE GRIP SPOT
        if item is not None:
            if not (self.move_to_pose(T_back) and self.rmrc_to(T_grip)): # SLIDE IN
                return False
            self.grab(item)
            return self.rmrc_to(T_up) # LIFT
        if not (self.move_to_pose(T_up) and self.rmrc_to(T_grip)): # LOWER ONTO THE TABLE
            return False
        self.release()
        return self.rmrc_to(T_back) # SLIDE OUT

    # CARRY: PICK UP AND PUT DOWN WITH RMRC, ONE JOINT MOVE ACROSS. PICK IK IS SEEDED FROM THE DROP SO THE ARM NEVER FLIPS SHAPE
    def carry(self, from_xyz, to_xyz, item, lift=0.1, tool=SE3.Rx(np.pi)):
        T_pick, T_drop = SE3(*from_xyz) * tool, SE3(*to_xyz) * tool
        T_up_pick, T_up_drop = SE3(0, 0, lift) * T_pick, SE3(0, 0, lift) * T_drop
        q_up_drop = self.solve_ik(T_up_drop, q0=self.home_q) # ARM SHAPE ABOVE THE DROP
        ik_sol = self.robot.ikine_LM(T_up_pick, q0=q_up_drop, joint_limits=True) # SAME SHAPE ABOVE THE PICK
        q_up_pick = self.closest_turn(ik_sol.q, q_up_drop) if ik_sol.success and self.above_table(ik_sol.q) else self.solve_ik(T_up_pick)
        if not (self.move_to_q(q_up_pick) and self.rmrc_to(T_pick)): # OVER IT, STRAIGHT DOWN
            return False
        self.grab(item)
        if not (self.rmrc_to(T_up_pick) and self.move_to_q(self.solve_ik(T_up_drop, q0=q_up_drop)) and self.rmrc_to(T_drop)): # UP, ACROSS, DOWN
            return False
        self.release()
        return self.rmrc_to(T_up_drop) # STRAIGHT BACK UP

    # NECK POUR: GRABS A BOTTLE BY ITS NECK AT neck_xyz, RMRC OVER THE SHAKER, TIPS IT, TIPS BACK, RMRC BACK AND PUTS IT DOWN
    def neck_pour(self, neck_xyz, item, over_xyz, length, tilt=-1.6, hold=20):
        T_up = SE3(over_xyz[0], over_xyz[1], over_xyz[2] + length + 0.03) * SE3.Rx(np.pi) # NECK OVER THE SHAKER, BOTTOM CLEARS THE RIM
        T_back = SE3(0, 0, 0.1) * SE3(*neck_xyz) * SE3.Rx(np.pi) # 10 CM ABOVE WHERE IT WAS PICKED UP
        if not (self.pick_drop(neck_xyz, item) and self.rmrc_to(T_up, duration=2.0, steps=40)):
            return False
        if not self.rmrc_to(T_up * SE3.Rx(tilt), orientation_weight=1.0, duration=2.0, steps=40): # RMRC TIP, HAND STAYS PUT
            return False
        for i in range(hold): # POURING. workcell.step SO THE E-STOP STILL WORKS
            if self.workcell.state != "RUNNING":
                return False
            self.workcell.step(0.05)
        return (self.rmrc_to(T_up, orientation_weight=1.0, duration=2.0, steps=40) and self.rmrc_to(T_back, duration=2.0, steps=40)
                and self.pick_drop(neck_xyz)) # TIP BACK, RMRC BACK, PUT IT DOWN

    # POUR: HOLDS THE BOTTLE BY ITS SIDE (radius, mouth = TOP ABOVE THE HAND) AND ROLLS THE WRIST WITH RMRC TO TIP IT OVER over_xyz
    def pour(self, over_xyz, radius, mouth, tilt=1.75, direction=None, hold=20):
        T_up, d = self.side_grip(over_xyz, radius, direction) # GRIPPER LEVEL, POINTING ALONG direction
        side = np.cross(d, [0, 0, 1]) # THE WAY THE TOP TIPS WHEN THE WRIST ROLLS
        T_up = SE3(*([0, 0, 0.04] - side * mouth * np.sin(tilt) - [0, 0, mouth * np.cos(tilt)])) * T_up # MOUTH ENDS 4 CM OVER THE SHAKER
        if not self.move_to_q(self.solve_ik(T_up, q0=self.home_q)): # IK FROM HOME, SO THE WRIST HAS ROOM TO ROLL
            if not self.move_to_pose(T_up): # COLLISION IN THE WAY: NORMAL MOVE WITH ITS RE-PLAN
                return False
        if not self.rmrc_to(T_up * SE3.Rz(tilt), orientation_weight=1.0, duration=2.0, steps=40): # ROLL THE WRIST, HAND STAYS PUT
            return False
        for i in range(hold): # POURING. workcell.step SO THE E-STOP STILL WORKS
            if self.workcell.state != "RUNNING":
                return False
            self.workcell.step(0.05)
        return self.rmrc_to(T_up, orientation_weight=1.0, duration=2.0, steps=40) # ROLL BACK UPRIGHT

    # MOVE RMRC: lab 9 Q1 RMRC, straight line from where the hand is now to T_target. step numbers match the lab
    # differences from the lab:
    #   - each step is played as it's calculated, so the e-stop and collision check can stop it mid-line
    #   - epsilon is the lab's 0.1 scaled by robot size (manipulability shrinks with size cubed), so small robots aren't always damped
    #   - orientation turns smoothly from the start pose to the target instead of staying fixed
    #   - returns True if the hand finished within 5 mm of the target, False if e-stopped, blocked or out of reach
    # orientation_weight: the 0.1 in the lab's W matrix. 1.0 when the final orientation matters (pouring, tipping)
    def move_rmrc(self, T_target, duration=3.0, steps=60, orientation_weight=0.1):
        robot = self.robot
        n = robot.n

        # 1.1) SET PARAMETERS FOR THE SIMULATION
        delta_t = duration / (steps - 1) # DISCRETE TIME STEP (S)
        epsilon = 0.1 * (robot.reach / 1.706) ** 3 # LAB'S 0.1, SCALED FROM THE PUMA 560'S REACH (1.706 M) TO THIS ROBOT
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
                print("RMRC motion aborted")
                return False

            T =robot.fkine(q_matrix[i, :]).A # FORWARD TRANSFORMATION AT THE CURRENT JOINT STATE
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
                m_lambda = (1 - m / epsilon) * 0.05 # DAMPING GROWS AS WE GET CLOSER TO THE SINGULARITY (LAB: 0.05)
            else:
                m_lambda = 0
            inv_j = np.linalg.inv(J.T @ J + m_lambda * np.eye(n)) @ J.T # DAMPED LEAST SQUARES INVERSE
            qdot[i, :] = (inv_j @ xdot).T # SOLVE THE RMRC EQUATION
            for j in range(n): # LOOP THROUGH THE JOINTS
                if q_matrix[i, j] + delta_t * qdot[i, j] < qlim[j, 0]: # NEXT ANGLE BELOW THE LIMIT...
                    qdot[i, j] = 0 # STOP THE MOTOR
                elif q_matrix[i, j] + delta_t * qdot[i, j] > qlim[j, 1]: # NEXT ANGLE ABOVE THE LIMIT...
                    qdot[i, j] = 0 # STOP THE MOTOR
            q_matrix[i + 1, :] = q_matrix[i, :] + delta_t * qdot[i, :] # NEXT JOINT STATE FROM THE JOINT VELOCITIES

            if self.check_collision(q_matrix[i + 1, :]): # same table + robot check as move_to_q
                print("UNSAFE: RMRC stopped, collision ahead")
                return False
            robot.q = q_matrix[i + 1, :] # PLAY THIS STEP NOW (THE LAB PLAYS THEM ALL AT THE END)
            self.update_meshes(robot.q)
            self.workcell.step()

        # DID THE HAND ACTUALLY ARRIVE? (NOT IN THE LAB)
        error = np.linalg.norm(robot.fkine(q_matrix[-1, :]).t - T_target.t) # DISTANCE LEFT TO THE TARGET (METRES)
        if error > 0.005: # MORE THAN 5 MM OFF
            print("RMRC finished %.1f mm from the target (out of reach, near a singularity or at a joint limit)" % (error * 1000))
            return False
        return True

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