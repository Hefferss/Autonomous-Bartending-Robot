# comment author: will
# workcell = swift, robots, inputs and the e-stop states (running -> estopped -> reset -> running). RobotMover = the shared movement commands (needs .robot .meshes .offsets .home_q)

import swift                      
import numpy as np
from roboticstoolbox import jtraj, trapezoidal
from spatialmath import SE3       
from OO_Collision import CollisionChecker
from math import pi 

TABLE_HEIGHT = 0.25 # top of every table. no robot joint is allowed below this
TABLE_CLEARANCE = 0.03 # links have thickness, so every joint away from the gripper stays this far above the table


# the workcell class sets up swift and adds the robot, input and safety states to the 3D environment
class Workcell:

    # constructor: swift window, safety state, robots, input devices
    def __init__(self):
        self.env = swift.Swift()
        self.state = "RUNNING"
        self.movers = {}
        self.inputs = []
        self.collision_on = False


    # add robot function: stores the robot under its name (the name is used in the GUI drop down box)
    def add_robot(self, name, robot_setup):
        print("Added robot:", name)
        mover = RobotMover(robot_setup, self)
        self.movers[name] = mover
        return mover

    # add input function: the device needs a poll() method, called on every step() below
    def add_input(self, device):
        self.inputs.append(device)


    # step function: checks the inputs (e.g. xbox controller) then updates swift. replaces env.step
    def step(self, *args):
        for device in self.inputs:
            device.poll()
        self.env.step(*args)

    # launch function: opens swift, adds robots
    def launch(self):
        self.env.launch(realtime=True)
        for mover in self.movers.values():
            mover.add_to_scene()
        self.env.step()

    # safety state functions: each step only works from the right state, so the robots can never restart by accident
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


# robot mover class
class RobotMover:

    # constructor: the robot, its meshes + offsets, home pose, held items and collision ellipsoids
    def __init__(self, robot_setup, workcell):
        self.workcell = workcell
        self.robot = robot_setup.robot
        self.meshes = robot_setup.meshes
        self.offsets = robot_setup.offsets
        self.home_q = robot_setup.home_q
        self.held = []
        self.checker = CollisionChecker(self.robot)

    # grab (attachment): attaches an item to the end effector, same idea as the lab 4 pen
    def grab(self, item):
        T_ee = self.robot.fkine(self.robot.q).A
        self.held.append((item, np.linalg.inv(T_ee) @ np.array(item.T)))
        print("Grabbed item")

    # release (attachment): lets go of everything
    def release(self):
        self.held = []
        print("Released item")

    # update mesh function: moves every mesh (and any held item) to match the DH robot. same function for all robots
    def update_meshes(self, q):
        link_check = self.robot.fkine_all(q)
        for i in range(len(self.meshes)):
            self.meshes[i].T = link_check[i].A @ self.offsets[i]
        if self.held:
            T_ee = self.robot.fkine(q).A
            for item, offset in self.held:
                item.T = T_ee @ offset

    # adds the robot to the scene at its home pose
    def add_to_scene(self):
        env = self.workcell.env
        env.add(self.robot)
        self.robot.q = self.home_q
        env.step()
        self.update_meshes(self.home_q)
        for mesh in self.meshes:
            env.add(mesh)                     

    # move robot via giving joint angles. checks the e-stop and collisions (lab 5) before every step. if it starts touching something it's let move so it can get out
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

    # check collision: lab 5 check_collision. hits if a joint goes under the table, or the other robots' links (as points) are inside my lab 6 ellipsoids
    def check_collision(self, q):
        if not self.workcell.collision_on:
            return False
        if not self.above_table(q):
            return True
        if self.checker.self_hit(q): # a link going through its own robot
            return True
        for other in self.workcell.movers.values():
            if other is self:
                continue
            points = other.checker.link_points(other.robot.q)
            if self.checker.collides(q, points):
                return True
        return False

    # above table: the gripper end (within 10 cm of the tip) can go down to the table top so it can pick things up. every other joint stays TABLE_CLEARANCE above it
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

    # solve IK: tries several starts, keeps only answers inside the joint limits and above the table,
    # then picks the one closest to the current joint angles (avoids wrist flips and odd paths)
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
            q = self.closest_turn(q, q_now) # same pose, but no extra full spins of the wrist
            if np.any(q < self.robot.qlim[0]) or np.any(q > self.robot.qlim[1]):   # explicit limit check
                continue
            if not self.above_table(q):
                continue
            dist = np.linalg.norm(q - q_now)               # how far the joints have to travel
            if best_dist is None or dist < best_dist:
                best_q, best_dist = q, dist
        if best_q is None:
            raise ValueError("Inverse kinematics failed to find a valid solution for the target pose!")
        return best_q

    # closest turn: q and q +- 360 deg is the same pose. use whichever is closest to q_now if it's inside the limits
    def closest_turn(self, q, q_now):
        q_close = q_now + (q - q_now + np.pi) % (2 * np.pi) - np.pi
        inside = (q_close >= self.robot.qlim[0]) & (q_close <= self.robot.qlim[1])
        return np.where(inside, q_close, q)

    # move to pose: IK then move_to_q. if a collision stops it, re-plan like the A1 lift step: up, across above the target, down (tries 0.2 then 0.4 m)
    def move_to_pose(self, T_target, steps=50):
        if self.workcell.state != "RUNNING": # after an e-stop, don't even try the IK
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

    # pick drop: above, down, grab (if given an item) or let go, back up. stops on e-stop
    def pick_drop(self, xyz, item=None, lift=0.1, steps=50, tool=SE3.Rx(np.pi)):
        T_spot = SE3(*xyz) * tool
        T_above = SE3(0, 0, lift) * T_spot
        if not self.move_to_pose(T_above, steps):
            return False
        if not self.move_to_pose(T_spot, steps // 2):
            return False
        if item is not None:
            for it in (item if isinstance(item, (list, tuple)) else [item]): # one item or a list of them (e.g. glass + umbrella)
                self.grab(it)
        else:
            self.release()
        return self.move_to_pose(T_above, steps // 2)

    # tuck: folds the arm up over its own base, out of the way (UR3 tuck pose)
    def tuck(self):
        return self.move_to_q([-pi/2, -pi/2, pi/2, -pi/2, -pi/2, 0])

    # wait: holds still for a few steps (e.g. while pouring). workcell.step so the e-stop still works. false if e-stopped
    def wait(self, steps=20):
        for i in range(steps):
            if self.workcell.state != "RUNNING":
                return False
            self.workcell.step(0.05)
        return True

    # RMRC to: straight line with RMRC, normal move if RMRC fails (singularity, joint limit)
    def rmrc_to(self, T_target, orientation_weight=0.1, duration=1.0, steps=20):
        if self.move_rmrc(T_target, duration=duration, steps=steps, orientation_weight=orientation_weight):
            return True
        return self.workcell.state == "RUNNING" and self.move_to_pose(T_target)

    # carry: pick up and put down with RMRC, one joint move across. pick IK is seeded from the drop so the arm never flips shape
    def carry(self, from_xyz, to_xyz, item, lift=0.1, tool=SE3.Rx(np.pi)):
        T_pick, T_drop = SE3(*from_xyz) * tool, SE3(*to_xyz) * tool
        T_up_pick, T_up_drop = SE3(0, 0, lift) * T_pick, SE3(0, 0, lift) * T_drop
        q_up_drop = self.solve_ik(T_up_drop, q0=self.home_q) # arm shape above the drop
        ik_sol = self.robot.ikine_LM(T_up_pick, q0=q_up_drop, joint_limits=True) # same shape above the pick
        q_up_pick = self.closest_turn(ik_sol.q, q_up_drop) if ik_sol.success and self.above_table(ik_sol.q) else self.solve_ik(T_up_pick)
        if not (self.move_to_q(q_up_pick) and self.rmrc_to(T_pick)): # over it, straight down
            return False
        self.grab(item)
        if not (self.rmrc_to(T_up_pick) and self.move_to_q(self.solve_ik(T_up_drop, q0=q_up_drop)) and self.rmrc_to(T_drop)): # up, across, down
            return False
        self.release()
        return self.rmrc_to(T_up_drop) # straight back up

    # neck pour: grabs a bottle by its neck at neck_xyz, RMRC over the shaker, tips it, tips back, RMRC back and puts it down
    def neck_pour(self, neck_xyz, item, over_xyz, length, tilt=-1.6, hold=20):
        T_up = SE3(over_xyz[0], over_xyz[1], over_xyz[2] + length + 0.03) * SE3.Rx(np.pi) # neck over the shaker, bottom clears the rim
        T_back = SE3(0, 0, 0.1) * SE3(*neck_xyz) * SE3.Rx(np.pi) # 10 cm above where it was picked up
        if not (self.pick_drop(neck_xyz, item) and self.rmrc_to(T_up, duration=2.0, steps=40)):
            return False
        if not self.rmrc_to(T_up * SE3.Rx(tilt), orientation_weight=1.0, duration=2.0, steps=40): # RMRC tip, hand stays put
            return False
        if not self.wait(hold): # pouring
            return False
        return (self.rmrc_to(T_up, orientation_weight=1.0, duration=2.0, steps=40) and self.rmrc_to(T_back, duration=2.0, steps=40)
                and self.pick_drop(neck_xyz)) # tip back, RMRC back, put it down

    # move RMRC: lab 9 Q1 RMRC, straight line from where the hand is now to T_target. step numbers match the lab
    # differences from the lab:
    #   - each step is played as it's calculated, so the e-stop and collision check can stop it mid-line
    #   - epsilon is the lab's 0.1 scaled by robot size (manipulability shrinks with size cubed), so small robots aren't always damped
    #   - orientation turns smoothly from the start pose to the target instead of staying fixed
    #   - returns true if the hand finished within 5 mm of the target, false if e-stopped, blocked or out of reach
    # orientation_weight: the 0.1 in the lab's w matrix. 1.0 when the final orientation matters (pouring, tipping)
    def move_rmrc(self, T_target, duration=3.0, steps=60, orientation_weight=0.1):
        robot = self.robot
        num_joints = robot.n

        # 1.1) set parameters
        dt = duration / (steps - 1) # time per step (s)
        singularity_threshold = 0.1 * (robot.reach / 1.706) ** 3 # lab's epsilon 0.1, scaled from the puma 560'S reach (1.706 m) to this robot
        max_damping = 0.05 # lab's max damping (lambda)
        weights = np.diag([1, 1, 1, orientation_weight, orientation_weight, orientation_weight]) # lab's w: how much position vs orientation error matters

        # 1.2) allocate arrays
        q_path = np.zeros([steps, num_joints]) # joint angles at each step
        q_velocity = np.zeros([steps, num_joints]) # joint velocities at each step

        # 1.3) set up the straight line
        T_start = robot.fkine(robot.q) # where the hand is now (world frame)
        progress = trapezoidal(0, 1, steps).q # 0 -> 1 along the line, speeds up, cruises, slows down
        line_points = np.zeros([3, steps]) # points on the straight line
        rotations_wanted = [] # the orientation wanted at each point
        for i in range(steps):
            line_points[:, i] = (1 - progress[i]) * T_start.t + progress[i] * T_target.t # straight line from start to target
            rotations_wanted.append(T_start.interp(T_target, progress[i]).R) # orientation turns smoothly (the lab keeps it fixed)
        q_path[0, :] = robot.q # start from where the robot is (the lab uses IK here)
        joint_limits = np.transpose(robot.qlim) # one row per joint: [min, max]

        # 1.4) follow the line with RMRC, one step at a time
        for i in range(steps - 1):
            if self.workcell.state != "RUNNING": # same e-stop check as MOVE_TO_Q
                print("RMRC motion aborted")
                return False

            # how fast the hand needs to move to reach the next point on the line
            T_now = robot.fkine(q_path[i, :]).A # where the hand is at this step
            position_error = line_points[:, i + 1] - T_now[:3, 3] # gap to the next point on the line
            R_wanted = rotations_wanted[i + 1] # the next orientation wanted
            R_now = T_now[:3, :3] # current hand orientation
            R_rate = (1 / dt) * (R_wanted - R_now) # how fast the rotation needs to change
            skew = R_rate @ R_now.T # skew-symmetric matrix, holds the angular velocity
            linear_velocity = (1 / dt) * position_error
            angular_velocity = np.array([skew[2, 1], skew[0, 2], skew[1, 0]]) # pull the angular velocity out of the skew matrix
            hand_velocity = weights @ np.vstack((linear_velocity.reshape(3, 1),
                                                 angular_velocity.reshape(3, 1))) # 6x1 hand velocity (the lab's xdot)

            # turn hand velocity into joint velocities, with damping near a singularity
            J = robot.jacob0(q_path[i, :]) # jacobian: how each joint moves the hand, at this pose
            manipulability = np.sqrt(np.linalg.det(J @ J.T)) # how freely the hand can move. 0 = at a singularity
            if manipulability < singularity_threshold: # near a singularity: fade in damping, more the closer we get
                damping = (1 - manipulability / singularity_threshold) * max_damping
            else: # far from one: no damping, plain RMRC
                damping = 0
            J_inverse = np.linalg.inv(J.T @ J + damping * np.eye(num_joints)) @ J.T # damped least squares: the damping stops the inverse blowing up
            q_velocity[i, :] = (J_inverse @ hand_velocity).T # joint velocities = j inverse * hand velocity

            for j in range(num_joints): # stop any joint that would go past its limit
                if q_path[i, j] + dt * q_velocity[i, j] < joint_limits[j, 0]:
                    q_velocity[i, j] = 0
                elif q_path[i, j] + dt * q_velocity[i, j] > joint_limits[j, 1]:
                    q_velocity[i, j] = 0
            q_path[i + 1, :] = q_path[i, :] + dt * q_velocity[i, :] # next joint angles = now + velocity * time

            if self.check_collision(q_path[i + 1, :]): # same table + robot check as move_to_q
                print("UNSAFE: RMRC stopped, collision ahead")
                return False
            robot.q = q_path[i + 1, :] # play this step now (the lab plays them all at the end)
            self.update_meshes(robot.q)
            self.workcell.step()

        # did the hand actually arrive?
        distance_left = np.linalg.norm(robot.fkine(q_path[-1, :]).t - T_target.t) # metres from the target
        if distance_left > 0.005: # more than 5 mm off
            print("RMRC finished %.1f mm from the target (out of reach, near a singularity or at a joint limit)" % (distance_left * 1000))
            return False
        return True

    # jog: nudges the end effector in x, y, z (GUI jog buttons)
    def jog(self, dx, dy, dz, steps=10):
        T_now = self.robot.fkine(self.robot.q)
        T_new = SE3(dx, dy, dz) * T_now
        try:
            q_goal = self.solve_ik(T_new)
        except ValueError:
            print("Jog out of reach - move cancelled")
            return False
        return self.move_to_q(q_goal, steps)