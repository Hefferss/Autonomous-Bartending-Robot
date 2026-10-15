# Comment Author: Will
# The Intention of this file is to create a class containing functions that allow any robot set up correctly to share the same movement commands
# The file contains two classes:
# Workcell - the class sets up the swift environment and adds the robots, inputs & Emergency stop behaviour
# RobotMover - This contains all the actual movement commands that the robots can call
# For a robot to use these functions it requires .robot, .meshes, .offsets and .home_q

# For the Safety requirements, the file has a safety state. Running is default and allows movment
# Emergnecy stop stops the robot and blocks all movement until the restart button is pressed, which is irl equivalent of twisting e stop after pressing it
# This will allow robot movment again: RUNNING -> ESTOPPED -> RESET -> RUNNING

# NEW IN THIS VERSION (ALL COMMENTS IN CAPITALS ARE NEW):
# SOLVE_IK - ONE PLACE WHERE INVERSE KINEMATICS IS SOLVED, SO EVERY MOVE GIVES THE SAME ANSWER EVERY TIME
# MOVE_RMRC - STRAIGHT LINE MOVEMENT USING RESOLVED MOTION RATE CONTROL. FOLLOWS THE LAB 9 SOLUTION (QUESTION 1) STEP BY STEP
# PLOT_LAST_RMRC - DRAWS THE SAME RESULT PLOTS AS THE LAB (JOINT ANGLES, JOINT VELOCITIES, ERRORS, MANIPULABILITY)
# MOVE_TO_POSE AND JOG NOW CALL SOLVE_IK INSTEAD OF CALLING IKINE_LM THEMSELVES

import swift                      
import numpy as np                # NEW: NUMPY, USED FOR THE RMRC MATHS (ARRAYS, SINGULAR VALUES, CLIPPING)
from roboticstoolbox import jtraj, trapezoidal # Minimum jerk profile | NEW: TRAPEZOIDAL GIVES A SMOOTH 0 TO 1 SPEED PROFILE FOR RMRC
from spatialmath import SE3       
from spatialmath.base import tr2rpy     # NEW: TURNS A ROTATION MATRIX INTO ROLL-PITCH-YAW ANGLES (USED FOR THE RMRC ANGLE ERROR, AS IN THE LAB)


# The workcell class sets up swift and adds the robot, input and safety states to the 3D environment
class Workcell:

    # This is the constructor that allows the object that uses the workcell class to have these variables that are 
    # Required for the fucntions
    def __init__(self):
        self.env = swift.Swift()              # Creates simulation object - doesnt run until swift launches
        self.state = "RUNNING"                # Establishes the state as "RUNNING" string for If statements Later
        self.movers = {}                      # The Dictionary that stores both a robots given name and its object from the robot's class file
        self.inputs = []                      # List to store input devices (like the Xbox controller) to check every step uisng poll() - the function that determines whether its been pressed 


    # Add Robot Function
    # Gets called in the Main file, adds the robot dicitonary to be called later
    # The Robot is stored with name and the robot class where robot setup parameter accepts robots class (Which robot to use)
    # The name is required for the robot GUI drop down box
    def add_robot(self, name, robot_setup):
        print("Added robot:", name)                       # Print to terminal for debugging
        mover = RobotMover(robot_setup, self)             # give the mover the setup AND this workcell
        self.movers[name] = mover                         # stores the name and robot in the dictionary set up by constructor 
        return mover                                      # returns value

    # Add Input Function
    # The device needs a poll() method. It is called on every step() below.
    def add_input(self, device):
        self.inputs.append(device)            # appends the list with the device


    # Step function
    # This function is responsilbe for checking whether any inputs have happened
    # Runs each step of the animation
    # *args is a number e.g. step(0.05)
    # Replaces the original swift function
    def step(self, *args):
        for device in self.inputs:            # See each input device
            device.poll()                     # Check for Presses
        self.env.step(*args)                  # Update the window, read the GUI

    # Launch Function
    # Opens swift, adds robots
    def launch(self):
        self.env.launch(realtime=True)        # open the simulator window
        for mover in self.movers.values():    # go through every robot - takes the objects from dictionary
            mover.add_to_scene()              # add to swift at robots home pose
        self.env.step()                       # refresh the window

    # ---- Safety state functions: used by the GUI buttons ----------------
    # Overview: three separate steps, each only allowed from the right
    # state, so the robots can never restart by accident.

    # Safety State Functions
    # Determines Behaviour of the robot depending on the state
    def emergency_stop(self):
        self.state = "ESTOPPED"               # E Stop Doesn't need IF statement as state wise robot can be stopped at anytime
        print("STATE: ESTOPPED - all motion stopped")

    def reset(self):
        if self.state != "ESTOPPED":          # reset only makes sense after an e-stop
            print("Reset ignored - e-stop is not active") #If statement catches whether behaviour makes sense
            return
        self.state = "RESET"                  # e-stop cleared, functionally button twisted so it can be pressed again, but robots are still blocked
        print("STATE: RESET - press RESTART when the area is clear")

    def restart(self):
        if self.state != "RESET":             # Must be Reset button before restart can be pressed
            print("Restart ignored - reset the e-stop first") #IF Statement catches restart pressed before it should be
            return
        self.state = "RUNNING"                # Movement is allowed again.
        print("STATE: RUNNING")


# Robot Mover Class
class RobotMover:

    # This is the constructor that defines what info required from robot for movement to work
    def __init__(self, robot_setup, workcell):
        self.workcell = workcell              # It Requires the Swift environment in the 
        self.robot = robot_setup.robot        # The original DH Robot from AT1 and weekly lab same setup
        self.meshes = robot_setup.meshes      # Requires STL mesh list to update
        self.offsets = robot_setup.offsets    # The list of Mesh offsets for updating
        self.home_q = robot_setup.home_q      # The Starting Pose for the 'return to home' button
        self.held = [] # ATTACHMENT: items this robot is carrying, as (mesh, offset from the EE) pairs

    # Grab (ATTACHMENT)
    # Attaches an item to the end effector, same idea as the lab 4 pen: item pose = fkine().A @ offset
    def grab(self, item):
        T_ee = self.robot.fkine(self.robot.q).A # where the end effector is now
        self.held.append((item, np.linalg.inv(T_ee) @ np.array(item.T))) # item pose relative to the EE
        print("Grabbed item")

    # Release (ATTACHMENT)
    def release(self):
        self.held = []
        print("Released item")

    # Update Mesh Function
    # This is required for the Meshes to match the DH Parameters & Cylinder bot
    # This was formely done in the movement loop and in the robot file now it is seperated here
    # So that it is easy to edit, but it works the same way, only difference is its using a for loop to
    # update each mesh rather than having a line to each one. This also allows same function for all robots
    def update_meshes(self, q): # Accepts Joint Position as Parameters
        link_check = self.robot.fkine_all(q)  # list that has position of every link using forward kinematic
        for i in range(len(self.meshes)):     # go through each mesh one by one
            # link i position  @  alignment offset  ->  mesh position
            self.meshes[i].T = link_check[i].A @ self.offsets[i] # the mesh postion gets updated by mulyiplyin mathmatical location to the offset
        # ATTACHMENT: held items follow the end effector. every move (jtraj, rmrc, sliders, ur3 rail) calls this so they all carry items
        if self.held:
            T_ee = self.robot.fkine(q).A # end effector at this step
            for item, offset in self.held:
                item.T = T_ee @ offset # same as the lab 4 pen

    # Adds the Robot to the Scene
    # The Original add robot function just to dictionary that is used 
    def add_to_scene(self):
        env = self.workcell.env               # the swift env
        env.add(self.robot)                   # add the robot mathmaticallu
        self.robot.q = self.home_q            # Use Home pose
        env.step()                            # Update Swift
        self.update_meshes(self.home_q)       # Use update mesh function
        for mesh in self.meshes:              # Adds the meshes to the robot in the sim
            env.add(mesh)                     

    # Move Robot via giving Joint angles to move to
    # Before every step it checks the safety state and stops if its meant to
    # Used for Sliders in the GUI
    def move_to_q(self, q_goal, steps=50):
        traj = jtraj(self.robot.q, q_goal, steps) # jtrag from weekly labs. Minimum jerk movemet profile

        for q_step in traj.q:                     # In between each step in the movement
            if self.workcell.state != "RUNNING":  # If its not meant to be moving, dont move
                print("Motion aborted")           # Print
                return False                      # End function cancelled, didnt reach end of movement
            self.robot.q = q_step                 # set the robot to this step
            self.update_meshes(q_step)            # move the meshes
            self.workcell.step()                  # check inputs, updates swift
        return True                               # reached the end of movement

    # SOLVE IK (NEW)
    # THE ONE PLACE INVERSE KINEMATICS IS SOLVED. MOVE_TO_POSE AND JOG BOTH USE IT.
    # WHY IT GIVES THE SAME ANSWER EVERY TIME:
    #   - Q0 = CURRENT POSE: THE SOLVER STARTS FROM WHERE THE ROBOT IS, SO IT FINDS THE NEAREST SOLUTION.
    #     WITHOUT Q0, IKINE_LM STARTS FROM RANDOM JOINT ANGLES ON EVERY CALL, WHICH CAN FLIP THE ELBOW OR WRIST.
    #   - SEED = 0: IF THE FIRST TRY FAILS THE SOLVER RESTARTS FROM RANDOM ANGLES. A FIXED SEED MAKES THOSE
    #     RESTARTS THE SAME EVERY RUN.
    #   - JOINT_LIMITS = TRUE: NEVER RETURNS ANGLES OUTSIDE THE JOINT LIMITS.
    # RAISES VALUEERROR IF THERE IS NO SOLUTION (TARGET OUT OF REACH).
    def solve_ik(self, T_target, q0=None):
        if q0 is None:                                    # NO START POSE GIVEN?
            q0 = self.robot.q                             # USE THE ROBOT'S CURRENT POSE
        ik_sol = self.robot.ikine_LM(T_target, q0=q0, joint_limits=True, seed=0)    # solve inverse kinematics | NEW: START POSE, JOINT LIMITS AND FIXED SEED ADDED
        if not ik_sol.success:                    # if no solution was found...
            raise ValueError("Inverse kinematics failed to find a valid solution for the target pose!") #If Fails give reason why
        return ik_sol.q                                   # THE JOINT ANGLES THAT REACH THE TARGET

    # Move to Pose. This is the Essential IK function we are using, calculates then using function above
    # NOW USES SOLVE_IK ABOVE. THIS IS A JOINT-SPACE MOVE: EVERY JOINT BLENDS SMOOTHLY FROM START TO END,
    # SO THE HAND TRAVELS ALONG A CURVE. USE MOVE_RMRC BELOW WHEN THE PATH OF THE HAND MATTERS.
    def move_to_pose(self, T_target, steps=50):
        q_goal = self.solve_ik(T_target)          # solve inverse kinematics | NOW DONE BY SOLVE_IK
        print("IK Solution found (radians):", q_goal)  #print to terminal for debugging
        return self.move_to_q(q_goal, steps)    # use function above to move to joints found

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
    # THE LAB'S RESULT ARRAYS ARE SAVED IN self.last_rmrc SO THEY CAN BE PLOTTED WITH plot_last_rmrc().
    def move_rmrc(self, T_target, duration=3.0, steps=60, tolerance=0.005,
                  epsilon=None, orientation_weight=0.1, max_joint_step=None, max_damping=0.05):
        robot = self.robot                                # SHORT NAME FOR THE ROBOT
        n = robot.n                                       # NUMBER OF JOINTS

        # 1.1) SET PARAMETERS FOR THE SIMULATION
        delta_t = duration / (steps - 1)                  # DISCRETE TIME STEP (S)
        if epsilon is None:                               # NO EPSILON GIVEN?
            PUMA_REACH = 1.706                            # THE REACH (AS ROBOTICS TOOLBOX MEASURES IT) OF THE LAB'S PUMA 560
            try:
                size = (robot.reach / PUMA_REACH) ** 3    # MANIPULABILITY GROWS WITH SIZE CUBED
            except Exception:                             # THIS ROBOT HAS NO REACH VALUE
                size = 1.0                                # SO USE THE LAB'S VALUE AS IT IS
            epsilon = 0.1 * size                          # THE LAB'S 0.1, SCALED TO THIS ROBOT
        W = np.diag([1, 1, 1, orientation_weight, orientation_weight, orientation_weight])   # WEIGHTING MATRIX FOR THE VELOCITY VECTOR

        # 1.2) ALLOCATE ARRAY DATA
        m = np.zeros([steps, 1])                          # MEASURE OF MANIPULABILITY
        q_matrix = np.zeros([steps, n])                   # JOINT ANGLES
        qdot = np.zeros([steps, n])                       # JOINT VELOCITIES
        position_error = np.zeros([3, steps])             # TRAJECTORY ERROR, FOR PLOTTING
        angle_error = np.zeros([3, steps])                # TRAJECTORY ERROR, FOR PLOTTING

        # 1.3) SET UP TRAJECTORY, INITIAL POSE
        T_start = robot.fkine(robot.q)                    # WHERE THE HAND IS NOW (WORLD FRAME)
        s = trapezoidal(0, 1, steps).q                    # TRAPEZOIDAL TRAJECTORY SCALAR
        x = np.zeros([3, steps])                          # POINTS ON THE STRAIGHT LINE
        R_want = []                                       # THE ORIENTATION WANTED AT EACH POINT
        for i in range(steps):
            x[:, i] = (1 - s[i]) * T_start.t + s[i] * T_target.t   # STRAIGHT LINE FROM START TO TARGET
            R_want.append(T_start.interp(T_target, s[i]).R)        # ORIENTATION TURNS SMOOTHLY (THE LAB KEEPS IT FIXED)
        q_matrix[0, :] = robot.q                          # START FROM WHERE THE ROBOT IS (THE LAB USES IK HERE)
        qlim = np.transpose(robot.qlim)                   # JOINT LIMITS, ONE ROW PER JOINT

        # 1.4) TRACK THE TRAJECTORY WITH RMRC
        done = 0                                          # HOW MANY STEPS HAVE BEEN COMPLETED
        for i in range(steps - 1):
            if self.workcell.state != "RUNNING":          # SAME E-STOP CHECK AS MOVE_TO_Q
                print("RMRC motion aborted")              # PRINT
                self.last_rmrc = self._rmrc_results(m, q_matrix, qdot, position_error, angle_error, epsilon, done)
                return False                              # STOP IMMEDIATELY, DROP THE REST OF THE LINE
            # (A COLLISION CHECK WOULD GO HERE, BEFORE THE ROBOT MOVES)

            T = robot.fkine(q_matrix[i, :]).A             # FORWARD TRANSFORMATION AT THE CURRENT JOINT STATE
            delta_x = x[:, i + 1] - T[:3, 3]              # POSITION ERROR FROM THE NEXT WAYPOINT
            Rd = R_want[i + 1]                            # THE NEXT ORIENTATION WANTED
            Ra = T[:3, :3]                                # CURRENT END-EFFECTOR ROTATION MATRIX
            Rdot = (1 / delta_t) * (Rd - Ra)              # ROTATION MATRIX ERROR
            S = Rdot @ Ra.T                               # SKEW-SYMMETRIC MATRIX
            linear_velocity = (1 / delta_t) * delta_x
            angular_velocity = np.array([S[2, 1], S[0, 2], S[1, 0]])   # PICK THE ANGULAR VELOCITY OUT OF THE SKEW MATRIX
            delta_theta = tr2rpy(Rd @ Ra.T, order='xyz')  # ROTATION ERROR AS ROLL-PITCH-YAW ANGLES
            xdot = W @ np.vstack((linear_velocity.reshape(3, 1),
                                  angular_velocity.reshape(3, 1)))   # END-EFFECTOR VELOCITY TO REACH THE NEXT WAYPOINT
            J = robot.jacob0(q_matrix[i, :])              # JACOBIAN AT THE CURRENT JOINT STATE
            m[i] = np.sqrt(np.linalg.det(J @ J.T))        # MANIPULABILITY: CLOSE TO 0 MEANS NEAR A SINGULARITY
            if m[i] < epsilon:                            # IF MANIPULABILITY IS LESS THAN THE THRESHOLD
                m_lambda = (1 - m[i] / epsilon) * max_damping   # DAMPING GROWS AS WE GET CLOSER TO THE SINGULARITY (LAB: 0.05)
            else:
                m_lambda = 0
            inv_j = np.linalg.inv(J.T @ J + m_lambda * np.eye(n)) @ J.T    # DAMPED LEAST SQUARES INVERSE
            qdot[i, :] = (inv_j @ xdot).T                 # SOLVE THE RMRC EQUATION
            if max_joint_step is not None:                # OPTIONAL SAFE-SPEED LIMIT (OFF BY DEFAULT, NOT IN THE LAB)
                biggest = np.max(np.abs(qdot[i, :])) * delta_t
                if biggest > max_joint_step:
                    qdot[i, :] = qdot[i, :] * (max_joint_step / biggest)
            for j in range(n):                            # LOOP THROUGH THE JOINTS
                if q_matrix[i, j] + delta_t * qdot[i, j] < qlim[j, 0]:     # NEXT ANGLE BELOW THE LIMIT...
                    qdot[i, j] = 0                        # STOP THE MOTOR
                elif q_matrix[i, j] + delta_t * qdot[i, j] > qlim[j, 1]:   # NEXT ANGLE ABOVE THE LIMIT...
                    qdot[i, j] = 0                        # STOP THE MOTOR
            q_matrix[i + 1, :] = q_matrix[i, :] + delta_t * qdot[i, :]     # NEXT JOINT STATE FROM THE JOINT VELOCITIES
            position_error[:, i] = x[:, i + 1] - T[:3, 3] # FOR PLOTTING
            angle_error[:, i] = delta_theta               # FOR PLOTTING

            robot.q = q_matrix[i + 1, :]                  # THE LAB PLAYS THESE AT THE END. HERE EACH STEP IS PLAYED AS IT IS CALCULATED
            self.update_meshes(robot.q)                   # MOVE THE MESHES TO MATCH
            self.workcell.step()                          # CHECK INPUTS, UPDATE SWIFT
            done = i + 1

        self.last_rmrc = self._rmrc_results(m, q_matrix, qdot, position_error, angle_error, epsilon, done)

        # DID THE HAND ACTUALLY ARRIVE? (NOT IN THE LAB)
        error = np.linalg.norm(robot.fkine(q_matrix[-1, :]).t - T_target.t)   # DISTANCE LEFT TO THE TARGET (METRES)
        if error > tolerance:                             # FINISHED TOO FAR AWAY?
            print("RMRC finished %.1f mm from the target (out of reach, near a singularity or at a joint limit)" % (error * 1000))
            return False
        return True                                       # REACHED THE TARGET

    # SAVES THE LAB'S RESULT ARRAYS (CUT TO THE STEPS THAT WERE ACTUALLY DONE) FOR PLOTTING
    def _rmrc_results(self, m, q_matrix, qdot, position_error, angle_error, epsilon, done):
        k = max(done, 1)                                  # AT LEAST ONE ROW
        return {"m": m[:k], "q": q_matrix[:k + 1], "qdot": qdot[:k], "epsilon": epsilon,
                "position_error": position_error[:, :k], "angle_error": angle_error[:, :k]}

    # PLOT LAST RMRC (NEW)
    # DRAWS THE SAME PLOTS AS THE LAB FOR THE MOST RECENT move_rmrc CALL: JOINT ANGLES (WITH THEIR LIMITS),
    # JOINT VELOCITIES, POSITION AND ANGLE ERROR, AND MANIPULABILITY (WITH EPSILON). GOOD EVIDENCE FOR THE DEMO AND VIDEO.
    def plot_last_rmrc(self):
        import matplotlib.pyplot as plt                   # ONLY NEEDED HERE, SO IMPORTED HERE
        if not hasattr(self, "last_rmrc"):                # NOTHING TO PLOT YET?
            print("No RMRC move has been run yet")
            return
        r = self.last_rmrc                                # THE SAVED ARRAYS
        qlim = np.transpose(self.robot.qlim)              # JOINT LIMITS
        n = self.robot.n
        rows = int(np.ceil(n / 2))

        plt.figure("RMRC joint angles")
        for j in range(n):
            plt.subplot(rows, 2, j + 1)
            plt.plot(r["q"][:, j], 'k', linewidth=1)
            plt.title("Joint %d" % (j + 1)); plt.ylabel("Angle (rad)")
            plt.axhline(qlim[j, 0], color='r'); plt.axhline(qlim[j, 1], color='r')
        plt.tight_layout()

        plt.figure("RMRC joint velocities")
        for j in range(n):
            plt.subplot(rows, 2, j + 1)
            plt.plot(r["qdot"][:, j], 'k', linewidth=1)
            plt.title("Joint %d" % (j + 1)); plt.ylabel("Velocity (rad/s)")
            plt.axhline(0, color='r')
        plt.tight_layout()

        plt.figure("RMRC tracking error")
        plt.subplot(2, 1, 1)
        plt.plot(r["position_error"].T * 1000, linewidth=1); plt.axhline(0, color='k')
        plt.xlabel("Step"); plt.ylabel("Position error (mm)"); plt.legend(["X-Axis", "Y-Axis", "Z-Axis"])
        plt.subplot(2, 1, 2)
        plt.plot(r["angle_error"].T, linewidth=1); plt.axhline(0, color='k')
        plt.xlabel("Step"); plt.ylabel("Angle error (rad)"); plt.legend(["Roll", "Pitch", "Yaw"])

        plt.figure("RMRC manipulability")
        plt.plot(r["m"], 'k', linewidth=1); plt.axhline(r["epsilon"])
        plt.title("Manipulability (line = epsilon)")
        plt.show(block=False)                             # SHOW THE PLOTS WITHOUT FREEZING THE PROGRAM
        plt.pause(0.1)

    # Jog for testing
    # NOW USES SOLVE_IK, SO A JOG GIVES THE SAME RESULT AS EVERY OTHER MOVE
    def jog(self, dx, dy, dz, steps=10):
        T_now = self.robot.fkine(self.robot.q)                # current end effector pose
        T_new = SE3(dx, dy, dz) * T_now                       # same pose, shifted in the world frame
        try:                                                  # SOLVE_IK RAISES VALUEERROR WHEN THERE IS NO SOLUTION
            q_goal = self.solve_ik(T_new)                     # IK, starting from where we are
        except ValueError:                                    # if it can't reach
            print("Jog out of reach - move cancelled")        # cancel move, dont do
            return False
        return self.move_to_q(q_goal, steps)                # otherwise do the small movement