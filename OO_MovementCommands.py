
# Comment Author: Will
# The Intention of this file is to create a class containing functions that allow any robot set up correctly to share the same movement commands
# The file contains two classes:
# Workcell - the class sets up the swift environment and adds the robots, inputs & Emergency stop behaviour
# RobotMover - This contains all the actual movement commands that the robots can call
# For a robot to use these functions it requires .robot, .meshes, .offsets and .home_q

# For the Safety requirements, the file has a safety state. Running is default and allows movment
# Emergnecy stop stops the robot and blocks all movement until the restart button is pressed, which is irl equivalent of twisting e stop after pressing it
# This will allow robot movment again: RUNNING -> ESTOPPED -> RESET -> RUNNING

import swift                      
from roboticstoolbox import jtraj # Minimum jerk profile
from spatialmath import SE3       


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

    # Move to Pose. This is the Essential IK function we are using, calculates then using function above
    def move_to_pose(self, T_target, steps=50):
        ik_sol = self.robot.ikine_LM(T_target)    # solve inverse kinematics
        if not ik_sol.success:                    # if no solution was found...
            raise ValueError("Inverse kinematics failed to find a valid solution for the target pose!") #If Fails give reason why
        print("IK Solution found (radians):", ik_sol.q)  #print to terminal for debugging
        return self.move_to_q(ik_sol.q, steps)    # use function above to move to joints found

    # Jog for testing
    def jog(self, dx, dy, dz, steps=10):
        T_now = self.robot.fkine(self.robot.q)                # current end effector pose
        T_new = SE3(dx, dy, dz) * T_now                       # same pose, shifted in the world frame
        ik_sol = self.robot.ikine_LM(T_new, q0=self.robot.q)  # IK, starting from where we are
        if not ik_sol.success:                                # if it can't reach
            print("Jog out of reach - move cancelled")        # cancel move, dont do
            return False
        return self.move_to_q(ik_sol.q, steps)                # otherwise do the small movement