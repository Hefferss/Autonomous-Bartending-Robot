# This is the Draft Main File. I think even if we should keep a copy of this regardless
# of whether we keep building on this because in the current state its a really good DH Parameter
# Test file - Will

# Makes the shared workcell, adds each robot to it, opens the
# simulator, adds the GUI and hands control to the GUI loop.
# To add a robot: import its setup class and add one add_robot() line.

# Imports
from math import pi                                  
from spatialmath import SE3                          
from OO_MovementCommands import Workcell              # WorkCell is the Scene setup.
# It is a class defined in the MovementCommands file and is responsible for setting up swift, Emergency stop behaviour, Inputs that GUI & Xbox buttons control and adding the robots.
from OO_GUI import RobotGUI                           # RobotGUI sets up the Swift GUI
# The GUI is held in its own file and the RobotGUI class builds the buttons and the sliders for swift. 
from OO_Controller import XboxController              # Xbox controller e-stop
# We add the xbox controller here so that we can specify which buttons do when controlling the robot
from OO_WillRobotTest import KawasakiRS007N           # Imports the Kawasaki (Will's) robot class
# The Robot class defines the DH parameters and mesh offsets for each robot in this case Will's Robot

# Main Program
if __name__ == "__main__":                           # This line ensures that the following code only runs when the file is run directly
    workcell = Workcell()                            # creates instance of the workcell class - 'workcell' now becomes the swift environment & allows us to use for example .add_robot suffixes to call functions
    workcell.add_robot("Kawasaki RS007N", KawasakiRS007N())  # creates instance of the kawasaki robot using the movementcommands add_robot function. adds the kawasaki to the scene with the name in the quotations
    workcell.launch()                                # Launch Swift with all the added robots

    T_target = SE3(-0.2, 0.1, 0.5) * SE3.Rx(pi)      # Test Pose for the robots within reach if they are placed at origin. Don't try pressing demo if robot not at origin currently it will break
    gui = RobotGUI(workcell, demo_targets={"Kawasaki RS007N": T_target})  # Takes the Swift env(workcell), & demo_target to create instance of the RobotGUI class
    # demo target takes the robot name that was specified when it was added and its target pose for testing. This is not a permanent setup but is required to make the demo button work in swift
    # To have multiple robots in the scene working with the GUI it would be required to have multiple in the dictionary
    gui.build()                                      # actually adds all the GUI buttons to Swift
    workcell.add_input(XboxController(on_estop=gui.on_estop,        # X button
                                      on_back=gui.on_reset,         # B button
                                      on_start=gui.on_restart))     # A button
    gui.run()                                        # keep running so the GUI works (Ctrl+C to quit)
    # This is a while loop in the GUI file which is contiously taking inputs so that the buttons and sliders work