# ======================================================================
# OO_Main.py  --  the program you actually RUN
#
# Overview: makes the shared workcell, adds each robot to it, opens the
# simulator, adds the GUI and hands control to the GUI loop.
# To add a robot: import its setup class and add one add_robot() line.
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
from math import pi                                  # the constant pi
from spatialmath import SE3                          # poses (used for the target)
from OO_MovementCommands import Workcell                  # the shared scene
from OO_GUI import RobotGUI                             # the Swift GUI
from OO_WillRobotTest import KawasakiRS007N           # robot-specific setup class
from OO_JamesRobotTest import ABBIRB140              # robot-specific setup class
# from ur3_setup import UR3Setup                     # <- example: another robot's setup class

# --------------------------- MAIN PROGRAM -----------------------------
if __name__ == "__main__":                           # only run when this file is run directly
    workcell = Workcell()                            # the ONE shared scene
    workcell.add_robot("James' Robot", ABBIRB140())   # add the James Robot
    #workcell.add_robot("Will's Robot", KawasakiRS007N())   # add the Kawasaki under a name
    # workcell.add_robot("UR3", UR3Setup())          # <- add more robots the same way
    workcell.launch()                                # open Swift and show every robot

    T_target = SE3(-0.2, 0.1, 0.5) * SE3.Rx(pi)      # demo pose for the Kawasaki
    gui = RobotGUI(workcell, demo_targets={"James' Robot": T_target})  # robot name -> demo pose
    gui.build()                                      # add the buttons, dropdown and sliders
    gui.run()                                        # keep running so the GUI works (Ctrl+C to quit)