from math import pi
from spatialmath import SE3 
from OO_MovementCommands import Workcell
from OO_GUI import RobotGUI
from OO_WillRobotTest import KawasakiRS007N#Will's robot
from OO_JamesRobotTest import ABBIRB140#james' robot
from spatialmath.base import transl #needed to add this to move robots and such
from OO_JackRobotTest import StaubliTX60 #jacks stuff


if __name__ == "__main__":
    workcell = Workcell()
    workcell.add_robot("James' Robot", ABBIRB140())# add the James Robot
    workcell.add_robot("Jack's Robot", StaubliTX60(base=transl(1, 0, 0))) #add jacks robot
    #workcell.add_robot("Will's Robot", KawasakiRS007N())   # add the Kawasaki under a name
    # workcell.add_robot("UR3", UR3Setup())          # <- add more robots the same way
    workcell.launch()                                # open Swift and show every robot

    T_target = SE3(-0.2, 0.1, 0.5) * SE3.Rx(pi)      # demo pose for the Kawasaki
    gui = RobotGUI(workcell, demo_targets={"James' Robot": T_target})  # robot name -> demo pose
    gui.build()                                      # add the buttons, dropdown and sliders
    gui.run()                                        # keep running so the GUI works (Ctrl+C to quit)