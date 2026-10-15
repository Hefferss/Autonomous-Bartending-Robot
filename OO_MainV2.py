from math import pi
from spatialmath import SE3
from spatialmath.base import transl #needed to place the robots
from OO_MovementCommands import Workcell
from OO_GUI import RobotGUI
from OO_WillRobotTest import KawasakiRS007N
from OO_JamesRobotTest import ABBIRB140
from OO_JackRobotTest import StaubliTX60 #jacks robot
from OO_UR3 import UR3Robot #ur3 on its platform
from OO_Scene_Objects import BarScene #tables, rail, bottles and all that
from OO_Robot_Movements import RobotMovements # the robot movements (UR3 -> Will shaker handoff)

if __name__ == "__main__":
    workcell = Workcell()
    workcell.add_robot("James' Robot", ABBIRB140(base=transl(-1, 0.75, 0.25)))# add the James Robot. on the robot bar table
    workcell.add_robot("Jack's Robot", StaubliTX60(base=transl(0, 0.75, 0.25)))# add jacks robot. middle of the robot bar table
    ur3 = UR3Robot(base=transl(0.08, 0.1, 0.25)) # kept as its own variable because the movements need it for the rail
    workcell.add_robot("UR3", ur3) # add the UR3. same spot as MainRobotSimFile
    workcell.add_robot("Will's Robot", KawasakiRS007N(base=transl(1, 0.75, 0.25)))# add the Kawasaki under a name
    workcell.launch()# open Swift and show every robot
    scene = BarScene(workcell.env)# adds all the objects. has to be after launch
    RobotMovements(workcell, scene, ur3).shaker_to_will() # runs the handoff before the GUI takes over

    T_target = SE3(-1, 0.75, 0.25) * SE3(-0.2, 0.1, 0.5) * SE3.Rx(pi) #demo pose for the Kawasaki. shifted the same amount as james' robot
    gui = RobotGUI(workcell, demo_targets={"James' Robot": T_target})# robot name -> demo pose
    gui.build()# add the buttons, dropdown and sliders
    gui.run()