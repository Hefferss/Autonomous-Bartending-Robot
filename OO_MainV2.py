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

# MOVED HERE FROM OO_Robot_Movements.py. THE CODE IS THE SAME, IT ONLY USES THE FUNCTIONS FROM OO_MovementCommands.py
# (move_to_pose, move_to_q, grab, release) AND THE UR3 RAIL'S move_base

# where the EE goes to grab the shaker (its top). shaker is 0.21 tall, tables are 0.25 high
SHAKER_STORAGE = (0, -0.2, 0.465)
SHAKER_TOP = 0.465 # top of the shaker when it's sitting on any table
WILL_DROP_ZONE = (0.85, 0.45) # UR3 leaves things here for Will. front of the (stretched) robot table
WILL_SHAKER_SPOT = (0.72, 0.62) # Will keeps the shaker here to pour into


def move(mover, xyz, steps=50): # wills move_to_pose, tool pointing down
    mover.move_to_pose(SE3(*xyz) * SE3.Rx(pi), steps)

def pick_drop(mover, xyz, item=None): # above, down, grab (if given an item) or let go, back up
    move(mover, (xyz[0], xyz[1], xyz[2] + 0.1))
    move(mover, xyz, 25)
    if item is not None:
        mover.grab(item)
    else:
        mover.release()
    move(mover, (xyz[0], xyz[1], xyz[2] + 0.1), 25)

def shaker_to_will(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"]                          # THE UR3 AND WILL'S ROBOT, TAKEN FROM THE WORKCELL
    will = workcell.movers["Will's Robot"]
    shaker = scene.cocktail_shaker_scene

    ur3_setup.move_base(ur3, SHAKER_STORAGE[0]) # slide to the shaker
    pick_drop(ur3, SHAKER_STORAGE, shaker) # grab it
    move(ur3, (ur3.robot.base.t[0], 0.0, 0.7)) # TUCK: UR3 pulls back over the rail so it's out of the way (X IS READ EACH TIME BECAUSE THE BASE MOVES WITH THE RAIL)
    ur3_setup.move_base(ur3, WILL_DROP_ZONE[0]) # slide over to Will
    pick_drop(ur3, (*WILL_DROP_ZONE, SHAKER_TOP)) # put it in his drop off zone
    move(ur3, (ur3.robot.base.t[0], 0.0, 0.7)) # get out of the way before Will reaches in
    pick_drop(will, (*WILL_DROP_ZONE, SHAKER_TOP), shaker) # Will picks it up
    pick_drop(will, (*WILL_SHAKER_SPOT, SHAKER_TOP)) # and puts it on his shaker spot
    will.move_to_q(will.home_q) # Will stands back up


if __name__ == "__main__":
    workcell = Workcell()
    workcell.add_robot("James' Robot", ABBIRB140(base=transl(-1, 0.75, 0.25)))# add the James Robot. on the robot bar table
    workcell.add_robot("Jack's Robot", StaubliTX60(base=transl(0, 0.75, 0.25)))# add jacks robot. middle of the robot bar table
    ur3 = UR3Robot(base=transl(0.08, 0.1, 0.25)) # kept as its own variable because the movements need it for the rail
    workcell.add_robot("UR3", ur3) # add the UR3. same spot as MainRobotSimFile
    workcell.add_robot("Will's Robot", KawasakiRS007N(base=transl(1, 0.75, 0.25)))# add the Kawasaki under a name
    workcell.launch()# open Swift and show every robot
    scene = BarScene(workcell.env)# adds all the objects. has to be after launch

    shaker_to_will(workcell, scene, ur3) # runs the handoff before the GUI takes over

    T_target = SE3(-1, 0.75, 0.25) * SE3(-0.2, 0.1, 0.5) * SE3.Rx(pi) #demo pose for the Kawasaki. shifted the same amount as james' robot
    gui = RobotGUI(workcell, demo_targets={"James' Robot": T_target})# robot name -> demo pose
    gui.build()# add the buttons, dropdown and sliders
    gui.run()