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
# (move_to_pose, move_to_q, pick_drop, tuck, grab, release) AND THE UR3 RAIL'S move_base

# where the EE goes to grab the shaker (its top). shaker is 0.21 tall, tables are 0.25 high
SHAKER_STORAGE = (0, -0.2, 0.465)
SHAKER_START = SE3(0, -0.2, 0.255) # WHERE THE SHAKER SITS WHEN THE SCENE STARTS (SAME AS OO_Scene_Objects.py)
SHAKER_TOP = 0.465 # top of the shaker when it's sitting on any table
WILL_DROP_ZONE = (0.85, 0.45) # UR3 leaves things here for Will. front of the (stretched) robot table
WILL_SHAKER_SPOT = (0.72, 0.62) # Will keeps the shaker here to pour into


def shaker_to_will(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"]                          # robots from the workcell
    will = workcell.movers["Will's Robot"]
    shaker = scene.cocktail_shaker_scene

    ur3.release() # RESET SO THE BUTTON CAN BE PRESSED AGAIN
    will.release()
    shaker.T = SHAKER_START

    ur3_setup.move_base(ur3, SHAKER_STORAGE[0]) # slide to the shaker
    ur3.pick_drop(SHAKER_STORAGE, shaker) # grab it
    ur3.tuck() # UR3 pulls back over the rail so it's out of the way
    ur3_setup.move_base(ur3, WILL_DROP_ZONE[0]) # slide over to Will
    ur3.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP)) # put it in his drop off zone
    ur3.tuck() # get out of the way before Will reaches in
    will.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP), shaker) # Will picks it up
    will.pick_drop((*WILL_SHAKER_SPOT, SHAKER_TOP)) # and puts it on his shaker spot
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

    # SCENE BUTTONS: LABEL -> FUNCTION. TO ADD ONE:
    # 1. WRITE THE SCENE AS A FUNCTION LIKE SHAKER_TO_WILL, USING THE MOVER COMMANDS
    # 2. RESET ANY ITEMS IT MOVES AT THE START SO IT CAN BE REPLAYED
    # 3. ADD A LINE BELOW, E.G.  "Pour Drink": lambda: pour_drink(workcell, scene),  (LAMBDA STOPS IT RUNNING NOW)
    scenes = {
        "Run Demo (Shaker to Will)": lambda: shaker_to_will(workcell, scene, ur3),
    }
    gui = RobotGUI(workcell, scenes=scenes)# button label -> scene
    gui.build()# add the buttons, dropdown and sliders
    gui.run()