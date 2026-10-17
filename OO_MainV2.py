from spatialmath import SE3
from spatialmath.base import transl #needed to place the robots
from OO_MovementCommands import Workcell
from OO_GUI import RobotGUI
from OO_WillRobotTest import KawasakiRS007N
from OO_JamesRobotTest import ABBIRB140
from OO_JackRobotTest import StaubliTX60 #jacks robot
from OO_UR3 import UR3Robot #ur3 on its platform
from OO_Scene_Objects import BarScene #tables, rail, bottles and all that
from math import pi # the constant pi

# MOVED HERE FROM OO_Robot_Movements.py. THE CODE IS THE SAME, IT ONLY USES THE FUNCTIONS FROM OO_MovementCommands.py
# (move_to_pose, move_to_q, pick_drop, tuck, grab, release) AND THE UR3 RAIL'S move_base

# where the EE goes to grab the shaker (its top). shaker is 0.21 tall, tables are 0.25 high
SHAKER_STORAGE = (0, -0.2, 0.465)
SHAKER_START = SE3(0, -0.2, 0.255) # WHERE THE SHAKER SITS WHEN THE SCENE STARTS (SAME AS OO_Scene_Objects.py)
SHAKER_TOP = 0.465 # top of the shaker when it's sitting on any table
WILL_DROP_ZONE = (0.85, 0.45) # UR3 leaves things here for Will. front of the (stretched) robot table
WILL_SHAKER_SPOT = (0.72, 0.62) # Will keeps the shaker here to pour into
JACK_DROP_ZONE = (0.4, 0.45) # Will leaves the shaker here for Jack
LID_STORAGE = (0.7, -0.25, 0.267) # top of the lid on the storage table (lid is 12 mm thick)
LID_START = SE3(0.7, -0.25, 0.045) # where the lid sits when the scene starts (same as OO_Scene_Objects.py)
LID_DROP = (-0.3, 0.45, 0.267) # UR3 leaves the lid here for Jack
UR3_LID_GRAB = 0.31 # UR3's wrist sticks out ~4 cm past its tool point, so it holds the lid from 4 cm up or the wrist goes into the table
LID_ON_SHAKER = 0.477 # top of the lid once it's on the shaker

# the whole cocktail in one go: UR3 -> Will -> Jack, UR3 brings Jack the lid, Jack puts it on and shakes
def make_cocktail(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"] # robots from the workcell
    will = workcell.movers["Will's Robot"]
    jam = workcell.movers["James' Robot"]
    jack = workcell.movers["Jack's Robot"]
    shaker = scene.cocktail_shaker_scene
    lid = scene.cocktail_shaker_lid_scene

    for robot in (ur3, will, jack): # RESET SO THE BUTTON CAN BE PRESSED AGAIN
        robot.release()
    shaker.T = SHAKER_START
    lid.T = LID_START

    ur3_setup.move_base(ur3, SHAKER_STORAGE[0]) # slide to the shaker
    ur3.pick_drop(SHAKER_STORAGE, shaker) # grab it
    ur3.tuck(0.4) # UR3 pulls back over the rail so it's out of the way. tuck takes a y number, not an SE3
    ur3_setup.move_base(ur3, WILL_DROP_ZONE[0]) # slide over to Will
    ur3.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP)) # put it in his drop off zone
    ur3.tuck() # get out of the way before Will reaches in
    will.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP), shaker) # Will picks it up
    will.pick_drop((*WILL_SHAKER_SPOT, SHAKER_TOP)) # and puts it on his shaker spot
    will.move_to_q(will.home_q) # Will stands back up
    jam.move_to_q((0, pi/2, -pi/2, 0, 0, 0)) # James stands back up
    jack.move_to_q((0.5, pi/2, -pi/2, 1, 3, 9)) # Jack stands back up
    will.move_to_q((0.8, pi/2, -pi/2, 0.5, 7, 3)) # Will stands back up

    will.pick_drop((*WILL_SHAKER_SPOT, SHAKER_TOP), shaker) # Will passes the shaker to Jack
    will.pick_drop((*JACK_DROP_ZONE, SHAKER_TOP))
    will.move_to_q(will.home_q)

    ur3_setup.move_base(ur3, LID_STORAGE[0]) # UR3 fetches the lid for Jack
    ur3.pick_drop((*LID_STORAGE[:2], UR3_LID_GRAB), lid)
    ur3.tuck()
    ur3_setup.move_base(ur3, LID_DROP[0])
    ur3.pick_drop((*LID_DROP[:2], UR3_LID_GRAB))
    ur3.tuck()

    jack.pick_drop(LID_DROP, lid) # Jack puts the lid on, grabs both and shakes
    # RMRC: LOWER THE LID STRAIGHT DOWN ONTO THE SHAKER
    jack.move_to_pose(SE3(*JACK_DROP_ZONE, LID_ON_SHAKER + 0.1) * SE3.Rx(pi)) # NORMAL MOVE TO 10 CM ABOVE THE SHAKER (PATH DOESN'T MATTER HERE)
    T_lid_on = SE3(*JACK_DROP_ZONE, LID_ON_SHAKER) * SE3.Rx(pi) # WHERE THE HAND ENDS UP WHEN THE LID IS ON
    if not jack.move_rmrc(T_lid_on, duration=2.0, steps=40): # STRAIGHT LINE DOWN, EXISTING RMRC CODE
        jack.move_to_pose(T_lid_on) # IF RMRC FAILED (SINGULARITY, OUT OF REACH), FALL BACK TO THE NORMAL MOVE
    jack.grab(shaker)
    jack.move_to_pose(SE3(*JACK_DROP_ZONE, 0.8) * SE3.Rx(pi))
    q_shake = jack.robot.q
    for i in range(3):
        jack.move_to_q(q_shake + [0, 0, 0, 0, 0.4, pi], 10)
        jack.move_to_q(q_shake + [0, 0, 0, 0, -0.4, -pi], 10)
    jack.pick_drop((*JACK_DROP_ZONE, LID_ON_SHAKER))
    jack.move_to_q(jack.home_q)


if __name__ == "__main__":
    workcell = Workcell()
    workcell.add_robot("James' Robot", ABBIRB140(base=transl(-1, 0.75, 0.25))) # add the James Robot. on the robot bar table
    workcell.add_robot("Jack's Robot", StaubliTX60(base=transl(0, 0.75, 0.25))) # add jacks robot. middle of the robot bar table
    ur3 = UR3Robot(base=transl(0.08, 0.1, 0.25)) # kept as its own variable because the movements need it for the rail
    workcell.add_robot("UR3", ur3) # add the UR3. same spot as MainRobotSimFile
    workcell.add_robot("Will's Robot", KawasakiRS007N(base=transl(1, 0.75, 0.25))) # add the Kawasaki under a name
    workcell.launch() # open Swift and show every robot
    scene = BarScene(workcell.env) # adds all the objects. has to be after launch
    workcell.collision_on = True #every robot checks the other robots before each step (False to turn off)

    # SCENE BUTTONS: LABEL -> FUNCTION. TO ADD ONE:
    # 1. WRITE THE SCENE AS A FUNCTION LIKE SHAKER_TO_WILL, USING THE MOVER COMMANDS
    # 2. RESET ANY ITEMS IT MOVES AT THE START SO IT CAN BE REPLAYED
    # 3. ADD A LINE BELOW, E.G.  "Pour Drink": lambda: pour_drink(workcell, scene),  (LAMBDA STOPS IT RUNNING NOW)
    scenes = {
        "Make Cocktail": lambda: make_cocktail(workcell, scene, ur3),
    }
    gui = RobotGUI(workcell, scenes=scenes) # button label -> scene
    gui.build() # add the buttons, dropdown and sliders
    gui.run()