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
SHAKER_STORAGE = (-0.08, -0.2, 0.465)
SHAKER_START = SE3(0, -0.2, 0.255) # WHERE THE SHAKER SITS WHEN THE SCENE STARTS (SAME AS OO_Scene_Objects.py)
SHAKER_TOP = 0.465 # top of the shaker when it's sitting on any table
WILL_DROP_ZONE = (0.85, 0.45) # UR3 leaves things here for Will. front of the (stretched) robot table
WILL_SHAKER_SPOT = (0.48, 0.42) # Will keeps the shaker here to pour into. was (0.72, 0.62): too close to his base to carry it on to Jack without his elbow flipping
JACK_DROP_ZONE = (0.35, 0.45) # Will leaves the shaker here for Jack
JACK_DROP_LID = (0.43, 0.45) # Will leaves the shaker here for Jack
GLASS_GRAB = (-0.2, -0.1, 0.38) # top of the cocktail glass on the storage table (0.125 tall)
JAMES_GLASS_SPOT = (-0.7, 0.45) # UR3 leaves the glass here for James
UR3_JAMES_WAIT_X = 0.3 # where the UR3 waits on the rail while James works
JAMES_LID_SPOT = (-0.45, 0.65, 0.267) # James puts the lid down here (lid top on the table), behind the shaker
JAMES_POUR_HEIGHT = 0.62 # James holds the shaker rim this high over the glass, so the shaker clears the glass before tipping
JAMES_TILT = 2.1 # how far James tips the shaker (rad, ~120 deg)
UMBRELLA_GRAB = (-0.2, -0.3, 0.341) # top of the umbrella in its pot on the storage table (0.086 tall)
JAMES_UMBRELLA_SPOT = (-0.9, 0.45) # UR3 leaves the umbrella here for James
SENSORS_TABLE_SPOT = (1.75, 0.1) # UR3 puts the finished cocktail here on the SensorsTable (x 1.6 to 2.1)
UR3_SENSORS_X = 1.4 # UR3 rail position for reaching the SensorsTable
GLASS_RIM_GRAB = 0.05 # UR3 holds the finished glass by its rim, this far towards the UR3 from the centre, so it misses the umbrella
UMBRELLA_IN_GLASS = 0.416 # umbrella top once James sets it in the glass (its base 5 cm down inside the bowl)
SHAKE_STROKE = 0.06 # Jack's shake goes this far up and down (m)
JAMES_DROP_ZONE = (-0.45, 0.45) # Jack leaves the shaker (lid on) here for James. between their bases, both can reach it
LID_STORAGE = (0.7, -0.25, 0.267) # top of the lid on the storage table (lid is 12 mm thick)
LID_START = SE3(0.7, -0.25, 0.045) # where the lid sits when the scene starts (same as OO_Scene_Objects.py)
LID_DROP = (-0.3, 0.45, 0.267) # UR3 leaves the lid here for Jack
LID_ON_SHAKER = 0.477 # top of the lid once it's on the shaker

# INGREDIENTS WILL POURS INTO THE SHAKER. GRAB = TOP OF THE ITEM ON THE STORAGE TABLE
COGNAC_GRAB = (0.3, -0.097, 0.503) # cognac STL origin is off centre, its middle is at y -0.097. 0.256 tall
COINTREAU_GRAB = (0.3, -0.2, 0.409) # 0.159 tall
SHAKER_GRAB_OFFSET = SHAKER_START.t[0] - SHAKER_STORAGE[0] # shaker is grabbed by its edge, its centre is this far +x of the grab point
LEMON_DROP_ZONE = (0.85, 0.55) # UR3 leaves the Cointreau here (Will can't reach low enough at WILL_DROP_ZONE for it)
UR3_WAIT_X = -0.3 # where the UR3 waits on the rail while Will works (clear of his pours at x 0.5)
SHAKER_POUR_SPOT = (WILL_SHAKER_SPOT[0] + SHAKER_GRAB_OFFSET, WILL_SHAKER_SPOT[1], SHAKER_TOP) # top centre of the shaker on Will's spot

# UR3 BRINGS ONE BOTTLE TO WILL, WILL GRABS IT BY THE NECK AND POURS IT INTO THE SHAKER, UR3 PUTS IT BACK
#   length = HOW TALL IT IS, drop_xy = WHERE THE UR3 LEAVES IT FOR WILL
def will_pours(workcell, ur3_setup, item, grab, length, drop_xy):
    ur3 = workcell.movers["UR3"]
    will = workcell.movers["Will's Robot"]
    drop = (*drop_xy, grab[2]) # UR3 holds it by the top. same height as on the storage table (both tables are 0.25 high)

    ur3_setup.move_base(ur3, grab[0]) # UR3 fetches it
    ur3.pick_drop(grab, item)
    ur3.tuck()
    ur3_setup.move_base(ur3, drop_xy[0])
    ur3.pick_drop(drop) # leaves it in Will's drop zone
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_WAIT_X) # backs off along the rail so it's not in Will's way

    will.neck_pour(drop, item, SHAKER_POUR_SPOT, length) # WILL HOLDS IT BY THE NECK (A LOW SIDE GRAB FOLDS HIS ARM THROUGH ITSELF)
    will.move_to_q(will.home_q) # out of the way

    ur3_setup.move_base(ur3, drop_xy[0])
    ur3.pick_drop(drop, item) # UR3 takes it back to where it started
    ur3.tuck()
    ur3_setup.move_base(ur3, grab[0])
    ur3.pick_drop(grab)
    ur3.tuck()


# WILL'S PART: UR3 BRINGS THE SHAKER, WILL MOVES IT AND PUTS IT DOWN AT JACK'S DROP ZONE (THE HAND OFF TO JACK)
def will_part(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"] # robots from the workcell
    will = workcell.movers["Will's Robot"]
    jack = workcell.movers["Jack's Robot"]
    shaker = scene.cocktail_shaker_scene
    lid = scene.cocktail_shaker_lid_scene

    if workcell.state != "RUNNING": # E-STOPPED: DON'T RESET ANYTHING
        return
    for robot in (ur3, will, jack): # RESET SO THE BUTTON CAN BE PRESSED AGAIN
        robot.release()
    shaker.T = SHAKER_START
    lid.T = LID_START
    if not hasattr(scene, "start_poses"): # FIRST PRESS: REMEMBER WHERE EVERYTHING STARTS
        scene.start_poses = save_start_poses(scene)
    for item in (scene.cognac_scene, scene.cointreau_scene, scene.Better_lemon_scene): # INGREDIENTS BACK TO THEIR START
        item.T = scene.start_poses[item].copy()

    ur3_setup.move_base(ur3, SHAKER_STORAGE[0]) # slide to the shaker
    ur3.pick_drop(SHAKER_STORAGE, shaker) # grab it
    ur3.tuck() # UR3 pulls back over the rail so it's out of the way
    ur3_setup.move_base(ur3, WILL_DROP_ZONE[0]) # slide over to Will
    ur3.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP)) # put it in his drop off zone
    ur3.tuck() # get out of the way before Will reaches in
    will.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP), shaker) # Will picks it up
    will.pick_drop((*WILL_SHAKER_SPOT, SHAKER_TOP)) # and puts it on his shaker spot
    will.move_to_q(will.home_q) # Will stands back up
    will.move_to_q((0.8, pi/2, -pi/2, 0.5, 7, 3)) # Will stands back up

    # WILL POURS THE INGREDIENTS INTO THE SHAKER: (ITEM, UR3 GRAB SPOT, HEIGHT, DROP SPOT). LEMON IS OFF: TOO LOW FOR WILL WITHOUT HIS ARM GOING THROUGH ITSELF
    for item, grab, length, drop_xy in [(scene.cognac_scene, COGNAC_GRAB, 0.256, WILL_DROP_ZONE),
                                        (scene.cointreau_scene, COINTREAU_GRAB, 0.159, LEMON_DROP_ZONE)]:
        if workcell.state != "RUNNING": # E-STOP: SKIP THE REST
            return
        will_pours(workcell, ur3_setup, item, grab, length, drop_xy)

    # Will passes the shaker to Jack: RMRC up and down, one short joint move across in the same arm shape
    will.carry((*WILL_SHAKER_SPOT, SHAKER_TOP), (*JACK_DROP_ZONE, SHAKER_TOP), shaker) # HAND OFF: THE SHAKER IS NOW DOWN FOR JACK
    will.move_to_q(will.home_q)


# JACK'S PART: UR3 BRINGS THE LID, JACK PUTS IT ON, SHAKES, AND HANDS THE SHAKER TO JAMES
def jack_part(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"] # robots from the workcell
    jack = workcell.movers["Jack's Robot"]
    shaker = scene.cocktail_shaker_scene
    lid = scene.cocktail_shaker_lid_scene
    if workcell.state != "RUNNING": # E-STOPPED: DON'T RESET ANYTHING
        return
    for robot in (ur3, jack): # RESET SO THE BUTTON CAN BE PRESSED AGAIN
        robot.release()
    shaker.T = SE3(JACK_DROP_ZONE[0] + SHAKER_GRAB_OFFSET, JACK_DROP_ZONE[1], 0.255) # SHAKER WHERE WILL LEAVES IT, SO JACK'S BUTTON WORKS ON ITS OWN TOO
    lid.T = LID_START # LID BACK IN STORAGE FOR THE UR3 TO FETCH

    ur3_setup.move_base(ur3, LID_STORAGE[0]) # UR3 fetches the lid for Jack
    ur3.pick_drop(LID_STORAGE, lid) # grabs the lid at its top surface, straight down (same as Jack's)
    ur3.tuck()
    ur3_setup.move_base(ur3, LID_DROP[0])
    ur3.pick_drop(LID_DROP)
    ur3.tuck()
    ur3_setup.move_base(ur3, -1.2)

    jack.pick_drop(LID_DROP, lid) # Jack puts the lid on, grabs both and shakes
    # RMRC: LOWER THE LID STRAIGHT DOWN ONTO THE SHAKER
    jack.move_to_pose(SE3(*JACK_DROP_LID, LID_ON_SHAKER + 0.1) * SE3.Rx(pi)) # NORMAL MOVE TO 10 CM ABOVE THE SHAKER (PATH DOESN'T MATTER HERE)
    T_lid_on = SE3(*JACK_DROP_LID, LID_ON_SHAKER) * SE3.Rx(pi) # WHERE THE HAND ENDS UP WHEN THE LID IS ON
    jack.rmrc_to(T_lid_on, duration=2.0, steps=40) # RMRC STRAIGHT DOWN (NORMAL MOVE IF RMRC FAILS)
    jack.grab(shaker)
    T_shake = SE3(*JACK_DROP_ZONE, 0.8) * SE3.Rx(pi) # SHAKE HEIGHT, ABOVE THE DROP ZONE
    jack.rmrc_to(T_shake, duration=2.0, steps=40) # RMRC STRAIGHT UP WITH THE SHAKER
    for i in range(4): # RMRC SHAKE: SHORT STRAIGHT UP-DOWN STROKES, LIKE A BARTENDER
        jack.rmrc_to(SE3(0, 0, SHAKE_STROKE) * T_shake, duration=0.3, steps=8)
        jack.rmrc_to(SE3(0, 0, -SHAKE_STROKE) * T_shake, duration=0.3, steps=8)
    T_james = SE3(*JAMES_DROP_ZONE, LID_ON_SHAKER) * SE3.Rx(pi) # HAND ON THE LID, SHAKER DOWN ON THE TABLE AT JAMES' DROP ZONE
    jack.rmrc_to(SE3(0, 0, 0.1) * T_james, duration=3.0, steps=60) # RMRC ACROSS TO ABOVE JAMES' DROP ZONE
    jack.rmrc_to(T_james) # RMRC STRAIGHT DOWN
    jack.release() # HAND OFF: SHAKER AND LID DOWN FOR JAMES
    jack.rmrc_to(SE3(0, 0, 0.1) * T_james) # RMRC STRAIGHT BACK UP
    jack.move_to_q(jack.home_q)


# JAMES' PART: UR3 BRINGS THE GLASS, JAMES TAKES THE LID OFF, POURS, ADDS THE UMBRELLA, UR3 TAKES THE COCKTAIL TO THE SENSORSTABLE
def james_part(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"]
    jam = workcell.movers["James' Robot"]
    glass = scene.cocktail_glass_scene
    shaker = scene.cocktail_shaker_scene
    lid = scene.cocktail_shaker_lid_scene
    umbrella = scene.Umbrella_scene
    if workcell.state != "RUNNING": # E-STOPPED: DON'T RESET ANYTHING
        return
    if not hasattr(scene, "start_poses"): # FIRST PRESS: REMEMBER WHERE EVERYTHING STARTS
        scene.start_poses = save_start_poses(scene)
    for robot in (ur3, jam): # RESET SO THE BUTTON CAN BE PRESSED AGAIN
        robot.release()
    glass.T = scene.start_poses[glass].copy()
    umbrella.T = scene.start_poses[umbrella].copy()
    shaker.T = SE3(*JAMES_DROP_ZONE, 0.255) # SHAKER + LID WHERE JACK LEAVES THEM, SO JAMES' BUTTON WORKS ON ITS OWN TOO
    lid.T = SE3(*JAMES_DROP_ZONE, 0.255) # (THE LID STL'S ORIGIN IS AT THE SHAKER'S BASE)

    ur3_setup.move_base(ur3, GLASS_GRAB[0]) # UR3 fetches the glass for James
    ur3.pick_drop(GLASS_GRAB, glass)
    ur3.tuck()
    ur3_setup.move_base(ur3, JAMES_GLASS_SPOT[0])
    ur3.pick_drop((*JAMES_GLASS_SPOT, GLASS_GRAB[2]))
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_JAMES_WAIT_X) # out of James' way

    jam.pick_drop((*JAMES_DROP_ZONE, LID_ON_SHAKER), lid) # James takes the lid off
    jam.pick_drop(JAMES_LID_SPOT) # and puts it down behind the shaker
    shaker_rim = (JAMES_DROP_ZONE[0] - SHAKER_GRAB_OFFSET, JAMES_DROP_ZONE[1], SHAKER_TOP) # rim, same grab as Will and Jack
    jam.pick_drop(shaker_rim, shaker)
    T_pour = SE3(*JAMES_GLASS_SPOT, JAMES_POUR_HEIGHT) * SE3.Rx(pi) # rim over the glass, shaker upright
    jam.move_to_pose(T_pour)
    jam.rmrc_to(T_pour * SE3.Ry(JAMES_TILT), orientation_weight=1.0, duration=2.0, steps=40) # RMRC TIP: HAND STAYS PUT, SHAKER TIPS INTO THE GLASS
    if not jam.wait(): # POURING
        return
    jam.rmrc_to(T_pour, orientation_weight=1.0, duration=2.0, steps=40) # TIP BACK UP
    jam.pick_drop(shaker_rim) # shaker back where it was
    jam.move_to_q(jam.home_q)

    ur3_setup.move_base(ur3, UMBRELLA_GRAB[0]) # UR3 fetches the umbrella for James
    ur3.pick_drop(UMBRELLA_GRAB, umbrella)
    ur3.tuck()
    ur3_setup.move_base(ur3, JAMES_UMBRELLA_SPOT[0])
    ur3.pick_drop((*JAMES_UMBRELLA_SPOT, UMBRELLA_GRAB[2]))
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_JAMES_WAIT_X)
    jam.pick_drop((*JAMES_UMBRELLA_SPOT, UMBRELLA_GRAB[2]), umbrella) # James puts the umbrella in the glass
    jam.pick_drop((*JAMES_GLASS_SPOT, UMBRELLA_IN_GLASS))
    jam.move_to_q(jam.home_q)

    ur3_setup.move_base(ur3, JAMES_GLASS_SPOT[0]) # UR3 takes the finished cocktail to the SensorsTable
    ur3.pick_drop((JAMES_GLASS_SPOT[0], JAMES_GLASS_SPOT[1] - GLASS_RIM_GRAB, GLASS_GRAB[2]), [glass, umbrella])
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_SENSORS_X)
    ur3.pick_drop((SENSORS_TABLE_SPOT[0], SENSORS_TABLE_SPOT[1] - GLASS_RIM_GRAB, GLASS_GRAB[2]))
    ur3.tuck()


# THE WHOLE COCKTAIL: WILL, THEN JACK, THEN JAMES. SKIPS THE REST IF AN E-STOP HAPPENED
def make_cocktail(workcell, scene, ur3_setup):
    for part in (will_part, jack_part, james_part):
        if workcell.state != "RUNNING":
            return
        part(workcell, scene, ur3_setup)


# SAVES THE START POSE OF EVERYTHING ON THE SCENE (ANY OBJECT WITH A .T). ALSO LOOKS INSIDE LISTS.
# SO ANY OBJECT YOU ADD TO BarScene AS self.something IS TRACKED WITHOUT CHANGING THIS CODE
def save_start_poses(scene):
    start_poses = {}
    for name, thing in vars(scene).items():
        if name == "env":                          # SKIP THE SWIFT WINDOW ITSELF
            continue
        items = thing if isinstance(thing, (list, tuple)) else [thing]   # A LIST OF OBJECTS (E.G. BOTTLES) IS HANDLED TOO
        for obj in items:
            if hasattr(obj, "T") and not callable(obj.T):
                start_poses[obj] = obj.T.copy()    # .copy() SO LATER MOVES DON'T CHANGE THE SAVED POSE
    return start_poses

# PUTS EVERY ROBOT AND EVERY OBJECT BACK TO HOW THE SCENE STARTED
def reset_scene(workcell, ur3_setup, start_poses):
    for mover in workcell.movers.values():         # LET GO OF ANYTHING A ROBOT IS HOLDING
        mover.release()
    ur3_setup.move_base(workcell.movers["UR3"], 0.08)   # RAIL BACK TO ITS START X (0.08 = THE UR3 BASE X). CHANGE IF YOUR RAIL STARTS ELSEWHERE
    for mover in workcell.movers.values():         # ALL ROBOTS TO HOME, INSTANTLY
        mover.robot.q = mover.home_q
        mover.update_meshes(mover.home_q)
    for obj, pose in start_poses.items():          # ALL OBJECTS TO THEIR START POSE
        obj.T = pose.copy()
    print("Scene reset")



if __name__ == "__main__":
    workcell = Workcell()
    workcell.add_robot("James' Robot", ABBIRB140(base=transl(-1, 0.75, 0.25))) # add the James Robot. on the robot bar table
    workcell.add_robot("Jack's Robot", StaubliTX60(base=transl(0, 0.75, 0.25))) # add jacks robot. middle of the robot bar table
    ur3 = UR3Robot(base=transl(0.08, 0.1, 0.25)) # kept as its own variable because the movements need it for the rail
    workcell.add_robot("UR3", ur3) # add the UR3. same spot as MainRobotSimFile
    workcell.add_robot("Will's Robot", KawasakiRS007N(base=transl(1, 0.75, 0.25))) # add the Kawasaki under a name
    workcell.launch() # open Swift and show every robot
    scene = BarScene(workcell.env) # adds all the objects. has to be after launch
    start_poses = save_start_poses(scene) # SAVE WHERE EVERYTHING STARTS
    workcell.collision_on = True #every robot checks the other robots before each step (False to turn off)

    # SCENE BUTTONS: LABEL -> FUNCTION. TO ADD ONE:
    # 1. WRITE THE SCENE AS A FUNCTION LIKE SHAKER_TO_WILL, USING THE MOVER COMMANDS
    # 2. RESET ANY ITEMS IT MOVES AT THE START SO IT CAN BE REPLAYED
    # 3. ADD A LINE BELOW, E.G.  "Pour Drink": lambda: pour_drink(workcell, scene),  (LAMBDA STOPS IT RUNNING NOW)
    scenes = {
        "Make Cocktail": lambda: make_cocktail(workcell, scene, ur3), # ALL THREE IN ORDER: WILL, JACK, JAMES
        "Will's Robot": lambda: will_part(workcell, scene, ur3),
        "Jack's Robot": lambda: jack_part(workcell, scene, ur3),
        "James' Robot": lambda: james_part(workcell, scene, ur3),
        "Reset Scene": lambda: reset_scene(workcell, ur3, start_poses),
    }
    gui = RobotGUI(workcell, scenes=scenes) # button label -> scene
    gui.build() # add the buttons, dropdown and sliders
    gui.run()