from spatialmath import SE3
from spatialmath.base import transl, trotz
from OO_MovementCommands import Workcell
from OO_GUI import RobotGUI
from OO_WillRobotTest import KawasakiRS007N
from OO_JamesRobotTest import ABBIRB140
from OO_JackRobotTest import StaubliTX60
from OO_UR3 import UR3Robot
from OO_Scene_Objects import BarScene
from math import pi
import numpy as np 


# where the EE goes to grab the shaker (its top). shaker is 0.21 tall, tables are 0.25 high
SHAKER_STORAGE = (-0.08, -0.2, 0.465)
SHAKER_START = SE3(0, -0.2, 0.255) # where the shaker sits when the scene starts (same as OO_Scene_Objects.py)
SHAKER_TOP = 0.465 # top of the shaker when it's sitting on any table
WILL_DROP_ZONE = (0.85, 0.45) # UR3 leaves things here for will. front of the (stretched) robot table
WILL_SHAKER_SPOT = (0.48, 0.42) # will keeps the shaker here to pour into. was (0.72, 0.62): too close to his base to carry it on to jack without his elbow flipping
JACK_DROP_ZONE = (0.35, 0.45) # will leaves the shaker here for jack
JACK_DROP_LID = (0.43, 0.45) # will leaves the shaker here for jack
GLASS_GRAB = (-0.2, -0.1, 0.38) # top of the cocktail glass on the storage table (0.125 tall)
JAMES_GLASS_SPOT = (-0.7, 0.45) # UR3 leaves the glass here for james
UR3_JAMES_WAIT_X = 0.3 # where the UR3 waits on the rail while james works
JAMES_LID_SPOT = (-0.45, 0.65, 0.267) # james puts the lid down here (lid top on the table), behind the shaker
JAMES_POUR_HEIGHT = 0.62 # james holds the shaker rim this high over the glass, so the shaker clears the glass before tipping
JAMES_TILT = 2.1 # how far james tips the shaker (rad, ~120 deg)
UMBRELLA_GRAB = (-0.2, -0.3, 0.341) # top of the umbrella in its pot on the storage table (0.086 tall)
JAMES_UMBRELLA_SPOT = (-0.9, 0.45) # UR3 leaves the umbrella here for james
SENSORS_TABLE_SPOT = (1.75, 0.1) # UR3 puts the finished cocktail here on the SensorsTable (x 1.6 to 2.1)
UR3_SENSORS_X = 1.4 # UR3 rail position for reaching the SensorsTable
GLASS_RIM_GRAB = 0.05 # UR3 holds the finished glass by its rim, this far towards the UR3 from the centre, so it misses the umbrella
UMBRELLA_IN_GLASS = 0.416 # umbrella top once james sets it in the glass (its base 5 cm down inside the bowl)
SHAKE_STROKE = 0.06 # jack's shake goes this far up and down (m)
JACK_TEST_Q = [30, 60, -40, 0, 30, 0] # test pose for the marker just in case
JAMES_DROP_ZONE = (-0.45, 0.45) # jack leaves the shaker (lid on) here for james. between their bases, both can reach it
LID_STORAGE = (0.7, -0.25, 0.267) # top of the lid on the storage table (lid is 12 mm thick)
LID_START = SE3(0.7, -0.25, 0.045) # where the lid sits when the scene starts (same as OO_Scene_Objects.py)
LID_DROP = (-0.3, 0.45, 0.267) # UR3 leaves the lid here for jack
LID_ON_SHAKER = 0.477 # top of the lid once it's on the shaker

# ingredients will pours into the shaker. grab = top of the item on the storage table
COGNAC_GRAB = (0.3, -0.097, 0.503) # cognac STL origin is off centre, its middle is at y -0.097. 0.256 tall
COINTREAU_GRAB = (0.3, -0.2, 0.409) # 0.159 tall
SHAKER_GRAB_OFFSET = SHAKER_START.t[0] - SHAKER_STORAGE[0] # shaker is grabbed by its edge, its centre is this far +x of the grab point
LEMON_DROP_ZONE = (0.85, 0.55) # UR3 leaves the cointreau here (will can't reach low enough at WILL_DROP_ZONE for it)
UR3_WAIT_X = -0.3 # where the UR3 waits on the rail while will works (clear of his pours at x 0.5)
SHAKER_POUR_SPOT = (WILL_SHAKER_SPOT[0] + SHAKER_GRAB_OFFSET, WILL_SHAKER_SPOT[1], SHAKER_TOP) # top centre of the shaker on will's spot

# UR3 brings one bottle to will, will grabs it by the neck and pours it into the shaker, UR3 puts it back
#   length = how tall it is, drop_xy = where the UR3 leaves it for will
def will_pours(workcell, ur3_setup, item, grab, length, drop_xy):
    ur3 = workcell.movers["UR3"]
    will = workcell.movers["Will's Robot"]
    drop = (*drop_xy, grab[2]) # UR3 holds it by the top. same height as on the storage table (both tables are 0.25 high)

    ur3_setup.move_base(ur3, grab[0]) # UR3 fetches it
    ur3.pick_drop(grab, item)
    ur3.tuck()
    ur3_setup.move_base(ur3, drop_xy[0])
    ur3.pick_drop(drop) # leaves it in will's drop zone
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_WAIT_X) # backs off along the rail so it's not in will's way

    will.neck_pour(drop, item, SHAKER_POUR_SPOT, length) # will holds it by the neck (a low side grab folds his arm through itself)
    will.move_to_q(will.home_q) # out of the way

    ur3_setup.move_base(ur3, drop_xy[0])
    ur3.pick_drop(drop, item) # UR3 takes it back to where it started
    ur3.tuck()
    ur3_setup.move_base(ur3, grab[0])
    ur3.pick_drop(grab)
    ur3.tuck()


# will's part: UR3 brings the shaker, will moves it and puts it down at jack's drop zone (the hand off to jack)
def will_part(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"] # robots from the workcell
    will = workcell.movers["Will's Robot"]
    jack = workcell.movers["Jack's Robot"]
    shaker = scene.cocktail_shaker_scene
    lid = scene.cocktail_shaker_lid_scene

    if workcell.state != "RUNNING": # e-stopped: don't reset anything
        return
    for robot in (ur3, will, jack): # reset so the button can be pressed again
        robot.release()
    shaker.T = SHAKER_START
    lid.T = LID_START
    if not hasattr(scene, "start_poses"): # first press: remember where everything starts
        scene.start_poses = save_start_poses(scene)
    for item in (scene.cognac_scene, scene.cointreau_scene, scene.Better_lemon_scene): # ingredients back to their start
        item.T = scene.start_poses[item].copy()

    ur3_setup.move_base(ur3, SHAKER_STORAGE[0]) # slide to the shaker
    ur3.pick_drop(SHAKER_STORAGE, shaker) # grab it
    ur3.tuck() # UR3 pulls back over the rail so it's out of the way
    ur3_setup.move_base(ur3, WILL_DROP_ZONE[0]) # slide over to will
    ur3.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP)) # put it in his drop off zone
    ur3.tuck() # get out of the way before will reaches in
    will.pick_drop((*WILL_DROP_ZONE, SHAKER_TOP), shaker) # will picks it up
    will.pick_drop((*WILL_SHAKER_SPOT, SHAKER_TOP)) # and puts it on his shaker spot
    will.move_to_q(will.home_q) # will stands back up
    will.move_to_q((0.8, pi/2, -pi/2, 0.5, 7, 3)) # will stands back up

    # will pours the ingredients into the shaker: (item, UR3 grab spot, height, drop spot). lemon is off: too low for will without his arm going through itself
    for item, grab, length, drop_xy in [(scene.cognac_scene, COGNAC_GRAB, 0.256, WILL_DROP_ZONE),
                                        (scene.cointreau_scene, COINTREAU_GRAB, 0.159, LEMON_DROP_ZONE)]:
        if workcell.state != "RUNNING": # e-stop: skip the rest
            return
        will_pours(workcell, ur3_setup, item, grab, length, drop_xy)

    # will passes the shaker to jack: RMRC up and down, one short joint move across in the same arm shape
    will.carry((*WILL_SHAKER_SPOT, SHAKER_TOP), (*JACK_DROP_ZONE, SHAKER_TOP), shaker) # hand off: the shaker is now down for jack
    will.move_to_q(will.home_q)


# jack's part: UR3 brings the lid, jack puts it on, shakes, and hands the shaker to james
def jack_part(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"] # robots from the workcell
    jack = workcell.movers["Jack's Robot"]
    shaker = scene.cocktail_shaker_scene
    lid = scene.cocktail_shaker_lid_scene
    if workcell.state != "RUNNING": # e-stopped: don't reset anything
        return
    for robot in (ur3, jack): # reset so the button can be pressed again
        robot.release()
    shaker.T = SE3(JACK_DROP_ZONE[0] + SHAKER_GRAB_OFFSET, JACK_DROP_ZONE[1], 0.255) # shaker where will leaves it, so jack's button works on its own too
    lid.T = LID_START # lid back in storage for the UR3 to fetch

    ur3_setup.move_base(ur3, LID_STORAGE[0]) # UR3 fetches the lid for jack
    ur3.pick_drop(LID_STORAGE, lid) # grabs the lid at its top surface, straight down (same as jack's)
    ur3.tuck()
    ur3_setup.move_base(ur3, LID_DROP[0])
    ur3.pick_drop(LID_DROP)
    ur3.tuck()
    ur3_setup.move_base(ur3, -1.2)

    jack.pick_drop(LID_DROP, lid) # jack puts the lid on, grabs both and shakes
    jack.move_to_pose(SE3(*JACK_DROP_LID, LID_ON_SHAKER + 0.1) * SE3.Rx(pi)) # normal move to 10 cm above the shaker (path doesn't matter here)
    T_lid_on = SE3(*JACK_DROP_LID, LID_ON_SHAKER) * SE3.Rx(pi) # where the hand ends up when the lid is on
    jack.rmrc_to(T_lid_on, duration=2.0, steps=40) # RMRC straight down (normal move if RMRC fails)
    jack.grab(shaker)
    T_shake = SE3(*JACK_DROP_ZONE, 0.8) * SE3.Rx(pi) # shake height, above the drop zone
    jack.rmrc_to(T_shake, duration=2.0, steps=40) # RMRC straight up with the shaker
    for i in range(4): # RMRC shake: short straight up-down strokes, like a bartender
        jack.rmrc_to(SE3(0, 0, SHAKE_STROKE) * T_shake, duration=0.3, steps=8)
        jack.rmrc_to(SE3(0, 0, -SHAKE_STROKE) * T_shake, duration=0.3, steps=8)
    T_james = SE3(*JAMES_DROP_ZONE, LID_ON_SHAKER) * SE3.Rx(pi) # hand on the lid, shaker down on the table at james' drop zone
    jack.rmrc_to(SE3(0, 0, 0.1) * T_james, duration=3.0, steps=60) # RMRC across to above james' drop zone
    jack.rmrc_to(T_james) # RMRC straight down
    jack.release() # hand off: shaker and lid down for james
    jack.rmrc_to(SE3(0, 0, 0.1) * T_james) # RMRC straight back up
    jack.move_to_q(jack.home_q)


# james' part: UR3 brings the glass, james takes the lid off, pours, adds the umbrella, UR3 takes the cocktail to the sensorstable
def james_part(workcell, scene, ur3_setup):
    ur3 = workcell.movers["UR3"]
    jam = workcell.movers["James' Robot"]
    glass = scene.cocktail_glass_scene
    shaker = scene.cocktail_shaker_scene
    lid = scene.cocktail_shaker_lid_scene
    umbrella = scene.Umbrella_scene
    if workcell.state != "RUNNING": # e-stopped: don't reset anything
        return
    if not hasattr(scene, "start_poses"): # first press: remember where everything starts
        scene.start_poses = save_start_poses(scene)
    for robot in (ur3, jam): # reset so the button can be pressed again
        robot.release()
    glass.T = scene.start_poses[glass].copy()
    umbrella.T = scene.start_poses[umbrella].copy()
    shaker.T = SE3(*JAMES_DROP_ZONE, 0.255) # shaker + lid where jack leaves them, so james' button works on its own too
    lid.T = SE3(*JAMES_DROP_ZONE, 0.255) # (the lid stl's origin is at the shaker's base)

    ur3_setup.move_base(ur3, GLASS_GRAB[0]) # UR3 fetches the glass for james
    ur3.pick_drop(GLASS_GRAB, glass)
    ur3.tuck()
    ur3_setup.move_base(ur3, JAMES_GLASS_SPOT[0])
    ur3.pick_drop((*JAMES_GLASS_SPOT, GLASS_GRAB[2]))
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_JAMES_WAIT_X) # out of james' way

    jam.pick_drop((*JAMES_DROP_ZONE, LID_ON_SHAKER), lid) # james takes the lid off
    jam.pick_drop(JAMES_LID_SPOT) # and puts it down behind the shaker
    shaker_rim = (JAMES_DROP_ZONE[0] - SHAKER_GRAB_OFFSET, JAMES_DROP_ZONE[1], SHAKER_TOP) # rim, same grab as will and jack
    jam.pick_drop(shaker_rim, shaker)
    T_pour = SE3(*JAMES_GLASS_SPOT, JAMES_POUR_HEIGHT) * SE3.Rx(pi) # rim over the glass, shaker upright
    jam.move_to_pose(T_pour)
    jam.rmrc_to(T_pour * SE3.Ry(JAMES_TILT), orientation_weight=1.0, duration=2.0, steps=40) # RMRC tip: hand stays put, shaker tips into the glass
    if not jam.wait(): # pouring
        return
    jam.rmrc_to(T_pour, orientation_weight=1.0, duration=2.0, steps=40) # tip back up
    jam.pick_drop(shaker_rim) # shaker back where it was
    jam.move_to_q(jam.home_q)

    ur3_setup.move_base(ur3, UMBRELLA_GRAB[0]) # UR3 fetches the umbrella for james
    ur3.pick_drop(UMBRELLA_GRAB, umbrella)
    ur3.tuck()
    ur3_setup.move_base(ur3, JAMES_UMBRELLA_SPOT[0])
    ur3.pick_drop((*JAMES_UMBRELLA_SPOT, UMBRELLA_GRAB[2]))
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_JAMES_WAIT_X)
    jam.pick_drop((*JAMES_UMBRELLA_SPOT, UMBRELLA_GRAB[2]), umbrella) # james puts the umbrella in the glass
    jam.pick_drop((*JAMES_GLASS_SPOT, UMBRELLA_IN_GLASS))
    jam.move_to_q(jam.home_q)

    ur3_setup.move_base(ur3, JAMES_GLASS_SPOT[0]) # UR3 takes the finished cocktail to the SensorsTable
    ur3.pick_drop((JAMES_GLASS_SPOT[0], JAMES_GLASS_SPOT[1] - GLASS_RIM_GRAB, GLASS_GRAB[2]), [glass, umbrella])
    ur3.tuck()
    ur3_setup.move_base(ur3, UR3_SENSORS_X)
    ur3.pick_drop((SENSORS_TABLE_SPOT[0], SENSORS_TABLE_SPOT[1] - GLASS_RIM_GRAB, GLASS_GRAB[2]))
    ur3.tuck()


# the whole cocktail: will, then jack, then james. skips the rest if an e-stop happened
def make_cocktail(workcell, scene, ur3_setup):
    for part in (will_part, jack_part, james_part):
        if workcell.state != "RUNNING":
            return
        part(workcell, scene, ur3_setup)


# saves the start pose of every object in the scene (anything with a .t, including inside lists)
def save_start_poses(scene):
    start_poses = {}
    for name, thing in vars(scene).items():
        if name == "env":# skip the swift window itself
            continue
        items = thing if isinstance(thing, (list, tuple)) else [thing]# a list of objects (e.g. bottles) is handled too
        for obj in items:
            if hasattr(obj, "T") and not callable(obj.T):
                start_poses[obj] = obj.T.copy()# .copy() so later moves don't change the saved pose
    return start_poses

# puts every robot and every object back to how the scene started
def reset_scene(workcell, ur3_setup, start_poses):
    for mover in workcell.movers.values():# let go of anything a robot is holding
        mover.release()
    ur3_setup.move_base(workcell.movers["UR3"], 0.08)# rail back to its start x (0.08 = the UR3 base x). change if your rail starts elsewhere
    for mover in workcell.movers.values():# all robots to home, instantly
        mover.robot.q = mover.home_q
        mover.update_meshes(mover.home_q)
    for obj, pose in start_poses.items():# all objects to their start pose
        obj.T = pose.copy()
    print("Scene reset")

# jack's test pose: plans to JACK_TEST_Q with jtraj (move_to_q), so the e-stop and collision checks still work
def jack_test_pose(workcell):
    jack = workcell.movers["Jack's Robot"]
    q = np.radians(JACK_TEST_Q)
    if np.any(q < jack.robot.qlim[0]) or np.any(q > jack.robot.qlim[1]): # outside the staubli's joint limits
        print("Test pose is outside the joint limits (deg):", np.round(np.degrees(jack.robot.qlim), 1).tolist())
        return
    if jack.move_to_q(q):
        print("Reached test pose:", JACK_TEST_Q)

if __name__ == "__main__":
    workcell = Workcell()
    workcell.add_robot("James' Robot", ABBIRB140(base=transl(-1, 0.75, 0.25))) # add the james robot. on the robot bar table
    workcell.add_robot("Jack's Robot", StaubliTX60(base=transl(0, 0.75, 0.25))) # add jacks robot. middle of the robot bar table
    ur3 = UR3Robot(base=transl(0.08, 0.1, 0.25)) # kept as its own variable because the movements need it for the rail
    workcell.add_robot("UR3", ur3) # add the UR3. same spot as MainRobotSimFile
    workcell.add_robot("Will's Robot", KawasakiRS007N(base=transl(1, 0.75, 0.25) @ trotz(-pi/2))) # add the kawasaki under a name
    workcell.launch() # open swift and show every robot
    scene = BarScene(workcell.env) # adds all the objects. has to be after launch
    start_poses = save_start_poses(scene) # save where everything starts
    workcell.collision_on = True #every robot checks the other robots before each step (false to turn off)

    # scene buttons: label -> function (lambda so it doesn't run straight away)
    scenes = {
        "Make Cocktail": lambda: make_cocktail(workcell, scene, ur3), # all three in order: will, jack, james
        "Will's Robot": lambda: will_part(workcell, scene, ur3),
        "Jack's Robot": lambda: jack_part(workcell, scene, ur3),
        "James' Robot": lambda: james_part(workcell, scene, ur3),
        "Reset Scene": lambda: reset_scene(workcell, ur3, start_poses),
        "Jack: Go To Test Pose": lambda: jack_test_pose(workcell),
    }
    gui = RobotGUI(workcell, scenes=scenes) # button label -> scene
    gui.build() # add the buttons, dropdown and sliders
    gui.run()