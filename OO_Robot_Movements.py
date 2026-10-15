from math import pi
from spatialmath import SE3

# where the EE goes to grab the shaker (its top). shaker is 0.21 tall, tables are 0.25 high
SHAKER_STORAGE = (0, -0.2, 0.465)
SHAKER_TOP = 0.465 # top of the shaker when it's sitting on any table
WILL_DROP_ZONE = (0.85, 0.45) # UR3 leaves things here for Will. front of the (stretched) robot table
WILL_SHAKER_SPOT = (0.72, 0.62) # Will keeps the shaker here to pour into


class RobotMovements:
    def __init__(self, workcell, scene, ur3_setup):
        self.ur3 = workcell.movers["UR3"]
        self.will = workcell.movers["Will's Robot"]
        self.ur3_setup = ur3_setup # UR3Robot object, needed for move_base (the rail)
        self.shaker = scene.cocktail_shaker_scene

    def move(self, mover, xyz, steps=50): # wills move_to_pose, tool pointing down
        mover.move_to_pose(SE3(*xyz) * SE3.Rx(pi), steps)

    def pick_drop(self, mover, xyz, item=None): # above, down, grab (if given an item) or let go, back up
        self.move(mover, (xyz[0], xyz[1], xyz[2] + 0.1))
        self.move(mover, xyz, 25)
        if item is not None:
            mover.grab(item)
        else:
            mover.release()
        self.move(mover, (xyz[0], xyz[1], xyz[2] + 0.1), 25)

    def ur3_tuck(self): # UR3 pulls back over the rail so it's out of the way
        self.move(self.ur3, (self.ur3.robot.base.t[0], 0.0, 0.7))

    def shaker_to_will(self):
        self.ur3_setup.move_base(self.ur3, SHAKER_STORAGE[0]) # slide to the shaker
        self.pick_drop(self.ur3, SHAKER_STORAGE, self.shaker) # grab it
        self.ur3_tuck()
        self.ur3_setup.move_base(self.ur3, WILL_DROP_ZONE[0]) # slide over to Will
        self.pick_drop(self.ur3, (*WILL_DROP_ZONE, SHAKER_TOP)) # put it in his drop off zone
        self.ur3_tuck() # get out of the way before Will reaches in
        self.pick_drop(self.will, (*WILL_DROP_ZONE, SHAKER_TOP), self.shaker) # Will picks it up
        self.pick_drop(self.will, (*WILL_SHAKER_SPOT, SHAKER_TOP)) # and puts it on his shaker spot
        self.will.move_to_q(self.will.home_q) # Will stands back up