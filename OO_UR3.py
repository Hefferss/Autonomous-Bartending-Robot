import os
import numpy as np
from spatialgeometry import Mesh
from spatialmath import SE3
from spatialmath.base import transl
from roboticstoolbox import models

#toolbox UR3 ignores its base when it does ikine_LM so it aims at the wrong spot once its moved off the origin. this shifts the target into the robots own frame first
class UR3BaseFix(models.UR3):
    def ikine_LM(self, target_pose, **extra_settings):
        return super().ikine_LM(self.base.inv() * target_pose, **extra_settings) #adds all the GUI extra settings. will be updated depending on whatever WIll changes in the GUi code

#same as UR3.py but class (object orientated):) the OO files need .robot .meshes .offsets .home_q
class UR3Robot:
    def __init__(self, base=None): #base is where the UR3 itself sits. the purple platform goes under it by itself

        self.robot = UR3BaseFix() #toolbox UR3. comes with its own meshes so no stls needed for the arm
        self.home_q = np.zeros(self.robot.n) #same start pose as UR3.py
        #joint limits (deg) tighter than the toolbox's +-360 so it can't fold into itself: shoulder stays above horizontal, elbow can't fold flat, no full spins
        self.robot.qlim = np.radians([[-180, -180, -160, -180, -180, -180], [ 180,    0,  160,  180,  180,  180]])
        if base is not None:
            self.robot.base = base

        #purple platform the UR3 sits on
        item_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Scene_parts")
        platform_location = os.path.join(item_folder, "BasePlatForUR3.stl")
        ur3_platform = Mesh(filename=platform_location, color="#430F98", scale=[0.001, 0.001, 0.001])
        platform_offset = transl(-0.08, 0, -0.1) #platform is behind and under the robot base. same numbers as set_UR3_pose in UR3.py, just from the robot instead of from the platform

        #new OO mesh mover stuff because i have to do it different now
        self.meshes = [ur3_platform]
        self.offsets = [platform_offset]

    def move_base(self, mover, target_x, steps=30): #moves the UR3 base along the x-axis to a target position over a number of steps. should lock on rail. need to add a thing that makes sure it doesnt slide off lol.
        start_x = self.robot.base.t[0]
        move_step = (target_x - start_x) / steps

        for i in range(steps):
            if mover.workcell.state != "RUNNING": #estop pressed, stop sliding. new estop stuff from will.  
                return False
            self.robot.base = self.robot.base * SE3(move_step, 0, 0) 
            mover.update_meshes(self.robot.q) #platform follows the base. instead of (ur3_base_mesh.T = ur3_base_mesh.T * SE3(move_step, 0, 0))
            mover.workcell.step(0.05) #workcell.step instead of env.step so the xbox estop gets checked while sliding too
        return True