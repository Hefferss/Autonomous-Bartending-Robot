# ======================================================================
# OO_IRB140Robot.py  --  ROBOT-SPECIFIC setup for the ABB IRB 140
#
# This class describes ONLY the robot (links, meshes, alignment).
# It has the same four attributes as the other robot setup classes, so
# the Workcell and GUI can use it without any changes:
#     .robot  .meshes  .offsets  .home_q
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
import os                                               # for building file paths
from math import pi                                     # the constant pi
from roboticstoolbox import DHLink, DHRobot             # to build the DH robot
from ir_support import CylindricalDHRobotPlot           # draws cylinders on the robot
from spatialgeometry import Mesh                        # loads STL files
from spatialmath.base import trotx, troty, trotz, transl  # rotation/translation matrices


# ------------------------ THE ROBOT CLASS -----------------------------
class ABBIRB140:

    # ---- Constructor: builds everything about this robot --------------
    # base is optional: pass a 4x4 matrix (for example transl(1, 0, 0))
    # to place this robot somewhere other than the origin, so it doesn't
    # sit on top of the other robots in the shared scene.
    def __init__(self, base=None):

        # -- Block 1: the DH links --
        # Overview: one DHLink per joint. d = offset along z, a = link
        # length, alpha = twist between joints, qlim = joint limits (radians).


        link1 = DHLink(d=0.352, a=0.07, alpha=-pi/2, qlim=[-pi, pi])
        link2 = DHLink(d=0.0,   a=0.36, alpha=0, qlim=[-100*pi/180, 100*pi/180], offset=-pi/2)
        link3 = DHLink(d=0.0,   a=0.0,  alpha=-pi/2, qlim=[-140*pi/180, 140*pi/180], offset=-pi/2)
        link4 = DHLink(d=0.38,  a=0.0,  alpha=pi/2,  qlim=[-200*pi/180, 200*pi/180])
        link5 = DHLink(d=0.0,   a=0.0,  alpha=-pi/2, qlim=[-120*pi/180, 120*pi/180])
        link6 = DHLink(d=0.065, a=0.0,  alpha=0,     qlim=[-400*pi/180, 400*pi/180])


        # -- Block 2: build the robot and add cylinders --
        # Overview: joins the links into one robot, then draws cylinders
        # along it. (The original file built the robot twice; once is enough.)
        self.robot = DHRobot([link1, link2, link3, link4, link5, link6], name='ABB IRB 140')
        #cyl_viz = CylindricalDHRobotPlot(self.robot, cylinder_radius=0.05, color="#702963")
        #self.robot = cyl_viz.create_cylinders()

        # -- Block 3: optional base position --
        if base is not None:                             # was a base position given?
            self.robot.base = base                       # yes: move the whole robot there

        # -- Block 4: home pose --
        self.home_q = [0, 0, 0, 0, 0, 0]          # vertical pose, same as the STLs

        # -- Block 5: load the STL meshes --
        # Overview: finds the STL folder next to this file, then loads the
        # 7 STL files (base + 6 links) into a list in order.
        mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "IRB_Robot_Files")
        file_names = ["base.stl", "link 1.stl", "link 2.stl", "link 3.stl",
                      "link 4.stl", "link 5.stl", "link 6.stl"]
        self.meshes = []                                 # empty list to fill
        for name in file_names:                          # go through each filename
            location = os.path.join(mesh_folder, name)   # full path to the file
            mesh = Mesh(filename=location, color="#FFAA00", scale=[0.001, 0.001, 0.001])  # load it (mm -> m)
            self.meshes.append(mesh)                     # add it to the list

        # -- Block 6: mesh alignment offsets --
        # Overview: each STL needs shifting/rotating so it sits correctly
        # on its link. One matrix per mesh, in the same order as above.
        self.offsets = [
            transl(-0.31, -0.2, 0),                                                 
            trotx(pi/2) @ transl(-0.22, -0.13, -0.35),                                            
            trotx(pi/2) @ troty(pi/2) @ transl(-0.099, -0.175, -0.7055),              
            troty(pi/2) @ trotz(pi) @ transl(-0.15, -0.0825, -0.715),                      # link 3
            trotx(pi/2) @ troty(pi/2) @ transl(-0.45, -0.065, -0.715),                 # link 4
            troty(-pi/2) @ transl(-0.45, -0.0155, -0.71),                                        
            troty(-pi/2) @ transl(-0.51, -0.025, -0.71),                                          
        ]

