# ======================================================================
# OO_WillRobotTest.py  --  ROBOT-SPECIFIC setup for the Kawasaki RS007N
#
# This class describes ONLY the robot (links, meshes, alignment).
# For another robot, copy this file and change the numbers/filenames.
# It must provide:  .robot  .meshes  .offsets  .home_q
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
import os                                               # for building file paths
from math import pi                                     # the constant pi
from roboticstoolbox import DHLink, DHRobot             # to build the DH robot
from ir_support import CylindricalDHRobotPlot           # draws cylinders on the robot
from spatialgeometry import Mesh                        # loads STL files
from spatialmath.base import trotx, trotz, troty, transl  # rotation/translation matrices


# ------------------------ THE ROBOT CLASS -----------------------------
class KawasakiRS007N:

    # ---- Constructor: builds everything about this robot --------------
    def __init__(self):

        # -- Block 1: the DH links --
        # Overview: one DHLink per joint, values from the Kawasaki
        # documentation (pg23). qlim is the joint limit in radians.
        link1 = DHLink(d=0.36,  a=0,     alpha=pi/2,  qlim=[-180*pi/180, 180*pi/180])  # JT1 Arm Rotation
        link2 = DHLink(d=0,     a=0.355, alpha=0,     qlim=[-135*pi/180, 135*pi/180])  # JT2 Arm Out-in
        link3 = DHLink(d=0,     a=0,     alpha=-pi/2, qlim=[-155*pi/180, 155*pi/180])  # JT3 Arm Up-down
        link4 = DHLink(d=0.375, a=0,     alpha=pi/2,  qlim=[-200*pi/180, 200*pi/180])  # JT4 Wrist Swivel
        link5 = DHLink(d=0,     a=0,     alpha=-pi/2, qlim=[-125*pi/180, 125*pi/180])  # JT5 Wrist Bend
        link6 = DHLink(d=0.078, a=0,     alpha=0,     qlim=[-360*pi/180, 360*pi/180])  # JT6 Wrist Twist

        # -- Block 2: build the robot and add cylinders --
        self.robot = DHRobot([link1, link2, link3, link4, link5, link6], name='Kawasaki RS007N')
        cyl_viz = CylindricalDHRobotPlot(self.robot, cylinder_radius=0.025, color="#7b1d1d")
        self.robot = cyl_viz.create_cylinders()

        # -- Block 3: home pose --
        self.home_q = [0, pi/2, -pi/2, 0, -pi/4, 0]     # vertical pose, same as the STLs

        # -- Block 4: load the STL meshes --
        mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Kawasaki_Robot_Files")
        file_names = ["wBase.stl", "wLink1.stl", "wLink2.stl", "wLink3.stl",
                      "wLink4.stl", "wLink5.stl", "wLink6.stl"]
        self.meshes = []                                 # empty list to fill
        for name in file_names:                          # go through each filename
            location = os.path.join(mesh_folder, name)   # full path to the file
            mesh = Mesh(filename=location, color="#5B5B5B8D", scale=[0.001, 0.001, 0.001])  # load it (mm -> m)
            self.meshes.append(mesh)                     # add it to the list

        # -- Block 5: mesh alignment offsets (one per mesh, same order) --
        self.offsets = [
            trotz(-pi/2) @ transl(-0.15, -0.2675, 0),                                             # base
            trotx(-pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -0.36),                          # link 1
            trotx(-pi/2) @ troty(pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -0.715),           # link 2
            trotz(-pi/2) @ transl(-0.15, -0.2675, -0.715),                                        # link 3
            trotx(-pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -1.090),                         # link 4
            trotz(-pi/2) @ transl(-0.15, -0.2675, -1.090),                                        # link 5
            trotz(-pi/2) @ transl(-0.15, -0.2675, -1.168),                                        # link 6
        ]