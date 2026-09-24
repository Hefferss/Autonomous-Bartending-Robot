import os
import roboticstoolbox as rtb
from ir_support.robots import UR3e
from spatialgeometry import Mesh
import swift
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHLink, DHRobot
from math import pi
from ir_support import CylindricalDHRobotPlot
from ir_support_extra_robots.robots import Turtlebot3Waffle
from ir_support_extra_parts.parts import part_mesh
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, trotz, tr2rpy, r2q
from roboticstoolbox import jtraj
from ir_support import tranimate_custom

env = swift.Swift()
env.launch(realtime=True)
item_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Scene_parts")

#cognac scene load
cognac_location = os.path.join(item_folder, "Cognac.stl")
cognac_scene = Mesh(filename=cognac_location, color="#522B0F", scale=[0.001, 0.001, 0.001])
cognac_scene.T = SE3(0.5, 0.5, 0.125) 
env.add(cognac_scene)

#cointreau.stl
cointreau_location = os.path.join(item_folder, "Cointreau.stl")
cointreau_scene = Mesh(filename=cointreau_location, color="#C47C07", scale=[0.02, 0.02, 0.02])
cointreau_scene.T = SE3(0.5, 0.5, 0.01)
env.add(cointreau_scene)

#lemon.stl 
lemon_location = os.path.join(item_folder, "Lemon.stl")
lemon_scene = Mesh(filename=lemon_location, color="#F7F7A1")
lemon_scene.T = SE3(0.2, 0.2, 0.02)
env.add(lemon_scene)

#bar_table.stl
bar_table_location = os.path.join(item_folder, "bar.stl")
bar_table_scene = Mesh(filename=bar_table_location, color="#8B4513", scale=[0.001, 0.001, 0.001])
bar_table_scene.T = SE3(0.5, 0.5, 0.01)
env.add(bar_table_scene)
env.step()

input("Press Enter to continue...")