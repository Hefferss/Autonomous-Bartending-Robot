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
mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Scene_parts")
#loading hennessy lol
hennessy_location = os.path.join(mesh_folder, "hennessy.stl")
hennessy_mesh = Mesh(filename=hennessy_location, scale=[0.0254, 0.0254, 0.0254], color="#4e2a12")
hennessy_mesh.T = transl(-0.3, 0.3, 0.9) @ trotx(pi/2)
env.add(hennessy_mesh)

env.step()
input("Enter to finish\n")