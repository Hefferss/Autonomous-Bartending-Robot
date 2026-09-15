import numpy as np
import swift
from roboticstoolbox import DHLink, DHRobot
from math import pi
from ir_support import CylindricalDHRobotPlot
import os
from spatialgeometry import Mesh

#links for the staubli
link1 = DHLink(d=0.375, a=0, alpha=pi/2, qlim=[-pi, pi])
link2 = DHLink(d=0, a=0.29, alpha=0, qlim=[-pi/2, pi/2])
link3 = DHLink(d=0.02, a=0, alpha=-pi/2, qlim=[-pi/2, pi/2])
link4 = DHLink(d=0.31, a=0, alpha=pi/2, qlim=[-pi, pi])
link5 = DHLink(d=0, a=0, alpha=-pi/2, qlim=[-pi, pi])
link6 = DHLink(d=0.07, a=0, alpha=0, qlim=[-pi, pi])

robot = DHRobot([link1, link2, link3, link4, link5, link6], name='Staubli TX60')
q = np.array([0, 0, 0, 0, 0, 0])

#test cylinders to add to robot. comment out later. same as A1
cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.025, color="#7b1d1d")
robot = cyl_viz.create_cylinders()

#sets up environment and robot.
env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
robot.q = q
env.step()

#mesh attached to robot inspired by the pen mesh import in week 4
#Base joint
mesh_folder = "Staubli Robot Files"
base_location = os.path.join(mesh_folder, "base_link.stl")
base_mesh = Mesh(filename=base_location)
base_mesh = Mesh(filename=base_location, color="#7b1d1d") #red because its the same as my last one and it was ugly as gray.
env.add(base_mesh)
env.step()
input("Enter to continue\n")

#Joint 1
mesh_folder = "Staubli Robot Files"
joint_1_location = os.path.join(mesh_folder, "link_1.stl")
joint_1_mesh = Mesh(filename=joint_1_location, color="#7b1d1d") 
env.add(joint_1_mesh)
env.step()
input("Enter to continue\n")