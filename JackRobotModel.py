import numpy as np
import swift
from roboticstoolbox import DHLink, DHRobot
from math import pi
from ir_support import CylindricalDHRobotPlot
import os
from spatialgeometry import Mesh
from spatialmath.base import trotx, trotz, troty, transl

#links for the staubli. the qlims are specific. yay.
link1 = DHLink(d=0.3275, a=0, alpha=pi/2, qlim=[-pi, pi]) #diff from sheet in d because it didnt align with the stl. 
link2 = DHLink(d=0, a=0.29, alpha=0, qlim=[-127.5*pi/180, 127.5*pi/180])
link3 = DHLink(d=0.02, a=0, alpha=-pi/2, qlim=[-142.5*pi/180, 142.5*pi/180])
link4 = DHLink(d=0.31, a=0, alpha=pi/2, qlim=[-270*pi/180, 270*pi/180])
link5 = DHLink(d=0, a=0, alpha=-pi/2, qlim=[-121*pi/180, 132.5*pi/180])
link6 = DHLink(d=0.07, a=0, alpha=0, qlim=[-270*pi/180, 270*pi/180])

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
#Base link
mesh_folder = "Staubli Robot Files"
base_location = os.path.join(mesh_folder, "base_link.stl")
base_mesh = Mesh(filename=base_location)
base_mesh = Mesh(filename=base_location, color="#7b1d1d") #red because its the same as my last one and it was ugly as gray.
env.add(base_mesh)
env.step()

#link 1
mesh_folder = "Staubli Robot Files"
link_1_location = os.path.join(mesh_folder, "link_1.stl")
link_1_mesh = Mesh(filename=link_1_location, color="#7b1d1d") 
link_check = robot.fkine_all(q) #checks all locations of the robot and lists. used to update position each time. maybe theres a better way?
link_1_mesh_place = link_check[1].A
link_1_mesh.T = link_1_mesh_place @ trotx(pi) @ troty(-pi/2)

#link 2
mesh_folder = "Staubli Robot Files"
link_2_location = os.path.join(mesh_folder, "link_2.stl")
link_2_mesh = Mesh(filename=link_2_location, color="#7b1d1d") 
link_check = robot.fkine_all(q) 
link_2_mesh_place = link_check[2].A
link_2_mesh.T = link_2_mesh_place @ trotx(pi) @ trotz(pi/2) @ troty(pi) @ trotx(pi) @ transl(0, -0.23, 0)

env.add(link_1_mesh)
env.add(link_2_mesh)
env.step()
input("Enter to continue\n")
