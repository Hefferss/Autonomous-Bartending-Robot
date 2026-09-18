import numpy as np
import swift
from roboticstoolbox import DHLink, DHRobot, jtraj
from math import pi
from ir_support import CylindricalDHRobotPlot
import os
from spatialgeometry import Mesh
from spatialmath.base import trotx, trotz, troty, transl

# Z is d, X is a

#Links for the Kawasaki RS007N.                                                              Specifications from Kawasaki Documentation
link1 = DHLink(d=0, a=0, alpha=0, qlim=[-180*pi/180, 180*pi/180])                         # JT1, "Arm Rotation"
link2 = DHLink(d=0, a=0, alpha=0, qlim=[-135*pi/180, 135*pi/180])                            # JT2, "Arm Out-in"
link3 = DHLink(d=-0, a=0, alpha=0, qlim=[-155*pi/180, 155*pi/180])                       # JT3, "Arm Up-down"
link4 = DHLink(d=0, a=0, alpha=0, qlim=[-200*pi/180, 200*pi/180])                         # JT4, "Wrist Swivel"
link5 = DHLink(d=0, a=0, alpha=0, qlim=[-125*pi/180, 125*pi/180])                        # JT5, "Wrist Bend"
link6 = DHLink(d=0, a=0, alpha=0, qlim=[-360*pi/180, 360*pi/180])                            # JT6, "Wrist Twist"

robot = DHRobot([link1, link2, link3, link4, link5, link6], name='Kawasaki RS007N')
q = np.array([0, pi/2, -pi/2, 0, 0, 0]) #vertical pose, same as the stls

#test cylinders to add to robot. comment out later. same as A1
#cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.025, color="#7b1d1d")
#robot = cyl_viz.create_cylinders()

#sets up environment and robot.
env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
robot.q = q
env.step()

#mesh attached to robot inspired by the pen mesh import in week 4
mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Kawasaki_Robot_Files")
link_check = robot.fkine_all(q) #checks all locations of the robot and lists. used to update position each time. maybe theres a better way?

#Base link
base_location = os.path.join(mesh_folder, "wlink1.stl")
base_mesh = Mesh(filename=base_location, color="#6e6e6e") #red because its the same as my last one and it was ugly as gray.
base_mesh.T = link_check[0].A

#link 1
link_1_location = os.path.join(mesh_folder, "wLink2.stl")
link_1_mesh = Mesh(filename=link_1_location, color="#6e6e6e")
link_1_mesh.T = link_check[1].A

#link 2
link_2_location = os.path.join(mesh_folder, "wLink3.stl")
link_2_mesh = Mesh(filename=link_2_location, color="#6e6e6e")
link_2_mesh.T = link_check[2].A

#link 3
link_3_location = os.path.join(mesh_folder, "wLink4.stl")
link_3_mesh = Mesh(filename=link_3_location, color="#6e6e6e")
link_3_mesh.T = link_check[3].A

#link 4
link_4_location = os.path.join(mesh_folder, "wLink5.stl")
link_4_mesh = Mesh(filename=link_4_location, color="#6e6e6e")
link_4_mesh.T = link_check[4].A 

#link 5
link_5_location = os.path.join(mesh_folder, "wLink6.stl")
link_5_mesh = Mesh(filename=link_5_location, color="#6e6e6e")
link_5_mesh.T = link_check[5].A 

#link 6 
link_6_location = os.path.join(mesh_folder, "wLink7.stl")
link_6_mesh = Mesh(filename=link_6_location, color="#6e6e6e")
link_6_mesh.T = link_check[6].A #no transform needed, lines up already


env.add(base_mesh)
env.add(link_1_mesh)
env.add(link_2_mesh)
env.add(link_3_mesh)
env.add(link_4_mesh)
env.add(link_5_mesh)
env.add(link_6_mesh)
env.step()
input("Enter to continue\n")