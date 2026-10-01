import numpy as np
import swift
from roboticstoolbox import DHLink, DHRobot, jtraj
from math import pi
from ir_support import CylindricalDHRobotPlot
import os
from spatialmath.base import ellipsoid, trotx, trotz, troty, transl
from spatialmath import SE3
from spatialgeometry import Mesh

#links for the staubli. the qlims are specific. yay.
link1 = DHLink(d=0.375, a=0, alpha=pi/2, qlim=[-pi, pi]) #0.375 from sheet, lines up with stl now mesh transforms are right
link2 = DHLink(d=0, a=0.29, alpha=0, qlim=[-127.5*pi/180, 127.5*pi/180])
link3 = DHLink(d=-0.02, a=0, alpha=-pi/2, qlim=[-142.5*pi/180, 142.5*pi/180]) #negative so offset is same side as stl
link4 = DHLink(d=0.31, a=0, alpha=pi/2, qlim=[-270*pi/180, 270*pi/180])
link5 = DHLink(d=0, a=0, alpha=-pi/2, qlim=[-121*pi/180, 132.5*pi/180])
link6 = DHLink(d=0.07, a=0, alpha=0, qlim=[-270*pi/180, 270*pi/180])

robot = DHRobot([link1, link2, link3, link4, link5, link6], name='Staubli TX60')
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
mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Staubli Robot Files")
link_check = robot.fkine_all(q) #checks all locations of the robot and lists. used to update position each time. maybe theres a better way?

#Base link
base_location = os.path.join(mesh_folder, "base_link.stl")
base_mesh = Mesh(filename=base_location, color="#6e6e6e") #red because its the same as my last one and it was ugly as gray.
base_mesh.T = link_check[0].A

#link 1
link_1_location = os.path.join(mesh_folder, "link_1.stl")
link_1_mesh = Mesh(filename=link_1_location, color="#6e6e6e")
link_1_mesh.T = link_check[1].A @ trotx(-pi/2)

#link 2
link_2_location = os.path.join(mesh_folder, "link_2.stl")
link_2_mesh = Mesh(filename=link_2_location, color="#6e6e6e")
link_2_mesh.T = link_check[2].A @ transl(-0.29, 0, 0) @ trotz(-pi/2) @ trotx(-pi/2) #dh frame is at elbow, stl origin at shoulder

#link 3
link_3_location = os.path.join(mesh_folder, "link_3.stl")
link_3_mesh = Mesh(filename=link_3_location, color="#6e6e6e")
link_3_mesh.T = link_check[3].A

#link 4
link_4_location = os.path.join(mesh_folder, "link_4.stl")
link_4_mesh = Mesh(filename=link_4_location, color="#6e6e6e")
link_4_mesh.T = link_check[4].A @ transl(0, -0.31, 0) @ trotx(-pi/2) #according to the schematics, its at the wrist point so it needs to be translatsed and offset. 4th adjustment. 0-.25, -0.3, -0.325. -0.32.

#link 5
link_5_location = os.path.join(mesh_folder, "link_5.stl")
link_5_mesh = Mesh(filename=link_5_location, color="#6e6e6e")
link_5_mesh.T = link_check[5].A 

#link 6 
link_6_location = os.path.join(mesh_folder, "link_6.stl")
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

#COLLISION STUFF BELOW. based mostly off lab 5 question 1 and lab 6 q2 elipsoids. using elipsoid method

#create an elipsoid. 
elipsoid_fatness = 0.1 

def elipsoids_for_robot_around_links(q): #will be called every time j traj runs for this robot. Idea here is to make an elipsoid for each link and then use this to attach them to Q's to check for collisions.
    Joint_Pos_Rotation = [T.A for T in robot.fkine_all(q)] #same way as in lab 6 q2. gets a 4x4 matrix for each joint so we can work out positoin adn rotation. get_transforms from lab 6.
    Ellipsoid_all = [] # makes an empty array for the elipsoids to be added to. same idea as week 6 q2.
    for i in range(len(Joint_Pos_Rotation)-1): #for each link (between joint i and i+1):
            Elipsoid_vector = Joint_Pos_Rotation[i+1][:3,3] - Joint_Pos_Rotation[i][:3,3] #arrow from this joint to the next, along the link.
            Elipsoid_width = np.array([-Elipsoid_vector[1], Elipsoid_vector[0], 0]) #left and right of the link
            Elipsoid_height = np.cross(Elipsoid_vector, Elipsoid_width) #up and down from the link
            Elipsoid_centre = (Joint_Pos_Rotation[i+1][:3,3] + Joint_Pos_Rotation[i][:3,3])/2 #middle of the link, where the elipsoid sits. halfway between the two joints. Lab 6 2.5
            
            if np.linalg.norm(Elipsoid_vector) != 0:
                Elipsoid_direction = Elipsoid_vector / np.linalg.norm(Elipsoid_vector) #along the link, length 1
            else:
                Elipsoid_direction = Elipsoid_vector #link with no length, leave it as zeros. This is for link 5, because it doesnt have a length. just a direction.
            if np.linalg.norm(Elipsoid_width) != 0:
                Elipsoid_width = Elipsoid_width / np.linalg.norm(Elipsoid_width)
            if np.linalg.norm(Elipsoid_height) != 0:
                Elipsoid_height = Elipsoid_height / np.linalg.norm(Elipsoid_height)
                
            Elipsoid_rotation_matrix = np.column_stack((Elipsoid_direction, Elipsoid_width, Elipsoid_height)) #which way the elipsoid points. lab 6 2.7. makes it so the matrix lines up the long axis with the link
            Elipsoid_size = np.diag(np.square([np.linalg.norm(Elipsoid_vector/2), elipsoid_fatness, elipsoid_fatness])) #radii squared on the diagonal. lab 6 2.1
            Elipsoid_size_rotation = Elipsoid_rotation_matrix @ Elipsoid_size @ Elipsoid_rotation_matrix.T #size and direction packed into one 3x3. lab 6 2.7
            Ellipsoid_all.append({'matrix': Elipsoid_size_rotation, 'center': Elipsoid_centre}) #save this link's elipsoid. lab 6 2.10
    return Ellipsoid_all #give the list back once every link is done. updates it to Ellispod_all every time jtraj runs.

#Notes for me when im not sleepy:
# - Need to draw the elipsoids in swift so i can check them
# - Need to make sure they update when the robot moves, so they are always in the right place
# - Need to add an obstacle and check that this robot works
# - need to code a safety solution and brainstorm how i can make it work properly when there are obstacles
            