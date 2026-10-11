import numpy as np
from roboticstoolbox import DHLink, DHRobot
from math import pi
import os
from spatialmath.base import trotx, trotz, transl
from spatialgeometry import Mesh

#same as jackrobotmodel but class (object orientated):)
class StaubliTX60:
    def __init__(self, base=None): #base is where it sits in the scene, e.g. transl(0, 0.75, 0.25). leave empty for the origin

        #links for the staubli. the qlims are specific. yay.
        link1 = DHLink(d=0.375, a=0, alpha=pi/2, qlim=[-pi, pi]) #0.375 from sheet, lines up with stl now mesh transforms are right
        link2 = DHLink(d=0, a=0.29, alpha=0, qlim=[-127.5*pi/180, 127.5*pi/180])
        link3 = DHLink(d=-0.02, a=0, alpha=-pi/2, qlim=[-142.5*pi/180, 142.5*pi/180]) #negative so offset is same side as stl
        link4 = DHLink(d=0.31, a=0, alpha=pi/2, qlim=[-270*pi/180, 270*pi/180])
        link5 = DHLink(d=0, a=0, alpha=-pi/2, qlim=[-121*pi/180, 132.5*pi/180])
        link6 = DHLink(d=0.07, a=0, alpha=0, qlim=[-270*pi/180, 270*pi/180])

        self.robot = DHRobot([link1, link2, link3, link4, link5, link6], name='Staubli TX60')
        self.home_q = [0, pi/2, -pi/2, 0, 0, 0] #vertical pose, same as the stls
        if base is not None:
            self.robot.base = base

        #mesh attached to robot inspired by the pen mesh import in week 4
        mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Staubli Robot Files")

        #Base link
        base_location = os.path.join(mesh_folder, "base_link.stl")
        base_mesh = Mesh(filename=base_location, color="#6e6e6e")
        base_offset = np.eye(4) #no transform needed

        #link 1
        link_1_location = os.path.join(mesh_folder, "link_1.stl")
        link_1_mesh = Mesh(filename=link_1_location, color="#6e6e6e")
        link_1_offset = trotx(-pi/2)

        #link 2
        link_2_location = os.path.join(mesh_folder, "link_2.stl")
        link_2_mesh = Mesh(filename=link_2_location, color="#6e6e6e")
        link_2_offset = transl(-0.29, 0, 0) @ trotz(-pi/2) @ trotx(-pi/2) #dh frame is at elbow, stl origin at shoulder

        #link 3
        link_3_location = os.path.join(mesh_folder, "link_3.stl")
        link_3_mesh = Mesh(filename=link_3_location, color="#6e6e6e")
        link_3_offset = np.eye(4)

        #link 4
        link_4_location = os.path.join(mesh_folder, "link_4.stl")
        link_4_mesh = Mesh(filename=link_4_location, color="#6e6e6e")
        link_4_offset = transl(0, -0.31, 0) @ trotx(-pi/2) #according to the schematics, its at the wrist point so it needs to be translatsed and offset. 4th adjustment. 0-.25, -0.3, -0.325. -0.32.

        #link 5
        link_5_location = os.path.join(mesh_folder, "link_5.stl")
        link_5_mesh = Mesh(filename=link_5_location, color="#6e6e6e")
        link_5_offset = np.eye(4)

        #link 6
        link_6_location = os.path.join(mesh_folder, "link_6.stl")
        link_6_mesh = Mesh(filename=link_6_location, color="#6e6e6e")
        link_6_offset = np.eye(4) #no transform needed, lines up already

        #the OO mover does link_check[i].A @ offsets[i] for each mesh every step, so these two lists have to be in the same order
        self.meshes = [base_mesh, link_1_mesh, link_2_mesh, link_3_mesh, link_4_mesh, link_5_mesh, link_6_mesh]
        self.offsets = [base_offset, link_1_offset, link_2_offset, link_3_offset, link_4_offset, link_5_offset, link_6_offset]