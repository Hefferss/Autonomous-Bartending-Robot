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

        #offsets for each stl, same order as the files below
        mesh_files = ["base_link.stl", "link_1.stl", "link_2.stl", "link_3.stl", "link_4.stl", "link_5.stl", "link_6.stl"]
        self.offsets = [np.eye(4), #base link, no transform needed
                        trotx(-pi/2), #link 1
                        transl(-0.29, 0, 0) @ trotz(-pi/2) @ trotx(-pi/2), #link 2. dh frame is at elbow, stl origin at shoulder
                        np.eye(4), #link 3
                        transl(0, -0.31, 0) @ trotx(-pi/2), #link 4. according to the schematics, its at the wrist point so it needs to be translatsed and offset. 4th adjustment. 0-.25, -0.3, -0.325. -0.32.
                        np.eye(4), #link 5
                        np.eye(4)] #link 6. no transform needed, lines up already

        #the OO mover does link_check[i].A @ offsets[i] for each mesh every step, so these two lists have to be in the same order
        self.meshes = []
        for file in mesh_files:
            self.meshes.append(Mesh(filename=os.path.join(mesh_folder, file), color="#6e6e6e"))