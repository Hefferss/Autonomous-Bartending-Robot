import numpy as np
from roboticstoolbox import DHLink, DHRobot
from math import pi
import os
from spatialmath.base import trotx, trotz, transl
from spatialgeometry import Mesh

# Jack's robot: Staubli TX2-60 (standard, not the L). 6 dof, 670 mm reach, 4.5 kg payload. does the lid + shake station
# dims and joint limits from the official Staubli TX2-60 datasheet, cross checked with the ros-industrial staubli_tx2_60_support xacro
# datasheet checks: vertical height 375+290+310+70 = 1045 mm, axis 1-5 reach 290+310 = 600 mm, axis 3-5 reach 310 mm (= d4), flange adds 70 -> 670 mm
class StaubliTX60:
    def __init__(self, base=None): #base is where it sits in the scene, e.g. transl(0, 0.75, 0.25). leave empty for the origin

        # DH links (standard DH: d along z, a along x, alpha = twist between joints). no joint offsets, so q = 0 is an L shape
        # qlims: datasheet ranges are measured from the staubli zero (arm straight up). straight up is q2 = +90, q3 = -90 in my DH, so those two are shifted
        link1 = DHLink(d=0.375, a=0, alpha=pi/2, qlim=[-pi, pi]) #base height 375 mm (shoulder above the base). +-180
        link2 = DHLink(d=0, a=0.29, alpha=0, qlim=[-127.5*pi/180, 127.5*pi/180], offset=pi/2) #upper arm 290 mm. +-127.5 from straight up, shifted +90
        link3 = DHLink(d=-0.02, a=0, alpha=-pi/2, qlim=[-142.5*pi/180, 142.5*pi/180], offset=-pi/2) #20 mm elbow offset, negative so it's on the same side as the stl. +-142.5 shifted -90
        link4 = DHLink(d=0.31, a=0, alpha=pi/2, qlim=[-270*pi/180, 270*pi/180]) #forearm 310 mm (elbow to wrist centre). +-270
        link5 = DHLink(d=0, a=0, alpha=-pi/2, qlim=[-121*pi/180, 132.5*pi/180]) #wrist bend, no length (spherical wrist). -121 / +132.5, not symmetric on the datasheet
        link6 = DHLink(d=0.07, a=0, alpha=0, qlim=[-270*pi/180, 270*pi/180]) #wrist centre to flange 70 mm. +-270

        self.robot = DHRobot([link1, link2, link3, link4, link5, link6], name='Staubli TX60')
        self.home_q = [0, 0, 0, 0, 0, 0] #straight up, same pose as the stls. it's a singularity (wrist and arm lined up), so RMRC never starts from here, IK to a pose first
        if base is not None:
            self.robot.base = base

        #stls are the ros-industrial TX60 meshes. loading them like the pen mesh in week 4
        mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Staubli Robot Files")

        #offsets: where each stl sits relative to its DH frame. each mesh is drawn at fkine_all(q)[i] @ offsets[i] every step
        #worked out by putting the robot in the vertical pose and lining each stl up with its link
        mesh_files = ["base_link.stl", "link_1.stl", "link_2.stl", "link_3.stl", "link_4.stl", "link_5.stl", "link_6.stl"]
        self.offsets = [np.eye(4), #base link, no transform needed
                        trotx(-pi/2), #link 1. DH frame 1 is twisted 90 deg (alpha = pi/2), stl isn't, so rotate it back
                        transl(-0.29, 0, 0) @ trotz(-pi/2) @ trotx(-pi/2), #link 2. DH frame is at the elbow but the stl origin is at the shoulder, so step back the 290 mm then rotate
                        np.eye(4), #link 3. lines up already
                        transl(0, -0.31, 0) @ trotx(-pi/2), #link 4. DH frame is at the wrist but the stl origin is at the elbow, so step back the 310 mm then rotate
                        np.eye(4), #link 5. lines up already
                        np.eye(4)] #link 6. lines up already

        #the OO mover does link_check[i].A @ offsets[i] for each mesh every step, so these two lists have to be in the same order
        self.meshes = []
        for file in mesh_files:
            self.meshes.append(Mesh(filename=os.path.join(mesh_folder, file), color="#5e5e5e"))

#Guide for stuff for my robot

# Q = 0 homing and the joint limits, and the DH parameters are above. (lines 17-22 and 24 for DH, lines 15,16,18,19 and 25 for qlim shift and home_q respectively)

#fkine/fkine_all/basein mainV2 as well as in movement commands, OO_MovementCommands.py 98 for the fkine of the meshes, 87 and 102 for teh fkine used for hold and 151 and 152 for the table

# grab release has OO_MovementCommands.py grab line 86 (stores item pose relative to the gripper), release line 92

#Mesh offsets are applied in DH in line 34 and updated in OO_MovementCommands.py lines 97 to 100, 

#Singularity/ manipulatbility stuff. OO_MovementCommands.py gives the jacobian and 324 to 330 has manipuability and the damping. 

#Ikine mostly in movement commands, 163 for solve, 170 for ikine_LM, 193 for move_to_pose, which has solve ikine in it, and mainv2, in which lnes 137 and 139 use IK to move jack_part

#jtraj in 117, 118 of movment commands (move_to_q)   , used again in 153 for my move to home

# RMRC, MovementCOmmands has the 285 move_RMRC, 300 for trapezoid, 325 for the jacobian and t245 has the rmrc_to wrap. Main V2 has the 141 lid down, 144 lift, 146 shake, 149 carry to james and the 150,152 down and up movements, all of which use the RMRC to move the robot.

#Collision has OO_MovementCommands.py for check_collisoin (134), and above_table(150), as well as 205 replan for move_to_pose. OO_Collisoin as the elipsoids on 24, collides on 68, 85 for the link_points and 97 for the self_hit. Line 83 of movement commands adds in the collisoin for ma robot. Also, for the end effectory, its in above_table in line 150 of  movenet commands. 

#Grab and hand off has OO_MovementCommands.py with grab (line86) and release (line 92). MainV2 has 142 for the grab and 151 for the release.

#GUI, 120 is the joint slider, and 151 is the jog, and movemenet commands as teh jog functoin itself, in line 355.