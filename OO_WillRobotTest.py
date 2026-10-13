
# Comment Author - Will:
# This is the file for my Robot the Kawasaki RS007N. It was useful for setting them up and testing but in future we should just have all robot
# Classes in the one file i think. The point of the robot class in this case the KawasakiRS007N class is to specify the links, the meshes and the folder they're stored in
# and the offsets that each mesh file needs to be reaaranged correctly. To work out whether this was correct we used Cylinder bot and matched the Meshes as well as we could with
# These Offsets. the Robot class should match around the same with different DH links, stls and offsets.

# In previous versions it was easier to edit with all of the STLs setup piece by piece, but the new simpler alternative is that the a for loop iterates through them all which just shrinks the code
# The other disadvantage is that you cant assign one link a different colours to the others. To be honest we could go back to former way

# Also important to note we got the import stl code from the weekly labs pen example, which makes it easy to assign colours and scale STLs rather than the 5.1 Dh robot example code

# For a robot class to work with the other movement files setup they need to provide the .robot, .meshes, .offsets and .home_q.
# What these are is explained and specified down below


import os                                               # Building file paths
from math import pi                                     
from roboticstoolbox import DHLink, DHRobot             # To Build the DH robot
from ir_support import CylindricalDHRobotPlot           # Cylinders that are visual rep of DH Paramters. Essential for Testing & Fixing Mesh offsets
from spatialgeometry import Mesh                        # loads STL files
from spatialmath.base import trotx, trotz, troty, transl  # rotation/translation matrices


# Defines the Robot Class - In this case Will's Robot the Kawasaki RS007N
class KawasakiRS007N:

    # This is the Constructor. This is what specifies the properties of the robot that the other files require
    # This is namely the robot, .meshes, .offsets and .home_q parts.
    def __init__(self):

        # These are the DH Parameters. Joint Lims taken from the Kawasaki Manual
        # The DH Paramaters were inferred using the Alpha from Jack's robot as they're very similar
        # The d and a values were taken from the manual and confirmed after trial and error
        link1 = DHLink(d=0.36,  a=0,     alpha=pi/2,  qlim=[-180*pi/180, 180*pi/180])  # JT1 Arm Rotation
        link2 = DHLink(d=0,     a=0.355, alpha=0,     qlim=[-135*pi/180, 135*pi/180])  # JT2 Arm Out-in
        link3 = DHLink(d=0,     a=0,     alpha=-pi/2, qlim=[-155*pi/180, 155*pi/180])  # JT3 Arm Up-down
        link4 = DHLink(d=0.375, a=0,     alpha=pi/2,  qlim=[-200*pi/180, 200*pi/180])  # JT4 Wrist Swivel
        link5 = DHLink(d=0,     a=0,     alpha=-pi/2, qlim=[-125*pi/180, 125*pi/180])  # JT5 Wrist Bend
        link6 = DHLink(d=0.078, a=0,     alpha=0,     qlim=[-360*pi/180, 360*pi/180])  # JT6 Wrist Twist

        # The self. prefix is what allows us to share the functions in the other files between robots using OO Programming
        # When an instance of this robot is created the its name may be different but the .robot spec remains the same
        # This is the line that uses the DHRobot from IR support to actually build the robot mathmatically allowing IK and jtrag functions
        self.robot = DHRobot([link1, link2, link3, link4, link5, link6], name='Kawasaki RS007N')
        
        # This is the CylinderBot Code which is essential for testing - Not currently needed, leave commented 

        #cyl_viz = CylindricalDHRobotPlot(self.robot, cylinder_radius=0.025, color="#7b1d1d")
        #self.robot = cyl_viz.create_cylinders()

        # -- Block 2b: where this robot stands (edit this line) --
        # transl(x, y, z) is in metres. To turn the robot as well, multiply by a
        # rotation, e.g. transl(1, 0, 0) @ trotz(pi) turns it round to face -x.
        # Setting the base moves the whole robot; the meshes follow automatically.
        self.robot.base = transl(0.0, 0.0, 0.0)

        # This is the home pose. Really its a starting pose now but its inherited from previous versions where
        # it was required to make robot stand up straight for testing. Now it is a required part of constructor for the starting pose
        self.home_q = [0, pi/2, -pi/2, 0, 0, 0]     #Vertical pose by default

        # This is just a neater way of expressing the previous code where it would each had 4 lines for each mesh its doing the same function
        # it finds the mesh folder for the robot, iterates through each stl link file name in this case wlink for Will's link, and adds to a list that
        # uses a for loop to build the meshes. As discussed in the comments at the top it means you cant give individual links different colour but it 
        # is far neater. The .meshes list is required by the constuctor to know the mesh for each link and apply offsets in the movement function/file

        mesh_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Kawasaki_Robot_Files")
        file_names = ["wBase.stl", "wLink1.stl", "wLink2.stl", "wLink3.stl", "wLink4.stl", "wLink5.stl", "wLink6.stl"]
        self.meshes = []                                 # empty list to fill
        for name in file_names:                          # go through each filename
            mesh_path = os.path.join(mesh_folder, name)  # full path to the file
            mesh = Mesh(filename=mesh_path, color="#5B5B5B8D", scale=[0.001, 0.001, 0.001])  # Swift Assumes unit in metres when its actually meant to be mm hence the conversion
            self.meshes.append(mesh)                     # add it to the list

        # These are the Mesh Offsets obtained by matching with cylinder bot and trial and error. For me (Will) i think it was required because I
        # Dowloaded stp files then used soldiworks to export to stls all at once. This unintentionally kept its locations and meshes 
        # didnt spawn at origin, they spawned standing perfectly connected at positive 0.15, 0.2675, 0. This is why its translating each mesh by that amount
        # I also rotated 90 deg in all parts to keep robot facing x axis consistent with group members.
        
        # It stores in this array, replacing the old link check lines that had to be applied for each step in the movement code to the linkCheck for each mesh.
        # Doing it this way is a neater of updating the mesh for each step of the movement. This array of offsets is taken and then used in movements code
        # Structuring it this way allows us to have different offset for each robot when using the movement commands, so each robot is required to have and offsets list in its constructor
        self.offsets = [
            trotz(-pi/2) @ transl(-0.15, -0.2675, 0),                                             # Base
            trotx(-pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -0.36),                          # link 1
            trotx(-pi/2) @ troty(pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -0.715),           # link 2
            trotz(-pi/2) @ transl(-0.15, -0.2675, -0.715),                                        # link 3
            trotx(-pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -1.090),                         # link 4
            trotz(-pi/2) @ transl(-0.15, -0.2675, -1.090),                                        # link 5
            trotz(-pi/2) @ transl(-0.15, -0.2675, -1.168),                                        # link 6
        ]