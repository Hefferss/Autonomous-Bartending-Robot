import os
from math import pi
import numpy as np
import swift
from roboticstoolbox import DHLink, DHRobot, jtraj
from spatialmath import SE3
from spatialmath.base import trotx, trotz, troty, transl
from spatialgeometry import Mesh
from ir_support import CylindricalDHRobotPlot

# 1. Robot Setup
links = [
    DHLink(d=0.36, a=0, alpha=pi/2, qlim=[-pi, pi]),
    DHLink(d=0, a=0.355, alpha=0, qlim=[-135*pi/180, 135*pi/180]),
    DHLink(d=0, a=0, alpha=-pi/2, qlim=[-155*pi/180, 155*pi/180]),
    DHLink(d=0.375, a=0, alpha=pi/2, qlim=[-200*pi/180, 200*pi/180]),
    DHLink(d=0, a=0, alpha=-pi/2, qlim=[-125*pi/180, 125*pi/180]),
    DHLink(d=0.078, a=0, alpha=0, qlim=[-2*pi, 2*pi])
]
robot = DHRobot(links, name='Kawasaki RS007N')
robot = CylindricalDHRobotPlot(robot, cylinder_radius=0.025, color="#7b1d1d").create_cylinders()

# 2. Environment Setup
env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
robot.q = np.zeros(6)
env.step()

# 3. Load and Add Meshes via Loop
mesh_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Kawasaki_Robot_Files")
mesh_files = ["wBase.stl"] + [f"wLink{i}.stl" for i in range(1, 7)]
meshes = [Mesh(filename=os.path.join(mesh_dir, f), color="#D9C0FF", scale=[0.001]*3) for f in mesh_files]

for m in meshes:
    env.add(m)

# 4. Helper for Link Transformations
def get_mesh_transforms(link_check):
    return [
        link_check[0].A @ trotz(-pi/2) @ transl(-0.15, -0.2675, 0),
        link_check[1].A @ trotx(-pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -0.36),
        link_check[2].A @ trotx(-pi/2) @ troty(pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -0.715),
        link_check[3].A @ trotz(-pi/2) @ transl(-0.15, -0.2675, -0.715),
        link_check[4].A @ trotx(-pi/2) @ transl(-0.15, -0.2675, -1.090),
        link_check[5].A @ trotz(-pi/2) @ transl(-0.15, -0.2675, -1.090),
        link_check[6].A @ trotz(-pi/2) @ trotz(-pi/2) @ transl(-0.15, -0.2675, -1.168)
    ]

# 5. IK and Trajectory Execution
ik_sol = robot.ikine_LM(SE3(0.3, 0.3, 0.1))
if not ik_sol.success:
    raise ValueError("Inverse kinematics failed to find a valid solution!")

for q_step in jtraj(robot.q, ik_sol.q, 50).q:
    robot.q = q_step
    for mesh, T in zip(meshes, get_mesh_transforms(robot.fkine_all(q_step))):
        mesh.T = T
    env.step()

input("Enter to continue\n")