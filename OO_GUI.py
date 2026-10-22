# OO_GUI.py: swift GUI (scene buttons, safety buttons, robot/camera dropdowns, joint sliders, jog buttons) and the state beacon
# swift only reads GUI clicks during env.step(), so the program ends with gui.run()

import swift
import numpy as np
from spatialgeometry import Sphere
from spatialmath.base import transl
from OO_Camera import CAMERA_ANGLES


class RobotGUI:

    # scenes: button label -> function to run (see OO_MainV2.py). jog_step in metres
    def __init__(self, workcell, jog_step=0.02, beacon_position=(-1.5, 2.3, 1.2), camera_angles=None, scenes=None, live_buttons=None, hazard_scenes=None):
        self.workcell = workcell
        self.scenes = scenes or {}
        self.hazard_scenes = hazard_scenes or [] # labels from scenes that are shown under HAZARDS instead of OPERATIONS (still queued like a scene, so the robots can move)
        self.live_buttons = live_buttons or {} # label -> function that runs straight away when clicked (not queued), so it works while a scene is running. must not call env.step
        self.jog_step = jog_step
        self.beacon_position = beacon_position
        self.names = list(workcell.movers.keys()) # robot names in dropdown order
        self.selected = self.names[0]
        self.pending = None # the next queued action, run in run()
        self.beacons = {}
        self.camera_angles = camera_angles or CAMERA_ANGLES
        self.camera_names = list(self.camera_angles.keys())

    def current(self):
        return self.workcell.movers[self.selected]

    def build(self):
        env = self.workcell.env

        # state beacons: one sphere per state, the others parked far away (removing objects crashes older swift). float colours, ints break swift on windows
        self.beacons["RUNNING"] = Sphere(radius=0.06, color=[0.0, 1.0, 0.0, 1.0])    # green
        self.beacons["ESTOPPED"] = Sphere(radius=0.06, color=[1.0, 0.0, 0.0, 1.0])   # red
        self.beacons["RESET"] = Sphere(radius=0.06, color=[1.0, 0.7, 0.0, 1.0])      # amber
        for beacon in self.beacons.values():
            env.add(beacon)
        self.show_state()

        # scene buttons: queued here, played in run()
        env.add(swift.Label("--- OPERATIONS ---"))
        for label in self.scenes:
            if label not in self.hazard_scenes:
                env.add(swift.Button(lambda *args, label=label: self.queue("scene", label), label))

        # live buttons: run inside the click itself, so they can happen while a scene is running (the man walking in)
        env.add(swift.Label("--- HAZARDS ---"))
        for label in self.hazard_scenes:
            env.add(swift.Button(lambda *args, label=label: self.queue("scene", label), label))
        for label, function in self.live_buttons.items():
            env.add(swift.Button(lambda *args, function=function: function(), label))

        # safety buttons (all robots)
        env.add(swift.Label("--- SAFETY ---"))
        env.add(swift.Button(self.on_estop, "EMERGENCY STOP"))
        env.add(swift.Button(self.on_reset, "1. RESET E-STOP"))
        env.add(swift.Button(self.on_restart, "2. RESTART"))

        # robot and camera dropdowns
        env.add(swift.Label("- SELECTED ROBOT -"))
        env.add(swift.Select(self.on_select_robot, desc="Control robot:",
                             options=self.names, value=0))
        env.add(swift.Label("--- CAMERA ---"))
        env.add(swift.Select(self.on_select_camera, desc="View:",
                             options=self.camera_names, value=0))
        self.set_camera(self.camera_names[0])

        # joint sliders: as many as the robot with the most joints, starting at the first robot's home angles
        env.add(swift.Label("-- JOINT CONTROL --"))
        most_joints = max(m.robot.n for m in self.workcell.movers.values())
        first_home = self.current().home_q
        for j in range(most_joints):
            if j < len(first_home):
                start = round(float(np.degrees(first_home[j])))
            else:
                start = 0
            # j=j locks in each slider's joint number
            slider = swift.Slider(lambda value, j=j: self.on_joint_slider(j, value),
                                  min=-180, max=180, step=1, value=start,
                                  desc="Joint " + str(j + 1), unit="deg")
            env.add(slider)

        # end effector jog buttons: (name, (dx, dy, dz))
        env.add(swift.Label("- END EFFECTOR JOG -"))
        s = self.jog_step
        jog_buttons = [("+X", (s, 0, 0)), ("-X", (-s, 0, 0)),
                       ("+Y", (0, s, 0)), ("-Y", (0, -s, 0)),
                       ("+Z", (0, 0, s)), ("-Z", (0, 0, -s))]
        for name, direction in jog_buttons:
            env.add(swift.Button(lambda *args, d=direction: self.queue("jog", d), "Jog " + name))

    # current state's sphere to beacon_position, the others parked 1000 m away. no env.step() here (unsafe inside a callback)
    def show_state(self):
        for state, beacon in self.beacons.items():
            if state == self.workcell.state:
                x, y, z = self.beacon_position
            else:
                x, y, z = 1000, 1000, -1000
            beacon.T = transl(x, y, z)

    def on_estop(self, *args):
        self.pending = None # cancel anything waiting to start
        self.workcell.emergency_stop()
        self.show_state()

    def on_reset(self, *args):
        self.workcell.reset() # ESTOPPED -> RESET (still blocked)
        self.show_state()

    def on_restart(self, *args):
        self.workcell.restart() # RESET -> RUNNING
        self.show_state()

    def on_select_robot(self, index):
        self.selected = self.names[int(index)]
        print("Selected robot:", self.selected)

    def on_select_camera(self, index):
        self.set_camera(self.camera_names[int(index)])

    def set_camera(self, name):
        position, look_at = self.camera_angles[name]
        # plain python floats, numpy types break swift on windows
        self.workcell.env.set_camera_pose([float(v) for v in position],
                                          [float(v) for v in look_at])
        print("Camera view:", name)

    def on_joint_slider(self, j, value):
        if self.workcell.state != "RUNNING":
            return
        mover = self.current()
        robot = mover.robot
        if j >= robot.n: # this robot has no joint j
            return
        q = np.array(robot.q)
        angle = np.radians(float(value))
        angle = np.clip(angle, robot.qlim[0, j], robot.qlim[1, j]) # keep inside the joint limits
        q[j] = angle
        if mover.check_collision(q): # same table + robot check as move_to_q, so the slider can't push into the table
            print("UNSAFE: slider move blocked, collision")
            return
        robot.q = q
        mover.update_meshes(q)

    def queue(self, action, data=None):
        if self.workcell.state != "RUNNING":
            print("Ignored - workcell is not in RUNNING state")
            return
        self.pending = (action, data, self.selected) # remember which robot was selected when it was clicked

    # loops forever: runs any queued action, then env.step() so swift sees the clicks
    def run(self):
        while True:
            if self.pending is not None:
                action, data, name = self.pending
                self.pending = None
                mover = self.workcell.movers[name]
                if action == "jog":
                    mover.jog(data[0], data[1], data[2])
                elif action == "scene":
                    self.scenes[data]() # e-stop still works mid-scene
            self.workcell.step(0.05)