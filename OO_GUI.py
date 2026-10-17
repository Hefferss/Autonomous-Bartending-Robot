# The GUI file handles the GUI, including sliders buttons and drop down menus.
# It is also responsible for the Safety Beacon (Coloured Sphere)
# Swift only takes GUI inputs if env.step() is called
# The program must finish by calling gui.run() to do so

import swift                                  # gives us Button, Slider, Select and Label
import numpy as np                            # for degree/radian conversion and arrays
from spatialgeometry import Sphere            # the coloured state beacons
from spatialmath.base import transl           # builds a 4x4 position matrix
from OO_Camera import CAMERA_ANGLES     # the list of camera views (edit that file to change them)


# --------------------------- THE GUI CLASS ----------------------------
class RobotGUI:

    # ---- Constructor ----------------------------------------------------
    # Overview: stores the workcell, a dictionary of demo poses (one per
    # robot name), the jog size, and which robot is selected to start with.
    # SCENES (NEW): BUTTON LABEL -> FUNCTION, ONE BUTTON EACH (SEE OO_MainV2.py)
    def __init__(self, workcell, demo_targets=None, jog_step=0.02,
                 beacon_position=(0, 0, 1.5), camera_angles=None, scenes=None):
        self.workcell = workcell                          # the shared scene
        self.demo_targets = demo_targets or {}            # robot name -> demo pose (may be empty)
        self.scenes = scenes or {}                        # BUTTON LABEL -> FUNCTION TO RUN (MAY BE EMPTY)
        self.jog_step = jog_step                          # how far one jog click moves (metres)
        self.beacon_position = beacon_position            # where the active beacon is shown
        self.names = list(workcell.movers.keys())         # robot names in dropdown order
        self.selected = self.names[0]                     # start with the first robot selected
        self.pending = None                               # the next action waiting to run
        self.beacons = {}                                 # will hold one sphere per state
        self.camera_angles = camera_angles or CAMERA_ANGLES   # name -> (position, look_at)
        self.camera_names = list(self.camera_angles.keys())   # view names in dropdown order

    # ---- Current mover: the RobotMover of the selected robot ------------
    def current(self):
        return self.workcell.movers[self.selected]        # look it up by name

    # ---- Build: adds every control to the Swift window -----------------
    def build(self):
        env = self.workcell.env                           # shortcut to the Swift window

        # -- Block 1: state beacons --
        # One sphere per state, all added once. show_state() moves the sphere for
        # the current state into view and parks the others very far away, where
        # they are too small to see. (We never REMOVE spheres: removing objects
        # from Swift crashes older versions on the next env.step().)
        # Colours are floats (0.0, 1.0) because whole-number lists break Swift on Windows.
        self.beacons["RUNNING"] = Sphere(radius=0.06, color=[0.0, 1.0, 0.0, 1.0])    # green
        self.beacons["ESTOPPED"] = Sphere(radius=0.06, color=[1.0, 0.0, 0.0, 1.0])   # red
        self.beacons["RESET"] = Sphere(radius=0.06, color=[1.0, 0.7, 0.0, 1.0])      # amber
        for beacon in self.beacons.values():              # go through each sphere
            env.add(beacon)                               # add it to the simulator
        self.show_state()                                 # put the right one in view

        # -- Block 2: operations (preset movements for the selected robot) --
        env.add(swift.Label("--- OPERATIONS ---"))

        # SCENE BUTTONS: QUEUED HERE, PLAYED IN RUN() ("label=label" LOCKS IN THE LABEL LIKE THE JOG BUTTONS)
        for label in self.scenes:
            env.add(swift.Button(lambda *args, label=label: self.queue("scene", label), label))

        # -- Block 3: safety buttons (these affect ALL robots) --
        env.add(swift.Label("--- SAFETY ---"))
        env.add(swift.Button(self.on_estop, "EMERGENCY STOP"))
        env.add(swift.Button(self.on_reset, "1. RESET E-STOP"))
        env.add(swift.Button(self.on_restart, "2. RESTART"))

        # -- Block 4: robot selector (dropdown) --
        # Swift calls on_select_robot with the INDEX of the chosen option (0, 1, 2...).
        env.add(swift.Label("- SELECTED ROBOT -"))
        env.add(swift.Select(self.on_select_robot, desc="Control robot:",
                             options=self.names, value=0))

        # -- Block 5: camera view (dropdown) --
        # The views come from OO_CameraAngles.py. The first one is applied now.
        env.add(swift.Label("--- CAMERA ---"))
        env.add(swift.Select(self.on_select_camera, desc="View:",
                             options=self.camera_names, value=0))
        self.set_camera(self.camera_names[0])             # start in the first view

        # -- Block 6: joint sliders --
        # Make as many sliders as the robot with the MOST joints. Sliders
        # beyond the selected robot's joint count are simply ignored.
        env.add(swift.Label("-- JOINT CONTROL --"))
        most_joints = max(m.robot.n for m in self.workcell.movers.values())
        first_home = self.current().home_q                # start values from the first robot
        for j in range(most_joints):                      # one slider per joint
            if j < len(first_home):                       # does the first robot have this joint?
                start = round(float(np.degrees(first_home[j])))   # yes: use its home angle
            else:
                start = 0                                 # no: start at zero
            # "j=j" locks in the current j so each slider keeps its own joint number
            slider = swift.Slider(lambda value, j=j: self.on_joint_slider(j, value),
                                  min=-180, max=180, step=1, value=start,
                                  desc="Joint " + str(j + 1), unit="deg")
            env.add(slider)

        # -- Block 7: jog buttons --
        env.add(swift.Label("- END EFFECTOR JOG -"))
        s = self.jog_step                                 # short name for the step size
        jog_buttons = [("+X", (s, 0, 0)), ("-X", (-s, 0, 0)),     # name and (dx, dy, dz)
                       ("+Y", (0, s, 0)), ("-Y", (0, -s, 0)),
                       ("+Z", (0, 0, s)), ("-Z", (0, 0, -s))]
        for name, direction in jog_buttons:               # one button per entry
            # "d=direction" locks in this button's direction
            env.add(swift.Button(lambda *args, d=direction: self.queue("jog", d), "Jog " + name))

    # ---- Show state: moves the matching beacon into view -----------------
    # Overview: the sphere for the current state goes to beacon_position; the
    # other two are parked 1000 m away, far too small to see. There is no
    # env.step() here: the next normal step redraws the change, and calling
    # step inside a button callback can cause trouble.
    def show_state(self):
        for state, beacon in self.beacons.items():        # go through each beacon
            if state == self.workcell.state:              # is this the current state?
                x, y, z = self.beacon_position            # yes: bring it into view
            else:
                x, y, z = 1000, 1000, -1000               # no: park it far away
            beacon.T = transl(x, y, z)                    # move the sphere there

    # ---- Callbacks: run when the user clicks something -----------------
    def on_estop(self, *args):
        self.pending = None                   # cancel anything waiting to start
        self.workcell.emergency_stop()        # block ALL robots (aborts any move in progress)
        self.show_state()                     # beacon turns red

    def on_reset(self, *args):
        self.workcell.reset()                 # ESTOPPED -> RESET (still blocked)
        self.show_state()                     # beacon turns amber

    def on_restart(self, *args):
        self.workcell.restart()               # RESET -> RUNNING
        self.show_state()                     # beacon turns green

    def on_select_robot(self, index):
        self.selected = self.names[int(index)]            # the dropdown gives an index; look up the name
        print("Selected robot:", self.selected)           # tell the terminal

    def on_select_camera(self, index):
        self.set_camera(self.camera_names[int(index)])    # the dropdown gives an index; look up the name

    def set_camera(self, name):
        position, look_at = self.camera_angles[name]      # the two (x, y, z) points for this view
        # plain Python floats, because numpy integer types break Swift on Windows
        self.workcell.env.set_camera_pose([float(v) for v in position],
                                          [float(v) for v in look_at])
        print("Camera view:", name)                       # tell the terminal

    def on_joint_slider(self, j, value):
        if self.workcell.state != "RUNNING":              # ignore the slider unless running
            return
        mover = self.current()                            # the selected robot
        robot = mover.robot                               # its DHRobot
        if j >= robot.n:                                  # this robot has no joint j
            return
        q = np.array(robot.q)                             # copy the current joint angles
        angle = np.radians(float(value))                  # slider is in degrees -> radians
        angle = np.clip(angle, robot.qlim[0, j], robot.qlim[1, j])  # keep inside this robot's joint limits
        q[j] = angle                                      # change joint j
        if mover.check_collision(q): # same table + robot check as move_to_q, so the slider can't push into the table
            print("UNSAFE: slider move blocked, collision")
            return
        robot.q = q                                       # set the robot to the new angles
        mover.update_meshes(q)                            # move the meshes to match

    def queue(self, action, data=None):
        if self.workcell.state != "RUNNING":              # don't accept movements unless running
            print("Ignored - workcell is not in RUNNING state")
            return
        self.pending = (action, data, self.selected)      # remember the action AND which robot

    # ---- Run: keeps the window alive and does queued actions ------------
    # Overview: loops forever. Each time round it runs any queued action,
    # then calls env.step() so Swift can see the GUI clicks. Ctrl+C quits.
    def run(self):
        while True:                                       # loop until the program is closed
            if self.pending is not None:                  # is there a queued action?
                action, data, name = self.pending         # take it out of the queue
                self.pending = None                       # clear the queue
                mover = self.workcell.movers[name]        # the robot chosen when it was clicked
                if action == "jog":                                   # jog button
                    mover.jog(data[0], data[1], data[2])              # nudge in x, y, z
                elif action == "home":                                # go home button
                    mover.move_to_q(mover.home_q)                     # move to the home pose
                elif action == "demo" and name in self.demo_targets:  # demo button
                    mover.move_to_pose(self.demo_targets[name])       # move to its demo target
                elif action == "scene":                               # SCENE BUTTON
                    self.scenes[data]()                               # PLAY THE SCENE (E-STOP STILL WORKS)
            self.workcell.step(0.05)                      # check inputs, let Swift update and read the GUI