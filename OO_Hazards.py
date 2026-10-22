# OO_Hazards.py: the man's hand and the light curtain
#   ManActor      a person whose hand walks through the scene. only the light curtain reacts to him
#   LightCurtain  two posts and a see-through plane (green = clear, red = broken). breaking it triggers the e-stop
# both have poll(): add them with workcell.add_input(...), the man first so he has moved before the curtain looks

import numpy as np
from math import pi
from spatialgeometry import Mesh, Cuboid
from spatialmath import SE3
from ir_support_extra_parts.parts import part_path

PARK = SE3(1000.0, 1000.0, -1000.0) # hidden things wait here (removing objects crashes older swift, so they are only moved)


class ManActor:
    SHOULDER_FULL = np.array([0.15, -0.22, 1.33]) # measured on personMaleCasual.dae at full size (1.87 m): +x is the way he faces, feet on z = 0
    HAND_FULL = np.array([0.593, -0.189, 1.175])

    # scale: every table here is 0.25 m, so 0.6 (1.12 m) suits the scene. speed: metres the hand walks per step (keep it under 2 x the curtain's thickness)
    def __init__(self, env, scale=0.6, speed=0.03):
        self.speed = speed
        self.SHOULDER = self.SHOULDER_FULL * scale
        self.HAND = self.HAND_FULL * scale
        self.mesh = Mesh(filename=str(part_path("personMaleCasual")), scale=[scale, scale, scale])
        env.add(self.mesh)
        self.pose = SE3()
        self.yaw = 0.0
        self.hide()

    def hand_position(self):
        return self.pose.R @ self.HAND + self.pose.t

    # the points the light curtain can see: a line from his shoulder to his hand. empty when hidden
    def points(self):
        if not self.active:
            return np.empty((0, 3))
        shoulder = self.pose.R @ self.SHOULDER + self.pose.t
        return shoulder + np.outer(np.linspace(0, 1, 10), self.hand_position() - shoulder)

    # stand on the floor so his hand is at (x, y), facing yaw (0 = +x, pi = -x)
    def stand(self, x, y, yaw):
        body = np.array([x, y]) - (SE3.Rz(yaw).R @ self.HAND)[:2]
        self.pose = SE3(float(body[0]), float(body[1]), 0.0) * SE3.Rz(yaw)
        self.yaw = yaw
        self.mesh.T = self.pose
        self.active = True

    # start with the hand at start_xyz and walk it in a straight line to end_xyz (only x and y are used, his hand height is fixed by his size). poll() does the walking
    def walk_hand(self, start_xyz, end_xyz, yaw):
        self.stand(start_xyz[0], start_xyz[1], yaw)
        self.goal = np.array(end_xyz[:2], dtype=float)

    def stop(self): # stop walking, stay where he is
        self.goal = None

    def hide(self):
        self.goal = None
        self.active = False
        self.mesh.T = PARK

    # called by Workcell.step() every animation step
    def poll(self):
        if self.goal is None or not self.active:
            return
        hand = self.hand_position()[:2]
        to_go = self.goal - hand
        distance = float(np.linalg.norm(to_go))
        if distance <= self.speed: # arrived
            self.stand(self.goal[0], self.goal[1], self.yaw)
            self.goal = None
        else:
            step = hand + to_go / distance * self.speed
            self.stand(step[0], step[1], self.yaw)


class LightCurtain:
    POST_HEIGHT = 0.816 # SafetyLightCurtain.dae is 0.816 m tall

    # a flat curtain across the y direction at x: posts at (x, y_min) and (x, y_max), the plane between them from z = bottom to z = top
    # thickness: a point within this distance of the plane (in x) counts as breaking it
    def __init__(self, workcell, env, x, y_min, y_max, bottom=0.30, top=0.95, thickness=0.04):
        self.workcell = workcell
        self.x, self.y_min, self.y_max = x, y_min, y_max
        self.bottom, self.top, self.thickness = bottom, top, thickness
        self.sources = [] # things that can break it (each has points() -> n x 3)
        self.was_broken = False
        self.on_trip = None # what to run the moment it breaks (main sets this to the GUI e-stop)

        for y in (y_min, y_max): # the posts, stretched up so they stand just above the plane
            post = Mesh(filename=str(part_path("SafetyLightCurtain")), scale=[1.0, 1.0, (top + 0.05) / self.POST_HEIGHT])
            post.T = SE3(x, y, 0.0) * SE3.Rz(pi / 2)
            env.add(post)

        # the plane: a green box and a red box, only one is in place at a time (the other waits far away)
        size = [0.01, y_max - y_min, top - bottom]
        self.place = SE3(x, (y_min + y_max) / 2, (bottom + top) / 2)
        self.green = Cuboid(scale=size, color=[0.0, 1.0, 0.0, 0.3]) # float colours, ints break swift on windows
        self.red = Cuboid(scale=size, color=[1.0, 0.0, 0.0, 0.3])
        env.add(self.green)
        env.add(self.red)
        self.show(False)

    def add_source(self, source):
        self.sources.append(source)

    # true if any point of any source is inside the curtain right now
    def broken(self):
        for source in self.sources:
            for px, py, pz in source.points():
                if abs(px - self.x) <= self.thickness and self.y_min <= py <= self.y_max and self.bottom <= pz <= self.top:
                    return True
        return False

    def show(self, broken): # green plane when clear, red when broken
        self.green.T = PARK if broken else self.place
        self.red.T = self.place if broken else PARK

    # called by Workcell.step() every animation step. only the moment it BREAKS triggers the e-stop
    def poll(self):
        now = self.broken()
        if now != self.was_broken:
            self.show(now)
        if now and not self.was_broken:
            print("LIGHT CURTAIN BROKEN - emergency stop")
            if self.on_trip is not None:
                self.on_trip()
            else:
                self.workcell.emergency_stop()
            self.workcell.fault = "LIGHT CURTAIN BROKEN"
        self.was_broken = now

    # for Workcell.add_interlock: while the curtain is still broken RESET is refused (a message), otherwise None
    def interlock(self):
        return "light curtain is still broken - clear the beam first" if self.broken() else None
