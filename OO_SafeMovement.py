import numpy as np
from roboticstoolbox import jtraj #smooth joint path, same as move_to_q uses
from spatialgeometry import Sphere #the visible obstacle
from spatialmath import SE3
from spatialmath.base import transl
from OO_Collision import CollisionChecker, sphere_points #the ellipsoid checker and a point cloud for a sphere

class ObstacleWorld: #the list of things the robots must not hit (and optionally the other robots)

    def __init__(self): #starts with no obstacles and no robots registered
        self.items = {} #name -> n x 3 array of obstacle points
        self.robots = {} #name -> (RobotMover, CollisionChecker), for robot against robot checks

    def add_obstacle(self, name, points): #add (or replace) a named obstacle. points: an n x 3 array, e.g. from sphere_points
        self.items[name] = np.asarray(points, dtype=float).reshape(-1, 3)

    def remove_obstacle(self, name): #remove a named obstacle (nothing happens if it isn't there)
        self.items.pop(name, None)

    def register_robot(self, name, mover, checker): #remember a robot so other robots can avoid it. SafeMover does this for you
        self.robots[name] = (mover, checker)

    def points_for(self, mover, include_robots=False): #every point this robot should avoid right now. include_robots also adds the other robots' links
        parts = list(self.items.values()) #the placed obstacles
        if include_robots:
            for other, checker in self.robots.values():
                if other is not mover: #never avoid yourself
                    parts.append(checker.link_points(other.robot.q))
        return np.vstack(parts) if parts else np.empty((0, 3))

class PlacedObstacle: #an obstacle you can drop into the scene on demand: a red sphere you can see, plus the points the checker uses
    PARK = (1000.0, 1000.0, -1000.0) #where it waits when hidden (too far away to see)

    def __init__(self, world, env, name="obstacle", radius=0.05, color=None): #makes the red sphere and parks it. env is workcell.env, so create this after launch()
        self.world = world
        self.name = name
        self.radius = radius #size in metres
        self.placed = False #is it in the scene right now?
        self.mesh = Sphere(radius=radius, color=color or [1.0, 0.0, 0.0, 1.0]) #colours are floats because whole number lists break swift on windows
        self.mesh.T = transl(*self.PARK)
        env.add(self.mesh) #added once and never removed: removing crashes older swift

    def place(self, xyz): #drop the obstacle at (x, y, z). it appears in swift and the checkers start avoiding it
        xyz = [float(v) for v in xyz]
        self.mesh.T = transl(*xyz)
        self.world.add_obstacle(self.name, sphere_points(xyz, self.radius))
        self.placed = True

    def hide(self): #take it away again
        self.mesh.T = transl(*self.PARK)
        self.world.remove_obstacle(self.name)
        self.placed = False


class SafeMover: #wraps one robot's RobotMover. moves like move_to_q / move_to_pose, but checks for collisions first, again every step, and can re-plan round a blocked path

    def __init__(self, mover, world, fatness=0.1, check_robots=False, name=None): #check_robots: also avoid the other robots (off by default, handoffs are close on purpose)
        self.mover = mover
        self.world = world
        self.checker = CollisionChecker(mover.robot, fatness) #fatness = how thick the link ellipsoids are
        self.check_robots = check_robots
        self.last_result = None #"reached", "blocked ahead", "hit mid-move", "e-stop" or "no clear path"
        world.register_robot(name or ("robot %d" % len(world.robots)), mover, self.checker)

    def obstacle_points(self): #the points this robot must avoid right now (recalculated each time, so new obstacles are seen)
        return self.world.points_for(self.mover, self.check_robots)

    def first_hit_in_path(self, q_path): #look ahead: index of the first step of q_path that would hit something, or None if it's clear
        points = self.obstacle_points()
        if len(points) == 0: #nothing to hit
            return None
        return self.checker.first_collision_in_path(q_path, points)

    def move_to_q_safe(self, q_goal, steps=50, look_ahead=True): #like move_to_q, but doesn't start if the path would hit, and re-checks obstacles before every step. True if it reached q_goal
        mover = self.mover
        workcell = mover.workcell
        path = jtraj(mover.robot.q, q_goal, steps).q #the planned path
        if look_ahead: #if the planned path would hit, don't start
            hit = self.first_hit_in_path(path)
            if hit is not None:
                print("COLLISION AHEAD at step %d of %d - not moving" % (hit, steps))
                self.last_result = "blocked ahead"
                return False
        for q_step in path:
            if workcell.state != "RUNNING": #same e-stop check as move_to_q
                print("Motion aborted")
                self.last_result = "e-stop"
                return False
            points = self.obstacle_points() #obstacles as they are right now
            if len(points) and self.checker.collides(q_step, points): #stop before the robot moves into one
                print("COLLISION - stopped before the robot hit it")
                self.last_result = "hit mid-move"
                return False
            mover.robot.q = q_step #set the robot to this step
            mover.update_meshes(q_step) #move the meshes
            workcell.step() #check inputs, update swift
        self.last_result = "reached"
        return True

    def spot_in_path(self, q_goal, steps=50): #a spot the robot passes through on the way to q_goal but is clear at both ends, for dropping an obstacle in its path. None if none found
        q_start = np.array(self.mover.robot.q)
        path = jtraj(q_start, q_goal, steps).q
        middle = path[len(path) // 2]
        for e in reversed(self.checker.elipsoids_for_robot_around_links(middle)): #last links move the most
            for along in np.linspace(-0.8, 0.8, 9): #slide along the link
                spot = e['center'] + along * e['radii'][0] * e['rotation'][:, 0]
                probe = sphere_points(spot, 0.05) #about the size of a PlacedObstacle
                if (self.checker.collides(middle, probe) and not self.checker.collides(q_start, probe)
                        and not self.checker.collides(q_goal, probe)):
                    return spot
        return None

    def move_to_pose_avoiding(self, T_target, steps=50, lifts=(0.15, 0.3, 0.45), sides=(0.0, 0.2, -0.2)): #active avoidance: direct path if clear, else the first via-point whose two legs are both clear, else don't move
        mover = self.mover
        q_start = np.array(mover.robot.q)
        q_goal = mover.solve_ik(T_target) #the same consistent IK as everything else (raises ValueError if out of reach)
        self.last_via = None #which via-point was used
        if self.first_hit_in_path(jtraj(q_start, q_goal, steps).q) is None:
            return self.move_to_q_safe(q_goal, steps, look_ahead=False) #clear: go (still checked every step)

        print("COLLISION AHEAD - looking for a way round")
        T_start = mover.robot.fkine(q_start)
        for lift in lifts: #try going higher and higher
            for side in sides: #and left / right
                middle = (T_start.t + T_target.t) / 2 + np.array([0.0, side, lift]) #via-point halfway between start and goal, raised and shifted
                try:
                    q_via = mover.solve_ik(SE3.Rt(T_target.R, middle)) #arm pose at the via-point
                except ValueError: #via-point out of reach
                    continue
                leg1 = jtraj(q_start, q_via, steps).q
                leg2 = jtraj(q_via, q_goal, steps).q
                if self.first_hit_in_path(leg1) is None and self.first_hit_in_path(leg2) is None: #take the first one whose two legs are both clear
                    print("Going round: via-point %.2f m up, %.2f m sideways" % (lift, side))
                    self.last_via = (lift, side)
                    if not self.move_to_q_safe(q_via, steps, look_ahead=False):
                        return False
                    return self.move_to_q_safe(q_goal, steps, look_ahead=False)
        print("No clear path found - not moving") #nothing clear, don't move
        self.last_result = "no clear path"
        return False