# ======================================================================
# OO_SafeMovement.py  --  collision-aware movement for ANY robot (including the UR3)
#
# EVERY COMMENT IN THIS FILE IS NEW (ADDED BY CLAUDE), SO THEY ARE ALL IN CAPITALS TO MATCH THE CONVENTION.
#
# THIS FILE SITS ON TOP OF OO_Collision.py AND OO_MovementCommands.py WITHOUT CHANGING EITHER, SO IT CANNOT CLASH WITH
# ANYONE ELSE'S EDITS TO THOSE FILES. IT HAS THREE CLASSES:
#   ObstacleWorld    - THE LIST OF THINGS THE ROBOTS MUST NOT HIT (AND OPTIONALLY THE OTHER ROBOTS)
#   PlacedObstacle   - AN OBSTACLE YOU CAN DROP INTO THE SCENE ON DEMAND: A RED SPHERE YOU CAN SEE, PLUS THE POINTS THE CHECKER USES
#   SafeMover        - WRAPS ONE ROBOT'S RobotMover. MOVES LIKE move_to_q / move_to_pose, BUT CHECKS FOR COLLISIONS FIRST
#                      (LOOK-AHEAD), CHECKS AGAIN BEFORE EVERY STEP (IN CASE SOMETHING IS PLACED MID-MOVE), AND CAN
#                      RE-PLAN AROUND A BLOCKED PATH
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
import numpy as np                                       # ARRAYS AND MATHS
from math import pi                                      # THE CONSTANT PI
from roboticstoolbox import jtraj                        # SMOOTH JOINT PATH, SAME AS move_to_q USES
from spatialgeometry import Sphere                       # THE VISIBLE OBSTACLE
from spatialmath import SE3                              # POSES
from spatialmath.base import transl                      # POSITION MATRIX
from OO_Collision import CollisionChecker, sphere_points # THE ELLIPSOID CHECKER AND A POINT CLOUD FOR A SPHERE


# ---------------------------- THE OBSTACLE WORLD ------------------------
class ObstacleWorld:

    # CONSTRUCTOR: STARTS WITH NO OBSTACLES AND NO ROBOTS REGISTERED
    def __init__(self):
        self.items = {}                                  # NAME -> N x 3 ARRAY OF OBSTACLE POINTS
        self.robots = {}                                 # NAME -> (RobotMover, CollisionChecker), FOR ROBOT-AGAINST-ROBOT CHECKS

    # ADD (OR REPLACE) A NAMED OBSTACLE. points: AN N x 3 ARRAY, E.G. FROM box_points OR sphere_points
    def add_obstacle(self, name, points):
        self.items[name] = np.asarray(points, dtype=float).reshape(-1, 3)

    # REMOVE A NAMED OBSTACLE (NOTHING HAPPENS IF IT ISN'T THERE)
    def remove_obstacle(self, name):
        self.items.pop(name, None)

    # REMEMBER A ROBOT SO OTHER ROBOTS CAN AVOID IT. SafeMover DOES THIS FOR YOU.
    def register_robot(self, name, mover, checker):
        self.robots[name] = (mover, checker)

    # EVERY POINT THIS ROBOT SHOULD AVOID RIGHT NOW. include_robots=True ALSO ADDS POINTS ALONG THE OTHER ROBOTS' LINKS.
    def points_for(self, mover, include_robots=False):
        parts = list(self.items.values())                # THE PLACED OBSTACLES
        if include_robots:
            for other, checker in self.robots.values():
                if other is not mover:                   # NEVER AVOID YOURSELF
                    parts.append(checker.link_points(other.robot.q))
        return np.vstack(parts) if parts else np.empty((0, 3))


# ---------------------------- A DROPPABLE OBSTACLE ------------------------
class PlacedObstacle:
    PARK = (1000.0, 1000.0, -1000.0)                     # WHERE IT WAITS WHEN HIDDEN (TOO FAR AWAY TO SEE)

    # CONSTRUCTOR: MAKES THE RED SPHERE AND PARKS IT. env IS THE SWIFT WINDOW (workcell.env), SO CREATE THIS AFTER launch().
    #   world: THE ObstacleWorld     name: A NAME FOR IT IN THE WORLD     radius: SIZE IN METRES
    def __init__(self, world, env, name="obstacle", radius=0.05, color=None):
        self.world = world
        self.name = name
        self.radius = radius
        self.placed = False                              # IS IT IN THE SCENE RIGHT NOW?
        # COLOURS ARE FLOATS BECAUSE WHOLE-NUMBER LISTS BREAK SWIFT ON WINDOWS
        self.mesh = Sphere(radius=radius, color=color or [1.0, 0.0, 0.0, 1.0])
        self.mesh.T = transl(*self.PARK)
        env.add(self.mesh)                               # ADDED ONCE AND NEVER REMOVED: REMOVING CRASHES OLDER SWIFT

    # DROP THE OBSTACLE AT (x, y, z). IT APPEARS IN SWIFT AND THE CHECKERS START AVOIDING IT.
    def place(self, xyz):
        xyz = [float(v) for v in xyz]
        self.mesh.T = transl(*xyz)
        self.world.add_obstacle(self.name, sphere_points(xyz, self.radius))
        self.placed = True

    # TAKE IT AWAY AGAIN
    def hide(self):
        self.mesh.T = transl(*self.PARK)
        self.world.remove_obstacle(self.name)
        self.placed = False


# ---------------------------- THE SAFE MOVER ------------------------------
class SafeMover:

    # CONSTRUCTOR
    #   mover: THE ROBOT'S RobotMover (FROM workcell.movers[...])     world: THE ObstacleWorld
    #   fatness, length_margin, link_offsets: PASSED STRAIGHT TO THE CollisionChecker (SEE OO_Collision.py)
    #   check_robots: True ALSO AVOIDS THE OTHER REGISTERED ROBOTS. OFF BY DEFAULT BECAUSE ROBOTS HANDING OVER AN ITEM
    #                 ARE CLOSE TOGETHER ON PURPOSE
    #   name: THE ROBOT'S NAME, USED WHEN REGISTERING IT IN THE WORLD
    def __init__(self, mover, world, fatness=0.1, length_margin=1.0, link_offsets=None,
                 check_robots=False, name=None):
        self.mover = mover
        self.world = world
        self.checker = CollisionChecker(mover.robot, fatness, length_margin, link_offsets)
        self.check_robots = check_robots
        self.last_result = None                          # "reached", "blocked ahead", "hit mid-move", "e-stop" OR "no clear path"
        world.register_robot(name or ("robot %d" % len(world.robots)), mover, self.checker)

    # THE POINTS THIS ROBOT MUST AVOID RIGHT NOW (RECALCULATED EACH TIME, SO NEW OBSTACLES ARE SEEN)
    def obstacle_points(self):
        return self.world.points_for(self.mover, self.check_robots)

    # LOOK AHEAD: THE INDEX OF THE FIRST STEP OF q_path THAT WOULD HIT SOMETHING, OR None IF THE PATH IS CLEAR
    def first_hit_in_path(self, q_path):
        points = self.obstacle_points()
        if len(points) == 0:                             # NOTHING TO HIT
            return None
        return self.checker.first_collision_in_path(q_path, points)

    # MOVE TO JOINT ANGLES LIKE RobotMover.move_to_q, BUT SAFELY:
    #   1. LOOK AHEAD: IF THE PLANNED PATH WOULD HIT, DON'T START (look_ahead=False SKIPS THIS)
    #   2. BEFORE EVERY STEP: CHECK THE E-STOP, THEN CHECK THE NEXT POSE AGAINST THE OBSTACLES AS THEY ARE RIGHT NOW,
    #      AND STOP BEFORE THE ROBOT MOVES INTO ONE
    # RETURNS True IF IT REACHED q_goal. self.last_result SAYS WHAT HAPPENED IF IT DIDN'T.
    def move_to_q_safe(self, q_goal, steps=50, look_ahead=True):
        mover = self.mover
        workcell = mover.workcell
        path = jtraj(mover.robot.q, q_goal, steps).q                 # THE PLANNED PATH
        if look_ahead:
            hit = self.first_hit_in_path(path)
            if hit is not None:
                print("COLLISION AHEAD at step %d of %d - not moving" % (hit, steps))
                self.last_result = "blocked ahead"
                return False
        for q_step in path:
            if workcell.state != "RUNNING":                          # SAME E-STOP CHECK AS move_to_q
                print("Motion aborted")
                self.last_result = "e-stop"
                return False
            points = self.obstacle_points()                          # OBSTACLES AS THEY ARE RIGHT NOW
            if len(points) and self.checker.collides(q_step, points):
                print("COLLISION - stopped before the robot hit it")
                self.last_result = "hit mid-move"
                return False
            mover.robot.q = q_step                                   # SET THE ROBOT TO THIS STEP
            mover.update_meshes(q_step)                              # MOVE THE MESHES
            self.checker.update_display(q_step)                      # MOVE THE COLLISION ELLIPSOIDS (IF SHOWN)
            workcell.step()                                          # CHECK INPUTS, UPDATE SWIFT
        self.last_result = "reached"
        return True

    # A SPOT THE ROBOT WILL PASS THROUGH ON ITS WAY TO q_goal, BUT THAT IS CLEAR AT THE START AND THE END.
    # USED TO "DROP" AN OBSTACLE INTO A PLANNED PATH. RETURNS (x, y, z) OR None IF THE MOVE IS TOO SMALL TO FIND ONE.
    def spot_in_path(self, q_goal, steps=50):
        q_start = np.array(self.mover.robot.q)
        path = jtraj(q_start, q_goal, steps).q
        middle = path[len(path) // 2]
        for e in reversed(self.checker.elipsoids_for_robot_around_links(middle)):   # LAST LINKS MOVE THE MOST
            for along in np.linspace(-0.8, 0.8, 9):                  # SLIDE ALONG THE LINK
                spot = e['center'] + along * e['radii'][0] * e['rotation'][:, 0]
                probe = sphere_points(spot, 0.05)                    # ABOUT THE SIZE OF A PlacedObstacle
                if (self.checker.collides(middle, probe) and not self.checker.collides(q_start, probe)
                        and not self.checker.collides(q_goal, probe)):
                    return spot
        return None

    # MOVE TO A POSE, GOING AROUND ANY OBSTACLE IN THE WAY (ACTIVE AVOIDANCE):
    #   1. SOLVE IK FOR T_target (RAISES ValueError IF IT CAN'T BE REACHED, LIKE move_to_pose)
    #   2. IF THE DIRECT PATH IS CLEAR, TAKE IT
    #   3. IF NOT, TRY A VIA-POINT HALFWAY BETWEEN START AND GOAL, RAISED BY EACH LIFT (AND SHIFTED SIDEWAYS BY EACH SIDE
    #      VALUE), AND TAKE THE FIRST ONE WHOSE TWO LEGS ARE BOTH CLEAR
    #   4. IF NOTHING IS CLEAR, DON'T MOVE
    # RETURNS True IF IT REACHED THE POSE. self.last_result SAYS WHAT HAPPENED, self.last_via SAYS WHICH VIA-POINT WAS USED.
    def move_to_pose_avoiding(self, T_target, steps=50, lifts=(0.15, 0.3, 0.45), sides=(0.0, 0.2, -0.2)):
        mover = self.mover
        q_start = np.array(mover.robot.q)
        q_goal = mover.solve_ik(T_target)                            # THE SAME CONSISTENT IK AS EVERYTHING ELSE
        self.last_via = None
        if self.first_hit_in_path(jtraj(q_start, q_goal, steps).q) is None:
            return self.move_to_q_safe(q_goal, steps, look_ahead=False)   # CLEAR: GO (STILL CHECKED EVERY STEP)

        print("COLLISION AHEAD - looking for a way round")
        T_start = mover.robot.fkine(q_start)
        for lift in lifts:                                           # TRY GOING HIGHER AND HIGHER
            for side in sides:                                       # AND LEFT / RIGHT
                middle = (T_start.t + T_target.t) / 2 + np.array([0.0, side, lift])
                try:
                    q_via = mover.solve_ik(SE3.Rt(T_target.R, middle))   # ARM POSE AT THE VIA-POINT
                except ValueError:                                   # VIA-POINT OUT OF REACH
                    continue
                leg1 = jtraj(q_start, q_via, steps).q
                leg2 = jtraj(q_via, q_goal, steps).q
                if self.first_hit_in_path(leg1) is None and self.first_hit_in_path(leg2) is None:
                    print("Going round: via-point %.2f m up, %.2f m sideways" % (lift, side))
                    self.last_via = (lift, side)
                    if not self.move_to_q_safe(q_via, steps, look_ahead=False):
                        return False
                    return self.move_to_q_safe(q_goal, steps, look_ahead=False)
        print("No clear path found - not moving")
        self.last_result = "no clear path"
        return False
