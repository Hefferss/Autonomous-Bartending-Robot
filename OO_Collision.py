# OO_Collision.py: lab 6 ellipsoid collision checking, one CollisionChecker per robot (works for every robot incl. the UR3)
# a point is inside a link's ellipsoid when (p - centre)^T A^-1 (p - centre) <= 1, where A is the ellipsoid matrix (lab 6)
# obstacles are point clouds: other robots' links (link_points) or a sphere dropped into a path (sphere_points)

import numpy as np
import trimesh # reads the item's STL for its size (already needed by ir_support)


# sphere points: points spread evenly over a sphere (fibonacci spiral). used for the obstacle dropped into a path
def sphere_points(centre, radius, count=200):
    k = np.arange(count) + 0.5                                       # one index per point
    phi = np.arccos(1 - 2 * k / count)                               # angle down from the top
    theta = np.pi * (1 + 5 ** 0.5) * k                               # golden-angle spiral around
    unit = np.column_stack((np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)))
    return unit * radius + np.array(centre, dtype=float)


_bounds_cache = {} # filename -> box, so each STL is only read once


# item bounds: the box (low corner, high corner) around an item's STL in its own frame. used so an item a robot is holding counts as part of the robot
def item_bounds(item):
    if item.filename not in _bounds_cache:
        _bounds_cache[item.filename] = trimesh.load(item.filename, force="mesh").bounds * np.asarray(item.scale)
    return _bounds_cache[item.filename]


class CollisionChecker:

    # robot: the DH robot to put ellipsoids around. fatness: how thick every link ellipsoid is (m)
    def __init__(self, robot, fatness=0.1):
        self.robot = robot
        self.fatness = fatness

    def elipsoids_for_robot_around_links(self, q): #will be called every time j traj runs for this robot. idea here is to make an elipsoid for each link and then use this to attach them to q's to check for collisions.
        Joint_Pos_Rotation = [T.A for T in self.robot.fkine_all(q)] #same way as in lab 6 q2. gets a 4x4 matrix for each joint so we can work out positoin adn rotation. get_transforms from lab 6.
        Ellipsoid_all = [] # makes an empty array for the elipsoids to be added to. same idea as week 6 q2.
        
        # restrict to robot.n links to ignore extra tool or EE frames
        num_links = min(len(Joint_Pos_Rotation) - 1, self.robot.n)
        for i in range(num_links): #for each link (between joint i and i+1):
                Elipsoid_vector = Joint_Pos_Rotation[i+1][:3,3] - Joint_Pos_Rotation[i][:3,3] #arrow from this joint to the next, along the link.
                Elipsoid_width = np.array([-Elipsoid_vector[1], Elipsoid_vector[0], 0]) #left and right of the link
                if np.linalg.norm(Elipsoid_width) == 0: #link 5 has no length, so the width is zero. make it a flat direction instead of zero.
                    Elipsoid_width = np.array([1, 0, 0]) #any flat direction is sideways to a vertical link
                Elipsoid_height = np.cross(Elipsoid_vector, Elipsoid_width) #up and down from the link
                Elipsoid_centre = (Joint_Pos_Rotation[i+1][:3,3] + Joint_Pos_Rotation[i][:3,3])/2 #middle of the link, where the elipsoid sits. halfway between the two joints. lab 6 2.5

                if np.linalg.norm(Elipsoid_vector) != 0:
                    Elipsoid_direction = Elipsoid_vector / np.linalg.norm(Elipsoid_vector) #along the link, length 1
                else:
                    Elipsoid_direction = Elipsoid_vector #link with no length, leave it as zeros. this is for link 5, because it doesnt have a length. just a direction.
                if np.linalg.norm(Elipsoid_width) != 0:
                    Elipsoid_width = Elipsoid_width / np.linalg.norm(Elipsoid_width)
                if np.linalg.norm(Elipsoid_height) != 0:
                    Elipsoid_height = Elipsoid_height / np.linalg.norm(Elipsoid_height)

                Elipsoid_rotation_matrix = np.column_stack((Elipsoid_direction, Elipsoid_width, Elipsoid_height)) #which way the elipsoid points. lab 6 2.7. makes it so the matrix lines up the long axis with the link
                
                radii = np.array([np.linalg.norm(Elipsoid_vector/2), self.fatness, self.fatness]) #half the link long, fatness wide
                Elipsoid_size = np.diag(np.square(radii)) #radii squared on the diagonal. lab 6 2.1
                Elipsoid_size_rotation = Elipsoid_rotation_matrix @ Elipsoid_size @ Elipsoid_rotation_matrix.T #size and direction packed into one 3x3. lab 6 2.7
                # a link with no length gives a flat matrix that can't be inverted, so make those a sphere of radius fatness
                if np.linalg.norm(Elipsoid_vector) < 1e-9:
                    Elipsoid_size_rotation = np.eye(3) * self.fatness ** 2
                    radii = np.array([self.fatness, self.fatness, self.fatness])
                    Elipsoid_rotation_matrix = np.eye(3)

                Ellipsoid_all.append({
                    'matrix': Elipsoid_size_rotation, 
                    'center': Elipsoid_centre,
                    'rotation': Elipsoid_rotation_matrix,
                    'radii': radii
                }) #save this link's elipsoid. lab 6 2.10
        return Ellipsoid_all #give the list back once every link is done. updates it to Ellispod_all every time jtraj runs.


    # collides: true if any obstacle point (n x 3) is inside any link ellipsoid of the robot at q
    def collides(self, q, points):
        points = np.atleast_2d(np.asarray(points, dtype=float))
        for e in self.elipsoids_for_robot_around_links(q):
            diff = points - e['center'] # every point relative to the ellipsoid centre
            distance = np.einsum('ij,jk,ik->i', diff, np.linalg.inv(e['matrix']), diff) # (p - c)^T A^-1 (p - c) for all points at once
            if np.any(distance <= 1.0):
                return True
        return False

    # look ahead: index of the first step of a planned path (e.g. jtraj(...).q) that hits, or none if it's all clear
    def first_collision_in_path(self, q_path, points):
        for step, q in enumerate(q_path):                            # every step of the path
            if self.collides(q, points):
                return step
        return None

    # link points: points along every link of this robot, used as the obstacle for the other robots' checkers
    def link_points(self, q, spacing=0.05):
        frames = [T.A for T in self.robot.fkine_all(q)]              # same frames as above
        pts = []
        num_links = min(len(frames) - 1, self.robot.n)
        for i in range(num_links):
            a, b = frames[i][:3, 3], frames[i + 1][:3, 3]            # the two joints of this link
            count = max(int(np.linalg.norm(b - a) / spacing) + 1, 2) # how many points along it
            for s in np.linspace(0, 1, count):
                pts.append(a + s * (b - a))
        return np.array(pts)

    # self hit: true if a link goes through a non-neighbouring link of the same robot (neighbours always touch at their joint)
    def self_hit(self, q, spacing=0.03):
        frames = [T.A[:3, 3] for T in self.robot.fkine_all(q)]
        ellipsoids = self.elipsoids_for_robot_around_links(q)
        real = [i for i in range(len(ellipsoids)) if np.linalg.norm(frames[i + 1] - frames[i]) > 1e-6]
        for a_i, i in enumerate(real):
            a, b = frames[i], frames[i + 1]
            count = max(int(np.linalg.norm(b - a) / spacing) + 1, 2)
            pts = np.array([a + s * (b - a) for s in np.linspace(0.25, 0.75, count)]) # middle half of the link, away from its joints
            for j in real[a_i + 2:]: # every real link at least two along
                e = ellipsoids[j]
                diff = pts - e['center']
                if np.any(np.einsum('ij,jk,ik->i', diff, np.linalg.inv(e['matrix']), diff) <= 1.0):
                    return True
        return False