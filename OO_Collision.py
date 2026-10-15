# ======================================================================
# OO_Collision.py  --  ellipsoid collision checking for ANY robot (including the UR3)
#
# Comment convention (same as OO_MovementCommands.py):
#   comments in normal case  = YOURS (kept exactly as you wrote them)
#   COMMENTS IN CAPITALS     = NEW, ADDED WHEN THIS WAS TURNED INTO A CLASS
#
# THIS FILE HAS:
#   CollisionChecker      - ONE PER ROBOT. YOUR ELLIPSOID CODE, TURNED INTO A CLASS, PLUS THE CHECKS AND THE DISPLAY
#   box_points            - TURNS A BOX INTO A POINT CLOUD TO CHECK AGAINST (LIKE THE POINT CLOUDS IN LAB 6)
#   sphere_points         - TURNS A SPHERE INTO A POINT CLOUD
#   make_unit_sphere_stl  - WRITES A SPHERE MESH FILE SO SWIFT CAN DRAW THE ELLIPSOIDS
#
# HOW A COLLISION IS FOUND (LAB 6): THE OBSTACLE IS A SET OF POINTS. A POINT IS INSIDE AN ELLIPSOID WHEN ITS
# "ALGEBRAIC DISTANCE"  (p - centre)^T  A^-1  (p - centre)  IS 1 OR LESS, WHERE A IS THE ELLIPSOID MATRIX.
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
import os                                                # FOR FILE PATHS
import struct                                            # FOR WRITING A BINARY STL FILE
import tempfile                                          # A SAFE PLACE TO PUT THE GENERATED SPHERE FILE
import numpy as np                                       # ARRAYS AND MATHS
from spatialgeometry import Mesh                         # DRAWS A MESH IN SWIFT (THE SAME CLASS THE ROBOT STLS USE)
from spatialmath.base import transl                      # BUILDS A 4x4 POSITION MATRIX


# ---------------------------- OBSTACLE HELPERS ------------------------

# BOX POINTS (NEW)
# POINTS COVERING THE SURFACE OF A BOX, SO A BOX CAN BE CHECKED LIKE ANY OTHER POINT CLOUD.
#   centre: (x, y, z) OF THE MIDDLE OF THE BOX      size: (length, width, height) IN METRES
#   spacing: DISTANCE BETWEEN POINTS. SMALLER = SAFER (CANNOT SLIP BETWEEN POINTS) BUT SLOWER
def box_points(centre, size, spacing=0.03):
    half = np.array(size, dtype=float) / 2                           # HALF EXTENTS
    axes = [np.linspace(-h, h, max(int(round(2 * h / spacing)) + 1, 2)) for h in half]   # POINTS ALONG EACH EDGE
    gx, gy, gz = np.meshgrid(*axes, indexing='ij')                   # A 3D GRID OF POINTS
    grid = np.column_stack((gx.ravel(), gy.ravel(), gz.ravel()))     # AS AN N x 3 LIST
    on_surface = np.any(np.isclose(np.abs(grid), half), axis=1)      # KEEP ONLY POINTS ON THE OUTSIDE
    return grid[on_surface] + np.array(centre, dtype=float)          # MOVE TO WHERE THE BOX IS


# SPHERE POINTS (NEW)
# POINTS SPREAD EVENLY OVER THE SURFACE OF A SPHERE (FIBONACCI SPIRAL). USE FOR BOTTLES, BALLS, A HAND.
def sphere_points(centre, radius, count=200):
    k = np.arange(count) + 0.5                                       # ONE INDEX PER POINT
    phi = np.arccos(1 - 2 * k / count)                               # ANGLE DOWN FROM THE TOP
    theta = np.pi * (1 + 5 ** 0.5) * k                               # GOLDEN-ANGLE SPIRAL AROUND
    unit = np.column_stack((np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)))
    return unit * radius + np.array(centre, dtype=float)


# MAKE UNIT SPHERE STL (NEW)
# SWIFT DRAWS MESHES FROM FILES. THIS WRITES A SPHERE OF RADIUS 1 AS A BINARY STL, THEN EACH ELLIPSOID IS THAT
# SPHERE SCALED TO ITS THREE RADII. RETURNS THE FILE PATH. IT IS ONLY WRITTEN ONCE (IN YOUR TEMP FOLDER).
def make_unit_sphere_stl(path=None):
    if path is None:                                                 # NO PATH GIVEN?
        path = os.path.join(tempfile.gettempdir(), "unit_sphere_for_collision.stl")
    if os.path.exists(path):                                         # ALREADY MADE?
        return path
    t = (1 + 5 ** 0.5) / 2                                           # GOLDEN RATIO, USED FOR THE ICOSAHEDRON
    verts = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t),
             (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2),
             (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5),
             (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]           # 20 TRIANGLES
    verts = [np.array(v, dtype=float) / np.linalg.norm(v) for v in verts]   # ALL ON THE UNIT SPHERE
    for _ in range(2):                                               # SPLIT EVERY TRIANGLE INTO 4, TWICE (320 TRIANGLES)
        cache, new_faces = {}, []
        def midpoint(a, b):
            key = tuple(sorted((a, b)))
            if key not in cache:
                m = (verts[a] + verts[b]) / 2
                verts.append(m / np.linalg.norm(m))                  # PUSH THE NEW POINT OUT TO THE SPHERE
                cache[key] = len(verts) - 1
            return cache[key]
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    with open(path, "wb") as f:                                      # BINARY STL: 80 BYTE HEADER, COUNT, THEN TRIANGLES
        f.write(b"unit sphere".ljust(80, b" "))
        f.write(struct.pack("<I", len(faces)))
        for a, b, c in faces:
            f.write(struct.pack("<3f", 0, 0, 0))                     # NORMAL (VIEWERS RECOMPUTE IT)
            for idx in (a, b, c):
                f.write(struct.pack("<3f", *verts[idx]))
            f.write(struct.pack("<H", 0))
    return path


# ---------------------------- THE COLLISION CLASS -----------------------

# ONE COLLISION CHECKER PER ROBOT. IT ONLY NEEDS THE ROBOT (ANY TOOLBOX ROBOT WITH fkine_all), SO IT WORKS FOR
# EVERY STUDENT'S ROBOT AND THE UR3.
class CollisionChecker:

    # CONSTRUCTOR
    #   robot: THE DH ROBOT TO PUT ELLIPSOIDS AROUND
    #   fatness: YOUR elipsoid_fatness (HOW THICK EVERY LINK ELLIPSOID IS, IN METRES). 0.1 = YOUR ORIGINAL VALUE
    #   length_margin: 1.0 = YOUR ORIGINAL (THE ELLIPSOID ENDS ARE EXACTLY AT THE JOINTS). 1.2 MAKES EACH ONE 20% LONGER
    #                  SO THE ENDS OF A LINK ARE COVERED TOO
    #   link_offsets: DICTIONARY OF LOCAL (X, Y, Z) SHIFTS PER LINK TO ALIGN WITH OFF-CENTER MESHES (E.G. LINK 3)
    def __init__(self, robot, fatness=0.1, length_margin=1.0, link_offsets=None):
        self.robot = robot                               # THE ROBOT
        self.fatness = fatness                           # replaces the global elipsoid_fatness, so every robot can have its own
        self.length_margin = length_margin
        self.link_offsets = link_offsets or {}           # ALLOWS SHIFTING OFF-CENTER LINK MESHES IN LOCAL COORDINATES
        self.meshes = []                                 # THE DRAWN ELLIPSOIDS (EMPTY UNTIL add_to_scene)
        self.visible = False                             # ARE THE DRAWN ELLIPSOIDS SHOWN?

    # ---- YOUR CODE ------------------------------------------------------

    def elipsoids_for_robot_around_links(self, q): #will be called every time j traj runs for this robot. Idea here is to make an elipsoid for each link and then use this to attach them to Q's to check for collisions.
        Joint_Pos_Rotation = [T.A for T in self.robot.fkine_all(q)] #same way as in lab 6 q2. gets a 4x4 matrix for each joint so we can work out positoin adn rotation. get_transforms from lab 6.
        Ellipsoid_all = [] # makes an empty array for the elipsoids to be added to. same idea as week 6 q2.
        
        # RESTRICT TO robot.n LINKS TO IGNORE EXTRA TOOL OR EE FRAMES
        num_links = min(len(Joint_Pos_Rotation) - 1, self.robot.n)
        for i in range(num_links): #for each link (between joint i and i+1):
                Elipsoid_vector = Joint_Pos_Rotation[i+1][:3,3] - Joint_Pos_Rotation[i][:3,3] #arrow from this joint to the next, along the link.
                Elipsoid_width = np.array([-Elipsoid_vector[1], Elipsoid_vector[0], 0]) #left and right of the link
                if np.linalg.norm(Elipsoid_width) == 0: #link 5 has no length, so the width is zero. make it a flat direction instead of zero.
                    Elipsoid_width = np.array([1, 0, 0]) #any flat direction is sideways to a vertical link
                Elipsoid_height = np.cross(Elipsoid_vector, Elipsoid_width) #up and down from the link
                Elipsoid_centre = (Joint_Pos_Rotation[i+1][:3,3] + Joint_Pos_Rotation[i][:3,3])/2 #middle of the link, where the elipsoid sits. halfway between the two joints. Lab 6 2.5

                if np.linalg.norm(Elipsoid_vector) != 0:
                    Elipsoid_direction = Elipsoid_vector / np.linalg.norm(Elipsoid_vector) #along the link, length 1
                else:
                    Elipsoid_direction = Elipsoid_vector #link with no length, leave it as zeros. This is for link 5, because it doesnt have a length. just a direction.
                if np.linalg.norm(Elipsoid_width) != 0:
                    Elipsoid_width = Elipsoid_width / np.linalg.norm(Elipsoid_width)
                if np.linalg.norm(Elipsoid_height) != 0:
                    Elipsoid_height = Elipsoid_height / np.linalg.norm(Elipsoid_height)

                Elipsoid_rotation_matrix = np.column_stack((Elipsoid_direction, Elipsoid_width, Elipsoid_height)) #which way the elipsoid points. lab 6 2.7. makes it so the matrix lines up the long axis with the link
                
                # APPLY LOCAL LINK OFFSET IF SPECIFIED, SO OFF-CENTER MESHES ARE FULLY COVERED
                if i in self.link_offsets:
                    local_offset = np.array(self.link_offsets[i], dtype=float)
                    Elipsoid_centre = Elipsoid_centre + Elipsoid_rotation_matrix @ local_offset

                # DEFINE RADII EXPLICITLY TO AVOID EIGENVALUE SORTING ISSUES
                length_radius = np.linalg.norm(Elipsoid_vector/2) * self.length_margin
                radii = np.array([length_radius, self.fatness, self.fatness])

                # NEW: length_margin (1.0 = unchanged) STRETCHES THE LONG AXIS SO THE LINK ENDS ARE COVERED
                Elipsoid_size = np.diag(np.square(radii)) #radii squared on the diagonal. lab 6 2.1
                Elipsoid_size_rotation = Elipsoid_rotation_matrix @ Elipsoid_size @ Elipsoid_rotation_matrix.T #size and direction packed into one 3x3. lab 6 2.7
                # NEW (BUG FIX): A LINK WITH NO LENGTH GIVES A "FLAT" MATRIX THAT CANNOT BE INVERTED (RANK 1, NOT 3), SO THE
                # COLLISION CHECK WOULD DIVIDE BY ZERO. THE FIX: SHAPE THOSE LINKS AS A SPHERE OF RADIUS fatness INSTEAD.
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


    def elipsoid_parameters_for_robots(self, elipsoid_info): #takes the elipsoid info and makes it into a 3D mesh for swift. uses the ellipsoid method from week 4.
        return elipsoid_info['center'], elipsoid_info['rotation'], elipsoid_info['radii']

    # ---- THE CHECKS (NEW) -----------------------------------------------

    # WHICH LINKS HIT ANY OF THE POINTS?
    # RETURNS A LIST OF LINK NUMBERS (0 = THE FIRST LINK). EMPTY LIST = NO COLLISION.
    #   q: JOINT ANGLES TO CHECK      points: AN N x 3 ARRAY OF OBSTACLE POINTS (SEE box_points, sphere_points)
    def links_hit(self, q, points):
        points = np.atleast_2d(np.asarray(points, dtype=float))      # MAKE SURE IT IS N x 3
        hit = []                                                     # LINKS THAT ARE HIT
        for link, e in enumerate(self.elipsoids_for_robot_around_links(q)):
            diff = points - e['center']                              # EVERY POINT RELATIVE TO THE ELLIPSOID CENTRE
            inverse = np.linalg.inv(e['matrix'])                     # A^-1
            distance = np.einsum('ij,jk,ik->i', diff, inverse, diff) # (p - c)^T A^-1 (p - c), FOR ALL POINTS AT ONCE
            if np.any(distance <= 1.0):                              # ANY POINT INSIDE THIS ELLIPSOID?
                hit.append(link)
        return hit

    # TRUE IF THE ROBOT AT q TOUCHES ANY OF THE POINTS
    def collides(self, q, points):
        return len(self.links_hit(q, points)) > 0

    # LOOK AHEAD: THE INDEX OF THE FIRST STEP IN A PLANNED PATH THAT HITS, OR None IF THE WHOLE PATH IS CLEAR.
    #   q_path: A LIST OR ARRAY OF JOINT ANGLES, ONE ROW PER STEP (FOR EXAMPLE jtraj(...).q)
    # USE THIS BEFORE MOVING SO THE ROBOT NEVER STARTS A MOVE THAT WILL HIT.
    def first_collision_in_path(self, q_path, points):
        for step, q in enumerate(q_path):                            # EVERY STEP OF THE PATH
            if self.collides(q, points):
                return step
        return None

    # POINTS ALONG EVERY LINK OF THIS ROBOT, TO USE AS THE "OBSTACLE" FOR ANOTHER ROBOT'S CHECKER.
    # THIS IS HOW ROBOT-AGAINST-ROBOT CHECKING WORKS.   spacing: METRES BETWEEN POINTS
    def link_points(self, q, spacing=0.05):
        frames = [T.A for T in self.robot.fkine_all(q)]              # SAME FRAMES AS ABOVE
        pts = []
        num_links = min(len(frames) - 1, self.robot.n)
        for i in range(num_links):
            a, b = frames[i][:3, 3], frames[i + 1][:3, 3]            # THE TWO JOINTS OF THIS LINK
            count = max(int(np.linalg.norm(b - a) / spacing) + 1, 2) # HOW MANY POINTS ALONG IT
            for s in np.linspace(0, 1, count):
                pts.append(a + s * (b - a))
        return np.array(pts)

    # ---- THE DISPLAY (NEW) ----------------------------------------------
    # DRAWS EACH ELLIPSOID IN SWIFT SO YOU CAN SEE THEM. THE RADII NEVER CHANGE (LINK LENGTHS ARE FIXED), ONLY THE
    # CENTRE AND ROTATION, SO EACH STEP ONLY HAS TO MOVE THE MESHES.

    # ADD THE ELLIPSOID MESHES TO THE SWIFT WINDOW (CALL AFTER THE WINDOW IS LAUNCHED). THEY START HIDDEN.
    #   env: THE SWIFT ENVIRONMENT (workcell.env)     q: THE JOINT ANGLES TO SIZE THEM FROM (DEFAULT: THE ROBOT'S CURRENT q)
    def add_to_scene(self, env, q=None, color="#FF8C0050"):
        if q is None:
            q = self.robot.q
        sphere_file = make_unit_sphere_stl()                         # THE UNIT SPHERE MESH
        self.meshes = []
        for info in self.elipsoids_for_robot_around_links(q):
            center, rotation, radii = self.elipsoid_parameters_for_robots(info)   # YOUR FUNCTION
            mesh = Mesh(filename=sphere_file, color=color, scale=[float(r) for r in radii])   # A SPHERE STRETCHED TO THE RADII
            env.add(mesh)                                            # NEVER REMOVED: REMOVING CRASHES OLDER SWIFT
            self.meshes.append(mesh)
        self.set_visible(False)                                      # HIDDEN UNTIL ASKED
        return self.meshes

    # MOVE THE DRAWN ELLIPSOIDS TO MATCH THE ROBOT AT q. CALL ONCE PER STEP (ONLY DOES WORK WHILE VISIBLE).
    def update_display(self, q):
        if not self.visible:                                         # HIDDEN: NOTHING TO DO
            return
        for mesh, info in zip(self.meshes, self.elipsoids_for_robot_around_links(q)):
            center, rotation, radii = self.elipsoid_parameters_for_robots(info)
            T = np.eye(4)
            T[:3, :3] = rotation                                     # WHICH WAY THE ELLIPSOID POINTS
            T[:3, 3] = center                                        # WHERE IT IS
            mesh.T = T

    # SHOW OR HIDE THE ELLIPSOIDS. "HIDING" PARKS THEM 1000 m AWAY (SAME TRICK AS THE STATE BEACONS).
    # THIS IS WHAT A "SHOW COLLISION SPHERES" BUTTON SHOULD CALL.
    def set_visible(self, visible):
        self.visible = bool(visible)
        if self.visible:
            self.update_display(self.robot.q)                        # MOVE THEM ONTO THE ROBOT
        else:
            for mesh in self.meshes:
                mesh.T = transl(1000, 1000, -1000)                   # PARK THEM OUT OF SIGHT