# OO_TestCollision.py: PASS/FAIL tests for OO_Collision.py on every robot (ellipsoids, hits, look-ahead, robot vs robot, speed)

import time
import numpy as np
from roboticstoolbox import jtraj
from OO_Collision import CollisionChecker, sphere_points
from OO_TestMain import check, results, find_test_pose, nearby_pose, put_at, build_workcell


# run_collision_tests: runs tests 1 to 6 on one robot. "others" are the other movers, for test 5.
def run_collision_tests(name, mover, others):
    print("\n=== %s ===" % name)
    robot = mover.robot
    checker = CollisionChecker(robot, fatness=0.1)
    q_a = find_test_pose(mover)
    q_b = nearby_pose(mover, q_a, size=0.5)
    put_at(mover, q_a)

    # 1. every link has a proper ellipsoid
    ellipsoids = checker.elipsoids_for_robot_around_links(q_a)
    solid = all(np.linalg.matrix_rank(e['matrix']) == 3 for e in ellipsoids)
    check("1.  one ellipsoid per link, and every one is a solid (rank 3)", len(ellipsoids) == robot.n and solid,
          "%d ellipsoids" % len(ellipsoids))

    # 2. inside / outside
    centre = ellipsoids[len(ellipsoids) // 2]['center']
    check("2a. a point at an ellipsoid centre is a hit", checker.collides(q_a, [centre]))
    check("2b. a point 1 m above every link is not a hit", not checker.collides(q_a, [np.max([e['center'] for e in ellipsoids], axis=0) + [0, 0, 1.0]]))

    # 3. ellipsoids follow the robot
    moved = checker.elipsoids_for_robot_around_links(q_b)
    frames = [T.A for T in robot.fkine_all(q_b)]
    follows = all(np.allclose(moved[i]['center'], (frames[i][:3, 3] + frames[i + 1][:3, 3]) / 2) for i in range(len(moved)))
    check("3.  ellipsoid centres sit halfway between the joints after the robot moves", follows)

    # 4. obstacle in a planned path
    q_far = nearby_pose(mover, q_a, size=1.0)       # A BIGGER MOVE, SO THE MIDDLE OF THE PATH IS CLEAR OF THE START AND END
    path = jtraj(q_a, q_far, 40).q
    clear_before = checker.first_collision_in_path(path, sphere_points((0, 0, 5), 0.05))   # far away obstacle
    # FIND A SPOT THE ROBOT HITS MID-MOVE BUT NOT AT THE START OR END (LAST LINKS FIRST, THEY MOVE THE MOST)
    bottle = None
    for e in reversed(checker.elipsoids_for_robot_around_links(path[20])):
        for s_along in np.linspace(-0.8, 0.8, 9):
            spot = e['center'] + s_along * e['radii'][0] * e['rotation'][:, 0]       # a point along this link
            candidate = sphere_points(spot, 0.02)                                     # a small bottle there
            if (checker.collides(path[20], candidate) and not checker.collides(q_a, candidate)
                    and not checker.collides(q_far, candidate)):
                bottle = candidate
                break
        if bottle is not None:
            break
    hit_step = checker.first_collision_in_path(path, bottle) if bottle is not None else None
    check("4a. an obstacle placed in the path is found before moving", hit_step is not None and hit_step > 0,
          "could not find a spot that is clear at both ends" if bottle is None else "first hit at step %s of 40" % hit_step)
    check("4b. the path is clear when the obstacle is far away", clear_before is None)
    check("4c. the start and end poses do not hit that obstacle",
          bottle is not None and not checker.collides(q_a, bottle) and not checker.collides(q_far, bottle))

    # 5. robot against robot
    if others:
        other = others[0]
        other_checker = CollisionChecker(other.robot)
        points = other_checker.link_points(other.robot.q)
        separate = not checker.collides(robot.q, points)
        old_base = other.robot.base
        try:
            other.robot.base = robot.base            # pretend the other robot stands on top of this one
            overlap = checker.collides(robot.q, other_checker.link_points(other.robot.q))
        finally:
            other.robot.base = old_base              # always put it back
        check("5.  robots that are apart do not hit, and robots on top of each other do", separate and overlap)
    else:
        print("   (test 5 skipped: only one robot)")

    # 6. speed
    cloud = sphere_points((0.5, 0.5, 0.5), 0.2, count=2000)
    start = time.perf_counter()
    for _ in range(20):
        checker.collides(q_a, cloud)
    per_check = (time.perf_counter() - start) / 20 * 1000
    check("6.  one check against %d points takes under 50 ms" % len(cloud), per_check < 50, "%.1f ms" % per_check)

    put_at(mover, mover.home_q)


def main():
    workcell = build_workcell()
    movers = list(workcell.movers.items())
    for i, (name, mover) in enumerate(movers):
        others = [m for j, (n, m) in enumerate(movers) if j != i]
        run_collision_tests(name, mover, others)
    passed = sum(1 for _, ok in results if ok)
    print("\n==================== SUMMARY ====================")
    print("%d of %d checks passed" % (passed, len(results)))
    for name, ok in results:
        if not ok:
            print("   FAILED:", name)


if __name__ == "__main__":
    main()