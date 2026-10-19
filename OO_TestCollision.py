from roboticstoolbox import jtraj #simple pass/fail checks for OO_Collision.py on every robot. run this file on its own
from OO_Collision import CollisionChecker, sphere_points
from OO_TestMain import check, results, find_test_pose, nearby_pose, put_at, build_workcell


def run_collision_tests(name, mover, other): #4 checks on one robot. other = any other robot, for check 3
    print("\n=== %s ===" % name)
    robot = mover.robot
    checker = CollisionChecker(robot)
    q = find_test_pose(mover) #a normal pose, not near a singularity
    put_at(mover, q)

    ellipsoids = checker.elipsoids_for_robot_around_links(q)
    centre = ellipsoids[2]['center'] #middle of link 3
    check("1. a point in the middle of a link is a hit", checker.collides(q, [centre]))

    far_point = robot.fkine(q).t + [0, 0, 2.0] #2 m above the hand
    check("2. a point 2 m away is not a hit", not checker.collides(q, [far_point]))

    other_points = CollisionChecker(other.robot).link_points(other.robot.q) #the other robot's links as points
    apart = not checker.collides(q, other_points)
    old_base = other.robot.base
    other.robot.base = robot.base #stand the other robot right on top of this one
    stacked = checker.collides(q, CollisionChecker(other.robot).link_points(other.robot.q))
    other.robot.base = old_base #put it back
    check("3. robots apart don't hit, robots on top of each other do", apart and stacked)

    q_end = nearby_pose(mover, q, size=1.0) #a big move
    path = jtraj(q, q_end, 40).q
    middle_of_last_link = checker.elipsoids_for_robot_around_links(path[20])[-1]['center'] #where the last link will be halfway through the move
    obstacle = sphere_points(middle_of_last_link, 0.05) #a 5 cm ball there
    hit = checker.first_collision_in_path(path, obstacle)
    check("4. an obstacle in the path is found before moving", hit is not None and hit > 0, "first hit at step %s of 40" % hit)

    put_at(mover, mover.home_q)


def main():
    workcell = build_workcell()
    movers = list(workcell.movers.items())
    for i, (name, mover) in enumerate(movers):
        other = movers[(i + 1) % len(movers)][1] #the next robot in the list
        run_collision_tests(name, mover, other)
    passed = sum(1 for _, ok in results if ok)
    print("\n%d of %d checks passed" % (passed, len(results)))
    for name, ok in results:
        if not ok:
            print("   FAILED:", name)


if __name__ == "__main__":
    main()