"""Draws a 200 mm square with the tool, in straight lines, then goes home.

Start MotionStudio with a robot (Tools › Robot Builder) and the Robot API on,
or run:  motionstudio-server --robot "6-axis 0.9"
"""
import motionstudio as ms

with ms.connect() as studio:
    robot = studio.robot()                 # the scene's only robot
    print(robot, "tool at", robot.pose())

    start = robot.pose()
    corner = start.offset(y=-150)          # 150 mm lower: the first corner
    robot.move_l(corner, speed=300)
    for dx, dz in [(0, 200), (-200, 0), (0, -200), (200, 0)]:
        robot.move_by(x=dx, z=dz, speed=200)
    robot.move_l(start)
    robot.home()
    print("done at", round(studio.time, 2), "s")
