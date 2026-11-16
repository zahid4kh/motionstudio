"""Walks a humanoid round a square, then turns it back to face the way it started.

Run with a humanoid on legs (Tools › Humanoid Builder) or:
    motionstudio-server --humanoid "Full-size"
"""
import motionstudio as ms

with ms.connect() as studio:
    walker = studio.humanoid()            # the only robot with legs
    print(walker, walker.default_gait)

    # One side at a time: walk, then a quarter turn to the left
    for _ in range(4):
        walker.walk(1000)                 # mm ahead
        walker.turn(90)                   # degrees to its left
    print("back at", walker.place)

    # The same square as one path, faster, without the arm swing
    side = 1000
    start = walker.place
    walker.walk_path([(side, 0), (side, -side), (0, -side), (0, 0)], heading=0, frame="robot",
                     speed=500, arm_swing=0)
    print("and again, at", walker.place, "started at", start)
