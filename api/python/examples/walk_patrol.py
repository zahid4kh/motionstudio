"""A humanoid patrols between waypoints, and a second program can stop it.

Run with a humanoid on legs (Tools › Humanoid Builder) or:
    motionstudio-server --humanoid "Full-size"

It walks a figure through four waypoints (world coordinates, mm) three times,
without waiting, and prints where it is twice a second. Ctrl+C stops it
gracefully: it finishes its step and brings its feet together.
"""
import motionstudio as ms

WAYPOINTS = [(1500, 0), (1500, -1200), (0, -1200), (0, 0)]

with ms.connect() as studio:
    walker = studio.humanoid()
    gait = ms.Gait(step_length=280, speed=450, arm_swing=15)
    try:
        for lap in range(3):
            walk = walker.walk_path(WAYPOINTS, heading=0, gait=gait, wait=False)
            while not walk.done:
                st = walker.state()
                print(f"lap {lap + 1}: step {st.step}/{st.steps} at {st.place}")
                studio.sleep(0.5)
            walk.wait()   # raises if it failed
    except KeyboardInterrupt:
        walker.stop_walking()
        print("stopping at", walker.place)
