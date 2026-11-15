"""Moves two things at once: start motions with wait=False, then wait for both."""
import motionstudio as ms

with ms.connect() as studio:
    robot = studio.robot()
    names = robot.joint_names
    first = robot.move_joints({names[0]: 45}, speed=40, wait=False)
    print("moving", first)
    lift = robot.move_by(y=50, wait=False)  # queued: it shares joints with the first motion
    lift.wait()
    print(first.state, lift.state)

    # Motions queue per joint, in order; stop() cancels what's left
    robot.move_joints({names[0]: -45}, wait=False)
    studio.sleep(0.5)                       # simulation seconds
    print("stopped", robot.stop(), "motion(s) at", robot.joints[names[0]])
