"""Waves a humanoid's right hand and points its left hand forward.

Run with a humanoid (Tools › Humanoid Builder) or:
    motionstudio-server --humanoid "Full-size"
"""
import motionstudio as ms

with ms.connect() as studio:
    humanoid = studio.robot("Humanoid")
    print(humanoid.tool_names)
    humanoid.home()
    humanoid.move_joints(RShoulderRoll=100, RElbow=60, speed=60)
    for _ in range(3):
        humanoid.move_joints(RShoulderYaw=25, speed=80)
        humanoid.move_joints(RShoulderYaw=-25, speed=80)

    # The left hand as a tool, moved by its arm only (not the waist)
    hand = humanoid.tool("Left Hand TCP", base="Torso")
    humanoid.move_joints(LShoulderPitch=40, LElbow=70)   # bend the arm: a straight one is at full reach
    hand.move_by(x=60, speed=150)          # straight line, 60 mm forward
    hand.move_by(z=-60, speed=150)         # and 60 mm to its left (-Z)
    humanoid.home()
