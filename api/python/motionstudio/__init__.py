"""Control MotionStudio robots and stations from Python.

    import motionstudio as ms

    with ms.connect() as studio:
        robot = studio.robot("Robot")
        robot.move_l(z=100)              # straight line, only z changes
        robot.move_by(y=-50)             # 50 mm down
        robot.move_joints(A1=30)         # a joint move
        studio.signals["Start"] = True

        walker = studio.humanoid()       # a humanoid on legs
        walker.walk(1000)                # a metre ahead
        walker.turn(90)                  # a quarter turn to its left
        walker.walk_path([(1000, 0), (1000, 1000), (0, 1000)])

        with walker.record() as rec:     # its joint values and place, every scan
            walker.walk_to(2000, 0)
        rec.take.save("walk.csv")        # the CSV that Record Values writes
        walker.play(rec.take, relative=True)   # again, from where it stands

Units are millimetres, degrees and seconds; Y is up, as in MotionStudio.
"""

from .client import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    PROTOCOL,
    Actuator,
    Gait,
    Humanoid,
    JointInfo,
    Motion,
    MotionCancelled,
    MotionStudioError,
    NotFoundError,
    OutOfLimitsError,
    OutOfReachError,
    Place,
    Pose,
    Robot,
    Signals,
    SimulationError,
    Studio,
    Tool,
    WaitTimeout,
    WalkState,
    connect,
)
from .take import Recording, Take

__version__ = "0.2.0"

__all__ = [
    "connect", "Studio", "Robot", "Humanoid", "Tool", "Actuator", "Motion", "Pose", "Place", "Gait", "WalkState",
    "JointInfo", "Signals", "Take", "Recording",
    "MotionStudioError", "NotFoundError", "OutOfReachError", "OutOfLimitsError", "MotionCancelled",
    "WaitTimeout", "SimulationError", "DEFAULT_HOST", "DEFAULT_PORT", "PROTOCOL",
]
