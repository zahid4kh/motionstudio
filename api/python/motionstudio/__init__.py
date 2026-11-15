"""Control MotionStudio robots and stations from Python.

    import motionstudio as ms

    with ms.connect() as studio:
        robot = studio.robot("Robot")
        robot.move_l(z=100)              # straight line, only z changes
        robot.move_by(y=-50)             # 50 mm down
        robot.move_joints(A1=30)         # a joint move
        studio.signals["Start"] = True

Units are millimetres, degrees and seconds; Y is up, as in MotionStudio.
"""

from .client import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    PROTOCOL,
    Actuator,
    JointInfo,
    Motion,
    MotionCancelled,
    MotionStudioError,
    NotFoundError,
    OutOfLimitsError,
    OutOfReachError,
    Pose,
    Robot,
    Signals,
    SimulationError,
    Studio,
    Tool,
    WaitTimeout,
    connect,
)

__version__ = "0.1.0"

__all__ = [
    "connect", "Studio", "Robot", "Tool", "Actuator", "Motion", "Pose", "JointInfo", "Signals",
    "MotionStudioError", "NotFoundError", "OutOfReachError", "OutOfLimitsError", "MotionCancelled",
    "WaitTimeout", "SimulationError", "DEFAULT_HOST", "DEFAULT_PORT", "PROTOCOL",
]
