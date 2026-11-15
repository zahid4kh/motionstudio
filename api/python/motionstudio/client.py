"""The MotionStudio robot API client.

Everything goes through one TCP connection to MotionStudio (the app, or the
headless ``motionstudio-server``). Requests are JSON lines; a background thread
reads the replies and hands each to the call waiting for it, so several
motions can run at once and handles can be shared between threads.

Units: millimetres, degrees and seconds. Y is up, as in MotionStudio.
"""

from __future__ import annotations

import itertools
import json
import socket
import threading
from dataclasses import dataclass, replace
from typing import Any, Dict, Iterator, List, Mapping, Optional, Union

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 5800
PROTOCOL = 1

Number = Union[int, float]


# ── Errors ──────────────────────────────────────────────────────────────────


class MotionStudioError(Exception):
    """An error reported by MotionStudio. ``code`` says what kind it is."""

    def __init__(self, code: str, message: str, motion: Optional[int] = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.motion = motion

    def __str__(self) -> str:
        return f"{self.message} ({self.code})"


class NotFoundError(MotionStudioError):
    """No robot, tool, joint, pose or signal has that name (or several do: ``ambiguous``)."""


class OutOfReachError(MotionStudioError):
    """The tool can't get there: the target, or a point on the straight line, is out of reach."""


class OutOfLimitsError(MotionStudioError):
    """A joint value is outside the joint's range."""


class MotionCancelled(MotionStudioError):
    """The motion was stopped, cancelled by an earlier failure, or the simulation was reset."""


class WaitTimeout(MotionStudioError, TimeoutError):
    """A wait ran out of time (the motion or condition carries on)."""


class SimulationError(MotionStudioError):
    """The simulation could not start (the automation has errors), or is not running."""


_ERRORS = {
    "not_found": NotFoundError,
    "ambiguous": NotFoundError,
    "out_of_reach": OutOfReachError,
    "out_of_limits": OutOfLimitsError,
    "cancelled": MotionCancelled,
    "reset": MotionCancelled,
    "timeout": WaitTimeout,
    "simulation": SimulationError,
    "not_running": SimulationError,
}


def _error(err: Mapping[str, Any]) -> MotionStudioError:
    code = str(err.get("code", "error"))
    return _ERRORS.get(code, MotionStudioError)(code, str(err.get("message", code)), err.get("motion"))


# ── Values ──────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Pose:
    """A position (mm) and orientation (Euler XYZ, degrees) of a tool frame."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def offset(self, x: Number = 0, y: Number = 0, z: Number = 0,
               rx: Number = 0, ry: Number = 0, rz: Number = 0) -> "Pose":
        """This pose with the given amounts added to its values."""
        return Pose(self.x + x, self.y + y, self.z + z, self.rx + rx, self.ry + ry, self.rz + rz)

    def replace(self, **values: Number) -> "Pose":
        """This pose with some values replaced: ``pose.replace(z=100)``."""
        return replace(self, **{k: float(v) for k, v in values.items()})

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "rx": self.rx, "ry": self.ry, "rz": self.rz}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Pose":
        return cls(*(float(d.get(k, 0.0)) for k in ("x", "y", "z", "rx", "ry", "rz")))

    def __str__(self) -> str:
        return ("Pose(x={:.2f}, y={:.2f}, z={:.2f}, rx={:.2f}, ry={:.2f}, rz={:.2f})"
                .format(self.x, self.y, self.z, self.rx, self.ry, self.rz))


@dataclass(frozen=True)
class JointInfo:
    """One joint of a robot: its actuator's name, range and speed."""

    name: str
    type: str                 # "revolute" or "prismatic"
    unit: str                 # "deg" or "mm"
    value: float
    min: Optional[float]      # None for a continuous joint
    max: Optional[float]
    speed: float              # the actuator's speed (100 %), per second
    acceleration: float


# ── Connection ──────────────────────────────────────────────────────────────


class _Connection:
    def __init__(self, host: str, port: int, timeout: Optional[float]):
        try:
            self._sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as e:
            raise ConnectionError(
                f"can't reach MotionStudio at {host}:{port} ({e.strerror or e}). Is the app running with "
                "Tools › Robot API switched on, or motionstudio-server started?") from None
        self._sock.settimeout(None)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._ids = itertools.count(1)
        self._send_lock = threading.Lock()
        self._lock = threading.Lock()
        self._waiting: Dict[int, List[Any]] = {}   # id → [event, reply]
        self._closed: Optional[str] = None
        self._reader = threading.Thread(target=self._read, name="motionstudio-reader", daemon=True)
        self._reader.start()

    def _read(self) -> None:
        buf = b""
        try:
            while True:
                chunk = self._sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    msg = json.loads(line)
                    with self._lock:
                        slot = self._waiting.pop(msg.get("id"), None)
                    if slot is not None:
                        slot[1] = msg
                        slot[0].set()
        except (OSError, ValueError):
            pass
        with self._lock:
            self._closed = self._closed or "the connection to MotionStudio was closed"
            waiting, self._waiting = self._waiting, {}
        for slot in waiting.values():
            slot[0].set()

    def call(self, method: str, params: Optional[Mapping[str, Any]] = None,
             timeout: Optional[float] = None) -> Any:
        rid = next(self._ids)
        slot: List[Any] = [threading.Event(), None]
        with self._lock:
            if self._closed:
                raise ConnectionError(self._closed)
            self._waiting[rid] = slot
        line = json.dumps({"id": rid, "method": method, "params": dict(params or {})}) + "\n"
        try:
            with self._send_lock:
                self._sock.sendall(line.encode())
        except OSError as e:
            with self._lock:
                self._waiting.pop(rid, None)
            raise ConnectionError(f"can't send to MotionStudio: {e}") from None
        if not slot[0].wait(timeout):
            with self._lock:
                self._waiting.pop(rid, None)
            raise WaitTimeout("timeout", f"no answer to {method} within {timeout} s")
        reply = slot[1]
        if reply is None:
            raise ConnectionError(self._closed or "the connection to MotionStudio was closed")
        if "error" in reply:
            raise _error(reply["error"])
        return reply.get("result")

    def close(self) -> None:
        with self._lock:
            self._closed = "the connection was closed"
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()


def _drop_none(d: Mapping[str, Any]) -> Dict[str, Any]:
    return {k: v for k, v in d.items() if v is not None}


def _target(target: Optional[Pose], **parts: Optional[Number]) -> Dict[str, float]:
    t: Dict[str, float] = target.to_dict() if target is not None else {}
    t.update({k: float(v) for k, v in parts.items() if v is not None})
    return t


# ── Motions ─────────────────────────────────────────────────────────────────


class Motion:
    """A motion MotionStudio is running (from a move with ``wait=False``)."""

    def __init__(self, studio: "Studio", reply: Mapping[str, Any]):
        self._studio = studio
        self.id: int = int(reply["motion"])
        self._last = dict(reply)

    def wait(self, timeout: Optional[float] = None) -> "Motion":
        """Waits until the motion ends. Raises if it failed or was cancelled.

        ``timeout`` is in simulation seconds; when it runs out, ``WaitTimeout``
        is raised and the motion carries on."""
        self._last = self._studio._call("motion.wait", _drop_none({"motion": self.id, "timeout": timeout}))
        return self

    def cancel(self) -> None:
        """Stops the motion; the joints brake and hold where they are."""
        self._last = self._studio._call("motion.cancel", {"motion": self.id})

    @property
    def state(self) -> str:
        """``queued``, ``running``, ``done``, ``failed`` or ``cancelled``."""
        self._last = self._studio._call("motion.status", {"motion": self.id})
        return str(self._last["state"])

    @property
    def done(self) -> bool:
        """True once the motion has ended (whichever way)."""
        return self.state in ("done", "failed", "cancelled")

    @property
    def progress(self) -> float:
        """0 … 1."""
        self._last = self._studio._call("motion.status", {"motion": self.id})
        return float(self._last.get("progress", 0.0))

    def __repr__(self) -> str:
        return f"<Motion {self.id} {self._last.get('state', '?')}>"


# ── Robots and tools ────────────────────────────────────────────────────────


class Tool:
    """A tool frame (TCP) of a robot: what moves to Cartesian targets."""

    def __init__(self, robot: "Robot", name: str, base: Optional[str] = None):
        self.robot = robot
        self.name = name
        self.base = base

    def _params(self, **more: Any) -> Dict[str, Any]:
        return _drop_none({"robot": self.robot.path, "tool": self.name, "base": self.base, **more})

    def pose(self, frame: str = "base") -> Pose:
        """Where the tool is: in the robot's base frame, ``"world"``, or any object's frame."""
        return Pose.from_dict(self.robot.studio._call("tool.pose", self._params(frame=frame)))

    def move_l(self, target: Optional[Pose] = None, *, x: Optional[Number] = None, y: Optional[Number] = None,
               z: Optional[Number] = None, rx: Optional[Number] = None, ry: Optional[Number] = None,
               rz: Optional[Number] = None, speed: Optional[Number] = None, frame: str = "base",
               wait: bool = True) -> Motion:
        """Moves the tool along a straight line to a pose.

        Give a ``Pose`` or any of ``x y z rx ry rz``: the values you leave out
        stay as they are (``move_l(z=100)`` only changes z). ``speed`` is in
        mm/s (default 250). ``frame``: ``"base"`` (the robot's), ``"world"``
        or an object's name."""
        return self._move("tool.moveL", _target(target, x=x, y=y, z=z, rx=rx, ry=ry, rz=rz), speed, frame,
                          False, wait)

    def move_j(self, target: Optional[Pose] = None, *, x: Optional[Number] = None, y: Optional[Number] = None,
               z: Optional[Number] = None, rx: Optional[Number] = None, ry: Optional[Number] = None,
               rz: Optional[Number] = None, speed: Optional[Number] = None, frame: str = "base",
               wait: bool = True) -> Motion:
        """Moves the tool to a pose with a joint move (faster, but not straight).

        Like ``move_l``, but ``speed`` is a percentage of the joints' speed (default 100)."""
        return self._move("tool.moveJ", _target(target, x=x, y=y, z=z, rx=rx, ry=ry, rz=rz), speed, frame,
                          False, wait)

    def move_by(self, x: Number = 0, y: Number = 0, z: Number = 0, rx: Number = 0, ry: Number = 0,
                rz: Number = 0, *, speed: Optional[Number] = None, frame: str = "base", linear: bool = True,
                wait: bool = True) -> Motion:
        """Moves the tool by an offset from where it is, along a straight line.

        ``frame="tool"`` moves along and turns about the tool's own axes;
        ``"base"`` (default), ``"world"`` or an object's name uses that frame's
        axes (turning about the tool's point). ``linear=False`` makes it a joint move."""
        offset = {k: float(v) for k, v in dict(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz).items() if v}
        return self._move("tool.moveL" if linear else "tool.moveJ", offset, speed, frame, True, wait)

    def _move(self, method: str, target: Dict[str, float], speed: Optional[Number], frame: str,
              relative: bool, wait: bool) -> Motion:
        reply = self.robot.studio._call(method, self._params(target=target, speed=speed, frame=frame,
                                                             relative=relative, wait=wait))
        return Motion(self.robot.studio, reply)

    def __repr__(self) -> str:
        return f"<Tool {self.name} of {self.robot.path}>"


class Robot:
    """A robot or other machine in the scene: the joints below one object."""

    def __init__(self, studio: "Studio", info: Mapping[str, Any]):
        self.studio = studio
        self.name: str = info["name"]
        self.path: str = info["path"]
        self.joint_names: List[str] = list(info.get("joints", []))
        self.tool_names: List[str] = list(info.get("tools", []))

    # Joints

    @property
    def joints(self) -> Dict[str, float]:
        """The joints' values now (degrees, or mm for sliding joints), by actuator name."""
        return {k: float(v) for k, v in self.studio._call("robot.joints", {"robot": self.path}).items()}

    def joint_info(self) -> List[JointInfo]:
        """Every joint with its range, unit and speed."""
        out = []
        for j in self.studio._call("robot.info", {"robot": self.path})["joints"]:
            out.append(JointInfo(j["name"], j["type"], j["unit"], float(j["value"]), j.get("min"), j.get("max"),
                                 float(j["speed"]), float(j["acceleration"])))
        return out

    def move_joints(self, values: Optional[Mapping[str, Number]] = None, *, speed: Optional[Number] = None,
                    relative: bool = False, wait: bool = True, **joints: Number) -> Motion:
        """Moves joints to values: ``robot.move_joints(J1=30, J2=-10)`` or ``robot.move_joints({"J1": 30})``.

        The joints arrive together. ``speed`` is a percentage of the joints'
        speed (default 100); ``relative=True`` moves them by the values."""
        targets = {**(values or {}), **joints}
        reply = self.studio._call("robot.moveJoints", _drop_none({
            "robot": self.path, "joints": {k: float(v) for k, v in targets.items()},
            "speed": speed, "relative": relative, "wait": wait}))
        return Motion(self.studio, reply)

    def move_to_pose(self, pose: str, *, speed: Optional[Number] = None, wait: bool = True) -> Motion:
        """Moves the joints to a pose taught in MotionStudio (Automation › Actuators & Poses)."""
        reply = self.studio._call("robot.moveToPose", _drop_none(
            {"robot": self.path, "pose": pose, "speed": speed, "wait": wait}))
        return Motion(self.studio, reply)

    def home(self, *, speed: Optional[Number] = None, wait: bool = True) -> Motion:
        """Moves to the robot's home pose ("<robot> Home" or "Home")."""
        reply = self.studio._call("robot.home", _drop_none({"robot": self.path, "speed": speed, "wait": wait}))
        return Motion(self.studio, reply)

    def stop(self) -> int:
        """Stops every motion of this robot; returns how many were stopped."""
        return int(self.studio._call("robot.stop", {"robot": self.path})["stopped"])

    @property
    def poses(self) -> List[str]:
        """Names of the poses taught for this robot."""
        return list(self.studio._call("robot.info", {"robot": self.path})["poses"])

    # Tools

    def tool(self, name: Optional[str] = None, *, base: Optional[str] = None) -> Tool:
        """The robot's tool frame. Without a name: its only tool (``TCP``).

        ``base`` limits the joints that move to those below that object, e.g.
        a humanoid's arm without its waist: ``tool("Right Hand TCP", base="Torso")``."""
        if name is None:
            if len(self.tool_names) == 1:
                name = self.tool_names[0]
            elif not self.tool_names:
                raise NotFoundError("not_found", f"{self.name} has no tool; give the name of an object to use")
            else:
                raise NotFoundError("ambiguous", f"{self.name} has {len(self.tool_names)} tools: pick one of "
                                                 + ", ".join(self.tool_names))
        return Tool(self, name, base)

    # The only tool's moves, so a one-tool robot reads naturally

    def pose(self, frame: str = "base") -> Pose:
        """Where the robot's tool is (see ``Tool.pose``)."""
        return self.tool().pose(frame)

    def move_l(self, target: Optional[Pose] = None, **kwargs: Any) -> Motion:
        """Straight-line move of the robot's tool (see ``Tool.move_l``)."""
        return self.tool().move_l(target, **kwargs)

    def move_j(self, target: Optional[Pose] = None, **kwargs: Any) -> Motion:
        """Joint move of the robot's tool to a pose (see ``Tool.move_j``)."""
        return self.tool().move_j(target, **kwargs)

    def move_by(self, *args: Number, **kwargs: Any) -> Motion:
        """Relative move of the robot's tool (see ``Tool.move_by``)."""
        return self.tool().move_by(*args, **kwargs)

    def __repr__(self) -> str:
        return f"<Robot {self.path}: {len(self.joint_names)} joints>"


class Actuator:
    """One actuator by name, on any machine: a door, a turntable, a conveyor's drive."""

    def __init__(self, studio: "Studio", name: str):
        self.studio = studio
        self.name = name

    @property
    def value(self) -> float:
        """Its position (degrees or mm)."""
        return float(self.studio.signals[self.name + ".pos"])

    def move_to(self, value: Number, *, speed: Optional[Number] = None, wait: bool = True) -> Motion:
        """Moves to a position; ``speed`` is a percentage of its speed (default 100)."""
        reply = self.studio._call("robot.moveJoints", _drop_none(
            {"joints": {self.name: float(value)}, "speed": speed, "wait": wait}))
        return Motion(self.studio, reply)

    def move_by(self, amount: Number, *, speed: Optional[Number] = None, wait: bool = True) -> Motion:
        """Moves by an amount from where it is."""
        reply = self.studio._call("robot.moveJoints", _drop_none(
            {"joints": {self.name: float(amount)}, "relative": True, "speed": speed, "wait": wait}))
        return Motion(self.studio, reply)

    def run(self, speed: Number) -> None:
        """Runs at a speed (deg/s or mm/s, negative for reverse) until stopped."""
        self.studio._call("actuator.run", {"name": self.name, "speed": float(speed)})

    def stop(self) -> None:
        """Brakes to a stop."""
        self.studio._call("actuator.run", {"name": self.name, "speed": 0.0})

    def __repr__(self) -> str:
        return f"<Actuator {self.name}>"


class Signals:
    """The automation's signals, like a dict: ``studio.signals["Start"] = True``.

    Reading works for every symbol the automation knows (``Box.count``,
    ``J1.pos`` …); only Input and Internal signals can be written."""

    def __init__(self, studio: "Studio"):
        self._studio = studio

    def __getitem__(self, name: str) -> float:
        return float(self._studio._call("signal.get", {"name": name})["value"])

    def __setitem__(self, name: str, value: Union[bool, Number]) -> None:
        self._studio._call("signal.set", {"name": name, "value": value})

    def __iter__(self) -> Iterator[str]:
        return iter(s["name"] for s in self.list())

    def __contains__(self, name: object) -> bool:
        return any(s["name"] == name for s in self.list())

    def list(self) -> List[Dict[str, Any]]:
        """Every signal with its kind, type, value and whether it can be written."""
        return list(self._studio._call("signal.list"))

    def __repr__(self) -> str:
        return "<Signals " + ", ".join(f"{s['name']}={s['value']:g}" for s in self.list()) + ">"


# ── Studio ──────────────────────────────────────────────────────────────────


class Studio:
    """A connection to MotionStudio. Make one with ``motionstudio.connect()``."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: Optional[float] = 10.0):
        self._conn = _Connection(host, port, timeout)
        hello = self._call("hello", {"client": "python"})
        self.server: str = hello.get("server", "")
        self.version: str = hello.get("version", "")
        if int(hello.get("protocol", 0)) != PROTOCOL:
            self.close()
            raise MotionStudioError("protocol", f"MotionStudio speaks protocol {hello.get('protocol')}, "
                                                f"this library {PROTOCOL}: update one of them")
        self.signals = Signals(self)

    def _call(self, method: str, params: Optional[Mapping[str, Any]] = None) -> Any:
        return self._conn.call(method, params)

    def close(self) -> None:
        """Closes the connection. Motions that are running carry on."""
        self._conn.close()

    def __enter__(self) -> "Studio":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # Scene

    def robots(self) -> List[Robot]:
        """Every robot and machine in the scene."""
        return [Robot(self, r) for r in self._call("scene.robots")]

    def robot(self, name: Optional[str] = None) -> Robot:
        """A robot by name (or path, ``"Cell/Robot"``). Without a name: the only robot."""
        robots = self._call("scene.robots")
        if name is None:
            if len(robots) == 1:
                return Robot(self, robots[0])
            raise NotFoundError("ambiguous" if robots else "not_found",
                                "the scene has no robot" if not robots else
                                "the scene has several robots: pick one of " + ", ".join(r["path"] for r in robots))
        info = self._call("robot.info", {"robot": name})
        return Robot(self, {"name": info["name"], "path": info["path"],
                            "joints": [j["name"] for j in info["joints"]],
                            "tools": [t["name"] for t in info["tools"]]})

    def actuator(self, name: str) -> Actuator:
        """An actuator by name, e.g. a door or a turntable."""
        return Actuator(self, name)

    # Signals

    def get(self, name: str) -> float:
        """A signal's value (``studio.signals[name]``)."""
        return self.signals[name]

    def set(self, name: str, value: Union[bool, Number]) -> None:
        """Sets an Input or Internal signal (``studio.signals[name] = value``)."""
        self.signals[name] = value

    def wait_until(self, condition: str, timeout: Optional[float] = None) -> float:
        """Waits until an expression is true, e.g. ``"PartSensor && !Robot.moving"``.

        Uses MotionStudio's expression language and signal names. ``timeout``
        is in simulation seconds. Returns the simulation time."""
        return float(self._call("signal.waitUntil", _drop_none({"condition": condition, "timeout": timeout}))["time"])

    # Simulation

    def play(self) -> None:
        """Starts or resumes the simulation (moves and signal writes do this themselves)."""
        self._call("sim.play")

    def pause(self) -> None:
        self._call("sim.pause")

    def reset(self) -> None:
        """Stops the simulation and puts everything back where it started."""
        self._call("sim.reset")

    def sleep(self, seconds: Number) -> float:
        """Waits for simulation time to pass; returns the simulation time."""
        return float(self._call("sim.wait", {"seconds": float(seconds)})["time"])

    @property
    def time(self) -> float:
        """Simulation time in seconds."""
        return float(self._call("sim.state")["time"])

    @property
    def running(self) -> bool:
        return bool(self._call("sim.state")["running"])

    @property
    def speed(self) -> float:
        """Simulation speed factor (1 = real time, up to 20)."""
        return float(self._call("sim.state")["speed"])

    @speed.setter
    def speed(self, factor: Number) -> None:
        self._call("sim.speed", {"factor": float(factor)})

    def __repr__(self) -> str:
        return f"<Studio {self.server} {self.version}>"


def connect(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: Optional[float] = 10.0) -> Studio:
    """Connects to MotionStudio (the app with Tools › Robot API on, or ``motionstudio-server``).

    ``timeout`` limits only the connection attempt."""
    return Studio(host, port, timeout)
