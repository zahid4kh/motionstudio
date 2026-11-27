"""Building scenes: objects, joints, physics, signals, sensors, poses, sequences
and part flow, plus opening, saving and importing models.

    import motionstudio as ms

    with ms.connect() as studio:
        scene = studio.scene
        scene.new()
        cell = scene.add_frame("Cell")
        table = scene.add_box("Table", size=(800, 40, 600), parent=cell, position=ms.Pose(y=700))
        arm = scene.add_frame("Arm", parent=cell, position=ms.Pose(x=300, y=800))
        scene.add_joint("Turn", parent=cell, child=arm, axis=(0, 1, 0), min=-170, max=170)
        scene.save("cell.mstudio")

Lengths are mm and angles degrees. Positions are in the parent's frame unless
``frame`` says otherwise (``"world"`` or any object). A number given for an
automation value (a Move target, a belt speed) is in mm, mm/s or degrees too;
a string is an expression in the automation's own units (m, m/s, degrees), as
typed in the app: ``target="Depth - 0.1"``.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple, Union

from .client import Number, Pose, _drop_none

if TYPE_CHECKING:
    from .client import Studio

Vector = Union["Vec3", Sequence[Number], Mapping[str, Number]]
ObjectRef = Union[str, "SceneObject"]
Value = Union[Number, str, bool]          # a number in API units, or an expression


# ── Values ──────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Vec3:
    """A point or a direction (mm for points)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __iter__(self) -> Iterator[float]:
        return iter((self.x, self.y, self.z))

    def __add__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, k: Number) -> "Vec3":
        return Vec3(self.x * k, self.y * k, self.z * k)

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Vec3":
        return cls(float(d.get("x", 0)), float(d.get("y", 0)), float(d.get("z", 0)))

    def __str__(self) -> str:
        return "Vec3({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z)


@dataclass(frozen=True)
class Box:
    """An axis-aligned box in some frame (mm)."""

    min: Vec3
    max: Vec3
    center: Vec3
    size: Vec3


@dataclass(frozen=True)
class Feature:
    """A round edge, a round face, a straight edge or a flat face of a CAD part:
    where a joint's axis and pivot can come from. ``kind`` is ``circle``,
    ``cylinder``, ``line`` or ``plane``; ``origin`` is the centre (of the
    circle, of the cylinder's middle, of the edge, of the face) and ``axis``
    its direction (a plane's normal). ``radius`` for circles and cylinders,
    ``length`` for cylinders and lines."""

    object: str
    kind: str
    origin: Vec3
    axis: Vec3
    radius: Optional[float] = None
    length: Optional[float] = None


def _vec(v: Optional[Vector]) -> Any:
    if v is None:
        return None
    if isinstance(v, Vec3):
        return [v.x, v.y, v.z]
    if isinstance(v, Mapping):
        return {k: float(val) for k, val in v.items()}
    values = [float(x) for x in v]
    if len(values) != 3:
        raise ValueError("a vector needs three numbers: (x, y, z)")
    return values


def _pose(p: Union[Pose, Mapping[str, Number], Sequence[Number], None]) -> Any:
    if p is None:
        return None
    if isinstance(p, Pose):
        return p.to_dict()
    if isinstance(p, Mapping):
        return {k: float(v) for k, v in p.items()}
    values = [float(x) for x in p]
    if len(values) not in (3, 6):
        raise ValueError("a position is (x, y, z) or (x, y, z, rx, ry, rz)")
    return dict(zip(("x", "y", "z", "rx", "ry", "rz"), values))


def _name(o: Optional[ObjectRef]) -> Optional[str]:
    if o is None:
        return None
    return o.ref if isinstance(o, SceneObject) else str(o)


def _size(size: Union[Number, Vector, None]) -> Any:
    if size is None or isinstance(size, (int, float)):
        return size
    return _vec(size)


def _path(p: Union[str, "os.PathLike[str]"]) -> str:
    return os.path.abspath(os.fspath(p))


# ── Objects and joints ──────────────────────────────────────────────────────


class SceneObject:
    """An object in the scene. Its methods change it in MotionStudio. It is
    found by its id (``ref``, ``"#12"``), so it stays the same object when it
    is renamed or moved; ids last until the scene is opened again."""

    def __init__(self, scene: "Scene", info: Mapping[str, Any]):
        self.scene = scene
        self._info: Dict[str, Any] = dict(info)
        self.id: int = int(info["id"])
        self.ref: str = f"#{self.id}"
        self.path: str = info["path"]

    def _update(self, info: Mapping[str, Any]) -> "SceneObject":
        self._info = dict(info)
        self.path = info["path"]
        return self

    def info(self) -> Dict[str, Any]:
        """Everything about it, fresh: type, parent, children, position, world, size, colour, body, belt, joint."""
        return self._update(self.scene._call("object.get", {"object": self.ref}))._info

    @property
    def name(self) -> str:
        return self._info["name"]

    @property
    def type(self) -> str:
        """``frame``, ``mesh``, ``box``, ``cylinder``, ``sphere``, ``capsule`` or ``basket``."""
        return self._info["type"]

    @property
    def parent(self) -> Optional["SceneObject"]:
        p = self.info().get("parent")
        return self.scene.object(p) if p else None

    @property
    def children(self) -> List["SceneObject"]:
        return self.scene.objects(under=self, depth=1)

    @property
    def position(self) -> Pose:
        """Where it is in its parent's frame (the world for a top object)."""
        return Pose.from_dict(self.info()["position"])

    @property
    def world(self) -> Pose:
        """Where it is in the world."""
        return Pose.from_dict(self.info()["world"])

    @property
    def size(self) -> Optional[Vec3]:
        """A box's, cylinder's or sphere's size in mm (None for frames and meshes)."""
        s = self.info().get("size")
        return Vec3.from_dict(s) if s else None

    def pose_in(self, frame: ObjectRef) -> Pose:
        """Where it is in another object's frame (or ``"world"``)."""
        return Pose.from_dict(self.scene._call("object.get", {"object": self.ref, "frame": _name(frame)})["inFrame"])

    def set(self, *, name: Optional[str] = None, parent: Union[ObjectRef, None, bool] = False,
            position: Union[Pose, Mapping[str, Number], Sequence[Number], None] = None, frame: Optional[ObjectRef] = None,
            size: Union[Number, Vector, None] = None, diameter: Optional[Number] = None, height: Optional[Number] = None,
            color: Union[str, None, bool] = False, opacity: Optional[Number] = None,
            hidden: Optional[bool] = None) -> "SceneObject":
        """Changes it. ``position`` may be partial (``{"z": 100}``) and is in its
        parent's frame unless ``frame`` is given; ``parent=None`` moves it to the
        top (keeping where it is); ``color=None`` makes it take its parent's."""
        params: Dict[str, Any] = _drop_none({"object": self.ref, "name": name, "position": _pose(position),
                                             "frame": _name(frame), "size": _size(size), "diameter": diameter,
                                             "height": height, "opacity": opacity, "hidden": hidden})
        if parent is not False:
            params["parent"] = _name(parent)   # type: ignore[arg-type]
        if color is not False:
            params["color"] = color
        return self._update(self.scene._call("object.set", params))

    def move_to(self, position: Union[Pose, Mapping[str, Number], Sequence[Number], None] = None, *,
                frame: Optional[ObjectRef] = None, **values: Number) -> "SceneObject":
        """Puts it somewhere: ``obj.move_to(x=100, z=-50)`` (the rest stays) or a Pose."""
        p = dict(_pose(position) or {})
        p.update({k: float(v) for k, v in values.items()})
        return self.set(position=p, frame=frame)

    def rename(self, name: str) -> "SceneObject":
        return self.set(name=name)

    def set_parent(self, parent: Optional[ObjectRef]) -> "SceneObject":
        """Hangs it under another object (None: the top), keeping where it is and its joint."""
        return self.set(parent=parent)

    def add(self, type: str = "frame", name: Optional[str] = None, **kwargs: Any) -> "SceneObject":
        """Adds a child: ``obj.add("box", "Plate", size=(100, 10, 100))``."""
        return self.scene.add(type, name, parent=self, **kwargs)

    def remove(self) -> int:
        """Removes it and everything below it; returns how many objects went."""
        return int(self.scene._call("object.remove", {"object": self.ref})["removed"])

    def duplicate(self, name: Optional[str] = None, **changes: Any) -> "SceneObject":
        """A copy with its children, joints and actuators (renamed)."""
        params = {"object": self.ref, "name": name}
        if "position" in changes:
            changes["position"] = _pose(changes["position"])
        if "frame" in changes:
            changes["frame"] = _name(changes["frame"])
        if "parent" in changes:
            changes["parent"] = _name(changes["parent"])
        params.update(changes)
        return SceneObject(self.scene, self.scene._call("object.duplicate", _drop_none(params)))

    def bounds(self, frame: ObjectRef = "world", *, children: bool = True) -> Box:
        """Its box (with its children's, unless ``children=False``) in a frame, mm."""
        b = self.scene._call("object.bounds", {"object": self.ref, "frame": _name(frame), "children": children})
        return Box(*(Vec3.from_dict(b[k]) for k in ("min", "max", "center", "size")))

    def features(self, kinds: Iterable[str] = ("circle", "cylinder"), *, frame: ObjectRef = "world",
                 min_radius: Optional[Number] = None, max_radius: Optional[Number] = None,
                 children: bool = True, limit: int = 500) -> List[Feature]:
        """The round edges and faces (or ``line``, ``plane``) of its CAD geometry,
        e.g. to put a joint's axis through a bearing or a shaft."""
        r = self.scene._call("object.features", _drop_none({
            "object": self.ref, "kinds": list(kinds), "frame": _name(frame), "minRadius": min_radius,
            "maxRadius": max_radius, "children": children, "limit": limit}))
        return [Feature(f["object"], f["kind"], Vec3.from_dict(f["origin"]), Vec3.from_dict(f["axis"]),
                        f.get("radius"), f.get("length")) for f in r["features"]]

    def set_body(self, mass: Optional[Number] = None, *, gravity: Optional[bool] = None,
                 collision: Optional[bool] = None) -> "SceneObject":
        """Makes it a rigid body (physics: it falls, rides belts, is gripped); mass in kg."""
        return self._update(self.scene._call("body.set", _drop_none(
            {"object": self.ref, "mass": mass, "gravity": gravity, "collision": collision})))

    def remove_body(self) -> "SceneObject":
        return self._update(self.scene._call("body.set", {"object": self.ref, "remove": True}))

    def set_belt(self, speed: Optional[Value] = None, direction: Optional[Vector] = None, *,
                 frame: Optional[ObjectRef] = None, enabled: Optional[bool] = None) -> "SceneObject":
        """Makes its top surface a conveyor belt. ``speed`` in mm/s, or an
        expression in m/s (``"Run ? 0.2 : 0"``); ``direction`` in its own frame
        unless ``frame`` says."""
        return self._update(self.scene._call("belt.set", _drop_none(
            {"object": self.ref, "speed": speed, "direction": _vec(direction), "frame": _name(frame), "enabled": enabled})))

    def remove_belt(self) -> "SceneObject":
        return self._update(self.scene._call("belt.set", {"object": self.ref, "remove": True}))

    def __repr__(self) -> str:
        return f"<SceneObject {self.path} ({self._info.get('type')})>"

    def __eq__(self, other: object) -> bool:
        return isinstance(other, SceneObject) and other.id == self.id

    def __hash__(self) -> int:
        return hash(self.id)


class Joint:
    """A joint by name (or its actuator's name)."""

    def __init__(self, scene: "Scene", info: Mapping[str, Any]):
        self.scene = scene
        self._info: Dict[str, Any] = dict(info)
        self.name: str = info["name"]

    def info(self, frame: ObjectRef = "world") -> Dict[str, Any]:
        """Type, parent, child, axis and pivot (in ``frame``), range, value, actuator, follows."""
        self._info = self.scene._call("joint.get", {"joint": self.name, "frame": _name(frame)})
        self.name = self._info["name"]
        return self._info

    @property
    def actuator(self) -> Optional[str]:
        a = self.info().get("actuator")
        return a["name"] if a else None

    @property
    def value(self) -> float:
        """Its value (degrees, or mm for a prismatic joint)."""
        return float(self.info()["value"])

    @value.setter
    def value(self, v: Number) -> None:
        self.set(value=v)

    def set(self, **changes: Any) -> "Joint":
        """Changes it: ``name``, ``axis``, ``pivot``, ``frame``, ``min``, ``max``,
        ``continuous``, ``value``, ``actuator`` (True / False / dict), ``follows``."""
        params = dict(changes)
        for k in ("axis", "pivot"):
            if k in params:
                params[k] = _vec(params[k])
        if "frame" in params:
            params["frame"] = _name(params["frame"])
        if "follows" in params and isinstance(params["follows"], Joint):
            params["follows"] = params["follows"].name
        params["joint"] = self.name
        self._info = self.scene._call("joint.set", params)
        self.name = self._info["name"]
        return self

    def remove(self) -> None:
        """Removes it (and its actuator); the part stays where it is now."""
        self.scene._call("joint.remove", {"joint": self.name})

    def __repr__(self) -> str:
        return f"<Joint {self.name} ({self._info.get('type')})>"


# ── Sequences ───────────────────────────────────────────────────────────────


class Ops:
    """The operations of a sequence, in order. Each runs after the one before
    unless ``after`` names others (names or 0-based indices; ``[]`` for none).

        ops = (ms.Ops()
               .move_to_pose("Above pick", "Above Pick", speed=100)
               .wait_for("Part there", "PartAtPick")
               .move("Down", "J3", 120)               # 120 mm
               .grip("Suck", "TCP", sensor="AtCup"))
        scene.add_sequence("Robot", ops, loop=True, enable="Start")

    Every operation also takes ``on_start`` and ``on_finish`` (actions such as
    ``"Count = Count + 1; Busy = true"``) and ``start_condition``."""

    def __init__(self) -> None:
        self.operations: List[Dict[str, Any]] = []

    def add(self, type: str, name: Optional[str] = None, *, after: Union[None, str, int, Sequence[Union[str, int]]] = None,
            on_start: Optional[str] = None, on_finish: Optional[str] = None, start_condition: Optional[str] = None,
            **fields: Any) -> "Ops":
        """Any operation type with its fields as the protocol names them."""
        op: Dict[str, Any] = {"type": type, "name": name, "onStart": on_start, "onFinish": on_finish,
                              "startCondition": start_condition}
        op.update(fields)
        op = _drop_none(op)
        if after is not None:
            op["after"] = [after] if isinstance(after, (str, int)) else list(after)
        self.operations.append(op)
        return self

    def move(self, name: str, actuator: str, target: Value, *, speed: Optional[Value] = None, **common: Any) -> "Ops":
        """An actuator to a position (degrees or mm); done when it is there."""
        return self.add("move", name, actuator=actuator, target=target, speed=speed, **common)

    def move_to_pose(self, name: str, pose: str, *, speed: Optional[Value] = None, part: Optional[ObjectRef] = None,
                     sensor: Optional[str] = None, **common: Any) -> "Ops":
        """Every joint of a pose at once; ``speed`` in % of the actuators' speeds."""
        return self.add("moveToPose", name, pose=pose, speed=speed, part=_name(part), sensor=sensor, **common)

    def run(self, name: str, actuator: str, speed: Value, **common: Any) -> "Ops":
        """An actuator runs at a speed (deg/s or mm/s) until stopped; done at once."""
        return self.add("run", name, actuator=actuator, speed=speed, **common)

    def stop(self, name: str, actuator: str, **common: Any) -> "Ops":
        return self.add("stop", name, actuator=actuator, **common)

    def set(self, name: str, actions: Optional[str] = None, **common: Any) -> "Ops":
        """Only its actions: ``ops.set("Reset", "Count = 0; Busy = false")``."""
        if actions is not None:
            common["on_start"] = actions
        return self.add("set", name, **common)

    def wait(self, name: str, seconds: Value, **common: Any) -> "Ops":
        return self.add("wait", name, seconds=seconds, **common)

    def wait_for(self, name: str, condition: str, **common: Any) -> "Ops":
        """Done when the condition (an expression) is true."""
        return self.add("waitFor", name, condition=condition, **common)

    def grip(self, name: str, gripper: ObjectRef, *, sensor: Optional[str] = None, part: Optional[ObjectRef] = None,
             **common: Any) -> "Ops":
        """The gripper holds what the sensor sees (or the part)."""
        return self.add("grip", name, gripper=_name(gripper), sensor=sensor, part=_name(part), **common)

    def release(self, name: str, gripper: ObjectRef, **common: Any) -> "Ops":
        return self.add("release", name, gripper=_name(gripper), **common)

    def move_l(self, name: str, tool: ObjectRef, pose: str, *, speed: Optional[Value] = None,
               part: Optional[ObjectRef] = None, sensor: Optional[str] = None, **common: Any) -> "Ops":
        """The tool in a straight line to where it is in a pose; ``speed`` in mm/s."""
        return self.add("moveL", name, tool=_name(tool), pose=pose, speed=speed, part=_name(part), sensor=sensor, **common)

    def walk(self, name: str, robot: ObjectRef, *, forward: Optional[Value] = None, sideways: Optional[Value] = None,
             turn: Optional[Value] = None, to: Optional[ObjectRef] = None,
             route: Optional[Sequence[Sequence[Number]]] = None, heading: Optional[Value] = None,
             speed: Optional[Value] = None, **common: Any) -> "Ops":
        """A legged robot walks by distances (mm) and a turn (degrees), to an
        object, or through ``route`` points [x, z] (mm)."""
        return self.add("walk", name, robot=_name(robot), forward=forward, sideways=sideways, turn=turn, to=_name(to),
                        route=[list(map(float, p)) for p in route] if route else None, heading=heading,
                        speed=speed, **common)

    def play_motion(self, name: str, motion: str, *, robot: Optional[ObjectRef] = None, relative: Optional[bool] = None,
                    speed: Optional[Value] = None, clip: Optional[str] = None, blend_clip: Optional[str] = None,
                    blend: Optional[Value] = None, seconds: Optional[Value] = None, **common: Any) -> "Ops":
        return self.add("playMotion", name, motion=motion, robot=_name(robot), relative=relative, speed=speed,
                        clip=clip, blendClip=blend_clip, blend=blend, seconds=seconds, **common)

    def __len__(self) -> int:
        return len(self.operations)


# ── Scene ───────────────────────────────────────────────────────────────────


class Scene:
    """The scene: ``studio.scene``. Changes need the simulation reset
    (``studio.reset()``); they show in the app at once and can be undone there."""

    def __init__(self, studio: "Studio"):
        self.studio = studio

    def _call(self, method: str, params: Optional[Mapping[str, Any]] = None) -> Any:
        return self.studio._call(method, params)

    # Projects

    def new(self, *, discard: bool = False) -> None:
        """An empty scene. In the app, unsaved changes need ``discard=True``."""
        self._call("scene.new", {"discard": discard})

    def open(self, path: Union[str, "os.PathLike[str]"], *, discard: bool = False) -> Dict[str, Any]:
        """Opens a .mstudio project in place of the scene; returns ``info()``."""
        return dict(self._call("scene.open", {"path": _path(path), "discard": discard}))

    def save(self, path: Union[str, "os.PathLike[str]", None] = None) -> str:
        """Saves the scene (to the file it came from when no path is given); returns the path."""
        return str(self._call("scene.save", _drop_none({"path": _path(path) if path is not None else None}))["path"])

    def info(self) -> Dict[str, Any]:
        """What the scene holds: top objects, robots, joints, signals, sensors,
        sequences, poses, sources, sinks and motions, by name."""
        return dict(self._call("scene.info"))

    def check(self) -> List[str]:
        """The automation's problems (what would stop Play); empty when it can run."""
        return list(self._call("scene.check")["problems"])

    def import_file(self, path: Union[str, "os.PathLike[str]"], *, parent: Optional[ObjectRef] = None,
                    name: Optional[str] = None, up_axis: Optional[str] = None) -> SceneObject:
        """Imports a STEP (.step/.stp), URDF, MJCF (.xml) or MotionStudio project
        (merged in). STEP models are taken to be Z-up (most CAD) and stood
        upright unless ``up_axis="y"``. Returns the new top object."""
        r = self._call("scene.import", _drop_none({"path": _path(path), "parent": _name(parent), "name": name,
                                                   "upAxis": up_axis}))
        return SceneObject(self, r["root"])

    # Objects

    def objects(self, name: Optional[str] = None, *, under: Optional[ObjectRef] = None, type: Optional[str] = None,
                depth: Optional[int] = None) -> List[SceneObject]:
        """Objects in tree order, all or those whose name matches a wildcard
        (``"legs:*"``), below ``under``, of a ``type``, down to ``depth`` levels."""
        found = self._call("scene.objects", _drop_none({"name": name, "under": _name(under), "type": type, "depth": depth}))
        return [SceneObject(self, o) for o in found]

    def object(self, name: ObjectRef) -> SceneObject:
        """An object by name or path (``"Station/Sorter/Lift"``)."""
        return SceneObject(self, self._call("object.get", {"object": _name(name)}))

    def exists(self, name: str) -> bool:
        from .client import NotFoundError
        try:
            self._call("object.get", {"object": name})
            return True
        except NotFoundError:
            return False

    def add(self, type: str = "frame", name: Optional[str] = None, *, parent: Optional[ObjectRef] = None,
            position: Union[Pose, Mapping[str, Number], Sequence[Number], None] = None, frame: Optional[ObjectRef] = None,
            size: Union[Number, Vector, None] = None, diameter: Optional[Number] = None, height: Optional[Number] = None,
            color: Optional[str] = None, opacity: Optional[Number] = None, hidden: Optional[bool] = None) -> SceneObject:
        """Adds an object: a ``frame`` (an empty, unscaled frame to build on and
        hang joints from), ``box``, ``cylinder``, ``sphere``, ``capsule`` or
        ``basket``. Sizes in mm (default 100); ``color`` as ``"#rrggbb"``."""
        return SceneObject(self, self._call("object.add", _drop_none({
            "type": type, "name": name, "parent": _name(parent), "position": _pose(position), "frame": _name(frame),
            "size": _size(size), "diameter": diameter, "height": height, "color": color, "opacity": opacity,
            "hidden": hidden})))

    def add_frame(self, name: str, **kwargs: Any) -> SceneObject:
        return self.add("frame", name, **kwargs)

    def add_box(self, name: str, size: Union[Number, Vector] = 100, **kwargs: Any) -> SceneObject:
        return self.add("box", name, size=size, **kwargs)

    def add_cylinder(self, name: str, diameter: Number = 100, height: Number = 100, **kwargs: Any) -> SceneObject:
        """A cylinder standing along its own Y axis."""
        return self.add("cylinder", name, diameter=diameter, height=height, **kwargs)

    def add_sphere(self, name: str, diameter: Number = 100, **kwargs: Any) -> SceneObject:
        return self.add("sphere", name, size=diameter, **kwargs)

    def group(self, objects: Iterable[ObjectRef], name: str = "Group") -> SceneObject:
        """A new frame at the middle of the objects, with them under it (nothing moves)."""
        return SceneObject(self, self._call("object.group", {"objects": [_name(o) for o in objects], "name": name}))

    # Joints

    def add_joint(self, name: Optional[str], parent: ObjectRef, child: ObjectRef, axis: Vector, *,
                  type: str = "revolute", pivot: Optional[Vector] = None, frame: Optional[ObjectRef] = None,
                  min: Optional[Number] = None, max: Optional[Number] = None, continuous: Optional[bool] = None,
                  value: Optional[Number] = None, actuator: Union[bool, Mapping[str, Any], None] = None,
                  follows: Union[str, Joint, Mapping[str, Any], None] = None) -> Joint:
        """A joint that moves ``child`` on ``parent`` (the child goes under the
        parent if it isn't). ``axis`` and ``pivot`` (mm; default the child's
        origin) are in the world unless ``frame`` says. ``min``/``max`` in
        degrees, or mm for ``type="prismatic"`` (needed then). The joint gets
        an actuator of its name unless ``actuator=False`` or it ``follows``
        another joint; ``actuator`` may give ``name``, ``speed``,
        ``acceleration``, ``mode`` ("position"/"speed"), ``target`` (an
        expression binding)."""
        if isinstance(follows, Joint):
            follows = follows.name
        return Joint(self, self._call("joint.add", _drop_none({
            "name": name, "parent": _name(parent), "child": _name(child), "axis": _vec(axis), "type": type,
            "pivot": _vec(pivot), "frame": _name(frame), "min": min, "max": max, "continuous": continuous,
            "value": value, "actuator": actuator, "follows": follows})))

    def joint(self, name: str) -> Joint:
        """A joint by its name, its actuator's name or the object it moves."""
        return Joint(self, self._call("joint.get", {"joint": name}))

    # Automation

    def add_signal(self, name: str, kind: str = "internal", type: str = "bool", value: Union[Number, bool] = 0, *,
                   expression: Optional[str] = None, description: Optional[str] = None,
                   address: Optional[int] = None) -> Dict[str, Any]:
        """A signal: ``input`` (set from outside: Runtime panel, PLC, this API),
        ``internal`` (set by operations) or ``derived`` (an ``expression``);
        ``bool``, ``int`` or ``real``. ``address`` exposes it over Modbus TCP."""
        return dict(self._call("signal.add", _drop_none({
            "name": name, "kind": kind, "type": type, "value": value, "expression": expression,
            "description": description, "address": address})))

    def add_sensor(self, name: str, type: str, *, mount: Optional[ObjectRef] = None, at: Optional[Vector] = None,
                   direction: Optional[Vector] = None, range: Optional[Number] = None, size: Optional[Vector] = None,
                   bounds: Optional[bool] = None, frame: Optional[ObjectRef] = None, detects: Optional[Union[str, SceneObject]] = None,
                   description: Optional[str] = None) -> Dict[str, Any]:
        """A sensor signal: ``beam`` (a light barrier from ``at`` along
        ``direction`` up to ``range`` mm), ``distance`` (the same, its value is
        the distance in m), ``zone`` (a box of ``size`` at ``at``, or the
        mount's box with ``bounds=True``) or ``position`` (a zone that also
        tells where the part is). It moves with ``mount``; ``at``, ``direction``
        and ``size`` are in the mount's frame unless ``frame`` says.
        ``detects``: ``"any"``, ``"bodies"`` (rigid bodies) or an object."""
        return dict(self._call("sensor.add", _drop_none({
            "name": name, "type": type, "mount": _name(mount), "at": _vec(at), "direction": _vec(direction),
            "range": range, "size": _vec(size), "bounds": bounds, "frame": _name(frame),
            "detects": _name(detects) if isinstance(detects, SceneObject) else detects, "description": description})))

    def add_pose(self, name: str, joints: Optional[Mapping[str, Number]] = None, *, robot: Optional[ObjectRef] = None,
                 replace: bool = False) -> Dict[str, Any]:
        """A pose: actuator values (degrees or mm), or where ``robot`` stands now."""
        return dict(self._call("pose.add", _drop_none({
            "name": name, "joints": dict(joints) if joints is not None else None, "robot": _name(robot),
            "replace": replace})))

    def add_sequence(self, name: str, operations: Union[Ops, Sequence[Mapping[str, Any]]], *, loop: bool = False,
                     enable: Optional[str] = None, start: Optional[str] = None, replace: bool = False) -> Dict[str, Any]:
        """A sequence of operations (an ``Ops`` or a list of protocol dicts).
        ``loop`` starts it again after each cycle; ``enable`` pauses it while
        false; ``start`` gates each cycle's start. Returns ``sequence()``."""
        ops = operations.operations if isinstance(operations, Ops) else [dict(o) for o in operations]
        return dict(self._call("sequence.add", _drop_none({
            "name": name, "operations": ops, "loop": loop, "enable": enable, "start": start, "replace": replace})))

    def sequence(self, name: str) -> Dict[str, Any]:
        """A sequence as stored: its operations with their predecessors as indices
        and their values as expressions (the automation's units)."""
        return dict(self._call("sequence.get", {"name": name}))

    def add_source(self, name: str, template: ObjectRef, *, interval: Optional[Value] = None,
                   trigger: Optional[str] = None, enable: Optional[str] = None, max: Optional[int] = None,
                   when_clear: Optional[bool] = None) -> Dict[str, Any]:
        """Makes copies of ``template`` while simulating: every ``interval``
        seconds, or each time ``trigger`` becomes true. The template itself
        is out of the run."""
        return dict(self._call("source.add", _drop_none({
            "name": name, "template": _name(template), "interval": interval, "trigger": trigger,
            "mode": "trigger" if trigger else None, "enable": enable, "max": max, "whenClear": when_clear})))

    def add_sink(self, name: str, *, mount: Optional[ObjectRef] = None, at: Optional[Vector] = None,
                 size: Optional[Vector] = None, bounds: Optional[bool] = None, frame: Optional[ObjectRef] = None,
                 source: Optional[str] = None, enable: Optional[str] = None) -> Dict[str, Any]:
        """Takes away the parts that touch its zone (a box like a zone sensor's)."""
        return dict(self._call("sink.add", _drop_none({
            "name": name, "mount": _name(mount), "at": _vec(at), "size": _vec(size), "bounds": bounds,
            "frame": _name(frame), "source": source, "enable": enable})))

    def remove(self, name: str) -> str:
        """Removes a signal, sensor, sequence, source, sink, pose or motion by name; says which it was."""
        return str(self._call("automation.remove", {"name": name})["removed"])

    def __repr__(self) -> str:
        return "<Scene>"
