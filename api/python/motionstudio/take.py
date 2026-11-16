"""Takes: what a robot did, sampled into rows of numbers.

A take comes from MotionStudio's **Record Values** button (a CSV file) or from
a program (``robot.record()``). Each row holds one number per column, in
millimetres, degrees and seconds:

- ``t``: seconds since the take started
- ``x y z rx ry rz``: the robot's top object in the world
- for a robot with legs: ``heading walking step swing progress goal_x goal_z
  goal_heading`` (where it faces; 1 while it walks; the step under way, -1
  none; the foot in the air, 0 left, 1 right, -1 none; how far through the
  step, 0..1; where the walk ends)
- then one column per joint, named after its actuator

No dependencies; ``Take.array()`` gives a NumPy array when NumPy is installed.
"""

from typing import Any, Dict, Iterator, List, Optional, Sequence, Union
import os

#: Columns that are not joints
PLACE_COLUMNS = ("t", "x", "y", "z", "rx", "ry", "rz")
WALK_COLUMNS = ("heading", "walking", "step", "swing", "progress", "goal_x", "goal_z", "goal_heading")
META_COLUMNS = PLACE_COLUMNS + WALK_COLUMNS


class Take:
    """Rows of numbers under named columns, with the robot's name and the sample rate."""

    def __init__(self, columns: Sequence[str], rows: Sequence[Sequence[float]], robot: str = "",
                 rate: float = 0.0, meta: Optional[Dict[str, str]] = None):
        self.columns: List[str] = list(columns)
        self.rows: List[List[float]] = [list(map(float, r)) for r in rows]
        self.robot = robot
        self.rate = float(rate)
        self.meta: Dict[str, str] = dict(meta or {})
        for i, r in enumerate(self.rows):
            if len(r) != len(self.columns):
                raise ValueError(f"row {i} has {len(r)} values for {len(self.columns)} columns")

    # ── Reading ──────────────────────────────────────────────────────────────

    def __len__(self) -> int:
        return len(self.rows)

    def __iter__(self) -> Iterator[Dict[str, float]]:
        for i in range(len(self.rows)):
            yield self.frame(i)

    @property
    def duration(self) -> float:
        """Seconds from the first row to the last."""
        return self.rows[-1][0] - self.rows[0][0] if self.rows else 0.0

    @property
    def joint_names(self) -> List[str]:
        """The joint columns (actuator names), in order."""
        return [c for c in self.columns if c not in META_COLUMNS]

    @property
    def walks(self) -> bool:
        """Whether it was recorded from a robot with legs."""
        return "walking" in self.columns

    def index(self, column: str) -> int:
        try:
            return self.columns.index(column)
        except ValueError:
            raise KeyError(f"no column {column!r}; the take has {', '.join(self.columns)}") from None

    def column(self, name: str) -> List[float]:
        """Every row's value of one column."""
        k = self.index(name)
        return [r[k] for r in self.rows]

    def __getitem__(self, name: str) -> List[float]:
        return self.column(name)

    def frame(self, i: int) -> Dict[str, float]:
        """Row ``i`` as a dict, column → value."""
        return dict(zip(self.columns, self.rows[i]))

    def joints(self, i: int) -> Dict[str, float]:
        """Row ``i``'s joint values, by actuator name."""
        return {c: v for c, v in zip(self.columns, self.rows[i]) if c not in META_COLUMNS}

    def array(self, columns: Optional[Sequence[str]] = None):
        """The rows as a NumPy array (rows × columns), optionally only some columns."""
        import numpy as np
        a = np.asarray(self.rows, dtype=float).reshape(len(self.rows), len(self.columns))
        return a if columns is None else a[:, [self.index(c) for c in columns]]

    # ── Making new takes ─────────────────────────────────────────────────────

    def between(self, t0: float, t1: float) -> "Take":
        """The rows from ``t0`` to ``t1`` seconds, with ``t`` starting at 0 again."""
        rows = [list(r) for r in self.rows if t0 - 1e-9 <= r[0] <= t1 + 1e-9]
        if rows:
            start = rows[0][0]
            for r in rows:
                r[0] -= start
        return Take(self.columns, rows, self.robot, self.rate, self.meta)

    def select(self, columns: Sequence[str]) -> "Take":
        """Only these columns (``t`` is kept first)."""
        cols = ["t"] + [c for c in columns if c != "t"]
        ks = [self.index(c) for c in cols]
        return Take(cols, [[r[k] for k in ks] for r in self.rows], self.robot, self.rate, self.meta)

    # ── Files ────────────────────────────────────────────────────────────────

    def to_csv(self) -> str:
        def num(v: float) -> str:
            s = f"{round(v, 3):.3f}".rstrip("0").rstrip(".")
            return "0" if s in ("-0", "") else s
        lines = ["# MotionStudio take", f"# robot: {self.robot}", f"# rate: {num(self.rate)}",
                 "# units: mm, degrees, seconds"]
        lines += [f"# {k}: {v}" for k, v in self.meta.items() if k not in ("robot", "rate", "units")]
        lines.append(",".join(self.columns))
        lines += [",".join(num(v) for v in r) for r in self.rows]
        return "\n".join(lines) + "\n"

    def save(self, path: Union[str, "os.PathLike[str]"]) -> None:
        """Writes the take as CSV (the same format as MotionStudio's Record Values)."""
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.to_csv())

    @classmethod
    def from_csv(cls, text: str) -> "Take":
        meta: Dict[str, str] = {}
        columns: Optional[List[str]] = None
        rows: List[List[float]] = []
        for n, line in enumerate(text.splitlines(), 1):
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                key, sep, value = line[1:].partition(":")
                if sep:
                    meta[key.strip()] = value.strip()
                continue
            if columns is None:
                columns = [c.strip() for c in line.split(",")]
                continue
            try:
                rows.append([float(v) for v in line.split(",")])
            except ValueError:
                raise ValueError(f"line {n}: not a row of numbers") from None
        if columns is None:
            raise ValueError("not a take: there is no header row")
        rate = float(meta.get("rate", 0) or 0)
        robot = meta.get("robot", "")
        return cls(columns, rows, robot, rate, {k: v for k, v in meta.items() if k not in ("robot", "rate", "units")})

    @classmethod
    def load(cls, path: Union[str, "os.PathLike[str]"]) -> "Take":
        """Reads a take saved by Record Values or ``Take.save``."""
        with open(path, encoding="utf-8") as f:
            return cls.from_csv(f.read())

    def __repr__(self) -> str:
        return (f"<Take {self.robot or '?'}: {len(self.rows)} rows, {self.duration:.2f} s, "
                f"{len(self.joint_names)} joints>")


class Recording:
    """A take being recorded (``robot.record()``). Stop it to get the take.

    As a context manager it stops when the block ends::

        with walker.record() as rec:
            walker.walk_to(2000, 0)
        rec.take.save("walk.csv")
    """

    def __init__(self, studio: Any, robot: str, reply: Dict[str, Any]):
        self.studio = studio
        self.robot = robot
        self.id: int = int(reply["take"])
        self.columns: List[str] = list(reply.get("columns", []))
        self.take: Optional[Take] = None

    @property
    def frames(self) -> int:
        """Rows recorded so far."""
        if self.take is not None:
            return len(self.take)
        return int(self.studio._call("take.status", {"take": self.id})["frames"])

    def stop(self) -> Take:
        """Stops recording and returns the take (again on later calls)."""
        if self.take is None:
            r = self.studio._call("take.stop", {"take": self.id})
            self.take = Take(r["columns"], r["rows"], r.get("robot", self.robot), r.get("rate", 0))
        return self.take

    def __enter__(self) -> "Recording":
        return self

    def __exit__(self, *exc: Any) -> None:
        try:
            self.stop()
        except Exception:
            if exc[0] is None:
                raise

    def __repr__(self) -> str:
        return f"<Recording {self.id} of {self.robot}{' (stopped)' if self.take is not None else ''}>"
