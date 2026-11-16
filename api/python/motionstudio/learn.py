"""Learning motion from takes (needs NumPy: ``pip install numpy``).

Record what a robot does (MotionStudio's **Record Values**, or
``robot.record()``), then teach a model to move like it:

- ``AnimationModel`` learns animations and poses: each take becomes a clip
  that the model can play at any length, or blend with others.
- ``WalkPolicy`` learns to walk: from takes of a humanoid walking to goals it
  learns, frame by frame, how the joints and the body move towards a goal.
  It then walks to goals it was never shown, on its own, and its motion plays
  back in MotionStudio with ``robot.play(take)``.

Both are small neural networks (``MLP``) trained with NumPy only, so they run
anywhere; nothing here talks to MotionStudio except ``WalkPolicy.walk_to``'s
``start=robot``.
"""

from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union
import math

try:
    import numpy as np
except ImportError as e:  # pragma: no cover
    raise ImportError("motionstudio.learn needs NumPy: pip install numpy") from e

from .take import META_COLUMNS, Take

__all__ = ["MLP", "AnimationModel", "WalkPolicy", "walk_steps"]


# ── A small neural network ───────────────────────────────────────────────────

class MLP:
    """A fully connected network (tanh hidden layers, linear output) trained
    with Adam on the mean squared error. Inputs and outputs are standardized
    inside, so give it raw numbers."""

    def __init__(self, n_in: int, n_out: int, hidden: Sequence[int] = (256, 256), seed: int = 0):
        rng = np.random.default_rng(seed)
        sizes = [n_in, *hidden, n_out]
        self.W = [rng.normal(0, math.sqrt(1.0 / a), (a, b)) for a, b in zip(sizes[:-1], sizes[1:])]
        self.b = [np.zeros(b) for b in sizes[1:]]
        self.x_mean = np.zeros(n_in)
        self.x_std = np.ones(n_in)
        self.y_mean = np.zeros(n_out)
        self.y_std = np.ones(n_out)
        self.history: List[Tuple[float, float]] = []   # (train, validation) loss per epoch

    @property
    def n_in(self) -> int:
        return self.W[0].shape[0]

    @property
    def n_out(self) -> int:
        return self.W[-1].shape[1]

    def _forward(self, x: "np.ndarray") -> Tuple["np.ndarray", List["np.ndarray"]]:
        acts = [x]
        h = x
        for k, (W, b) in enumerate(zip(self.W, self.b)):
            h = h @ W + b
            if k < len(self.W) - 1:
                h = np.tanh(h)
            acts.append(h)
        return h, acts

    def predict(self, x: "np.ndarray") -> "np.ndarray":
        """Outputs for inputs (one row or many)."""
        x = np.asarray(x, dtype=float)
        one = x.ndim == 1
        xs = (np.atleast_2d(x) - self.x_mean) / self.x_std
        y, _ = self._forward(xs)
        y = y * self.y_std + self.y_mean
        return y[0] if one else y

    def fit(self, X: "np.ndarray", Y: "np.ndarray", *, epochs: int = 100, batch: int = 256, lr: float = 1e-3,
            noise: float = 0.0, weights: Optional["np.ndarray"] = None, validation: Optional[Tuple["np.ndarray", "np.ndarray"]] = None,
            seed: int = 0, verbose: bool = False) -> List[Tuple[float, float]]:
        """Trains on inputs ``X`` and targets ``Y`` (rows). ``noise`` adds that
        much Gaussian noise (in standard deviations) to the inputs, which makes
        a model that feeds on its own outputs much steadier. ``weights``
        weighs each output in the loss. Returns the loss per epoch."""
        X = np.asarray(X, dtype=float)
        Y = np.asarray(Y, dtype=float)
        self.x_mean, self.x_std = X.mean(0), X.std(0)
        self.y_mean, self.y_std = Y.mean(0), Y.std(0)
        self.x_std[self.x_std < 1e-6] = 1.0
        self.y_std[self.y_std < 1e-6] = 1.0
        Xs = (X - self.x_mean) / self.x_std
        Ys = (Y - self.y_mean) / self.y_std
        w = np.ones(Y.shape[1]) if weights is None else np.asarray(weights, dtype=float)
        if validation is not None:
            Xv = (np.asarray(validation[0], dtype=float) - self.x_mean) / self.x_std
            Yv = (np.asarray(validation[1], dtype=float) - self.y_mean) / self.y_std
        rng = np.random.default_rng(seed)
        params = self.W + self.b
        m = [np.zeros_like(p) for p in params]
        v = [np.zeros_like(p) for p in params]
        b1, b2, eps, step = 0.9, 0.999, 1e-8, 0
        n = len(Xs)
        for epoch in range(epochs):
            rate = lr * (0.1 + 0.9 * 0.5 * (1 + math.cos(math.pi * epoch / max(1, epochs))))  # cosine, to lr/10
            order = rng.permutation(n)
            total = 0.0
            for s in range(0, n, batch):
                idx = order[s:s + batch]
                xb, yb = Xs[idx], Ys[idx]
                if noise > 0:
                    xb = xb + rng.normal(0, noise, xb.shape)
                out, acts = self._forward(xb)
                err = (out - yb) * w
                total += float((err * (out - yb)).sum())
                g = 2 * err / len(idx)
                grads_W, grads_b = [], []
                for k in range(len(self.W) - 1, -1, -1):
                    grads_W.append(acts[k].T @ g)
                    grads_b.append(g.sum(0))
                    if k:
                        g = (g @ self.W[k].T) * (1 - acts[k] ** 2)
                grads = grads_W[::-1] + grads_b[::-1]
                step += 1
                for p, gr, mi, vi in zip(params, grads, m, v):
                    mi *= b1
                    mi += (1 - b1) * gr
                    vi *= b2
                    vi += (1 - b2) * gr * gr
                    p -= rate * (mi / (1 - b1 ** step)) / (np.sqrt(vi / (1 - b2 ** step)) + eps)
            train = total / (n * Y.shape[1])
            val = float("nan")
            if validation is not None:
                out, _ = self._forward(Xv)
                val = float((((out - Yv) ** 2) * w).mean())
            self.history.append((train, val))
            if verbose and (epoch % max(1, epochs // 10) == 0 or epoch == epochs - 1):
                print(f"  epoch {epoch + 1:4d}/{epochs}: loss {train:.5f}" + ("" if validation is None else f", validation {val:.5f}"))
        return self.history

    def state(self, prefix: str = "") -> Dict[str, "np.ndarray"]:
        d = {f"{prefix}x_mean": self.x_mean, f"{prefix}x_std": self.x_std,
             f"{prefix}y_mean": self.y_mean, f"{prefix}y_std": self.y_std,
             f"{prefix}layers": np.array(len(self.W))}
        for k, (W, b) in enumerate(zip(self.W, self.b)):
            d[f"{prefix}W{k}"] = W
            d[f"{prefix}b{k}"] = b
        return d

    @classmethod
    def from_state(cls, d: Mapping[str, "np.ndarray"], prefix: str = "") -> "MLP":
        layers = int(d[f"{prefix}layers"])
        W = [np.asarray(d[f"{prefix}W{k}"]) for k in range(layers)]
        net = cls(W[0].shape[0], W[-1].shape[1], [w.shape[1] for w in W[:-1]])
        net.W = W
        net.b = [np.asarray(d[f"{prefix}b{k}"]) for k in range(layers)]
        for key in ("x_mean", "x_std", "y_mean", "y_std"):
            setattr(net, key, np.asarray(d[f"{prefix}{key}"]))
        return net


def _save(path: str, kind: str, meta: Mapping[str, Any], nets: Mapping[str, MLP]) -> None:
    import json
    arrays: Dict[str, Any] = {}
    for name, net in nets.items():
        arrays.update(net.state(f"{name}_"))
    np.savez_compressed(path, kind=np.array(kind), meta=np.array(json.dumps(meta)), nets=np.array(list(nets)), **arrays)


def _load(path: str, kind: str) -> Tuple[Dict[str, Any], Dict[str, MLP]]:
    import json
    with np.load(path, allow_pickle=False) as d:
        if str(d["kind"]) != kind:
            raise ValueError(f"{path} holds a {d['kind']}, not a {kind}")
        return json.loads(str(d["meta"])), {str(n): MLP.from_state(d, f"{n}_") for n in d["nets"]}


# ── Places on the floor ──────────────────────────────────────────────────────
# Heading 0 faces +X and positive turns to the robot's left, its -Z side
# (as everywhere in MotionStudio). Its forward axis is (cos h, -sin h) in
# (x, z) and its left axis is (-sin h, -cos h).

def _wrap(a: "np.ndarray") -> "np.ndarray":
    return (a + math.pi) % (2 * math.pi) - math.pi


def _to_body(dx, dz, h):
    """A floor vector (x, z) in a robot's frame facing h: (ahead, to its left)."""
    c, s = np.cos(h), np.sin(h)
    return dx * c - dz * s, -dx * s - dz * c


def _from_body(fwd, left, h):
    c, s = np.cos(h), np.sin(h)
    return fwd * c - left * s, -fwd * s - left * c


# ── Animations and poses ─────────────────────────────────────────────────────

class AnimationModel:
    """Learns animations: each named take is a clip, and the model learns the
    joint values (and how the robot moves on the floor) over the clip's
    normalized time. It can then play a clip at any length or rate, give the
    pose at any moment, or blend clips::

        model = AnimationModel().fit({"wave": wave_take, "bow": bow_take})
        robot.play(model.generate("wave", duration=3))
        robot.play(model.generate({"wave": 0.5, "bow": 0.5}))   # half of each
    """

    def __init__(self, hidden: Sequence[int] = (128, 128), harmonics: int = 12, seed: int = 0):
        self.hidden = tuple(hidden)
        self.harmonics = harmonics
        self.seed = seed
        self.clips: List[str] = []
        self.durations: Dict[str, float] = {}
        self.joints: List[str] = []
        self.robot = ""
        self.net: Optional[MLP] = None

    def _inputs(self, weights: "np.ndarray", tau: "np.ndarray") -> "np.ndarray":
        k = np.arange(1, self.harmonics + 1)
        ang = np.outer(tau, k) * math.pi
        return np.hstack([np.repeat(weights[None, :], len(tau), 0) if weights.ndim == 1 else weights,
                          np.sin(ang), np.cos(ang), tau[:, None]])

    def fit(self, clips: Mapping[str, Take], *, epochs: int = 400, lr: float = 2e-3, verbose: bool = False) -> "AnimationModel":
        """Learns the named takes. They must share their joints (one robot)."""
        if not clips:
            raise ValueError("no clips to learn")
        self.clips = list(clips)
        first = next(iter(clips.values()))
        self.joints = first.joint_names
        self.robot = first.robot
        X, Y = [], []
        for i, (name, take) in enumerate(clips.items()):
            if len(take) < 2:
                raise ValueError(f"clip {name!r} has fewer than two rows")
            missing = [j for j in self.joints if j not in take.columns]
            if missing:
                raise ValueError(f"clip {name!r} lacks the joints {', '.join(missing)}")
            a = take.array()
            t = a[:, take.index("t")]
            tau = (t - t[0]) / max(1e-9, t[-1] - t[0])
            self.durations[name] = float(t[-1] - t[0])
            one = np.zeros(len(self.clips))
            one[i] = 1
            X.append(self._inputs(one, tau))
            Y.append(np.hstack([a[:, [take.index(j) for j in self.joints]], self._place(take, a)]))
        self.net = MLP(X[0].shape[1], Y[0].shape[1], self.hidden, self.seed)
        self.net.fit(np.vstack(X), np.vstack(Y), epochs=epochs, lr=lr, batch=128, verbose=verbose)
        return self

    @staticmethod
    def _place(take: Take, a: "np.ndarray") -> "np.ndarray":
        """Where the robot is on the floor relative to the clip's start: ahead, left (mm), turn, height (mm)."""
        n = len(a)
        if not all(c in take.columns for c in ("x", "z")):
            return np.zeros((n, 4))
        x, z = a[:, take.index("x")], a[:, take.index("z")]
        y = a[:, take.index("y")] if "y" in take.columns else np.zeros(n)
        h = np.radians(a[:, take.index("heading")] if "heading" in take.columns else a[:, take.index("ry")])
        fwd, left = _to_body(x - x[0], z - z[0], h[0])
        return np.column_stack([fwd, left, np.degrees(_wrap(h - h[0])), y - y[0]])

    def _weights(self, clip: Union[str, Mapping[str, float]]) -> "np.ndarray":
        w = np.zeros(len(self.clips))
        mix = {clip: 1.0} if isinstance(clip, str) else dict(clip)
        for name, v in mix.items():
            if name not in self.clips:
                raise KeyError(f"no clip {name!r}; the model knows {', '.join(self.clips)}")
            w[self.clips.index(name)] = v
        total = w.sum()
        return w / total if total > 0 else w

    def pose(self, clip: Union[str, Mapping[str, float]], at: float = 0.0) -> Dict[str, float]:
        """The joint values ``at`` a moment of the clip (0 = its start, 1 = its end)."""
        if self.net is None:
            raise RuntimeError("the model has not learned anything yet: call fit()")
        y = self.net.predict(self._inputs(self._weights(clip), np.array([float(at)])))[0]
        return dict(zip(self.joints, map(float, y[:len(self.joints)])))

    def generate(self, clip: Union[str, Mapping[str, float]], *, duration: Optional[float] = None,
                 rate: float = 50.0, place: bool = True, start: Tuple[float, float, float, float] = (0, 0, 0, 0)) -> Take:
        """The clip (or a blend: ``{"wave": 0.7, "bow": 0.3}``) as a take,
        ``duration`` seconds long (default: the clip's, or the blend's mean).
        With ``place`` it carries ``x y z heading`` from ``start`` = (x, y, z,
        heading); play it with ``robot.play(take, relative=True)``."""
        if self.net is None:
            raise RuntimeError("the model has not learned anything yet: call fit()")
        w = self._weights(clip)
        if duration is None:
            duration = float(sum(w[i] * self.durations[c] for i, c in enumerate(self.clips)))
        n = max(2, int(round(duration * rate)) + 1)
        t = np.linspace(0, duration, n)
        y = self.net.predict(self._inputs(w, t / max(duration, 1e-9)))
        J = len(self.joints)
        cols = ["t"]
        data = [t]
        if place:
            x0, y0, z0, h0 = start
            dx, dz = _from_body(y[:, J], y[:, J + 1], math.radians(h0))
            cols += ["x", "y", "z", "heading"]
            data += [x0 + dx, y0 + y[:, J + 3], z0 + dz, h0 + y[:, J + 2]]
        cols += self.joints
        data += [y[:, k] for k in range(J)]
        name = clip if isinstance(clip, str) else "+".join(f"{k}*{v:g}" for k, v in clip.items())
        return Take(cols, np.column_stack(data).tolist(), self.robot, rate, {"clip": name})

    def save(self, path: str) -> None:
        if self.net is None:
            raise RuntimeError("nothing to save: call fit()")
        _save(path, "AnimationModel", {"clips": self.clips, "durations": self.durations, "joints": self.joints,
                                       "robot": self.robot, "harmonics": self.harmonics, "hidden": self.hidden}, {"net": self.net})

    @classmethod
    def load(cls, path: str) -> "AnimationModel":
        meta, nets = _load(path, "AnimationModel")
        m = cls(meta["hidden"], meta["harmonics"])
        m.clips, m.durations, m.joints, m.robot = meta["clips"], meta["durations"], meta["joints"], meta["robot"]
        m.net = nets["net"]
        return m


# ── Walking ──────────────────────────────────────────────────────────────────

GOAL_RANGE = 1500.0   # mm: a goal further than this looks this far away to the step policy
_KINDS = ("start", "step", "stop")   # the parts of a walk: crouch, steps, stand up
Place3 = Tuple[float, float, float]  # x, z (mm), heading (rad)


def _place_at(t: "np.ndarray", x, z, h, when: float) -> Place3:
    """The place at time ``when``, between rows."""
    i = int(np.clip(np.searchsorted(t, when) - 1, 0, len(t) - 2))
    f = float(np.clip((when - t[i]) / max(1e-9, t[i + 1] - t[i]), 0, 1))
    dh = float(_wrap(np.array(h[i + 1] - h[i])))
    return float(x[i] + (x[i + 1] - x[i]) * f), float(z[i] + (z[i + 1] - z[i]) * f), float(h[i] + dh * f)


def _relative(p: Place3, q: Place3) -> Tuple[float, float, float]:
    """``q`` seen from ``p``: ahead (mm), left (mm), turn (deg)."""
    fwd, left = _to_body(q[0] - p[0], q[1] - p[1], p[2])
    return float(fwd), float(left), float(math.degrees(_wrap(np.array(q[2] - p[2]))))


def _advance(p: Place3, inc: Sequence[float]) -> Place3:
    dx, dz = _from_body(inc[0], inc[1], p[2])
    return p[0] + float(dx), p[1] + float(dz), float(_wrap(np.array(p[2] + math.radians(inc[2]))))


def _mid(a: Place3, b: Place3) -> Place3:
    return (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, float(_wrap(np.array(a[2] + _wrap(np.array(b[2] - a[2])) / 2)))


def _first_side(start: Place3, first: Place3) -> int:
    """The foot that steps first, as MotionStudio chooses it: towards the turn, else the side it goes."""
    dh = float(_wrap(np.array(first[2] - start[2])))
    left = -(first[0] - start[0]) * math.sin(start[2]) - (first[1] - start[1]) * math.cos(start[2])
    return 0 if dh > 1e-6 else 1 if dh < -1e-6 else 0 if left > 1e-6 else 1 if left < -1e-6 else 0


def _body_steps(start: Place3, footsteps: Sequence[Place3]) -> List[Dict[str, Any]]:
    """Where the body goes over each step through ``footsteps`` (as MotionStudio
    plans it: the body is midway between the feet; after the last footstep the
    other foot comes alongside): per step its side, its place and increment."""
    side = _first_side(start, footsteps[0])
    feet = [start, start]
    body = [start]
    sides = []
    for f in footsteps:
        sides.append(side)
        feet[side] = f
        body.append(_mid(feet[0], feet[1]))
        side = 1 - side
    sides.append(side)
    body.append(footsteps[-1])
    return [{"side": sides[k], "place": body[k], "inc": _relative(body[k], body[k + 1])} for k in range(len(sides))]


def walk_steps(take: Take, step_time: Optional[float] = None) -> List[Dict[str, Any]]:
    """Splits a take of a walking robot into its walks, and each walk into its
    parts: the crouch before the first step, the steps, and standing up.

    Each walk is a dict with its ``goal`` and ``start`` (x, z in mm, heading
    in rad), ``footsteps`` (the places its steps put a foot, without the
    closing step that brings the other foot alongside), ``steps`` (per step,
    the closing one too: its ``side``, the body's ``place`` when it began and
    ``inc``, how the body moved over it: ahead, left in mm, turn in degrees),
    and ``rows``: for every row of the walk its part, step, how far through
    the part (0..1), its place, height and joint values."""
    for c in ("x", "y", "z", "heading", "walking", "step", "swing", "progress", "step_x", "step_z", "step_heading",
              "goal_x", "goal_z", "goal_heading"):
        if c not in take.columns:
            raise ValueError(f"a walk take needs the column {c!r} (record a humanoid on legs)")
    a = take.array()
    col = lambda c: a[:, take.index(c)]
    t, x, z, y, h = col("t"), col("x"), col("z"), col("y"), np.radians(col("heading"))
    walking, step, swing, progress = col("walking"), col("step").astype(int), col("swing").astype(int), col("progress")
    fx, fz, fh = col("step_x"), col("step_z"), np.radians(col("step_heading"))
    q = a[:, [take.index(j) for j in take.joint_names]]
    if step_time is None:   # from the rows: how fast the progress goes
        rates = [(progress[i + 1] - progress[i]) / (t[i + 1] - t[i]) for i in range(len(t) - 1)
                 if step[i] >= 0 and step[i + 1] == step[i] and t[i + 1] > t[i]]
        if not rates:
            return []
        step_time = 1.0 / float(np.median(rates))
    walks = []
    i = 0
    while i < len(t):
        if walking[i] < 0.5:
            i += 1
            continue
        j = i
        while j + 1 < len(t) and walking[j + 1] > 0.5:
            j += 1
        idx = np.arange(i, j + 1)
        ks = sorted(set(step[idx][step[idx] >= 0]))
        # A walk still under way when the take ended, or that never stepped, teaches nothing
        if len(ks) < 2 or ks != list(range(ks[-1] + 1)) or j + 1 >= len(t) or i == 0:
            i = j + 1
            continue
        begin = t[i - 1] + (t[i] - t[i - 1]) * 0.5            # it set off between the rows
        start = (float(x[i - 1]), float(z[i - 1]), float(h[i - 1]))
        first_row = {k: idx[step[idx] == k][0] for k in ks}
        starts = [t[first_row[k]] - progress[first_row[k]] * step_time for k in ks]
        end_steps = starts[-1] + step_time
        places = [_place_at(t, x, z, h, s) for s in starts] + [_place_at(t, x, z, h, end_steps)]
        steps = [{"side": int(swing[first_row[k]]), "place": places[n], "inc": _relative(places[n], places[n + 1])}
                 for n, k in enumerate(ks)]
        footsteps = [(float(fx[first_row[k]]), float(fz[first_row[k]]), float(fh[first_row[k]])) for k in ks[:-1]]
        prep = float(starts[0] - begin)
        rows = []
        for r in idx:
            if step[r] >= 0:
                rows.append(("step", int(step[r]), float(progress[r]), r))
            elif t[r] < starts[0]:
                rows.append(("start", -1, float(np.clip((t[r] - begin) / prep, 0, 1)), r))
            else:
                rows.append(("stop", -1, float(np.clip((t[r] - end_steps) / prep, 0, 1)), r))
        walks.append({
            "goal": (float(col("goal_x")[i]), float(col("goal_z")[i]), float(np.radians(col("goal_heading")[i]))),
            "start": start, "end": places[-1], "footsteps": footsteps, "steps": steps, "prep": prep,
            "step_time": step_time,
            "rows": [{"kind": kind, "step": k, "phase": ph, "place": (float(x[r]), float(z[r]), float(h[r])),
                      "y": float(y[r]), "q": q[r]} for kind, k, ph, r in rows],
        })
        i = j + 1
    return walks


class WalkPolicy:
    """Learns to walk to a goal from takes of a humanoid walking to goals.

    Two small networks, like a walker's mind and body:

    - the **step policy** decides where to step: from where the goal is, seen
      from the last footstep, and the step before, the next footstep (ahead,
      to its left, the turn), and when that one is the last;
    - the **motion model** moves the body through a step: from the phase in
      the step and the steps before, now and after, every joint's value and
      how the body sways and crouches about its path.

    Use them together or apart:

    - ``walk(robot, x, z)``: the robot steps where the policy decides, with
      its own walking (its feet are planted exactly where the steps put them);
    - ``motion(x, z)``: the whole motion comes from the networks, as a take
      for ``robot.play``;
    - ``footsteps(start, goal)``: only the policy's footsteps.

    Teach it with takes recorded around ``walker.walk_to(...)`` to many
    places (``robot.record()`` or Record Values).
    """

    def __init__(self, hidden: Sequence[int] = (256, 256), harmonics: int = 6, seed: int = 0):
        self.hidden = tuple(hidden)
        self.harmonics = harmonics
        self.seed = seed
        self.joints: List[str] = []
        self.robot = ""
        self.step_time = 0.0
        self.prep = 0.0
        self.stand_y = 0.0
        self.step_length = 0.0                   # mm, a typical footstep (median)
        self.steps_net: Optional[MLP] = None     # the step policy
        self.motion_net: Optional[MLP] = None    # the motion model
        self.history: Dict[str, List[Tuple[float, float]]] = {}

    # ── Inputs ───────────────────────────────────────────────────────────────

    @staticmethod
    def _step_input(at: Place3, goal: Place3, prev_inc: Sequence[float], first: bool) -> List[float]:
        fwd, left, turn = _relative(at, goal)
        d = math.hypot(fwd, left)
        k = GOAL_RANGE / d if d > GOAL_RANGE else 1.0
        dh = math.radians(turn)
        return [fwd * k, left * k, min(d, GOAL_RANGE * 2), math.sin(dh), math.cos(dh), *prev_inc, 1.0 if first else 0.0]

    def _motion_input(self, kind: str, phase: float, side: int, prev_inc, inc, next_inc) -> List[float]:
        k = np.arange(1, self.harmonics + 1)
        ang = 2 * math.pi * phase * k
        sign = 0.0 if kind != "step" else (1.0 if side == 0 else -1.0)
        return ([1.0 if kind == c else 0.0 for c in _KINDS] + [sign, phase] + list(np.sin(ang)) + list(np.cos(ang))
                + list(prev_inc) + list(inc) + list(next_inc))

    # ── Learning ─────────────────────────────────────────────────────────────

    def fit(self, takes: Sequence[Take], *, epochs: int = 300, motion_epochs: int = 100, lr: float = 1e-3,
            validation: float = 0.1, verbose: bool = False) -> "WalkPolicy":
        """Learns from takes of one humanoid walking: ``epochs`` passes for the
        step policy, ``motion_epochs`` for the motion model (it sees many more
        samples per pass). ``validation`` is the share of takes held back to
        report how well it does on walks it didn't learn."""
        takes = [t for t in takes if len(t) > 3]
        if not takes:
            raise ValueError("no takes to learn from")
        self.joints = takes[0].joint_names
        self.robot = takes[0].robot
        for t in takes:
            if t.joint_names != self.joints:
                raise ValueError("the takes must come from one robot (the same joints)")
        walks_of = [walk_steps(t) for t in takes]
        all_walks = [w for ws in walks_of for w in ws]
        if not all_walks:
            raise ValueError("no complete walk in the takes (each needs the robot standing before and after it)")
        self.step_time = float(np.median([w["step_time"] for w in all_walks]))
        self.prep = float(np.median([w["prep"] for w in all_walks]))
        self.stand_y = float(np.median([takes[i].rows[0][takes[i].index("y")] for i in range(len(takes))]))
        lengths = [math.hypot(*_relative(a, b)[:2]) for w in all_walks
                   for a, b in zip([w["start"]] + w["footsteps"][:-1], w["footsteps"])]
        self.step_length = float(np.median([v for v in lengths if v > 1] or [300.0]))

        rng = np.random.default_rng(self.seed)
        order = rng.permutation(len(takes))
        n_val = int(round(len(takes) * validation)) if len(takes) > 4 else 0
        held = set(order[:n_val].tolist())
        zero = (0.0, 0.0, 0.0)

        def samples(which):
            SX, SY, MX, MY = [], [], [], []
            for ti in which:
                for w in walks_of[ti]:
                    # The step policy: each footstep from the one before
                    fs = w["footsteps"]
                    at, prev = w["start"], zero
                    for k, f in enumerate(fs):
                        inc = _relative(at, f)
                        SX.append(self._step_input(at, w["goal"], prev, k == 0))
                        SY.append([*inc, 1.0 if k == len(fs) - 1 else 0.0])
                        at, prev = f, inc
                    # The motion model: every row about the body's path
                    steps = w["steps"]
                    n = len(steps)
                    for r in w["rows"]:
                        if r["kind"] == "start":
                            ref, side, prv, inc, nxt = w["start"], 0, zero, zero, steps[0]["inc"]
                        elif r["kind"] == "stop":
                            ref, side, prv, inc, nxt = w["end"], 0, steps[-1]["inc"], zero, zero
                        else:
                            k = r["step"]
                            ref, side = steps[k]["place"], steps[k]["side"]
                            prv = steps[k - 1]["inc"] if k else zero
                            inc = steps[k]["inc"]
                            nxt = steps[k + 1]["inc"] if k + 1 < n else zero
                        fwd, left, turn = _relative(_advance(ref, [v * r["phase"] for v in inc]), r["place"])
                        MX.append(self._motion_input(r["kind"], r["phase"], side, prv, inc, nxt))
                        MY.append(list(r["q"]) + [fwd, left, turn, r["y"] - self.stand_y])
            return [np.array(v, dtype=float) for v in (SX, SY, MX, MY)]

        SX, SY, MX, MY = samples([i for i in range(len(takes)) if i not in held])
        vSX = vSY = vMX = vMY = None
        if n_val:
            vSX, vSY, vMX, vMY = samples(sorted(held))
        if verbose:
            print(f"  {len(SX)} footsteps and {len(MX)} frames from {len(takes) - n_val} takes"
                  + (f" ({n_val} held back)" if n_val else "") + f"; a step takes {self.step_time:.2f} s")
            print("  the step policy:")
        self.steps_net = MLP(SX.shape[1], SY.shape[1], (128, 128), self.seed)
        self.history["steps"] = self.steps_net.fit(
            SX, SY, epochs=epochs, lr=lr, batch=64, weights=np.array([4, 4, 4, 1]),
            validation=(vSX, vSY) if n_val else None, seed=self.seed, verbose=verbose)
        if verbose:
            print("  the motion model:")
        self.motion_net = MLP(MX.shape[1], MY.shape[1], self.hidden, self.seed)
        self.history["motion"] = self.motion_net.fit(
            MX, MY, epochs=motion_epochs, lr=lr, batch=256,
            validation=(vMX, vMY) if n_val else None, seed=self.seed, verbose=verbose)
        return self

    # ── Deciding ─────────────────────────────────────────────────────────────

    @staticmethod
    def _where(start: Union[None, Mapping[str, float], Any]) -> Tuple[float, float, float]:
        """x, z, heading (degrees) of a robot, a Place or a dict; the origin for None."""
        if start is None:
            return 0.0, 0.0, 0.0
        if hasattr(start, "state") and callable(getattr(start, "state")):
            pl = start.state().place
            return pl.x, pl.z, pl.heading
        if hasattr(start, "heading") and hasattr(start, "x"):
            return float(start.x), float(start.z), float(start.heading)
        return float(start.get("x", 0)), float(start.get("z", 0)), float(start.get("heading", 0))

    @staticmethod
    def _goal(x0: float, z0: float, h0: float, x: float, z: float, heading: Optional[float]) -> float:
        if heading is not None:
            return float(heading)
        return math.degrees(math.atan2(-(z - z0), x - x0)) if math.hypot(x - x0, z - z0) > 1 else h0

    def footsteps(self, start: Union[None, Mapping[str, float], Any], x: float, z: float,
                  heading: Optional[float] = None, *, exact: bool = True,
                  max_steps: int = 300) -> List[Tuple[float, float, float]]:
        """The footsteps the policy chooses from ``start`` (a robot, a Place, or
        a dict with x, z, heading) to ``x``, ``z`` (mm), facing ``heading``
        (degrees; default: the way it goes there): ``[(x, z, heading), …]``
        in mm and degrees, as ``walker.walk_steps`` takes them.

        The policy decides every step, and where it stops; it ends within a
        few centimetres of the goal. With ``exact`` its last footstep is then
        put on the goal itself, when that is less than half a step and 30°
        from where the policy put it (else the steps are left as they are)."""
        if self.steps_net is None:
            raise RuntimeError("the policy has not learned anything yet: call fit()")
        x0, z0, h0 = self._where(start)
        heading = self._goal(x0, z0, h0, x, z, heading)
        at: Place3 = (x0, z0, math.radians(h0))
        goal: Place3 = (x, z, math.radians(heading))
        prev = (0.0, 0.0, 0.0)
        out = []
        for k in range(max_steps):
            o = self.steps_net.predict(np.array(self._step_input(at, goal, prev, k == 0)))
            inc = tuple(float(v) for v in o[:3])
            at = _advance(at, inc)
            prev = inc
            out.append((at[0], at[1], math.degrees(at[2])))
            if o[3] > 0.5:
                break
        if exact and out:
            fwd, left, turn = _relative((out[-1][0], out[-1][1], math.radians(out[-1][2])), goal)
            if math.hypot(fwd, left) < 0.5 * self.step_length and abs(turn) < 30:
                out[-1] = (x, z, heading)
        return out

    def walk(self, robot: Any, x: float, z: float, heading: Optional[float] = None, *, exact: bool = True,
             wait: bool = True, **gait: Any):
        """The robot walks to ``x``, ``z`` (mm) and faces ``heading``, stepping
        where the policy decides; its own walking moves its legs, so each foot
        lands exactly on its footstep. Returns the Motion."""
        steps = self.footsteps(robot, x, z, heading, exact=exact)
        return robot.walk_steps(steps, wait=wait, **gait)

    def motion(self, x: float, z: float, heading: Optional[float] = None, *,
               start: Union[None, Mapping[str, float], Any] = None, exact: bool = True, rate: float = 50.0) -> Take:
        """The walk to ``x``, ``z`` (mm) facing ``heading`` made entirely by the
        networks: the policy's footsteps, and every joint's value from the
        motion model. Returns a take (``t x y z heading``, the joints, and
        ``walking step swing progress``) to play with ``robot.play(take)``."""
        if self.motion_net is None:
            raise RuntimeError("the policy has not learned anything yet: call fit()")
        x0, z0, h0 = self._where(start)
        fs = self.footsteps({"x": x0, "z": z0, "heading": h0}, x, z, heading, exact=exact)
        begin: Place3 = (x0, z0, math.radians(h0))
        steps = _body_steps(begin, [(f[0], f[1], math.radians(f[2])) for f in fs])
        zero = (0.0, 0.0, 0.0)
        # The parts of the walk on a timeline: crouch, the steps, stand up
        parts = [("start", self.prep, begin, 0, zero, zero, steps[0]["inc"], -1)]
        for k, s in enumerate(steps):
            parts.append(("step", self.step_time, s["place"], s["side"], steps[k - 1]["inc"] if k else zero, s["inc"],
                          steps[k + 1]["inc"] if k + 1 < len(steps) else zero, k))
        parts.append(("stop", self.prep, _advance(steps[-1]["place"], steps[-1]["inc"]), 0, steps[-1]["inc"], zero, zero, -1))
        total = sum(p[1] for p in parts)
        n = int(math.floor(total * rate + 1e-9)) + 1
        J = len(self.joints)
        inputs, meta = [], []
        t0 = 0.0
        for idx, (kind, dur, ref, side, prv, inc, nxt, k) in enumerate(parts):
            i0 = int(math.ceil(t0 * rate - 1e-9))
            i1 = n - 1 if idx == len(parts) - 1 else int(math.ceil((t0 + dur) * rate - 1e-9)) - 1
            for i in range(i0, i1 + 1):
                ph = min(1.0, max(0.0, (i / rate - t0) / dur))
                inputs.append(self._motion_input(kind, ph, side, prv, inc, nxt))
                meta.append((i / rate, kind, k, side, ph, ref, inc))
            t0 += dur
        out = self.motion_net.predict(np.array(inputs))
        rows = []
        for (t, kind, k, side, ph, ref, inc), o in zip(meta, out):
            fx, fz, fh = _advance(_advance(ref, [v * ph for v in inc]), o[J:J + 3])
            stepping = kind == "step"
            rows.append([t, fx, self.stand_y + o[J + 3], fz, math.degrees(fh), 1.0,
                         float(k) if stepping else -1.0, float(side) if stepping else -1.0, ph if stepping else 0.0,
                         *o[:J]])
        rows[-1][5] = 0.0   # standing at the end
        cols = ["t", "x", "y", "z", "heading", "walking", "step", "swing", "progress", *self.joints]
        heading = self._goal(x0, z0, h0, x, z, heading)
        return Take(cols, rows, self.robot, rate, {"goal": f"{x:g}, {z:g}, {heading:g}", "steps": str(len(steps))})

    # ── Files ────────────────────────────────────────────────────────────────

    def save(self, path: str) -> None:
        if self.motion_net is None or self.steps_net is None:
            raise RuntimeError("nothing to save: call fit()")
        _save(path, "WalkPolicy", {"joints": self.joints, "robot": self.robot, "step_time": self.step_time,
                                   "prep": self.prep, "stand_y": self.stand_y, "step_length": self.step_length,
                                   "hidden": self.hidden, "harmonics": self.harmonics},
              {"steps": self.steps_net, "motion": self.motion_net})

    @classmethod
    def load(cls, path: str) -> "WalkPolicy":
        meta, nets = _load(path, "WalkPolicy")
        p = cls(meta["hidden"], meta["harmonics"])
        p.joints, p.robot, p.step_time, p.prep, p.stand_y = (meta["joints"], meta["robot"], meta["step_time"],
                                                             meta["prep"], meta["stand_y"])
        p.step_length = meta.get("step_length", 300.0)
        p.steps_net, p.motion_net = nets["steps"], nets["motion"]
        return p
