# MotionStudio — Usage Guide

## Launch

```bash
./run.sh
```

---

## Core concept

**Objects added while something is selected become children of it.**
Parent-child relationships are what joints connect.

```
Base (Cube)              ← root, no joint
 └── Arm1 (Cylinder)     ← child, has revolute joint to Base
      └── Arm2 (Cylinder) ← child, has revolute joint to Arm1
```

---

## Quick start: spinning roller

1. Click **`+ Cube`** → machine frame appears at origin
2. Cube is selected → click **`+ Cylinder`** → roller added as child
3. In **Properties**, set roller **Position Y = 1** to lift it above the base
4. Roller still selected → click **`Add Joint`**
5. Roller still selected → click **`Add Motor`**, set **Velocity = 90**
6. Click **`▶ Play`** → roller spins

---

## Quick start: 2-segment robot arm

1. **`+ Cube`** → Base
2. Select Base → **`+ Cylinder`** → Arm1, set Position Y = 1
3. Select Arm1 → **`+ Cylinder`** → Arm2, set Position Y = 1
4. Select Arm1 → **`Add Joint`** → **`Add Motor`** (velocity = 45)
5. Select Arm2 → **`Add Joint`** → **`Add Motor`** (velocity = -60)
6. **`▶ Play`** → both segments move independently

---

## Camera controls

| Action | Control |
|--------|---------|
| Orbit  | Alt + Left drag |
| Pan    | Right drag |
| Zoom   | Scroll wheel |
| Select | Left click on object |

---

## Toolbar reference

| Button | What it does |
|--------|-------------|
| `+ Cube / Cylinder / Sphere / Capsule` | Add primitive (child of selection, or root if nothing selected) |
| `Add Joint` | Add revolute joint between selected object and its parent |
| `Add Motor` | Add motor to selected object's joint |
| `Delete` | Remove selected object (and its children) |
| `▶ Play` | Start simulation |
| `⏸ Pause` | Pause simulation |
| `↺ Reset` | Stop and rewind to t = 0 |
| `New / Open / Save` | File operations |

---

## Properties panel

### Transform
- **Position** — local position relative to parent
- **Rotation** — Euler angles in degrees (XYZ order)
- **Scale** — local scale

### Joint (visible when object has a joint)

| Field | Meaning |
|-------|---------|
| Type | **Revolute** = rotates, **Prismatic** = slides |
| Axis | Direction of rotation/translation (e.g. `0,1,0` = Y-axis) |
| Limit Min / Max | Angle range (degrees) or displacement range (units) |
| Init Angle | Starting angle at t = 0 |

### Motor (visible when object has a joint + motor)

| Field | Meaning |
|-------|---------|
| Velocity | Speed: deg/sec (revolute) or units/sec (prismatic) |
| Target | Target position (not yet enforced by physics — sets reference) |
| Enabled | Toggle motor on/off without removing it |

Objects bounce back (ping-pong) when they hit Limit Min or Limit Max.

---

## Hierarchy panel

- Shows parent-child tree of all objects
- **Click** an item to select it
- **Double-click** to rename
- **Right-click** → Delete

---

## Save / Load

- **Ctrl+S** — save (prompts for filename on first save)
- **Ctrl+O** — open project
- **Ctrl+N** — new project
- Format: `.mstudio` (JSON)

---

## Joint axis cheat sheet

| Axis value | Effect |
|------------|--------|
| `0, 1, 0` | Spin around Y (vertical) — default |
| `1, 0, 0` | Tilt around X |
| `0, 0, 1` | Tilt around Z |
| `1, 0, 0` (prismatic) | Slide left/right |
| `0, 1, 0` (prismatic) | Slide up/down |
| `0, 0, 1` (prismatic) | Slide forward/back |
