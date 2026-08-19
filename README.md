<p align="center">
  <img src="icon-1024x1024.png" width="120" alt="MotionStudio icon">
</p>

<h1 align="center">MotionStudio</h1>

<p align="center">
  A desktop app for building mechanical assemblies — joints, motors, conveyors — and running real-time 3D kinematic and physics simulations.
</p>

---

MotionStudio lets you assemble simple machines out of primitive bodies, wire up revolute/prismatic joints and motors, and press Play to watch them move. It targets the workflow of a mechanical or automation engineer sketching out motion — conveyors, robot arms, linear actuators, pick-and-place cells — without needing a full CAD suite.

Born out of an interest in robotics, automation, and digital twins — a place to prototype how a machine moves before it exists anywhere else.

## Features

- **3D scene editor** — orbit/pan/zoom camera, grid floor, click-to-select, translate gizmo
- **Primitives** — Cube, Cylinder, Sphere, Capsule, parented by selection to build hierarchies
- **Joints** — Revolute and Prismatic, with axis, limits, and initial angle; objects ping-pong at their limits
- **Motors** — drive a joint at a target velocity, toggle on/off
- **Rigid bodies & conveyor belts** — opt in per-object to real MuJoCo physics (mass, gravity, collisions, surface velocity)
- **STEP import** — bring in CAD assemblies (`.step`/`.stp`) via OpenCASCADE, preserving the part hierarchy for rigid-body grouping
- **Undo/redo** — full history across every kind of scene edit
- **Hierarchy & Properties panels** — Unity-style tree view and inspector
- **Templates** — starter projects (crank arm, robotic arm, solar system, pendulum, spinning top)
- **Save/Load** — JSON-based `.mstudio` project files

MotionStudio runs fully kinematically out of the box. Installing [MuJoCo](https://mujoco.org/) unlocks real rigid-body physics (gravity, collisions, forces); installing [OpenCASCADE](https://dev.opencascade.org/) unlocks STEP import. Both are optional, detected at build time.

## Building

Requirements:

- CMake ≥ 3.20, a C++20 compiler
- Qt 6 (Core, Gui, Widgets, OpenGL, OpenGLWidgets, Concurrent)
- [MuJoCo 3.10.0](https://github.com/google-deepmind/mujoco/releases) extracted to `~/mujoco-3.10.0` (optional — falls back to a kinematic-only stub if not found)
- OpenCASCADE (optional — enables STEP import): `libocct-data-exchange-dev`, `libocct-ocaf-dev`, `libocct-modeling-algorithms-dev`

```bash
# Build and run
./run.sh

# Manual build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Run (MuJoCo must be on LD_LIBRARY_PATH)
LD_LIBRARY_PATH=~/mujoco-3.10.0/lib ./build/MotionStudio
```

Override the MuJoCo location with `-DMUJOCO_ROOT=<path>` or the `MUJOCO_ROOT` env var.

### Debian package

```bash
./packaging/build-deb.sh [version] [architecture]
```

## Quick start

Objects added while something is selected become children of it — parent-child relationships are what joints connect.

**Spinning roller:**

1. `+ Cube` → machine frame at origin
2. Cube selected → `+ Cylinder` → roller added as a child
3. In Properties, set the roller's Position Y to lift it above the base
4. Roller still selected → `Add Joint`, then `Add Motor` (set Velocity)
5. `▶ Play`

**2-segment robot arm:**

1. `+ Cube` → Base
2. Select Base → `+ Cylinder` → Arm1
3. Select Arm1 → `+ Cylinder` → Arm2
4. Select Arm1 → `Add Joint` → `Add Motor`
5. Select Arm2 → `Add Joint` → `Add Motor` (different velocity/direction)
6. `▶ Play`

See [USAGE.md](USAGE.md) for the full toolbar reference, camera controls, and joint axis cheat sheet.

## Architecture

`MainWindow` owns a shared `SceneGraph` (the single source of truth for all scene data) and wires it up to a `Viewport` (OpenGL 3.3 renderer), a `SimulationEngine` (kinematic by default, MuJoCo-backed when available), `HierarchyPanel`/`PropertiesPanel` (Qt UI), and an `UndoManager` (whole-scene snapshot undo/redo). Projects round-trip to `.mstudio` JSON files via `ProjectSerializer`.

## Status

MotionStudio is an early-stage, actively developed hobby project — not production software. There are no automated tests yet. Expect rough edges.

Development is AI-assisted, with features planned and reviewed deliberately rather than churned out — the goal is a tool that's actually coherent to use, not just one that accumulates checkboxes.

## License

[MIT](LICENSE)
