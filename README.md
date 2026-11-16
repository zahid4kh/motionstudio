<p align="center">
  <img src="icon-1024x1024.png" width="120" alt="MotionStudio icon">
</p>

<h1 align="center">MotionStudio</h1>

<p align="center">
  A desktop app for building mechanical assemblies — joints, actuators, conveyors — and running signal-driven, sequenced 3D kinematic and physics simulations.
</p>

<p align="center">
  <a href="https://buymeacoffee.com/zahidkh"><img src="https://img.shields.io/badge/Buy%20Me%20A%20Coffee-ffdd00?style=for-the-badge&logo=buy-me-a-coffee&logoColor=black" alt="Buy Me A Coffee"></a>
</p>

---

MotionStudio lets you assemble machines out of primitive bodies or imported CAD parts and add revolute/prismatic joints and actuators. You then program how they move with signals, expressions and operation sequences, and press Play to run the logic. It targets the workflow of a mechanical or automation engineer sketching out motion — conveyors, robot arms, linear actuators, pick-and-place cells — without needing a full CAD suite.

Born out of an interest in robotics, automation, and digital twins — a place to prototype how a machine moves before it exists anywhere else.

## Features

- **3D scene editor** — orbit/pan/zoom camera, grid floor, click-to-select, translate gizmo
- **Primitives** — Cube, Cylinder, Sphere, Capsule, parented by selection to build hierarchies
- **Joints** — Revolute (optionally continuous) and Prismatic, with axis, limits, pivot and initial value, jog-able at edit time
- **Actuators** — position or speed control with trapezoidal motion profiles; properties can be bound to expressions
- **Automation environment** — signals (inputs, internal flags, derived formulas), MCD-style operation sequences with predecessors, parallel branches, start conditions and signal actions, teach-point poses with synchronized moves, plus a runtime HMI (buttons, watch list, operation status, signal trace, Gantt timing)
- **Deterministic scan cycle** — PLC-style 16 ms scans with Play / Pause / Step / Reset and speed control
- **Rigid bodies & conveyor belts** — opt in per-object to real MuJoCo physics (mass, gravity, collisions, surface velocity)
- **STEP import** — bring in CAD assemblies (`.step`/`.stp`) via OpenCASCADE, preserving the part hierarchy for rigid-body grouping
- **Undo/redo** — full history across every kind of scene edit
- **Hierarchy & Properties panels** — Unity-style tree view and inspector
- **Robot Builder & Analyzer** — build a SCARA, palletizer or 6-axis robot from 28 presets and your own dimensions, axis ranges, end effector (16 tools: grippers, suction, magnet, drill, screwdriver, welding torch, dispenser, spray gun, laser, camera…) and style, with a live dimensioned drawing; analyse any robot's workspace, reach, DH parameters, dexterity and singularities
- **Humanoid Builder & walking** — build a humanoid on legs or a pedestal from 7 presets; humanoids walk ahead, sideways, turning, to a marker or along a path (feet planted on the floor, toe-off, arm swing), from a sequence's Walk operation or from code
- **Robot API** — drive robots and humanoids from Python, C++ or Kotlin programs over TCP: straight-line and joint moves, taught poses, walking paths, signals (`api/`, and `motionstudio-server` to run headless)
- **Templates** — working automation examples (pick & place arm, 6-axis robot cell, humanoid patrol, indexing table with press, pendulum, solar system, spinning top, crank arm)
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

## Quick start

Objects added while something is selected become children of it — parent-child relationships are what joints connect.

**Sequenced 2-axis arm:**

1. `+ Cube` → Base; select Base → `+ Cylinder` → Arm1; select Arm1 → `+ Cylinder` → Arm2 (lift each with Position Y)
2. Select Arm1 → `Add Joint` → `Add Actuator` (**J1**); select Arm2 → `Add Joint` → `Add Actuator` (**J2**)
3. **Automation › Signals** → `+ Input` → `Start`
4. **Automation › Sequences** → `+`, set *Start when* = `Start`, tick *Loop*
5. `+ Operation › Move` J1 → `60`, then `Move` J2 → `-45`, then `Move` J1 → `0`
6. `▶ Play`, then switch **Start** ON in **Automation › Runtime**

Or open a template (**File › Open Template…**), press Play and switch Start ON.

See [USAGE.md](USAGE.md) for the full toolbar reference, camera controls, and joint axis cheat sheet.

## Architecture

`MainWindow` owns a shared `SceneGraph`, the single source of truth for all scene data, including the automation definitions. It wires the scene up to:

- a `Viewport` (OpenGL 3.3 renderer)
- a `SimulationEngine`, which runs the `AutomationEngine` scan cycle and adds MuJoCo physics when rigid bodies are present
- `HierarchyPanel`, `PropertiesPanel` and `AutomationPanel` (the Qt UI)
- an `UndoManager` (whole-scene snapshot undo/redo) Projects round-trip to `.mstudio` JSON files via `ProjectSerializer`.

## Status

MotionStudio is an early-stage, actively developed hobby project — not production software. There are no automated tests yet. Expect rough edges.

Development is AI-assisted, with features planned and reviewed deliberately rather than churned out — the goal is a tool that's actually coherent to use, not just one that accumulates checkboxes.

## License

[MIT](LICENSE)
