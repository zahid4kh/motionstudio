# MotionStudio – Build Checklist

Tracks implementation progress against the PRD milestones.

---

## Milestone 1 – Viewport

Goal: See objects.

- [x] OpenGL 3.3 Core Profile context (MSAA × 4)
- [x] Orbit camera (Alt + Left drag)
- [x] Pan camera (Right drag / Alt + Shift + Left drag)
- [x] Zoom camera (scroll wheel)
- [x] Grid floor (XZ plane, 40 × 40 units)
- [x] Object rendering with Phong shading
- [x] Object selection by clicking the visible surface (bounding boxes prefilter, then a triangle test)
- [x] Selected object highlight (yellow tint)

---

## Milestone 2 – Scene Graph

Goal: Create assemblies.

- [x] SceneObject data structure (id, name, type, transform, parent/children)
- [x] SceneGraph (add, remove, select objects)
- [x] Transform hierarchy (world transform computed from parent chain)
- [x] Primitive types: Cube, Cylinder, Sphere, Capsule
- [x] Hierarchy panel (QTreeWidget, shows parent–child tree)
- [x] Rename via double-click in hierarchy
- [x] Delete via context menu in hierarchy
- [x] Properties panel (position, rotation, scale spinboxes)
- [x] Toolbar: add Cube / Cylinder / Sphere / Capsule buttons
- [x] Toolbar: Delete selected button

---

## Milestone 3 – MuJoCo Integration

Goal: Physics works.

- [x] CMakeLists.txt detects MuJoCo at `~/mujoco-3.10.0`
- [x] `MUJOCO_ENABLED` compile flag set when found
- [x] MuJoCo rigid-body simulation (scene → MJCF XML → simulate)
- [x] Read-back simulated body poses into SceneGraph transforms
- [x] Gravity / collisions driven by MuJoCo

> **Current state:** `buildMuJoCoModel()` generates MJCF from the scene (bodies, joints with ranges, collision
> shapes, gravity in −Y). The engine steps MuJoCo at 2 ms sub-steps and `syncFromMuJoCo()` writes the free
> bodies and unactuated joints back. Actuated joints are driven kinematically by the AutomationEngine
> (`prescribeActuatedJoints()`).

---

## Milestone 4 – Joint Editor

Goal: Mechanical motion works.

- [x] `Joint` data structure (Revolute / Prismatic, axis, limits, initial angle)
- [x] Add joint between selected child and its parent (toolbar button + SceneGraph)
- [x] Joint properties in Properties panel (type, axis, limits, initial angle)
- [x] Remove joint button in Properties panel
- [x] Joint current value applied to child body transform during rendering
- [ ] Joint limit visualisation in viewport (arc / arrow overlay)
  - Partly done: the selected joint's axis is drawn as an arrow, with a right-hand-rule curl for revolute joints, but the limit range isn't drawn yet.

---

## Milestone 5 – Motors

Goal: Automation motion works.

- [x] `Motor` data structure (velocity, target position, enabled flag, joint ID)
  - Now the "Actuator": Position/Speed mode, acceleration, and expression bindings. Position targets come from poses, operations or `targetExpr`.
- [x] Add motor to selected object's joint (toolbar button)
- [x] Motor properties in Properties panel (velocity, target, enabled checkbox)
- [x] Remove motor button in Properties panel
- [x] Simulation engine: motor advances joint angle / displacement each tick
- [x] Ping-pong bounce at joint limits
  - Replaced: joints now clamp at their limits. Back-and-forth motion is a looping sequence (see the pendulum template).
- [x] Play / Pause / Reset buttons in toolbar
- [x] Elapsed time display in status bar

---

## Milestone 6 – Save / Load

Goal: Projects persist.

- [x] JSON serialization (`ProjectSerializer::save`)
- [x] JSON deserialization (`ProjectSerializer::load`)
- [x] File menu: New / Open / Save / Save As
- [x] File format: `.mstudio` (JSON)
- [x] Keyboard shortcuts: Ctrl+N, Ctrl+O, Ctrl+S

---

## Post-MVP

- [x] MuJoCo full physics integration (Milestone 3 completion)
- [ ] Joint / motor limit visualisation overlays
- [ ] CAD import (STEP, SolidWorks)
  - STEP is done (OpenCASCADE XCAF, keeps the assembly tree, File › Import STEP… or drag and drop). Native SolidWorks files aren't supported.
- [x] Collision editing (Collision Body section in Properties: add/remove, shape type, dimensions)
- [ ] Inverse Kinematics
- [ ] Path Planning
- [ ] PLC Integration
  - There is an internal PLC-style scan engine (signals, sequences, expressions), but nothing connects to an external PLC yet (no Modbus/OPC UA).
- [ ] Robot Programming
- [ ] Sensors
- [ ] Digital Twin
- [x] Undo / Redo (whole-scene snapshots, Ctrl+Z / Ctrl+Shift+Z)
- [x] Object duplication (Ctrl+D, or Duplicate in the hierarchy menu): copies the subtree with its joints, actuators, rigid bodies and conveyor belts
- [x] File › Open Recent
- [ ] Multi-select
  - Partly done: the hierarchy has extended selection (Ctrl/Shift click) for deleting several items. The viewport and Properties panel still work on one object.
- [ ] Timeline scrubbing

---

## Done Beyond the Original Plan

- [x] Automation system: signals, actuators and poses, sequences with a Gantt view, a runtime page with signal traces, and an expression language
- [x] Starter templates (File › Open Template…)
- [x] Conveyor belts
- [x] Translate gizmo (drag along X/Y/Z)
- [x] Joint axis picking on model geometry (round edges, cylinders, straight edges, faces), with an arrow showing the rotation direction and a Flip button
- [x] UI redesign: Islands dark/light themes and Lucide icons
- [x] Per-theme viewport lighting controls
- [x] View cube, orthographic/perspective projection, Fit All (F)
