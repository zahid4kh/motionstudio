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
- [x] Object selection via ray–AABB intersection (left click)
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
- [ ] MuJoCo rigid-body simulation (scene → MJCF XML → simulate)
- [ ] Read-back simulated body poses into SceneGraph transforms
- [ ] Gravity / collisions driven by MuJoCo

> **Current state:** MuJoCo library linked but not yet wired into the simulation loop.
> The stub `SimulationEngine` performs basic motor integration (joint angle += velocity × dt) without real physics.

---

## Milestone 4 – Joint Editor

Goal: Mechanical motion works.

- [x] `Joint` data structure (Revolute / Prismatic, axis, limits, initial angle)
- [x] Add joint between selected child and its parent (toolbar button + SceneGraph)
- [x] Joint properties in Properties panel (type, axis, limits, initial angle)
- [x] Remove joint button in Properties panel
- [x] Joint current value applied to child body transform during rendering
- [ ] Joint limit visualisation in viewport (arc / arrow overlay)

---

## Milestone 5 – Motors

Goal: Automation motion works.

- [x] `Motor` data structure (velocity, target position, enabled flag, joint ID)
- [x] Add motor to selected object's joint (toolbar button)
- [x] Motor properties in Properties panel (velocity, target, enabled checkbox)
- [x] Remove motor button in Properties panel
- [x] Simulation engine: motor advances joint angle / displacement each tick
- [x] Ping-pong bounce at joint limits
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

## Not Started (Post-MVP)

- [ ] MuJoCo full physics integration (Milestone 3 completion)
- [ ] Joint / motor limit visualisation overlays
- [ ] CAD import (STEP, SolidWorks)
- [ ] Collision editing
- [ ] Inverse Kinematics
- [ ] Path Planning
- [ ] PLC Integration
- [ ] Robot Programming
- [ ] Sensors
- [ ] Digital Twin
- [ ] Undo / Redo
- [ ] Object duplication
- [ ] Multi-select
- [ ] Timeline scrubbing
