# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Run

```bash
# Build and run (preferred)
./run.sh

# Manual build
cmake -B build -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=ON
cmake --build build -j$(nproc)

# Run (MuJoCo must be on LD_LIBRARY_PATH)
LD_LIBRARY_PATH=~/mujoco-3.10.0/lib ./build/MotionStudio

# Build Debian package
./packaging/build-deb.sh [version] [architecture]
```

MuJoCo is expected at `~/mujoco-3.10.0`. Override with `-DMUJOCO_ROOT=<path>` or the `MUJOCO_ROOT` env var. Without it, CMake emits a warning and compiles a kinematic-only stub (no `MUJOCO_ENABLED` define).

There are no automated tests.

## Architecture

**MainWindow** owns all subsystems and wires their Qt signals together. Everything else is a subsystem that receives a pointer to the shared `SceneGraph`.

```
MainWindow
├── SceneGraph          ← single source of truth for all scene data
├── Viewport            ← OpenGL 3.3 rendering, reads SceneGraph
├── SimulationEngine    ← drives physics tick, writes back to SceneGraph
├── HierarchyPanel      ← Qt tree view, reads/signals SceneGraph        (src/ui/)
├── PropertiesPanel     ← Qt form, reads/writes SceneGraph              (src/ui/)
└── TemplateDialog      ← picks a starter .mstudio from Qt resources    (src/ui/)
```

### SceneGraph (`src/scene/`)

Central data store. All entities are keyed by integer IDs allocated monotonically from `m_nextId`. Separate maps hold:
- `SceneObject` — transform, primitive type, parent/child IDs
- `Joint` — connects a parent body to a child body (revolute or prismatic)
- `Motor` — drives a joint at a target velocity
- `RigidBody` — mass/gravity/collision properties; enables MuJoCo physics for that object
- `ConveyorBelt` — surface velocity applied to objects on top

`SceneGraph` emits Qt signals on every mutation; subsystems react to keep themselves in sync.

### SimulationEngine (`src/physics/SimulationEngine.h`)

Two backends selected at **compile time** via `MUJOCO_ENABLED`:

- **Kinematic (always present):** `applyMotors()` increments `Joint::currentValue` each tick and rebuilds the world transform chain. Supports joint limits with ping-pong bounce.
- **MuJoCo (when enabled):** `buildMuJoCoModel()` generates an MJCF string from the scene, loads it into MuJoCo, and `syncFromMuJoCo()` writes body positions/orientations back into `SceneObject::transform` after each `mj_step`. Objects with a `RigidBody` become dynamic bodies; conveyor belts apply `mjData.xfrc_applied` forces.

MuJoCo ID maps (`m_bodyMjId`, `m_jointQposAddr`, etc.) translate between our integer IDs and MuJoCo's internal indices.

### Viewport (`src/viewport/Viewport.h`)

`QOpenGLWidget` + `QOpenGLFunctions_3_3_Core`. Three GLSL shader programs:
- `m_objShader` — shaded primitives (per-vertex normals, Phong-ish)
- `m_gridShader` — infinite grid floor plane
- `m_wireShader` — wireframe overlays (collision shapes, conveyor arrows, joint axes)

Ray-AABB picking (`tryPickObject`) and hover detection run in `mousePressEvent`/`mouseMoveEvent`. A translate gizmo (3-axis drag) is drawn over the selected object.

### Serialization (`src/serialization/ProjectSerializer.h`)

`ProjectSerializer::save/load` round-trips the full scene to a `.mstudio` file (JSON via nlohmann/json). All five entity maps are serialized; `RigidBody` and `ConveyorBelt` entries are keyed by their `objectId`.

### Templates (`templates/`, `src/ui/TemplateDialog.h`)

Starter `.mstudio` files (crank arm, robotic arm, solar system, pendulum, spinning top) are embedded via `resources.qrc` and offered on startup through `TemplateDialog`, which returns a `:/templates/...` Qt resource path that is loaded through the normal `ProjectSerializer::load` path.

## Key conventions

- Integer IDs are allocated by `SceneGraph::m_nextId` and never reused within a session.
- Joints always connect `parentBodyId` → `childBodyId`; the motor references a `jointId`.
- Transform rotation is stored as Euler angles in **degrees, XYZ order**.
- The `MUJOCO_ENABLED` preprocessor guard must wrap all MuJoCo API calls; the stub `#else` branches must compile cleanly without MuJoCo headers.
- Project files use the `.mstudio` extension (registered as `application/x-motionstudio-project` in the Debian package).
