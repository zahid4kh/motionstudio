# Import & Export

## Bring models in

| Menu | Reads | Notes |
|---|---|---|
| **File › Import STEP…** | `.step`, `.stp` | CAD assemblies with their part tree and colours. Add joints yourself. |
| **File › Import URDF / MJCF…** | `.urdf` (ROS), `.xml` (MuJoCo) | Robots with their joints, limits and colours. Movable joints get actuators, so you can jog and program them right away. |
| **File › Insert Component…** | the built-in library | Grippers, conveyors, a gantry, a turntable, sensors, parts and bins, added to the current scene. |

**URDF:** links become frames and visuals become parts. Boxes, cylinders and spheres are supported, and so are STL and OBJ meshes (`package://` paths are found in the folders above the file). Revolute, continuous and prismatic joints are imported, and `mimic` joints become couplings. Floating and planar joints are imported as fixed.

**MJCF:** bodies, geoms (box, sphere, cylinder, ellipsoid, mesh; capsules as cylinders), hinge and slide joints, default classes and joint equalities (as couplings) are imported. Only a body's first joint is used, and free and ball joints are skipped. If the file has actuators, those joints get actuators; otherwise every movable joint gets one.

Both formats are Z-up, and MotionStudio is Y-up, so the model is placed under a root frame turned upright. The import notes list anything that was approximated or skipped.

## Send models out

**File › Export › URDF (ROS)…** and **MJCF (MuJoCo)…** write the mechanism with every part's geometry as STL files in a `meshes` folder next to the file. The pose with all joint values at 0 is the reference pose.

- Joint limits, continuous joints and couplings (`mimic` in URDF, joint equalities in MJCF) are kept.
- **MJCF** also gets a position actuator for each Position-mode actuator, a velocity actuator for each Speed-mode one, free joints for rigid bodies, colours and the floor. It opens directly in MuJoCo's `simulate`.
- Signals, sequences, sensors and conveyor belts are not part of these formats.

Other exports: **Export Video…** on the timeline (MP4, GIF, PNG frames), and the Modbus TCP server for a live PLC ([External I/O](help:io)).
