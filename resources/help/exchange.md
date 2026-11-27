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

**STEP:** you can also drop `.step` or `.stp` files onto the window. A progress window shows the three stages (**Read file**, **Build shapes**, **Mesh parts**), the percentage and the time left, which part is being meshed, and how long it has been running. Large assemblies can take a while:

- **Time left** is guessed from the file size at first. MotionStudio remembers how fast each stage ran, so the guess gets better with every import.
- **Working…** in place of a time means progress has stopped moving, usually while one very large part is meshing. The window names that part.
- **Cancel** (or Esc) stops the import and adds nothing. Reading the file can't be interrupted, so a cancel during **Read file** takes effect once the file is read.

Identical parts are meshed once, several parts are meshed at the same time, and the status bar shows how long the import took.

A part that holds several bodies (NX and other CAD systems export whole machines that way) becomes one object per body, named as in the CAD file.

**Organise the import** before you add joints. Each moving piece needs its own branch in the Hierarchy: the arm with everything it carries, the lift deck with its belt.

- **Group** (Ctrl+G, or right-click in the Hierarchy) puts the selected parts into a new empty frame at their centre, under the parent they share. Nothing moves.
- **Drag** parts in the Hierarchy: onto a part to put them under it, between two parts to put them next to it, onto the empty space below the list to make them top level. **Move to Top Level** in the right-click menu does the same.
- Parts keep their place in the world, and a part with a joint keeps it and moves the same way. Only at the top level is a joint removed, since there is no parent for it to move on.

## Send models out

**File › Export › URDF (ROS)…** and **MJCF (MuJoCo)…** write the mechanism with every part's geometry as STL files in a `meshes` folder next to the file. The pose with all joint values at 0 is the reference pose.

- Joint limits, continuous joints and couplings (`mimic` in URDF, joint equalities in MJCF) are kept.
- **MJCF** also gets a position actuator for each Position-mode actuator, a velocity actuator for each Speed-mode one, free joints for rigid bodies, colours and the floor. It opens directly in MuJoCo's `simulate`.
- Signals, sequences, sensors and conveyor belts are not part of these formats.

Other exports: **Export Video…** on the timeline (MP4, GIF, PNG frames), and the Modbus TCP server for a live PLC ([External I/O](help:io)).
