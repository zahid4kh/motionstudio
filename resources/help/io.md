# External I/O (Modbus TCP)

MotionStudio can act as the **plant** for a real controller. A PLC, CODESYS, OpenPLC or any other Modbus TCP client connects to the model and runs it: it reads the sensors and writes the commands, while MotionStudio moves the machine. That is virtual commissioning: you can test the PLC program before the real machine exists.

## Set it up

1. Open **Tools › External I/O (Modbus TCP)…** (or **External I/O…** on the Signals tab).
2. Give each signal the PLC should see an **address**. **Assign Free Addresses** numbers all the signals for you, or you can type addresses in the table, or in the Signals tab's *Modbus* column.
3. Tick **Run the server**. Port 5020 works without administrator rights; the standard port 502 needs them on Linux. Choose **All network interfaces** if the PLC is on another machine.
4. Press **Play**, then start the PLC program.

The status bar shows `Modbus :5020 · 1 client` while the server runs. It starts again with the app until you untick it.

## Which table a signal is in

| Signal | Modbus table | The PLC… | Functions |
|---|---|---|---|
| **Input**, Bool | Coil | writes it | 5, 15 (read: 1) |
| **Input**, Int or Real | Holding register | writes it | 6, 16 (read: 3) |
| Internal, Derived or **Sensor**, Bool | Discrete input | reads it | 2 |
| Internal or Derived, Int or Real | Input register | reads it | 4 |

- **Bool** is one bit.
- **Int** is one signed 16-bit register.
- **Real** is a 32-bit float in **two** registers, high word first, so it takes two addresses.

Addresses are per table, so coil 0 and discrete input 0 are different. Two signals can't overlap in one table; the problems indicator reports it if they do.

To give the PLC an actuator's position, add a Derived **Real** signal, for example:

```
J1.pos
```

## Typical mapping

| MotionStudio | Kind | Modbus | In the PLC program |
|---|---|---|---|
| `PartAtEnd` (beam sensor) | Sensor | discrete input 0 | `%IX0.0` |
| `AtGripper` (zone sensor) | Sensor | discrete input 1 | `%IX0.1` |
| `BeltRun` | Input Bool | coil 0 | `%QX0.0` |
| `GripperClose` | Input Bool | coil 1 | `%QX0.1` |
| `LiftTarget` | Input Real | holding registers 0–1 | `%QW0`–`%QW1` |

Then bind the model to those inputs: the belt's *Speed expr* `BeltRun ? 0.4 : 0`, the lift's Target binding `LiftTarget`, and so on. The PLC program then decides everything.

## Good to know

- While the simulation is stopped, reads return the signals' initial values, and writes are accepted but change nothing.
- PLC writes are applied at the start of the next scan, one value per scan, like the Runtime tab's inputs.
- The two halves of a Real can arrive in separate writes; the server combines them.
- Unmapped addresses read as 0. Writing a single coil or register that has no signal gets the *illegal data address* exception.
