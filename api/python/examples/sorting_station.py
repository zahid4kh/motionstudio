"""Builds a digital twin of a sorting station from its CAD, through the API.

    python3 sorting_station.py path/to/sorting.stp station.mstudio

The STEP file (an NX export, Z up) holds the whole station as one part with
named bodies. This script groups the bodies into what moves together, puts
joints through the CAD's own geometry, adds the belts, sensors, part feeders,
bins and the logic, and saves the project:

- a SCARA robot picks each part from the infeed belt with a suction cup (a
  laser measures how far down to go) and sets it on the transfer belt;
- the sorter at the end lets flat parts run straight on into bin A, and lifts,
  turns and tips tall ones into bin B;
- the bins fill up; the input EmptyBins empties them.

Coordinates in the station's own frame are the CAD's (mm, Z up), so
everything below is given with frame=station.
"""

import math
import sys

import motionstudio as ms


def build(studio: ms.Studio, step: str, out: str) -> None:
    scene = studio.scene
    studio.reset()
    scene.new(discard=True)

    station = scene.import_file(step, name="Sorting Station")     # Z-up CAD, stood upright
    station.move_to(y=670)                                        # the legs' feet are at CAD z = -670 mm
    S = station                                                   # the frame every CAD number below is in

    def move(names, to):
        for n in names:
            scene.object(n).set_parent(to)

    def numbered(base, a, b):
        return [f"{base}:{i}" for i in range(a, b + 1)]

    def frame(name, parent):
        return scene.add_frame(name, parent=parent)

    # ── Group the bodies into what moves together ───────────────────────────
    infeed = frame("Infeed Conveyor", station)
    move(["belt - Copy:2", "profile - Copy:2", "profile2 - Copy:2", "roller:3", "roller2:3", "bearing:3", "Part2:7",
          "servo_1:2", "servo_1:3", "servo pin gear:2", "servo pin gear:3", "stopper2:2", "stopper2:3"]
         + numbered("legs", 1, 8), infeed)
    transfer = frame("Transfer Conveyor", station)
    move(["belt - Copy:1", "profile - Copy:1", "profile2 - Copy:1", "roller:2", "roller2:2", "bearing:2", "Part2:6"]
         + numbered("legs", 9, 16), transfer)

    scara = frame("SCARA", station)
    move(["actuator_base"], scara)
    link1 = frame("J1 Arm", scara)
    move(["arm1"], link1)
    link2 = frame("J2 Arm", link1)
    move(["arm2:1", "arm2:2"], link2)
    quill = frame("J3 Quill", link2)
    move(["shaft"], quill)

    # The elbow: the round end of the first arm (its largest circle furthest along +X)
    rings = [f for f in scene.object("arm1").features(["circle"], frame=S) if abs(f.axis.z) > 0.99]
    far = max(f.origin.x + f.radius for f in rings)
    hub = max((f for f in rings if f.origin.x + f.radius > far - 1), key=lambda f: f.radius)
    elbow = (hub.origin.x, hub.origin.y)
    shaft = scene.object("shaft").bounds(S)
    wrist = (shaft.center.x, shaft.center.y)
    print(f"SCARA: elbow at {elbow[0]:.1f} {elbow[1]:.1f} mm, quill at {wrist[0]:.1f} {wrist[1]:.1f}, tip z {shaft.min.z:.1f}")
    tcp = scene.add_frame("TCP", parent=quill, frame=S, position=(wrist[0], wrist[1], shaft.min.z))

    sorter = frame("Sorter", station)
    move(["servo"], sorter)
    drive = frame("Servo Drive", sorter)
    move(["servo_shaft", "Bevel Gear21"] + numbered("Part2", 2, 5), drive)
    table = frame("Turntable", sorter)
    move(["Bevel Gear11", "circle", "static", "slider_holder"] + numbered("legs_rot", 1, 4), table)
    lift = frame("Lift", table)
    move(["linear", "conv_holder", "belt", "roller:1", "roller2:1", "bearing:1", "Part2:1", "profile", "profile2",
          "servo_1:1", "servo_1:4"] + numbered("bottom", 1, 4) + numbered("legs_rot_telescope", 1, 4), lift)
    # The two light-barrier heads have one name in the file: told apart by where they are
    for head in scene.objects("SENSORR"):
        side = "Right" if head.bounds(S).center.y < 0 else "Left"
        head.set(parent=lift, name=f"Light Barrier {side}")
    gate_l = frame("Gate Left", lift)
    move(["stopper2:1", "servo pin gear:1"], gate_l)
    gate_r = frame("Gate Right", lift)
    move(["stopper2:4", "servo pin gear:4"], gate_r)
    turn_axis = scene.object("circle").bounds(S).center
    pin_l = scene.object("servo pin gear:1").bounds(S).center
    pin_r = scene.object("servo pin gear:4").bounds(S).center
    servo_axis = scene.object("servo_shaft").bounds(S).center

    bin_a = frame("Bin A", station)
    move(numbered("table", 1, 7), bin_a)
    bin_b = frame("Bin B", station)
    move(numbered("table", 8, 14), bin_b)
    bodies = scene.object("Bodies")
    if not bodies.info()["children"]:
        bodies.remove()

    # ── Joints and actuators (axes and pivots in the CAD frame) ─────────────
    Z = (0, 0, 1)
    scene.add_joint("J1", scara, link1, Z, frame=S, pivot=(0, 0, 0), min=-170, max=170,
                    actuator={"speed": 150, "acceleration": 600})
    scene.add_joint("J2", link1, link2, Z, frame=S, pivot=(elbow[0], elbow[1], 0), min=-150, max=150,
                    actuator={"speed": 180, "acceleration": 700})
    scene.add_joint("J3", link2, quill, (0, 0, -1), type="prismatic", frame=S, min=0, max=150,
                    actuator={"speed": 250, "acceleration": 2000})
    scene.add_joint("Turn", sorter, table, Z, frame=S, pivot=(turn_axis.x, turn_axis.y, 0), min=-5, max=95,
                    actuator={"speed": 60, "acceleration": 120})
    scene.add_joint("Lift", table, lift, Z, type="prismatic", frame=S, min=0, max=100,
                    actuator={"speed": 100, "acceleration": 500})
    scene.add_joint("Gate", lift, gate_l, Z, frame=S, pivot=(pin_l.x, pin_l.y, 0), min=-90, max=0,
                    actuator={"speed": 180, "acceleration": 900})
    scene.add_joint("Gate Right", lift, gate_r, Z, frame=S, pivot=(pin_r.x, pin_r.y, 0), min=0, max=90,
                    follows={"joint": "Gate", "ratio": -1})
    # The servo turns the table through bevel gears (Ø 219 mm on the table, Ø 185 mm on the servo)
    scene.add_joint("Servo", sorter, drive, (1, 0, 0), frame=S, pivot=(0, servo_axis.y, servo_axis.z),
                    min=-360, max=360, follows={"joint": "Turn", "ratio": -219 / 185})

    # ── Parts: the file's two boxes become the part templates ──────────────
    belt_top = 121                                       # CAD z of the belts' tops, mm

    def make_part(body, name, y):
        part = scene.object(body)
        part.set(parent=None, name=name)
        part.set_body(mass=1.0)
        b = part.bounds(S)
        here = part.pose_in(S)
        # Centred on the infeed belt at CAD (-535, y), its bottom 1 mm above the belt
        part.move_to(frame=S, x=here.x - b.center.x - 535, y=here.y - b.center.y + y,
                     z=here.z - b.min.z + belt_top + 1)
        return part

    flat = make_part("MODEL1", "Flat Part", 870)
    tall = make_part("MODEL2", "Tall Part", 860)
    tall.move_to(frame=S, y=640)          # its template waits upstream, clear of the other

    # ── Signals and sensors ──────────────────────────────────────────────────
    scene.add_signal("Start", "input", value=True, description="Feed parts, run the belts, the robot and the sorter")
    scene.add_signal("FeedFlat", description="Pulse: the feeder puts a flat part on the infeed")
    scene.add_signal("FeedTall", description="Pulse: the feeder puts a tall part on the infeed")
    scene.add_signal("Depth", type="real", description="How far the quill goes down for the part it picks (m)")
    scene.add_signal("DeckRun", value=True, description="The sorter's belt runs")
    scene.add_signal("SorterBusy", description="The sorter has a part: hold the next at the end of the transfer belt")
    scene.add_signal("ToB", description="The part on the sorter goes to bin B (it is tall)")
    scene.add_signal("SortedA", type="int", description="Parts sorted into bin A (flat)")
    scene.add_signal("SortedB", type="int", description="Parts sorted into bin B (tall)")
    X, Y, DOWN = (1, 0, 0), (0, 1, 0), (0, 0, -1)

    def sensor(name, type, mount, **kw):
        scene.add_sensor(name, type, mount=mount, frame=S, detects="bodies", **kw)

    sensor("PartAtPick", "beam", infeed, at=(-760, -60, 150), direction=X, range=450,
           description="Light barrier across the end of the infeed: a part waits for the robot")
    sensor("PickGap", "distance", infeed, at=(-535, 20, 400), direction=DOWN, range=500,
           description="Laser above the pick spot: distance to the part's top (m)")
    # In the TCP's own frame (turned like the station's): just below the cup
    scene.add_sensor("AtSuction", "zone", mount=tcp, at=(0, 0, -10), size=(40, 40, 30), detects="bodies",
                     description="Zone under the suction cup")
    sensor("AtTransferEnd", "beam", transfer, at=(1450, -180, 150), direction=Y, range=360,
           description="Light barrier before the end of the transfer belt (far enough back that the turning sorter clears a waiting part)")
    sensor("OnDeck", "zone", lift, at=(2061, 5, 190), size=(760, 380, 120), description="A part on the sorter's belt")
    sensor("BinAFull", "zone", bin_a, at=(2708, 12, -40), size=(470, 470, 120), description="The pile in bin A reaches near its rim")
    sensor("BinBFull", "zone", bin_b, at=(2084, 624, -40), size=(470, 470, 120), description="The pile in bin B reaches near its rim")
    scene.add_signal("EmptyBins", "input", description="Switch on to empty both bins (an operator swaps them)")
    scene.add_sink("BinA", mount=bin_a, frame=S, at=(2708, 12, -224), size=(470, 470, 500), enable="EmptyBins")
    scene.add_sink("BinB", mount=bin_b, frame=S, at=(2084, 624, -224), size=(470, 470, 500), enable="EmptyBins")
    sensor("PlaceTaken", "zone", transfer, at=(560, 0, 147), size=(260, 300, 44),
           description="A part on the transfer belt where the robot puts the next one")
    sensor("AtGate", "beam", lift, at=(2132, -180, 159), direction=Y, range=360,
           description="The sorter's light barrier, in front of its gates")
    sensor("Tall", "beam", lift, at=(2080, -180, 210), direction=Y, range=360,
           description="Beam 90 mm above the sorter's belt: only tall parts cross it")

    # ── Belts (speeds as expressions, in m/s) ─────────────────────────────────
    scene.object("belt - Copy:2").set_belt("Start && !PartAtPick ? 0.15 : 0", (0, -1, 0), frame=S)
    scene.object("belt - Copy:1").set_belt("Start && !(AtTransferEnd && (SorterBusy || OnDeck)) ? 0.2 : 0", X, frame=S)
    scene.object("belt").set_belt("DeckRun ? (SorterBusy ? 0.35 : 0.2) : 0", X, frame=S)

    # ── Bins hold what falls in: their walls and a floor collide ────────────
    for n in numbered("table", 1, 14):
        if n not in ("table:4", "table:12"):      # the bins' base frames: their boxes fill the bin
            scene.object(n).set_body()
    for bin_, x0, y0 in ((bin_a, 2708, 12), (bin_b, 2084, 624)):
        floor = scene.add_box("Floor", size=(440, 440, 20), parent=bin_, frame=S, position=(x0, y0, -484),
                              color="#aaafb4")
        floor.set_body()

    # ── Part feeder ───────────────────────────────────────────────────────────
    scene.add_source("FlatFeeder", flat, trigger="FeedFlat")
    scene.add_source("TallFeeder", tall, trigger="FeedTall")
    scene.add_sequence("Feeder", ms.Ops()
                       .set("Flat part", "FeedFlat = true")
                       .wait("Pulse", 0.2, on_finish="FeedFlat = false")
                       .wait("Gap", 11)
                       .set("Tall part", "FeedTall = true")
                       .wait("Pulse", 0.2, on_finish="FeedTall = false")
                       .wait("Gap", 11),
                       loop=True, enable="Start")

    # ── SCARA poses: two-link inverse kinematics in the CAD plane ───────────
    l1 = math.hypot(*elbow)
    l2 = math.hypot(wrist[0] - elbow[0], wrist[1] - elbow[1])

    def pose(name, x, y):
        c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        q2 = math.acos(max(-1.0, min(1.0, c2)))
        q1 = math.atan2(y, x) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        print(f"pose {name}: J1 {math.degrees(q1):.1f} J2 {math.degrees(q2):.1f}")
        scene.add_pose(name, {"J1": math.degrees(q1), "J2": math.degrees(q2), "J3": 0})

    pose("Above Pick", -535, 20)
    pose("Above Place", 560, 0)
    scene.add_sequence("Robot", ms.Ops()
                       .move_to_pose("Above pick", "Above Pick", speed=100)
                       .wait_for("Wait for part", "PartAtPick")
                       .wait("Settle", 0.3)
                       .set("Measure", "Depth = PickGap - 0.108")
                       .move("Down", "J3", "PickGap - 0.108")         # Depth is written at the next scan
                       .grip("Suck", tcp, sensor="AtSuction")
                       .move("Up", "J3", 0)
                       .move_to_pose("Above place", "Above Place", speed=100)
                       .wait_for("Spot clear", "!PlaceTaken")
                       .move("Down", "J3", "Depth")
                       .release("Let go", tcp)
                       .move("Up", "J3", 0),
                       loop=True, enable="Start")

    # ── Sorter: flat parts straight on into bin A, tall ones lifted, turned and into bin B
    scene.add_sequence("Sorter", ms.Ops()
                       .set("Accept", "DeckRun = true; SorterBusy = false")
                       .wait_for("Wait for part", "AtGate")
                       .set("Hold", "DeckRun = false; SorterBusy = true; ToB = Tall")
                       .wait("Settle", 0.3)
                       .wait_for("Bin has room", "ToB ? !BinBFull : !BinAFull")
                       .move("Lift", "Lift", "ToB ? 0.1 : 0")
                       .move("Turn", "Turn", "ToB ? 90 : 0")
                       .move("Open gates", "Gate", -85)
                       .set("Run off", "DeckRun = true")
                       .wait_for("Off the deck", "!OnDeck")
                       .wait("Into the bin", 0.5,
                             on_finish="DeckRun = false; SortedA = SortedA + !ToB; SortedB = SortedB + ToB")
                       .move("Close gates", "Gate", 0)
                       .move("Turn back", "Turn", 0)
                       .move("Lower", "Lift", 0),
                       loop=True, enable="Start")

    problems = scene.check()
    for p in problems:
        print("problem:", p)
    info = scene.info()
    path = scene.save(out)
    print(f"saved {path}: {info['objects']} objects, {len(info['joints'])} joints, "
          f"{len(info['sensors'])} sensors, {len(info['sequences'])} sequences")
    if problems:
        sys.exit(2)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with ms.connect() as studio:
        build(studio, sys.argv[1], sys.argv[2])
