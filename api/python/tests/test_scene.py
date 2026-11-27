"""Live tests for building scenes: start an empty motionstudio-server, build a
small line through the library, check it, run it, save and open it.

    python3 -m unittest discover -s tests -p test_scene.py     (from api/python)
"""
import os
import tempfile
import unittest

from test_live import SERVER, Server, ms  # noqa: F401  (also puts the library on the path)


def close(a, b, tol=0.01):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


@unittest.skipUnless(os.path.exists(SERVER), "motionstudio-server is not built")
class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = Server()                       # an empty scene
        cls.studio = ms.connect(port=cls.server.port)
        cls.scene = cls.studio.scene

    @classmethod
    def tearDownClass(cls):
        cls.studio.close()
        cls.server.stop()

    def setUp(self):
        self.studio.reset()
        self.scene.new()

    def test_objects(self):
        scene = self.scene
        cell = scene.add_frame("Cell", position=ms.Pose(x=1000))
        table = scene.add_box("Table", size=(800, 40, 600), parent=cell, position=(0, 700, 0), color="#336699")
        self.assertEqual(table.path, "Cell/Table")
        self.assertTrue(close(table.world.__dict__.values(), [1000, 700, 0, 0, 0, 0]))
        self.assertEqual(tuple(table.size), (800, 40, 600))
        box = table.bounds()
        self.assertTrue(close(box.min, (600, 680, -300)) and close(box.size, (800, 40, 600)))
        cup = table.add("cylinder", "Cup", diameter=80, height=120, position=ms.Pose(x=100, y=80))
        self.assertTrue(close(cup.world.__dict__.values(), [1100, 780, 0]))
        # Placed in another frame: a Z-up frame stood upright
        cad = scene.add_frame("CAD", position=ms.Pose(rx=-90))
        pin = scene.add_frame("Pin", frame=cad, position=(10, 0, 250))
        self.assertTrue(close(list(pin.world.__dict__.values())[:3], (10, 250, 0)))
        self.assertTrue(close(list(pin.pose_in(cad).__dict__.values())[:3], (10, 0, 250)))
        # A handle follows its object through renames and moves
        pin.rename("Pin A").set_parent(cell)
        self.assertEqual(pin.path, "Cell/Pin A")
        self.assertTrue(close(list(pin.world.__dict__.values())[:3], (10, 250, 0)))
        pin.move_to(z=-40)
        self.assertAlmostEqual(pin.position.z, -40, places=3)
        self.assertEqual({o.name for o in cell.children}, {"Table", "Pin A"})
        self.assertEqual([o.path for o in scene.objects("Pin*")], ["Cell/Pin A"])
        copy = pin.duplicate("Pin B", position=ms.Pose(x=-50), frame="world")
        self.assertAlmostEqual(copy.world.x, -50, places=3)
        group = scene.group([pin, copy], "Pins")
        self.assertEqual(sorted(o.name for o in group.children), ["Pin A", "Pin B"])
        self.assertEqual(copy.remove(), 1)
        with self.assertRaises(ms.NotFoundError):
            scene.object("Nothing")
        with self.assertRaises(ms.MotionStudioError) as e:
            pin.set(size=10)
        self.assertEqual(e.exception.code, "bad_request")

    def test_joints(self):
        scene = self.scene
        cell = scene.add_frame("Cell", position=(1000, 0, 0))
        arm = scene.add_frame("Arm", parent=cell, position=(300, 800, 0))
        turn = scene.add_joint("Turn", cell, arm, (0, 1, 0), pivot=(1000, 800, 0), min=-170, max=170,
                               actuator={"speed": 90})
        self.assertEqual(turn.actuator, "Turn")
        slide = arm.add("frame", "Slide")
        lift = scene.add_joint("Lift", arm, slide, (0, 1, 0), type="prismatic", min=0, max=200,
                               actuator={"speed": 150})
        self.assertEqual(lift.info()["max"], 200)
        turn.value = 90
        lift.value = 100
        self.assertTrue(close(list(arm.world.__dict__.values())[:3], (1000, 800, -300)))
        self.assertTrue(close(list(slide.world.__dict__.values())[:3], (1000, 900, -300)))
        wheel = scene.add_frame("Wheel", parent=cell, position=(0, 0, 500))
        follower = scene.add_joint("Wheel", cell, wheel, (0, 0, 1), continuous=True, follows={"joint": turn.name, "ratio": -2})
        self.assertIsNone(follower.actuator)
        self.assertAlmostEqual(follower.value, -180, places=3)
        # A part moved under another keeps its joint
        base = scene.add_frame("Base", parent=cell, position=ms.Pose(y=50, ry=30))
        before = slide.world
        arm.set_parent(base)
        self.assertTrue(close(slide.world.__dict__.values(), before.__dict__.values()))
        self.assertEqual(scene.joint("Turn").info()["parent"], "Cell/Base")
        # A pose and a joint removed: the part stays
        scene.add_pose("Up", {"Lift": 150})
        lift.remove()
        self.assertTrue(close(slide.world.__dict__.values(), before.__dict__.values()))
        self.assertIn("Up", scene.info()["poses"])   # the pose stays, without the removed joint
        with self.assertRaises(ms.NotFoundError):
            scene.joint("Lift")

    def test_line_runs_saves_and_opens(self):
        scene = self.scene
        line = scene.add_frame("Line")
        belt = scene.add_box("Belt", size=(2000, 100, 400), parent=line, position=(0, 450, 0))
        belt.set_belt("Run ? 0.3 : 0", (1, 0, 0))
        part = scene.add_box("Box", size=100, position=(-900, 551, 0)).set_body(mass=0.5)
        self.assertEqual(part.info()["body"]["mass"], 0.5)
        stand = scene.add_frame("Stand", parent=line, position=(1200, 0, 0))
        ram = scene.add_box("Ram", size=50, parent=stand, position=(0, 300, 0))
        scene.add_joint("Ram", stand, ram, (0, 1, 0), type="prismatic", min=0, max=150, actuator={"speed": 300})
        scene.add_signal("Run", "input", value=True)
        scene.add_signal("Seen", type="int")
        scene.add_sensor("AtEnd", "beam", mount=belt, frame="world", at=(600, 560, -300), direction=(0, 0, 1),
                         range=600, detects="bodies")
        scene.add_pose("Up", {"Ram": 150})
        scene.add_source("Feeder", part, interval=3, max=3)
        scene.add_sink("Out", mount=belt, at=(950, 100, 0), size=(100, 200, 400))
        ops = (ms.Ops()
               .wait_for("Part", "AtEnd")
               .set("Count", "Seen = Seen + 1")
               .move_to_pose("Raise", "Up")
               .move("Lower", "Ram", 0, speed=600)
               .wait_for("Gone", "!AtEnd", after="Count")
               .wait("Both", 0.1, after=["Lower", 4]))
        seq = scene.add_sequence("Counter", ops, loop=True, enable="Run")
        self.assertEqual([o["after"] for o in seq["operations"]], [[], [0], [1], [2], [1], [3, 4]])
        self.assertEqual(seq["operations"][3]["speed"], "0.6")      # 600 mm/s as the automation's m/s
        self.assertEqual(scene.check(), [])
        with self.assertRaises(ms.MotionStudioError):
            scene.add_signal("Run")

        with tempfile.TemporaryDirectory() as d:
            path = scene.save(os.path.join(d, "line.mstudio"))
            scene.new()
            self.assertEqual(scene.objects(), [])
            info = scene.open(path)
            self.assertEqual(info["sequences"], ["Counter"])
            self.assertEqual(info["sources"], ["Feeder"])
            self.assertEqual(scene.sequence("Counter")["operations"], seq["operations"])

            # It runs: parts reach the beam, the ram rises for each, the sink takes them
            self.studio.sleep(12)
            self.assertGreaterEqual(self.studio.signals["Seen"], 2)
            self.assertGreaterEqual(self.studio.signals["Out.count"], 2)
            with self.assertRaises(ms.SimulationError):
                scene.add_frame("Late")
            self.studio.reset()

            # Merged in again under a parent; clashing names are renamed
            hall = scene.add_frame("Hall", position=(0, 0, 3000))
            root = scene.import_file(path, parent=hall)
            self.assertEqual(root.info()["parent"], "Hall")
            self.assertEqual(len(scene.info()["sequences"]), 2)
            self.assertEqual(scene.check(), [])
        self.assertEqual(scene.remove("Seen"), "signal")


if __name__ == "__main__":
    unittest.main()
