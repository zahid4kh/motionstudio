"""Live tests: start motionstudio-server and drive it through the library.

    python3 -m unittest discover -s tests          (from api/python)

The server is found through $MOTIONSTUDIO_SERVER, else build/motionstudio-server
at the top of the repository.
"""
import os
import tempfile
import socket
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import motionstudio as ms  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
SERVER = os.environ.get("MOTIONSTUDIO_SERVER", str(ROOT / "build" / "motionstudio-server"))


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("localhost", 0))
        return s.getsockname()[1]


class Server:
    def __init__(self, *args: str):
        self.port = free_port()
        self.proc = subprocess.Popen([SERVER, *args, "--port", str(self.port), "--speed", "20"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        line = self.proc.stdout.readline()
        if "Serving" not in line:
            self.proc.kill()
            raise RuntimeError("motionstudio-server did not start: " + line)

    def stop(self):
        self.proc.terminate()
        self.proc.wait(5)


@unittest.skipUnless(os.path.exists(SERVER), "motionstudio-server is not built")
class ArmTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = Server("--robot", "6-axis 0.9")
        cls.studio = ms.connect(port=cls.server.port)
        cls.robot = cls.studio.robot()

    @classmethod
    def tearDownClass(cls):
        cls.studio.close()
        cls.server.stop()

    def setUp(self):
        self.studio.reset()

    def assertPose(self, a: ms.Pose, b: ms.Pose, tol=0.05):
        for k in ("x", "y", "z"):
            self.assertAlmostEqual(getattr(a, k), getattr(b, k), delta=tol, msg=f"{k}: {a} vs {b}")

    def test_finds_the_robot_and_its_tool(self):
        self.assertEqual(self.robot.name, "Robot")
        self.assertEqual(self.robot.tool_names, ["TCP"])
        self.assertIn("A1", self.robot.joint_names)
        info = {j.name: j for j in self.robot.joint_info()}
        self.assertEqual(info["A1"].unit, "deg")
        self.assertLess(info["A2"].min, info["A2"].max)

    def test_straight_lines(self):
        start = self.robot.pose()
        m = self.robot.move_l(z=100, speed=400)
        self.assertEqual(m.state, "done")
        self.assertPose(self.robot.pose(), start.replace(z=100))
        self.robot.move_by(y=-80)
        self.assertPose(self.robot.pose(), start.replace(z=100).offset(y=-80))
        self.robot.move_l(start)
        self.assertPose(self.robot.pose(), start)
        self.assertTrue(self.studio.running)

    def test_tool_frame_moves(self):
        start = self.robot.pose()
        tool = self.robot.tool()
        tool.move_by(y=40, frame="tool")      # the tool's +Y points up (it approaches along -Y)
        self.assertAlmostEqual(tool.pose().y, start.y + 40, delta=0.05)
        w = tool.pose("world")
        self.assertAlmostEqual(w.y, start.y + 40, delta=0.05)   # the robot stands at the origin

    def test_joint_moves_and_poses(self):
        self.robot.move_joints(A1=30, A2=-10, speed=80)
        j = self.robot.joints
        self.assertAlmostEqual(j["A1"], 30, places=3)
        self.assertAlmostEqual(j["A2"], -10, places=3)
        self.robot.move_joints({"A1": 10}, relative=True)
        self.assertAlmostEqual(self.robot.joints["A1"], 40, places=3)
        self.robot.home()
        self.assertAlmostEqual(self.robot.joints["A1"], 0, places=3)
        self.assertIn("Robot Home", self.robot.poses)
        p = self.robot.pose().offset(x=-100, y=-100)
        self.robot.move_j(p)
        self.assertPose(self.robot.pose(), p)

    def test_motions_run_in_parallel_and_queue(self):
        a = self.robot.move_joints(A1=40, speed=30, wait=False)
        b = self.robot.move_joints(A1=0, speed=100, wait=False)     # same joint: waits for a
        self.assertIn(a.state, ("queued", "running"))
        b.wait()
        self.assertEqual(a.state, "done")
        self.assertAlmostEqual(self.robot.joints["A1"], 0, places=3)
        self.robot.move_joints(A1=-90, speed=10, wait=False)
        self.studio.sleep(0.3)
        self.assertEqual(self.robot.stop(), 1)
        held = self.robot.joints["A1"]
        self.assertTrue(-90 < held < 0)

    def test_threads_share_a_connection(self):
        results = []
        def worker(v):
            self.robot.move_joints(A6=v, speed=50)
            results.append(self.robot.joints["A6"])
        threads = [threading.Thread(target=worker, args=(v,)) for v in (20, 40)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(10)
        self.assertEqual(len(results), 2)

    def test_errors_say_what_is_wrong(self):
        with self.assertRaises(ms.OutOfReachError) as e:
            self.robot.move_l(x=5000)
        self.assertIn("out of reach", str(e.exception))
        with self.assertRaises(ms.OutOfLimitsError):
            self.robot.move_joints(A2=999)
        with self.assertRaises(ms.NotFoundError) as e:
            self.robot.move_joints(A9=1)
        self.assertIn("A1", str(e.exception))                     # lists the joints
        with self.assertRaises(ms.NotFoundError):
            self.studio.robot("Nobody")
        with self.assertRaises(ms.MotionCancelled):
            m = self.robot.move_joints(A1=-170, speed=5, wait=False)
            m.cancel()
            m.wait()
        with self.assertRaises(ms.WaitTimeout):
            self.robot.move_joints(A1=170, speed=5, wait=False).wait(timeout=0.2)

    def test_signals(self):
        self.assertIn("Grip", self.studio.signals)
        self.studio.signals["Grip"] = True
        self.studio.wait_until("Grip", timeout=2)
        self.assertEqual(self.studio.signals["Grip"], 1)
        self.studio.set("Grip", False)
        t = self.studio.sleep(0.5)
        self.assertGreaterEqual(t, 0.5)
        self.assertEqual(self.studio.get("Grip"), 0)
        with self.assertRaises(ms.WaitTimeout):
            self.studio.wait_until("Grip", timeout=0.2)


@unittest.skipUnless(os.path.exists(SERVER), "motionstudio-server is not built")
class HumanoidTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = Server("--humanoid", "Full-size")
        cls.studio = ms.connect(port=cls.server.port)

    @classmethod
    def tearDownClass(cls):
        cls.studio.close()
        cls.server.stop()

    def test_hands_are_tools(self):
        h = self.studio.robot("Humanoid")
        self.assertEqual(sorted(h.tool_names), ["Left Hand TCP", "Right Hand TCP"])
        with self.assertRaises(ms.NotFoundError):
            h.pose()                                              # which hand?
        hand = h.tool("Right Hand TCP", base="Torso")
        h.move_joints(RShoulderPitch=40, RElbow=70)             # a stretched arm can't move in a line
        start = hand.pose()
        hand.move_by(z=60, speed=150)
        self.assertAlmostEqual(hand.pose().z, start.z + 60, delta=0.1)
        hand.move_j(start.offset(x=40, y=40))
        self.assertAlmostEqual(hand.pose().x, start.x + 40, delta=0.1)
        with self.assertRaises(ms.OutOfReachError):              # straight down from the shoulder: too far
            hand.move_j(start.offset(y=-900))
        h.move_joints(LShoulderRoll=90, RShoulderRoll=90)
        self.assertAlmostEqual(h.joints["LShoulderRoll"], 90, places=3)
        h.home()


@unittest.skipUnless(os.path.exists(SERVER), "motionstudio-server is not built")
class WalkingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = Server("--humanoid", "Full-size")
        cls.studio = ms.connect(port=cls.server.port)

    @classmethod
    def tearDownClass(cls):
        cls.studio.close()
        cls.server.stop()

    def setUp(self):
        self.studio.reset()
        self.h = self.studio.humanoid()

    def assertPlace(self, p: ms.Place, x, z, heading=None, tol=0.5):
        self.assertAlmostEqual(p.x, x, delta=tol, msg=str(p))
        self.assertAlmostEqual(p.z, z, delta=tol, msg=str(p))
        if heading is not None:   # 180 and -180 are the same way
            self.assertAlmostEqual((p.heading - heading + 180) % 360 - 180, 0, delta=0.01, msg=str(p))

    def test_it_is_a_humanoid_with_a_gait(self):
        self.assertIsInstance(self.h, ms.Humanoid)
        self.assertTrue(self.h.walks)
        self.assertIsInstance(self.studio.robot("Humanoid"), ms.Humanoid)
        gait = self.h.default_gait
        self.assertGreater(gait.step_length, 100)
        self.assertGreater(gait.speed, 0)
        self.assertPlace(self.h.place, 0, 0, 0)
        self.assertFalse(self.h.walking)

    def test_walk_turn_and_side_step(self):
        self.assertEqual(self.h.walk(600).state, "done")
        self.assertPlace(self.h.place, 600, 0, 0)
        self.h.turn(90)
        self.assertPlace(self.h.place, 600, 0, 90)
        self.h.walk(400)                                    # facing -Z now
        self.assertPlace(self.h.place, 600, -400, 90)
        self.h.side_step(-200, speed=300)                   # to its right: +X when facing -Z
        self.assertPlace(self.h.place, 800, -400, 90)
        self.h.walk(-300, step_length=150)                  # backwards in short steps
        self.assertPlace(self.h.place, 800, -100, 90)

    def test_paths_and_places(self):
        square = [(800, 0), (800, 800), (0, 800), (0, 0)]
        self.h.walk_path(square, heading=0)
        self.assertPlace(self.h.place, 0, 0, 0)
        self.h.walk_to(500, 300, heading=180, gait=ms.Gait(speed=500, arm_swing=0))
        self.assertPlace(self.h.place, 500, 300, 180)
        self.h.walk_path([ms.Place(300, 0), {"x": 300, "z": 300}], frame="robot")   # x ahead, z to its right
        self.assertPlace(self.h.place, 200, 0)

    def test_without_waiting_and_stopping(self):
        m = self.h.walk(4000, wait=False)
        self.studio.sleep(1.2)
        st = self.h.state()
        self.assertTrue(st.walking)
        self.assertEqual(st.motion, m.id)
        self.assertGreater(st.steps, 5)
        self.assertEqual(self.h.stop_walking(), 1)
        m.wait()
        self.assertEqual(m.state, "done")
        self.assertLess(self.h.place.x, 2000)
        self.assertFalse(self.h.walking)

    def test_errors(self):
        with self.assertRaises(ms.MotionStudioError) as e:
            self.h.walk(500, step_length=5000)
        self.assertEqual(e.exception.code, "bad_request")
        with self.assertRaises(TypeError):
            self.h.walk(500, stride=3)
        with self.assertRaises(ms.MotionCancelled):
            m = self.h.walk(3000, wait=False)
            self.studio.sleep(0.5)
            self.h.stop()                                   # a hard stop: the robot halts mid-step
            m.wait()

    def test_record_save_load_and_play(self):
        with self.h.record(rate=25) as rec:
            self.h.walk_to(600, 0)
            self.assertGreater(rec.frames, 5)
        take = rec.take
        self.assertIsInstance(take, ms.Take)
        self.assertEqual(take.robot, "Humanoid")
        self.assertAlmostEqual(take.rate, 25)
        self.assertTrue(take.walks)
        self.assertIn("LKnee", take.joint_names)
        self.assertNotIn("walking", take.joint_names)
        self.assertAlmostEqual(take["x"][-1], 600, delta=0.5)
        self.assertEqual(max(take["walking"]), 1)
        self.assertEqual(set(take["swing"]) - {-1}, {0, 1})
        self.assertAlmostEqual(take.frame(len(take) // 2)["goal_x"], 600, delta=0.5)
        self.assertIs(rec.stop(), take)

        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "walk.csv")
            take.save(path)
            again = ms.Take.load(path)
        self.assertEqual(again.columns, take.columns)
        self.assertEqual(again.robot, "Humanoid")
        self.assertEqual(len(again), len(take))
        self.assertAlmostEqual(again.rows[-1][1], take.rows[-1][1], places=3)

        self.h.play(take, relative=True)                    # the same walk again, from 600
        self.assertPlace(self.h.place, 1200, 0, 0)
        self.h.play(take.select(["x", "z", "heading"] + take.joint_names), speed=3)   # back to where it was recorded
        self.assertPlace(self.h.place, 600, 0, 0)
        half = take.between(0, take.duration / 2)
        self.assertAlmostEqual(half.rows[0][0], 0)
        self.assertLess(len(half), len(take))
        with self.assertRaises(ms.MotionStudioError):
            self.h.play(ms.Take(["t", "nothing"], [[0, 1]]))


@unittest.skipUnless(os.path.exists(SERVER), "motionstudio-server is not built")
class ActuatorTests(unittest.TestCase):
    """A slide reports millimetres. The expression symbol stays in metres."""

    @classmethod
    def setUpClass(cls):
        cls.server = Server(str(ROOT / "templates" / "indexing_table.mstudio"))
        cls.studio = ms.connect(port=cls.server.port)

    @classmethod
    def tearDownClass(cls):
        cls.studio.close()
        cls.server.stop()

    def test_slide_is_millimetres_and_table_is_degrees(self):
        press, table = self.studio.actuator("Press"), self.studio.actuator("Table")
        # actuator.get answers before the simulation runs; Press.pos does not
        self.assertAlmostEqual(press.value, 0, delta=1e-6)
        with self.assertRaises(ms.SimulationError):
            _ = self.studio.signals["Press.pos"]
        press.move_to(250)
        table.move_to(90)
        self.assertAlmostEqual(press.value, 250, delta=1e-3)
        self.assertAlmostEqual(self.studio.signals["Press.pos"], 0.25, delta=1e-6)
        self.assertAlmostEqual(table.value, 90, delta=1e-3)
        self.assertAlmostEqual(self.studio.signals["Table.pos"], 90, delta=1e-3)
        with self.assertRaises(ms.NotFoundError):
            self.studio.actuator("Nobody").value


if __name__ == "__main__":
    unittest.main()
