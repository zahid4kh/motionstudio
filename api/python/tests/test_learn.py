"""motionstudio.learn against a live server (skipped without NumPy or the server).

    python3 -m unittest tests.test_learn
"""
import math
import os
import random
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import motionstudio as ms  # noqa: E402

try:
    import numpy as np
    from motionstudio.learn import AnimationModel, MLP, WalkPolicy, walk_steps
except ImportError:  # pragma: no cover
    np = None

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_live import SERVER, Server  # noqa: E402


@unittest.skipUnless(np is not None, "NumPy is not installed")
class MLPTests(unittest.TestCase):
    def test_learns_a_function_and_saves(self):
        rng = np.random.default_rng(1)
        X = rng.uniform(-2, 2, (2000, 2))
        Y = np.column_stack([np.sin(X[:, 0]) * 100, X[:, 0] * X[:, 1]])
        net = MLP(2, 2, (32, 32))
        net.fit(X, Y, epochs=150, lr=3e-3)
        err = np.abs(net.predict(X) - Y).mean(0)
        self.assertLess(err[0], 3.0)
        self.assertLess(err[1], 0.15)
        back = MLP.from_state(net.state("a_"), "a_")
        self.assertTrue(np.allclose(back.predict(X[:5]), net.predict(X[:5])))


@unittest.skipUnless(np is not None and os.path.exists(SERVER), "needs NumPy and motionstudio-server")
class LearnFromTakesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = Server("--humanoid", "Full-size")
        cls.studio = ms.connect(port=cls.server.port)
        cls.walker = cls.studio.humanoid()

    @classmethod
    def tearDownClass(cls):
        cls.studio.close()
        cls.server.stop()

    def test_animation_model(self):
        self.studio.reset()
        with self.walker.record(rate=50) as rec:
            self.walker.move_joints(RShoulderRoll=80, RElbow=60, speed=80)
            self.walker.move_joints(RElbow=10, speed=80)
            self.walker.home(speed=80)
        wave = rec.take
        model = AnimationModel().fit({"wave": wave, "still": wave.between(0, 0.1)}, epochs=500)
        half = wave.frame(len(wave) // 2)
        self.assertAlmostEqual(model.pose("wave", wave.rows[len(wave) // 2][0] / wave.duration)["RElbow"],
                               half["RElbow"], delta=4)
        slow = model.generate("wave", duration=2 * wave.duration, rate=25)
        self.assertAlmostEqual(slow.duration, 2 * wave.duration, delta=0.05)
        self.assertEqual(slow.columns[:5], ["t", "x", "y", "z", "heading"])
        self.assertAlmostEqual(max(slow["RShoulderRoll"]), max(wave["RShoulderRoll"]), delta=5)
        blend = model.generate({"wave": 1, "still": 1})
        self.assertLess(max(blend["RShoulderRoll"]), max(wave["RShoulderRoll"]) - 15)
        self.walker.play(slow, place=False)
        with tempfile.TemporaryDirectory() as d:
            model.save(os.path.join(d, "anim.npz"))
            again = AnimationModel.load(os.path.join(d, "anim.npz"))
        self.assertEqual(again.pose("wave", 0.3), model.pose("wave", 0.3))

    def test_walk_policy(self):
        self.studio.reset()
        rng = random.Random(3)
        takes = []
        for _ in range(30):
            p = self.walker.place
            a = math.radians(p.heading) + rng.gauss(0, 0.8)
            if math.hypot(p.x, p.z) > 2000:
                a = math.atan2(p.z, -p.x)
            d = rng.uniform(300, 1800)
            with self.walker.record(rate=25) as rec:
                self.studio.sleep(0.2)
                self.walker.walk_to(p.x + d * math.cos(a), p.z - d * math.sin(a))
                self.studio.sleep(0.3)
            takes.append(rec.take)
        walks = walk_steps(takes[0])
        self.assertEqual(len(walks), 1)
        self.assertGreater(len(walks[0]["footsteps"]), 1)
        self.assertEqual(len(walks[0]["steps"]), len(walks[0]["footsteps"]) + 1)   # and the closing step

        policy = WalkPolicy(hidden=(64, 64)).fit(takes, epochs=150, motion_epochs=30)
        self.assertAlmostEqual(policy.step_time, self.walker.default_gait.step_time, delta=0.01)
        start = self.walker.place
        goal = (start.x + 1000 * math.cos(math.radians(start.heading)), start.z - 1000 * math.sin(math.radians(start.heading)))
        own = policy.footsteps(self.walker, *goal, exact=False)
        self.assertGreater(len(own), 3)
        self.assertLess(math.hypot(own[-1][0] - goal[0], own[-1][1] - goal[1]), 250)   # roughly there, from 30 lessons
        steps = policy.footsteps(self.walker, *goal)
        policy.walk(self.walker, *goal)
        end = self.walker.place
        self.assertAlmostEqual(math.hypot(end.x - steps[-1][0], end.z - steps[-1][1]), 0, delta=0.5)
        take = policy.motion(start.x, start.z, start.heading, start=self.walker)
        self.assertIn("LKnee", take.columns)
        self.walker.play(take)
        back = self.walker.place
        self.assertLess(math.hypot(back.x - take.rows[-1][1], back.z - take.rows[-1][3]), 1)
        with tempfile.TemporaryDirectory() as d:
            policy.save(os.path.join(d, "walker.npz"))
            again = WalkPolicy.load(os.path.join(d, "walker.npz"))
        self.assertEqual(again.footsteps(None, 800, 300), policy.footsteps(None, 800, 300))


if __name__ == "__main__":
    unittest.main()
