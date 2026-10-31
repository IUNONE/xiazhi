from __future__ import annotations

import gc
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
import weakref

from data_pipeline.data_rerun.kinematics import EndEffectorKinematics
from scripts.export_rerun_episode import main


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        class Chain:
            def getNrOfJoints(self):
                return 2

            def getNrOfSegments(self):
                return 3

            def getSegment(self, index):
                joint = SimpleNamespace(
                    getTypeName=lambda: "Fixed" if index == 0 else "RotAxis",
                    getName=lambda: f"joint_{index}",
                )
                return SimpleNamespace(getJoint=lambda: joint)

        class Solver:
            def __init__(self, chain):
                # Model the non-owning C++ reference in the PyKDL binding.
                self.chain = weakref.ref(chain)
                self.result = 0

            def JntToCart(self, q, frame):
                if self.chain() is None:
                    return -4
                frame.p = [q[0], q[1], 0.0]
                return self.result

            def strError(self, result):
                return "The size of the input does not match the internal state"

        kdl = ModuleType("PyKDL")
        kdl.ChainFkSolverPos_recursive = Solver
        kdl.JntArray = lambda n: [0.0] * n
        kdl.Frame = lambda: SimpleNamespace(p=[0.0] * 3)
        parser = ModuleType("kdl_parser_py.urdf")
        parser.treeFromString = lambda _: (True, SimpleNamespace(getChain=lambda *_: Chain()))
        modules = patch.dict(sys.modules, {"PyKDL": kdl, "kdl_parser_py.urdf": parser})
        modules.start()
        self.addCleanup(modules.stop)

    def test_chain_survives_constructor_and_garbage_collection(self):
        fk = EndEffectorKinematics(b"<robot/>", "base", "tip")
        gc.collect()
        message = SimpleNamespace(name=["joint_2", "joint_1"], position=[0.2, 0.1])
        self.assertEqual(fk.position(message), (0.1, 0.2, 0.0))
        gc.collect()
        self.assertEqual(fk.position(message), (0.1, 0.2, 0.0))

    def test_solver_error_includes_code_and_chain(self):
        fk = EndEffectorKinematics(b"<robot/>", "base", "tip")
        fk.solver.result = -4
        message = SimpleNamespace(name=["joint_1", "joint_2"], position=[0.0, 0.0])
        with self.assertRaisesRegex(ValueError, "code=-4, chain=base->tip, joints=2, positions=2"):
            fk.position(message)

    def test_invalid_joint_data_is_still_rejected(self):
        fk = EndEffectorKinematics(b"<robot/>", "base", "tip")
        with self.assertRaisesRegex(ValueError, "missing URDF joints"):
            fk.position(SimpleNamespace(name=["joint_1"], position=[0.0]))
        with self.assertRaisesRegex(ValueError, "Non-finite"):
            fk.position(SimpleNamespace(name=["joint_1", "joint_2"], position=[float("nan"), 0.0]))


@unittest.skipUnless(importlib.util.find_spec("rerun"), "requires rerun-sdk 0.33")
class RerunExportTests(unittest.TestCase):
    def test_worker_flushes_real_recording_before_routing(self):
        import rerun as rr

        if not rr.__version__.startswith("0.33."):
            self.skipTest("requires rerun-sdk 0.33")
        cli = Path(sys.executable).parent / "rerun"
        if not cli.is_file():
            found = shutil.which("rerun")
            if found is None:
                self.skipTest("requires rerun CLI")
            cli = Path(found)

        def export_sample(visualizer):
            # Stand in for LeRobot data loading, but use the real Rerun SDK/sink.
            output = visualizer.config.output_dir / "source.rrd"
            rr.init("test_lerobot_export", recording_id="source-recording")
            rr.save(output)
            rr.set_time("frame_index", sequence=0)
            rr.log("observation.force", rr.Scalars([1, 2, 3, 4, 5, 6]))
            return output

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            output = root / "episode.rrd"
            request = root / "request.json"
            request.write_text(json.dumps({
                "kind": "lerobot", "path": str(root / "dataset"),
                "output": str(output), "repo_id": "local/test", "episode_index": 0,
                "label": "test/episode_000000", "recording_id": "routed-recording",
                "options": {"batch_size": 32, "num_workers": 0},
            }))
            try:
                with patch.dict(os.environ, {"PATH": str(cli.parent) + os.pathsep + os.environ["PATH"]}), \
                     patch.object(sys, "argv", ["export_rerun_episode.py", str(request)]), \
                     patch("scripts.export_rerun_episode.LeRobotRerunVisualizer.visualize", export_sample):
                    main()
                self.assertGreater(output.stat().st_size, 0)
                # This runs while Python's global recording is still alive.
                # Routing/reading must not depend on interpreter shutdown.
                result = subprocess.run([str(cli), "rrd", "print", str(output)],
                                        check=True, capture_output=True, text=True)
                self.assertIn("observation.force", result.stdout)
                self.assertIn("routed-recording", result.stdout)
            finally:
                rr.disconnect()


if __name__ == "__main__":
    unittest.main()
