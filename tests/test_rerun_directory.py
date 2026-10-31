"""Exercise directory layout registration against a real Rerun catalog."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from data_pipeline.data_rerun.directory import Episode, view_directory


@unittest.skipUnless(
    importlib.util.find_spec("rerun") and importlib.util.find_spec("datafusion"),
    "requires rerun-sdk[catalog] 0.33",
)
class DirectoryCatalogTests(unittest.TestCase):
    def test_mcap_and_lerobot_register_readable_default_blueprints(self):
        import rerun as rr
        import rerun.blueprint as rrb
        from rerun.catalog import DatasetEntry

        if not rr.__version__.startswith("0.33."):
            self.skipTest("requires rerun-sdk 0.33")
        register = DatasetEntry.register_blueprint
        registered = []

        def check_registration(dataset, uri, set_default=True):
            register(dataset, uri, set_default=set_default)
            self.assertEqual(len(dataset.segment_ids()), 2)
            self.assertEqual(len(dataset.blueprints()), 1)
            self.assertIn(dataset.default_blueprint(), dataset.blueprints())
            registered.append(uri)

        with tempfile.TemporaryDirectory(prefix="xiazhi 测试 # ") as temp:
            root = Path(temp)
            outputs = []
            for index in range(2):
                output = root / f"episode_{index}.rrd"
                with rr.RecordingStream("test", recording_id=f"episode-{index}") as recording:
                    recording.save(output)
                    recording.log("/signal", rr.Scalars(index))
                rrb.Blueprint(rrb.TimeSeriesView(origin="/signal")).save(
                    "test", output.with_suffix(".rbl"),
                )
                outputs.append(output)

            for kind in ("mcap", "lerobot"):
                with self.subTest(kind=kind):
                    episodes = []
                    for index in range(2):
                        path = root / f"episode_{index}.mcap" if kind == "mcap" else root
                        if kind == "mcap":
                            path.touch()
                        episodes.append(Episode(kind, path, f"episode_{index}", index, "local/test"))
                    # Keep real SDK layout creation, server, catalog registration and queries.
                    # Supply tiny cached recordings; do not launch a GUI during the test.
                    with patch("data_pipeline.data_rerun.directory.discover_episodes", return_value=episodes), \
                         patch("data_pipeline.data_rerun.directory.cache_episodes", return_value=outputs), \
                         patch.object(DatasetEntry, "register_blueprint", check_registration), \
                         patch("data_pipeline.data_rerun.directory.subprocess.Popen") as viewer:
                        viewer.return_value.wait.return_value = 0
                        viewer.return_value.poll.return_value = 0
                        view_directory(root, kind, root / "cache", {"camera_names": []})
                        viewer.assert_called_once()
            self.assertEqual(len(registered), 2)
            self.assertTrue(all(uri.startswith("file:///") for uri in registered))
            self.assertIn("%20", registered[1])
            self.assertIn("%23", registered[1])


if __name__ == "__main__":
    unittest.main()
