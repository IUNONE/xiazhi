import copy
import io
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from data_pipeline.data_collection.teleoperation.config import Calibration
from data_pipeline.data_rerun.leader_calibration import LeaderModelCalibration, joint_direction, sample_stable_raw
from data_pipeline.data_rerun.leader_preview import LEADER_URDF
from scripts import inspect_leader

ROOT = Path(__file__).resolve().parents[1]


class ModelCalibrationTest(unittest.TestCase):
    def setUp(self):
        base = Calibration.load(ROOT / 'configs/teleoperation/zhongling_eco63.example.json', require_verified=False)
        self.device = replace(base, raw_min=(0,)*6, raw_max=(1000,)*6,
                              angle_range_deg=(200,)*6, joint_indices=(5, 3, 1, 4, 2, 0))

    def test_restart_zero_direction_and_channel_order(self):
        zero = [450, 460, 470, 480, 490, 500]
        positive = [z + d for z, d in zip(zero, [100, -100]*3)]
        model = LeaderModelCalibration.from_readings(self.device, LEADER_URDF, zero, positive)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'model.json'
            path.write_text(json.dumps(model.document))
            loaded = LeaderModelCalibration.load(path, self.device, LEADER_URDF)
        for values, expected in [(zero, 0), (positive, np.deg2rad(20))]:
            raw = np.zeros(6)
            raw[list(self.device.joint_indices)] = values
            np.testing.assert_allclose(loaded.joint_positions(np.deg2rad(raw * .2)), expected, atol=1e-12)
        # Follower gains and signs must not change the U-Arm model mapping.
        changed = replace(self.device, direction=(-1,)*6, scale=(3,)*6)
        LeaderModelCalibration(model.document, changed, LEADER_URDF)
        with self.assertRaises(ValueError):
            LeaderModelCalibration(model.document, replace(self.device, joint_indices=tuple(range(6))), LEADER_URDF)
        invalid = copy.deepcopy(model.document)
        invalid['urdf_sha256'] = 'old'
        with self.assertRaises(ValueError):
            LeaderModelCalibration(invalid, self.device, LEADER_URDF)

    def test_reject_small_motion_wrap_and_invalid_readings(self):
        for positive in (500, 510, 990, float('nan'), 1001):
            with self.subTest(positive=positive), self.assertRaises(ValueError):
                joint_direction(self.device, 0, 500, positive)
        self.assertEqual(joint_direction(self.device, 0, 500, 400), -1)

    def test_stability_uses_current_channel_and_waits_for_slow_scans(self):
        state = SimpleNamespace(now=0.)

        def sleep(seconds):
            state.now += seconds

        def read():
            scan = int(state.now / .6)
            return SimpleNamespace(timestamp_ns=scan, raw=[500, 500 + 50 * scan])

        with patch('data_pipeline.data_rerun.leader_calibration.time.monotonic', side_effect=lambda: state.now), patch('data_pipeline.data_rerun.leader_calibration.time.sleep', side_effect=sleep):
            # Other joints (including a gripper) can move while joint 1 is held.
            result = sample_stable_raw(read, [0], [7, 8], .1)
            self.assertEqual(result[0], 500)
            self.assertGreater(state.now, .8)
            self.assertLess(state.now, 3)
            state.now = 0.
            with self.assertRaisesRegex(ValueError, '舵机 8 波动'):
                sample_stable_raw(read, [1], [7, 8], .1)
            state.now = 0.
            with self.assertRaisesRegex(ValueError, '仅收到 0 个新扫描'):
                sample_stable_raw(lambda: SimpleNamespace(timestamp_ns=1, raw=[500, 500]), [0], [7, 8], .1)
            state.now = 0.
            with self.assertRaisesRegex(ValueError, '舵机 8 波动'):
                sample_stable_raw(read, [0, 1], [7, 8], .1)

    def test_model_action_requires_output_and_viewer(self):
        for extra in (['--calibrate-model', '--viewer'],
                      ['--model-calibration-output', 'new.json'],
                      ['--calibrate-model', '--model-calibration-output', 'new.json'],
                      ['--calibrate-model', '--model-calibration-output', 'new.json',
                       '--viewer', '--model-calibration-input', 'existing.json']):
            argv = ['inspect_leader', '--port', 'fake', '--leader-config', 'fake.json'] + extra
            with self.subTest(extra=extra), patch.object(inspect_leader.sys, 'argv', argv), patch.object(inspect_leader.sys, 'stderr', io.StringIO()), patch.object(inspect_leader, 'ZhonglingLeader') as leader:
                with self.assertRaises(SystemExit) as error:
                    inspect_leader.main()
                self.assertEqual(error.exception.code, 2)
                leader.assert_not_called()

    def test_interactive_sequence_and_abort_without_hardware(self):
        for abort in (False, True):
            with self.subTest(abort=abort), tempfile.TemporaryDirectory() as d:
                output = Path(d) / 'model.json'
                state = SimpleNamespace(stage=0, unlock_reads=0, clock=0., sample=0, released=False, disconnected=False, closed=False)
                device = self.device

                def now():
                    state.clock += .05
                    return state.clock

                self_test = self

                class Input:
                    def isatty(self): return True
                    def fileno(self): return 0
                    def read(self, count):
                        state.unlock_reads += 1
                        # Enter and unrelated keys must not confirm a sample.
                        if state.unlock_reads < 3:
                            self_test.assertEqual(state.stage, 0)
                            return ['\n', 'x'][state.unlock_reads - 1]
                        state.stage += 1
                        return 'q' if abort and state.stage == 4 else 'y'

                    def readline(self):
                        raise AssertionError('Model calibration must use single-key input')

                class Leader:
                    def __init__(self, *a): pass
                    def connect(self): pass
                    def disconnect(self): state.disconnected = True
                    def release_torque(self): state.released = True
                    def latest(self):
                        state.sample += 1
                        raw = [500.] * 6
                        if state.stage >= 2 and state.stage % 2 == 0:
                            index = (state.stage - 2) // 2
                            raw[device.joint_indices[index]] += 100 if index % 2 == 0 else -100
                        return SimpleNamespace(raw=raw, positions=np.deg2rad(np.array(raw)*.2), timestamp_ns=state.sample)

                class Preview:
                    def __init__(self, *a, **kw): pass
                    def is_open(self): return not output.exists()
                    def update(self, sample, target_deg=None, message=None):
                        # Calibration must not override the live U-Arm pose.
                        self_test.assertIsNone(target_deg)
                    def close(self): state.closed = True

                argv = ['inspect_leader', '--port', 'fake', '--leader-config', 'fake.json', '--viewer', '--calibrate-model', '--model-calibration-output', str(output)]
                with patch.object(inspect_leader.sys, 'argv', argv), patch.object(inspect_leader.sys, 'stdin', Input()), patch.object(inspect_leader.sys, 'stdout', io.StringIO()), patch.object(inspect_leader.Calibration, 'load', return_value=device), patch.object(inspect_leader, 'ZhonglingLeader', Leader), patch('data_pipeline.data_rerun.leader_preview.LeaderPreview', Preview), patch.object(inspect_leader.select, 'select', return_value=([1], [], [])), patch.object(inspect_leader.time, 'monotonic', side_effect=now), patch.object(inspect_leader.time, 'sleep'), patch.object(inspect_leader.termios, 'tcgetattr', return_value=[]), patch.object(inspect_leader.termios, 'tcsetattr') as restore_terminal, patch.object(inspect_leader.tty, 'setcbreak'), patch.object(inspect_leader.os, 'read', side_effect=lambda fd, count: inspect_leader.sys.stdin.read(count).encode()):
                    inspect_leader.main()
                self.assertEqual(restore_terminal.call_count, 4 if abort else 12)
                self.assertEqual(state.unlock_reads, 6 if abort else 14)
                self.assertFalse(state.released)
                self.assertTrue(state.disconnected and state.closed)
                self.assertEqual(output.exists(), not abort)
                if not abort:
                    model = LeaderModelCalibration.load(output, device, LEADER_URDF)
                    self.assertEqual(model.document['zero_raw'], [500]*6)
                    self.assertEqual(model.document['direction'], [1, -1]*3)


if __name__ == '__main__':
    unittest.main()
