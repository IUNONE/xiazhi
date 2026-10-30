"""Check the reconstructed joint axes against the bundled CAD mesh geometry.

Run: python -m unittest discover -s tests -p 'test_leader_model.py'
No robot or Rerun viewer is opened.
"""
import struct
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

MODEL = Path(__file__).resolve().parents[1] / 'assets/robots/uarm_config2'
# Cylindrical surfaces measured from the assembled STL, in meters.
BEARINGS = [('link_1', .010), ('l2j2', .020), ('joint3', .010),
            ('joint4', .010), ('link3', .0099), ('handle_mix', .0099)]


class LeaderGeometryTest(unittest.TestCase):
    def test_joint_axes_match_circular_mesh_features(self):
        root = ET.parse(MODEL / 'uarm_config2_leader.urdf').getroot()
        pivot = np.zeros(3)
        for index, (mesh, radius) in enumerate(BEARINGS, 1):
            with self.subTest(joint=index):
                joint = root.find(f"joint[@name='joint{index}']")
                pivot += np.fromstring(joint.find('origin').get('xyz'), sep=' ')
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                self.assertAlmostEqual(np.linalg.norm(axis), 1, places=8)
                data = (MODEL / 'meshes' / f'{mesh}.stl').read_bytes()
                count = struct.unpack_from('<I', data, 80)[0]
                vertices = np.unique(np.array([
                    struct.unpack_from('<9f', data, 84 + 50 * k + 12)
                    for k in range(count)
                ]).reshape(-1, 3), axis=0)
                delta = vertices - pivot
                radial = delta - np.outer(delta @ axis, axis)
                matched = radial[np.abs(np.linalg.norm(radial, axis=1) - radius) < 4e-6]
                self.assertGreaterEqual(len(matched), 40)
                # Require a broad arc, not accidental matches on a few edges.
                u = matched[0] / np.linalg.norm(matched[0])
                w = np.cross(axis, u)
                angles = np.sort(np.arctan2(matched @ w, matched @ u))
                coverage = 2 * np.pi - np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi]))
                self.assertGreater(coverage, np.deg2rad(170))
                # Zero joint angles must preserve the original CAD assembly.
                for visual in root.find(f"link[@name='link{index}']").findall('visual'):
                    offset = np.fromstring(visual.find('origin').get('xyz'), sep=' ')
                    np.testing.assert_allclose(pivot + offset, 0, atol=3e-10)


if __name__ == '__main__':
    unittest.main()
