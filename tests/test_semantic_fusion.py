import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from semantic_lz.semantic_fusion import SemanticResult, canonical_class, labels_at_pixels


class SemanticFusionTests(unittest.TestCase):
    def test_canonical_classes_cover_safety_obstacles(self):
        self.assertEqual(canonical_class("person"), "person")
        self.assertEqual(canonical_class("cardboard box"), "cargo_box")
        self.assertEqual(canonical_class("car"), "vehicle")

    def test_projected_pixels_receive_mask_label(self):
        mask = np.zeros((1, 8, 8), dtype=bool)
        mask[0, 2:6, 2:6] = True
        result = SemanticResult(["person"], np.array([0.9]), mask)
        pixels = np.array([[3.0, 4.0], [7.0, 7.0], [3.0, 4.0]])
        valid = np.array([True, True, False])
        self.assertEqual(labels_at_pixels(result, pixels, valid).tolist(), [1, 0, 0])


if __name__ == "__main__":
    unittest.main()
