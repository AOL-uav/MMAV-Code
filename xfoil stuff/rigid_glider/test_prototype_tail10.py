import json
import unittest
from math import pi
from pathlib import Path

import prototype_tail10 as p


class Tail10Tests(unittest.TestCase):
    def test_frange(self):
        v = p.frange(-5, -1, 0.5)
        self.assertEqual(len(v), 9)
        self.assertEqual(v[0], -5)
        self.assertEqual(v[-1], -1)
        with self.assertRaises(ValueError):
            p.frange(0, 1, 0.3)

    def test_cg_values_include_required(self):
        cfg = json.loads(Path(p.CONFIG).read_text())
        v = p.cg_values_mm(cfg)
        for need in (171, 173.5, 176, 178.5, 181):
            self.assertIn(need, v)
        self.assertEqual(v[0], 166)
        self.assertEqual(v[-1], 186)
        self.assertEqual(len(v), 23)

    def test_tail_chord(self):
        area = 1.2 ** 2 / 7
        self.assertAlmostEqual(p.tail_chord(area, 1.2, 0.10, 0.35), 0.0489795918, places=8)

    def test_slope(self):
        self.assertAlmostEqual(p.slope([0, 1, 2], [1, 3, 5]), 2.0)
        self.assertAlmostEqual(p.slope([0, 1, 2, 3], [1, None, 5, 7]), 2.0)
        self.assertIsNone(p.slope([0], [1]))
        self.assertIsNone(p.slope([1, 1], [1, 2]))

    def test_equivalent_beta(self):
        self.assertAlmostEqual(p.equivalent_beta_deg(0.01, 0.5), 0.02 * 180 / pi)
        self.assertIsNone(p.equivalent_beta_deg(None, 0.5))
        self.assertIsNone(p.equivalent_beta_deg(0.1, 0.0))

    def test_flags(self):
        self.assertEqual(p.row_flags({"status": "positive-lift trim"}, False), "")
        self.assertEqual(p.row_flags({"status": "positive-lift trim"}, True), "WIDENED_ALPHA_PROBE")
        self.assertIn("NO_POSITIVE_LIFT_TRIM",
                      p.row_flags({"status": "no Cm=0 in screened alpha range"}, True))

    def test_compare(self):
        self.assertTrue(p.compare("x", 1.000001, 1.0, 1e-5)["ok"])
        self.assertFalse(p.compare("x", 1.1, 1.0, 1e-5)["ok"])


if __name__ == "__main__":
    unittest.main()
