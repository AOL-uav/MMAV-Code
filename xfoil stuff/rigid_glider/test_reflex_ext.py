"""Smoke tests for the reflex extension code.

Run: py -m unittest rigid_glider/test_reflex_ext.py
Writes only to a temporary directory; does not touch results/runs/.
"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from reflex_geometry import Reflex, deck  # noqa: E402
import reflex_ext_avl as ea  # noqa: E402
import reflex_ext_run as er  # noqa: E402


class ExtTests(unittest.TestCase):
    def setUp(self):
        self.base, self.ext, self.wing, self.flight = ea.load_configs()

    def test_lip_area_full_and_split(self):
        full = ea.lip_area(self.wing, Reflex(.10, 3, "full"))
        inner = ea.lip_area(self.wing, Reflex(.10, 3, "inner", 0.55))
        outer = ea.lip_area(self.wing, Reflex(.10, 3, "outer", 0.55))
        self.assertAlmostEqual(full, 0.10 * self.wing.area, places=9)
        self.assertAlmostEqual(inner + outer, full, places=9)
        self.assertGreater(inner, outer * 0.9)

    def test_deck_split_positions(self):
        text = deck(self.wing, "t", Reflex(.10, 3, "outer", 0.30))
        self.assertEqual(text.count("CONTROL\n"), 4)
        self.assertIn("reflR", text)

    def test_limit_interpolation(self):
        rows = [(e, {"trim_cl": 0.25 + 0.02 * e, "static_margin_mac": 0.10}) for e in range(-8, 9, 2)]
        hi, why = er.limit(rows, +1)
        self.assertEqual(hi, 8)
        self.assertIn("not reached", why)
        lo, why = er.limit(rows, -1)
        self.assertAlmostEqual(lo, -2.5, places=9)
        self.assertIn("trim_cl=0.2", why)

    def test_avl_case_runs_in_temp_dir(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Reflex(.10, 3, "full")
            row = ea.case("smoke", self.wing, f, self.flight, Path(tmp))
            self.assertIsNone(row["error"])
            self.assertGreater(row["trim_cl"], 0.0)


if __name__ == "__main__":
    unittest.main()
