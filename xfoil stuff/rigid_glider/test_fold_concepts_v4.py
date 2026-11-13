"""Tests for fold_concepts_v4.py.   py -B -m unittest test_fold_concepts_v4 -v   (no AVL; reads the v4 AVL strip file)"""
import sys
sys.dont_write_bytecode = True
import math
import unittest

import fold_concepts_v4 as fc


class FoldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.G = fc.geometry()
        cls.R = fc.solve_concepts(cls.G)
        cls.N = fc.analyse(cls.G, cls.R)

    def test_rotation_keeps_area_and_pivot(self):
        w = self.G["wingR"]
        P = self.R["c1"]["P"]
        for phi in (0.0, 35.0, 70.5):
            r = fc.rot(w, P, phi)
            self.assertAlmostEqual(fc.area(r), fc.area(w), places=6)
        self.assertEqual(fc.rot_pt(P, P, 50.0), P)

    def test_open_pose_is_the_v4_outline(self):
        self.assertAlmostEqual(self.G["s"] * 2, 400.0, places=6)
        self.assertAlmostEqual(self.G["yb"], 20.0, places=6)                       # body edge 12.5 mm at 250 -> 20 mm at 400
        self.assertAlmostEqual(fc.area(self.G["wingR"]), fc.area(self.G["armR"]) + fc.area(self.G["handR"]), delta=1.0)
        p0 = self.R["pose"]["c1"][0.0]["wingR"]
        for a, b in zip(p0, self.G["wingR"]):
            self.assertAlmostEqual(math.dist(a, b), 0.0, places=9)

    def test_folded_wings_lie_along_the_body(self):
        n = self.N["c1"]["n"]
        self.assertLess(n["wings_span"], 80.0)
        self.assertGreater(n["tips_past_tail"], 50.0)                              # swift-like: tips pass the tail
        self.assertAlmostEqual(n["stowed_span"], 92.0, places=0)                   # the 0.23 b tail fan sets the stowed span

    def test_symmetric_poses(self):
        pose = self.R["pose"]["c1"][1.0]
        a, _ = fc.overlap_runs(pose["wingR"], pose["wingL"])
        self.assertGreater(a, 1000.0)
        self.assertAlmostEqual(fc.area(pose["wingR"]), fc.area(pose["wingL"]), places=6)

    def test_band_lengthens_with_folding(self):
        b = self.N["c1"]["band_raw"]
        self.assertTrue(all(r > 0 for r in b["r_eff"]))
        self.assertLess(b["L"][-1] / b["L"][0], fc.BAND_MAX_STRETCH + 1e-9)

    def test_loads_and_masses_are_positive_and_sane(self):
        ld = self.N["ld"]
        self.assertAlmostEqual(ld["V"], 5.45, delta=0.2)
        self.assertGreater(ld["L_half"], 0.04)
        for c in ("c1", "c2"):
            self.assertGreater(self.N[c]["m_hw"], 0.5)
            self.assertLess(self.N[c]["m_hw"], 8.0)

    def test_string_pays_out_when_the_wings_open(self):
        self.assertGreater(self.N["c2"]["string"]["travel"], 5.0)
        self.assertLess(self.N["c2"]["string"]["travel"], math.dist(self.N["c2"]["string"]["ring"], self.N["c2"]["string"]["pts_gs"]) if "pts_gs" in self.N["c2"]["string"] else 1e9)


if __name__ == "__main__":
    unittest.main()
