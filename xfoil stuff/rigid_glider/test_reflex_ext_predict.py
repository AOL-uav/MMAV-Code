"""Unit tests for reflex_ext_predict (no solver calls). Run from rigid_glider/:
py -B -m unittest test_reflex_ext_predict
"""
import sys
sys.dont_write_bytecode = True
import unittest

import reflex_ext_predict as rp
import reflex_xfoil as rx


def polar(points):
    return [dict(alpha_deg=a, cl=c, cd=0.01, cdp=0.0, cm=m, top_xtr=1.0, bot_xtr=1.0) for a, c, m in points]


class GeometryTests(unittest.TestCase):
    def test_requested_bends_valid(self):
        for t in (0.04, 0.06, 0.08):
            for cf, a in ((0.10, 3), (0.15, 5), (0.15, 8), (0.20, 10)):
                r = rp.geometry_check(t, cf, a)
                self.assertTrue(r["valid"], (t, cf, a, r))
                self.assertLess(r["te_y_abs_err"], 1e-12)

    def test_absurd_bend_flagged(self):
        # 0.45 chord at 80 deg folds the upper surface back on itself
        self.assertFalse(rp.geometry_check(0.08, 0.45, 80)["valid"])


class MatchedClTests(unittest.TestCase):
    def test_interpolation_matches_at_cl(self):
        clean = polar([(0, 0.0, 0.0), (0.5, 0.05, 0.001), (1.0, 0.10, 0.002)])
        bent = polar([(0, 0.0, 0.01), (0.5, 0.05, 0.012), (1.0, 0.10, 0.014)])
        m = rp.matched(clean, bent, 0.075)
        self.assertAlmostEqual(m["dcm"], 0.013 - 0.0015, places=9)
        self.assertEqual(m["n_clean_brk"], 1)

    def test_gap_not_filled(self):
        clean = polar([(0, 0.0, 0.0), (0.5, 0.05, 0.0), (1.0, 0.10, 0.0)])
        bent = polar([(0, 0.0, 0.0), (2.0, 0.20, 0.0)])   # 2 deg gap
        self.assertIsNone(rp.matched(clean, bent, 0.10))


class MiscTests(unittest.TestCase):
    def test_log_counts(self):
        log = " VISCAL:  Convergence failed\n VISCAL:  Convergence failed\n Sequence halted since previous  4 points did not converge\n"
        c = rp.log_counts(log)
        self.assertEqual((c["viscal_convergence_failures"], c["sequence_halted"]), (2, 1))

    def test_interp_angle(self):
        r = rp.interp_angle([(2, 0.2), (3, 0.3), (4, 0.4)], 0.26)
        self.assertAlmostEqual(r["angle_deg"], 2.6)
        self.assertFalse(rp.interp_angle([(2, 0.2), (3, 0.3)], 0.5)["bracketed"])


if __name__ == "__main__":
    unittest.main()
