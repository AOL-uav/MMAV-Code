"""Unit tests for reflex_small_polars pure helpers (no solver calls). Run from rigid_glider/:
py -B -m unittest test_reflex_small_polars -v
"""
import sys
sys.dont_write_bytecode = True
import unittest

import reflex_small_polars as sp


def pol(points, xtr=1.0):
    return [dict(alpha=a, cl=c, cd=0.02, cdp=0.01, cm=m, xtr_top=xtr, xtr_bot=1.0) for a, c, m in points]


class ScriptTests(unittest.TestCase):
    def test_grid(self):
        self.assertEqual(len(sp.GRID), 29)
        self.assertEqual(sp.GRID[0], -4.0)
        self.assertEqual(sp.GRID[-1], 10.0)

    def test_viscous_script_matches_pure_wing_sweep_order(self):
        s = sp.build_script(20000, 5, 160)
        self.assertIn("VISC 20000\nVPAR\nN 5\n\nITER 400\n", s)
        self.assertIn("ALFA 0\nINIT\nALFA 0\nASEQ 0 10 0.5\n", s)
        self.assertIn("ALFA 0\nINIT\nALFA 0\nASEQ 0 -4 -0.5\n", s)
        self.assertLess(s.index("ASEQ 0 10 0.5"), s.index("ASEQ 0 -4 -0.5"))
        self.assertNotIn("PPAR", s)
        self.assertTrue(s.endswith("PACC\n\nQUIT\n"))

    def test_ppar_240_and_never_above_240(self):
        s = sp.build_script(40000, 9, 240)
        self.assertIn("PPAR\nN 240\n\n\nPANE", s)
        self.assertLessEqual(max(sp.PANELS), 240)

    def test_inviscid_script(self):
        s = sp.build_script(None, 0, 160)
        self.assertNotIn("VISC", s)
        self.assertIn("ASEQ -4 10 0.5", s)

    def test_job_count(self):
        n = len(sp.all_jobs())
        # per section and paneling: clean inv + 14 clean visc + 6 lips * (1 inv + 14 visc) = 105; x2 x2
        self.assertEqual(n, 2 * 2 * (1 + 14 + 6 * 15))


class DataTests(unittest.TestCase):
    def test_merge_sweeps_later_overrides_and_sorts(self):
        a = pol([(0, 0.1, 0.0), (0.5, 0.2, 0.0)])
        b = pol([(0, 0.11, 0.0), (-0.5, 0.0, 0.0)])
        m = sp.merge_sweeps([a, b])
        self.assertEqual([r["alpha"] for r in m], [-0.5, 0, 0.5])
        self.assertEqual(m[1]["cl"], 0.11)

    def test_coverage(self):
        c = sp.coverage_row(pol([(0, 0.1, 0), (0.5, 0.2, 0), (99, 0, 0)]))
        self.assertEqual(c["returned"], 2)
        self.assertEqual(c["requested"], 29)
        self.assertIn("-4", c["missing_alphas"].split())

    def test_lip_moment_cells(self):
        pts = [(a, 0.1 * a, -0.01) for a in [x / 2 for x in range(-8, 21)]]
        clean = pol(pts)
        bent = pol([(a, 0.1 * a + 0.1, 0.04) for a in [x / 2 for x in range(-8, 21)]])      # same slope, shifted: lip dcm = +0.05
        inv_clean = pol([(a, 0.1 * a, 0.0) for a in [x / 2 for x in range(-8, 21)]])
        inv_bent = pol([(a, 0.1 * a + 0.1, 0.1) for a in [x / 2 for x in range(-8, 21)]])
        cells = sp.lip_moment_cells(clean, bent, inv_clean, inv_bent)
        c25 = next(c for c in cells if c["cl"] == 0.25)
        self.assertAlmostEqual(c25["lip_dcm"], 0.05, places=9)
        self.assertAlmostEqual(c25["inviscid_lip_dcm"], 0.1, places=9)
        self.assertAlmostEqual(c25["fraction_of_inviscid"], 0.5, places=9)
        self.assertAlmostEqual(c25["alpha_bent"], 1.5, places=9)
        self.assertAlmostEqual(c25["alpha_clean"], 2.5, places=9)
        self.assertAlmostEqual(c25["bent_dcm_dcl_0p15_0p35"], 0.0, places=9)

    def test_missing_when_not_bracketed(self):
        clean = pol([(0, 0.0, 0.0), (0.5, 0.05, 0.0)])
        bent = pol([(0, 0.0, 0.0), (0.5, 0.05, 0.0)])
        cells = sp.lip_moment_cells(clean, bent, clean, bent)
        self.assertTrue(all(c["note"].startswith("MISSING") for c in cells))
        self.assertNotIn("lip_dcm", cells[0])

    def test_no_interpolation_across_gap(self):
        clean = pol([(0, 0.0, 0.0), (2, 0.4, 0.0)])         # 2 deg gap: must not bracket Cl 0.2
        bent = pol([(0, 0.0, 0.0), (0.5, 0.05, 0.0), (1, 0.1, 0.0), (1.5, 0.2, 0.0), (2, 0.4, 0.0)])
        cells = sp.lip_moment_cells(clean, bent, clean, bent)
        self.assertTrue(cells[1]["note"].startswith("MISSING"))

    def test_spread(self):
        self.assertEqual(sp.spread([1.0, None, -2.0]), (-2.0, 1.0, 2))
        self.assertIsNone(sp.spread([None]))

    def test_range_stats(self):
        t = sp.range_stats([(0.05, 0.06), (0.001, 0.003)])
        self.assertIn("n=2", t)
        self.assertIn("+20", t)             # % only from the pair with |ref| >= 0.01
        self.assertIn("(n=1)", t)
        self.assertEqual(sp.range_stats([]), "no pairs")

    def test_lip_label(self):
        self.assertEqual(sp.lip_label(0.15, 8.0), "15/8")


if __name__ == "__main__":
    unittest.main()
