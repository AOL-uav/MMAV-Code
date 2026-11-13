"""Unit tests for reflex_ext_gaps pure helpers (no solver calls). Run from rigid_glider/:
py -B -m unittest test_reflex_ext_gaps -v
"""
import sys
sys.dont_write_bytecode = True
import math
import unittest

import reflex_ext_gaps as g
import reflex_xfoil as rx


def synth_surface(xh, angle_deg, n=400, round_width=0.0):
    """Flat-ish surface LE->TE: slope 0 ahead of hinge, `angle_deg` aft of it.  round_width > 0 blends
    the angle in linearly over that x range (a rounded bend)."""
    pts, y = [], 0.0
    xs = [i / n for i in range(n + 1)]
    for i, x in enumerate(xs):
        if i:
            xm = (xs[i - 1] + x) / 2
            if round_width > 0:
                f = min(max((xm - (xh - round_width / 2)) / round_width, 0.0), 1.0)
            else:
                f = 1.0 if xm > xh else 0.0
            y += (x - xs[i - 1]) * math.tan(math.radians(angle_deg * f))
        pts.append((x, y))
    return pts


class SharpnessTests(unittest.TestCase):
    def test_ideal_corner(self):
        # hinge placed on a node so the corner is exactly one node
        b = synth_surface(0.85, 8.0)
        c = synth_surface(0.85, 0.0)
        m = g.bend_sharpness(b, c, 8.0, 0.85)
        self.assertAlmostEqual(m["max_node_turn_deg"], 8.0, delta=0.05)
        self.assertGreater(m["max_node_turn_frac"], 0.99)
        self.assertEqual(m["nodes_10_90"], 0)
        self.assertAlmostEqual(m["plateau_deg"], 8.0, delta=0.05)

    def test_rounded_bend_is_detected(self):
        b = synth_surface(0.85, 8.0, round_width=0.04)
        c = synth_surface(0.85, 0.0)
        m = g.bend_sharpness(b, c, 8.0, 0.85)
        self.assertLess(m["max_node_turn_frac"], 0.15)
        self.assertGreater(m["nodes_10_90"], 10)
        self.assertGreater(m["width_xc"], 0.02)
        self.assertAlmostEqual(m["plateau_deg"], 8.0, delta=0.05)

    def test_coarse_panels_spread_less_than_fine_for_same_input_rounding(self):
        # a corner sampled on a coarse grid is still one node (sharpness is node-based, not x-based)
        b = [(x / 20, 0.0 if x / 20 <= 0.85 else (x / 20 - 0.85) * math.tan(math.radians(5))) for x in range(21)]
        c = [(x / 20, 0.0) for x in range(21)]
        m = g.bend_sharpness(b, c, 5.0, 0.85, window=0.2)
        self.assertGreater(m["max_node_turn_frac"], 0.99)

    def test_split_surfaces(self):
        loop = [(1, 0.01), (0.5, 0.05), (0, 0), (0.5, -0.05), (1, -0.01)]
        up, lo = g.split_surfaces(loop)
        self.assertEqual(up[0], (0, 0))
        self.assertEqual(up[-1], (1, 0.01))
        self.assertEqual(lo[-1], (1, -0.01))


class GeometryTests(unittest.TestCase):
    def test_hinge_generator_shape(self):
        for cf, ang in ((0.10, 3), (0.15, 8), (0.20, 10)):
            pts = g.coordinates_hinge(0.08, cf, ang)
            up, lo = g.split_surfaces(pts)
            xh = 1 - cf
            self.assertAlmostEqual(pts[0][0], pts[-1][0], places=12)   # closed TE
            self.assertAlmostEqual(pts[0][1], pts[-1][1], places=12)
            self.assertAlmostEqual(pts[0][1], cf * math.sin(math.radians(ang)), places=9)
            self.assertAlmostEqual(pts[0][0], xh + cf * math.cos(math.radians(ang)), places=9)
            self.assertTrue(all(b[0] > a[0] for a, b in zip(up, up[1:])), "upper x monotonic")
            self.assertTrue(all(b[0] > a[0] for a, b in zip(lo, lo[1:])), "lower x monotonic")
            self.assertTrue(any(abs(x - xh) < 1e-12 for x, _ in lo), "unrotated hinge node on lower side")
            self.assertGreater(len(pts), 200)
            self.assertLess(len(pts), 360)

    def test_hinge_generator_clean_matches_naca(self):
        pts = g.coordinates_hinge(0.08, 0.15, 0.0)
        up, _ = g.split_surfaces(pts)
        for x, y in up:
            self.assertAlmostEqual(y, g._yt(0.08, x), places=12)

    def test_hinge_generator_agrees_with_baseline_generator_far_from_hinge(self):
        base = rx.coordinates(0.08, 0.15, 8.0)
        hg = g.coordinates_hinge(0.08, 0.15, 8.0)
        bu, bl = g.split_surfaces(base)
        hu, hl = g.split_surfaces(hg)
        # TE and an early-chord station agree (same rotation about the same hinge)
        self.assertAlmostEqual(base[0][0], hg[0][0], places=9)
        self.assertAlmostEqual(base[0][1], hg[0][1], places=9)
        self.assertAlmostEqual(g._interp([p[0] for p in hl], [p[1] for p in hl], 0.5),
                               g._interp([p[0] for p in bl], [p[1] for p in bl], 0.5), places=4)

    def test_ppar_block(self):
        self.assertEqual(g.ppar_block("base160", None), "")
        self.assertEqual(g.ppar_block("ppar240", None), "PPAR\nN 240\n\n\n")
        b = g.ppar_block("ref160", 0.85)
        self.assertIn("XT 0.8000 0.9000", b)
        self.assertIn("XB 0.8000 0.9000", b)
        self.assertNotIn("N 160", b)
        self.assertTrue(b.endswith("\n\n\n"))

    def test_asis_has_no_pane(self):
        self.assertNotIn("PANE", g.header("asis_hinge", None))
        self.assertIn("PANE", g.header("base160", None))

    def test_baseline_body_matches_reflex_xfoil_script(self):
        # same command sequence as reflex_xfoil.run() (viscous and inviscid)
        self.assertEqual(g.baseline_body(80000),
                         "VISC 80000\nVPAR\nN 5\n\nITER 150\nPACC\npolar.dat\n\nALFA 0\nINIT\nALFA -3\nASEQ -3 8 0.5\nPACC\n\nQUIT\n")
        self.assertEqual(g.baseline_body(None),
                         "PACC\npolar.dat\n\nALFA 0\nALFA -3\nASEQ -3 8 0.5\nPACC\n\nQUIT\n")


DUMP = """#    s        x        y     Ue/Vinf    Dstar     Theta      Cf       H
   0.0  1.00  0.02  1.00  0.005  0.003  0.0030     2.0
   0.1  0.90  0.03  1.05  0.005  0.003  0.0020     2.5
   0.2  0.80  0.04  1.05  0.005  0.003 -0.0010     4.0
   0.3  0.70  0.05  1.00  0.005  0.003 -0.0020     6.0
   0.4  0.60  0.05  0.90  0.005  0.003  0.0010     3.0
   0.5  0.50  0.05  0.50  0.005  0.003  0.0030     2.0
   0.6  0.00  0.00  0.00  0.005  0.003  0.0000     2.0
   0.7  0.10 -0.03 -0.50  0.005  0.003  0.0010     2.0
   0.8  0.40 -0.04 -1.00  0.005  0.003  0.0020     2.5
   0.9  0.70 -0.03 -1.00  0.005  0.003  0.0010     3.0
   1.0  0.90 -0.02 -1.00  0.005  0.003 -0.0005     5.0
   1.1  1.00 -0.01 -1.00  0.005  0.003 -0.0010     7.0
   1.2  1.20  0.00  1.00  0.005  0.003  0.0000     1.5
"""


class DumpTests(unittest.TestCase):
    def test_parse_skips_header(self):
        rows = g.parse_dump(DUMP)
        self.assertEqual(len(rows), 13)
        self.assertEqual(rows[0]["x"], 1.0)
        self.assertEqual(rows[2]["cf"], -0.001)

    def test_split_drops_wake_and_orders_flow(self):
        rows = g.parse_dump(DUMP)
        up, lo = g.split_dump(rows, 12)          # 12 surface rows, 1 wake row
        self.assertEqual(up[0]["x"], 0.0)        # starts at the stagnation node (flow order)
        self.assertEqual(up[-1]["x"], 1.0)       # ends at the TE
        self.assertEqual(lo[-1]["x"], 1.0)
        self.assertTrue(all(r["ue"] != 1.0 or r["x"] != 1.20 for r in up + lo))

    def test_split_requires_stagnation(self):
        rows = [dict(s=i, x=i, y=0, ue=1.0, dstar=0, theta=0, cf=0.1, h=2) for i in range(5)]
        with self.assertRaises(ValueError):
            g.split_dump(rows, 5)

    def test_separation_bubble_and_reattachment(self):
        rows = g.parse_dump(DUMP)
        up, lo = g.split_dump(rows, 12)
        s = g.surface_summary(up, xtr=0.55, hinge_x=0.85)
        # flow order x: 0, .5, .6, .7, .8, .9, 1.0 ; Cf: 0 .003 .001 -.002 -.001 .002 .003
        self.assertEqual(s["n_separation_regions"], 1)
        self.assertGreater(s["sep_x"], 0.6)
        self.assertLess(s["sep_x"], 0.7)
        self.assertGreater(s["reattach_x"], 0.8)
        self.assertLess(s["reattach_x"], 0.9)
        self.assertFalse(s["sep_at_te"])
        self.assertTrue(s["laminar_sep"] is False)      # separation is behind transition at 0.55
        self.assertAlmostEqual(s["max_h"], 6.0)
        self.assertAlmostEqual(s["sep_minus_hinge"], s["sep_x"] - 0.85)

    def test_laminar_separation_flag_and_te_separation_without_reattachment(self):
        rows = g.parse_dump(DUMP)
        up, lo = g.split_dump(rows, 12)
        s = g.surface_summary(lo, xtr=1.0, hinge_x=0.85)
        # lower flow order x: .1 .4 .7 .9 1.0 ; Cf: .001 .002 .001 -.0005 -.001 -> separates, never reattaches
        self.assertTrue(s["sep_at_te"])
        self.assertIsNone(s["reattach_x"])
        self.assertTrue(s["laminar_sep"])
        self.assertGreater(s["sep_x"], 0.7)
        self.assertLess(s["sep_x"], 0.9)

    def test_attached_flow_reports_none(self):
        rows = [dict(s=i, x=i / 10, y=0, ue=1.0, dstar=0, theta=0, cf=0.002, h=2.0 + i * 0.1) for i in range(10)]
        s = g.surface_summary(rows, xtr=0.5, hinge_x=0.8)
        self.assertIsNone(s["sep_x"])
        self.assertIsNone(s["laminar_sep"])
        self.assertFalse(s["sep_at_te"])
        self.assertAlmostEqual(s["max_h"], 2.9)


class PolarCompareTests(unittest.TestCase):
    def test_compare_polars_reports_missing_and_diff(self):
        a = [dict(alpha_deg=0.0, cl=0.1, cd=0.01, cdp=0.0, cm=0.0, top_xtr=1, bot_xtr=1),
             dict(alpha_deg=0.5, cl=0.2, cd=0.01, cdp=0.0, cm=0.0, top_xtr=1, bot_xtr=1)]
        b = [dict(a[0]), dict(alpha_deg=1.0, cl=0.3, cd=0.01, cdp=0.0, cm=0.0, top_xtr=1, bot_xtr=1)]
        b[0]["cl"] = 0.1001
        r = g.compare_polars(a, b)
        self.assertEqual(r["only_new"], [0.5])
        self.assertEqual(r["only_stored"], [1.0])
        self.assertAlmostEqual(r["max_abs_diff"], 0.0001, places=9)

    def test_alpha_list(self):
        self.assertEqual(g.alpha_list(0, 1, 0.25), [0.0, 0.25, 0.5, 0.75, 1.0])
        self.assertEqual(g.alpha_list(1, 0, 0.25)[-1], 0.0)
        self.assertEqual(g.alpha_list(7.75, 7.0, -0.25), [7.75, 7.5, 7.25, 7.0])

    def test_retry_scripts_have_distinct_settings(self):
        s = g.retry_strategies()
        self.assertIn("ITER 400", s["S1b_iter400_step0p5_from_m3"])
        self.assertIn("ASEQ 0 8 0.25", s["S2_up_from0_aseq0p25_iter300"])
        self.assertIn("ASEQ 8 -3 -0.25", s["S4_down_from8_aseq0p25_iter300"])
        self.assertTrue(all("QUIT" in v for v in s.values()))
        lad = g.re_ladder_script(80000, [85000, 90000])
        self.assertIn("Re 85000", lad)
        self.assertLess(lad.index("ASEQ 8 0 -0.5"), lad.index("Re 90000"))


if __name__ == "__main__":
    unittest.main()
