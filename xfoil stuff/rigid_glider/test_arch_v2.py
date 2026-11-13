"""Unit tests for the stage-3 helpers (arch_v2_model / arch_v2_study / arch_v2_items / arch_v2_figs)."""
import sys
sys.dont_write_bytecode = True
import math
import tempfile
import unittest
from pathlib import Path

import arch_model as A
import arch_v2_model as V2
import arch_v2_study as T
import pure_wing_model as m
import pure_wing_v2_model as v2

TMP = Path(tempfile.mkdtemp(prefix="arch_v2_test_"))
AR, SW, TP = 6.0, 15, 0.5
_c = {}


def lipM():
    if "l" not in _c:
        _c["l"] = A.build_lip(AR, SW, TP, 0.2, "outer", TMP / "lip")
    return _c["l"]


def pureM():
    if "p" not in _c:
        _c["p"] = A.build_pure(AR, SW, TP, TMP / "pure")
    return _c["p"]


class Tab:
    def __init__(self, fn, cl0=0.0, k=0.11):
        self.fn, self.cl0, self.k = fn, cl0, k
    def alpha_inv(self, cl): return (cl - self.cl0) / self.k
    def lookup(self, re, cl):
        return dict(alpha=self.alpha_inv(cl), cd=0.02, cm=self.fn(re, cl), nbranch=1, steep=False)


class Inv:
    def __init__(self, fn): self.fn = fn
    def cm_at(self, cl): return self.fn(cl)


INV = lambda cl: 0.03 + 0.01 * cl


def cfg(frac, cm_clean=0.0, inv_clean=lambda cl: 0.0):
    bent = Tab(lambda re, cl: frac * INV(cl) + cm_clean, cl0=0.2)
    return V2.Cfg2(Tab(lambda re, cl: cm_clean), bent, Inv(INV), Inv(inv_clean))


class FormC(unittest.TestCase):
    def test_k_equal_one_reduces_to_replace_form(self):
        M = lipM()
        c = cfg(1.0)
        for CL in (0.25, 0.4):
            r_c = V2.cm_extra_c(M, 0.4, 8.0, c, CL, ws=8.0)
            r_b = A.cm_extra_arch(M, 0.4, 8.0, c, CL, ws=8.0, form="replace")
            self.assertTrue(r_c["converged"]); self.assertAlmostEqual(r_c["k"], 1.0, places=9)
            self.assertAlmostEqual(r_c["cm_extra"], r_b["cm_extra"], places=10)

    def test_k_converges_to_constant_retained_fraction_and_scales_redistribution(self):
        M = lipM()
        r = V2.cm_extra_c(M, 0.4, 8.0, cfg(0.5), 0.3, ws=8.0)
        self.assertTrue(r["converged"]); self.assertAlmostEqual(r["k"], 0.5, delta=3e-3)
        self.assertAlmostEqual(r["cm_np"], A.cm_np_avl(M, r["k"] * 8.0, 0.3), places=12)     # AVL loading with the scaled deflection
        # the alpha needed for the same CL is smaller than with the full lip (lift loss reduced)
        self.assertLess(r["alpha_avl"], A.alpha_avl(M, 8.0, 0.3))

    def test_clipping_and_iteration_cap(self):
        M = lipM()
        hi = V2.cm_extra_c(M, 0.4, 8.0, cfg(2.0), 0.3, ws=8.0)
        self.assertAlmostEqual(hi["k"], 1.0, delta=2e-3)                                       # k clipped at 1
        lo = V2.cm_extra_c(M, 0.4, 8.0, cfg(-1.0), 0.3, ws=8.0)
        self.assertAlmostEqual(lo["k"], 0.0, delta=2e-3)                                       # and at 0
        one = V2.cm_extra_c(M, 0.4, 8.0, cfg(0.5), 0.3, ws=8.0, maxit=1)
        self.assertFalse(one["converged"]); self.assertEqual(one["iterations"], 1)

    def test_pessimistic_ordering(self):
        M = lipM()
        a = A.cm_extra_arch(M, 0.4, 8.0, cfg(0.4), 0.3, ws=8.0, form="coupled")
        c = V2.cm_extra_c(M, 0.4, 8.0, cfg(0.4), 0.3, ws=8.0)
        self.assertLess(abs(A.cm_np_avl(M, c["k"] * 8.0, 0.3)), abs(A.cm_np_avl(M, 8.0, 0.3)))   # smaller AVL lip moment when scaled


class CurvesAndWindows(unittest.TestCase):
    def test_build_curve_ev_matches_direct_evaluation(self):
        M = lipM()
        c = cfg(0.5, cm_clean=0.004)
        ev = lambda CL, V: A.cm_extra_arch(M, 0.4, 8.0, c, CL, ws=8.0, V=V, form="coupled")
        curve, nc = V2.build_curve_ev(ev, None)
        self.assertEqual(nc, 0)
        n = len(A.GRID)
        for i in (2, 30, 60, n - 10):
            CL = A.GRID[i]
            r = ev(CL, None)
            self.assertEqual(curve[i]["cm"], r["cm_extra"]); self.assertEqual(curve[i]["cov"], r["coverage"])
            self.assertAlmostEqual(curve[i]["dg"], (ev(A.GRID[i + 2], None)["cm_extra"] - ev(A.GRID[i - 2], None)["cm_extra"]) / 0.04, places=12)
            V = r["V"]
            self.assertAlmostEqual(curve[i]["df"], (ev(CL + 0.02, V)["cm_extra"] - ev(CL - 0.02, V)["cm_extra"]) / 0.04, places=12)
        self.assertIsNone(curve[0]["dg"]); self.assertIsNone(curve[-1]["dg"])

    def test_evaluators_avl_only(self):
        M = lipM()
        ev, ti, avl = V2.evaluators(M, 0.4, 8.0, "lip", None, "NACA_0006", "avl_only", None, None, [0.2, "outer"], 8.0)
        self.assertTrue(avl); self.assertIsNone(ti)
        r = ev(0.3, None)
        self.assertEqual(r["coverage"], 1.0); self.assertAlmostEqual(r["cm_extra"], A.cm_np_avl(M, 8.0, 0.3))

    def test_scan_and_edge_flags(self):
        curve = [dict(cl=c, cm=0.01 - 0.05 * c, cov=1.0, jag=0.0, dg=-0.05, df=-0.05, covd=1.0, ti=0.0) for c in A.GRID]
        sets, pts = V2.scan_curve2(curve)
        # Cm = 0.01 - 0.05 CL - SM CL = 0 -> CL = 0.01/(0.05+SM); in band 0.2..0.5 for SM in [-0.03, 0.0]
        self.assertTrue(all(-0.03 - 1e-9 <= s <= 0.0 + 1e-9 for s in sets["any"])); self.assertGreater(len(sets["any"]), 5)
        key = "lip|P07|0.2|outer"
        models = {"x": {key: dict(mac=0.23, xnp=0.175, area=0.22)}}
        mk = lambda form, nc, pan, sm: dict(deck="x", kind="lip", wing="P07", section="NACA_0006", cfg=[0.2, "outer"], u=10.0, ws=8, form=form, ncrit=nc, panels=pan,
                                             sets={md: list(sm) for md in V2.MODES}, pts=[[x, 0.3, -0.1, -0.1, 0.0, 0.0] for x in sm])
        edge_sm = [V2.SM_MAX - 0.01, V2.SM_MAX - 0.005, V2.SM_MAX]
        recs = [mk("avl_only", None, None, [0.1, 0.105]), mk("coupled", 5, 160, edge_sm), mk("coupled", 9, 160, edge_sm), mk("coupled", 5, 240, [0.1]), mk("coupled", 9, 240, [0.1])]
        rows = V2.window_rows2(recs, models)
        r = [x for x in rows if x["form"] == "coupled" and x["panels"] == 160 and x["tier"] == "both" and x["mode"] == "any"][0]
        self.assertEqual(r["touches_hi_edge"], 1); self.assertEqual(r["touches_lo_edge"], 0)
        self.assertAlmostEqual(r["sm_centre"], V2.SM_MAX - 0.005)
        mac_mm = 0.23 * 400
        self.assertAlmostEqual(r["width_mm"], 2 * 0.005 * mac_mm)
        self.assertAlmostEqual(r["cg_mm_centre"], 0.175 * 400 - r["sm_centre"] * mac_mm)
        lo = [x for x in rows if x["form"] == "avl_only"][0]
        self.assertEqual(lo["touches_hi_edge"], 0)

    def test_grid(self):
        self.assertEqual(V2.SM_GRID[0], -0.15); self.assertEqual(V2.SM_GRID[-1], 0.60); self.assertEqual(len(V2.SM_GRID), 151)


class DecksAndModels(unittest.TestCase):
    def setUp(self):
        self.w = m._wing(AR, SW, TP, 3)

    def test_deck_tail_height_tipfin_and_dihedral(self):
        cr = self.w.root_chord
        txt = V2.deck_full(self.w, "t", tail=dict(area_frac=0.1, x_qc=2 * cr, incidence=-3.0), tail_z=0.05 * cr)
        tail = txt.split("SURFACE\nTail\n")[1]
        secs = [[float(x) for x in l.split()] for i, l in enumerate(tail.splitlines()) if i > 0 and tail.splitlines()[i - 1] == "SECTION"]
        self.assertAlmostEqual(secs[0][2], 0.05 * cr, places=8); self.assertAlmostEqual(secs[1][2], 0.05 * cr, places=8)
        self.assertAlmostEqual(secs[0][3] * 0.35 * self.w.span_m, 0.1 * self.w.area, places=8)       # tail area = 10 % of wing area
        self.assertAlmostEqual(secs[0][0] + 0.25 * secs[0][3], 2 * cr, places=8)
        self.assertAlmostEqual(secs[0][4], -3.0)
        tz = txt.split("SURFACE\nRight\n")[1].split("SURFACE")[0].split("SECTION\n")[2].split()
        self.assertAlmostEqual(float(tz[2]), self.w.tip_z, places=8)
        for mode in ("sym", "above"):
            t2 = V2.deck_full(self.w, "t", tip_fin=dict(area_frac=0.04, mode=mode)).split("SURFACE\nTipFin\n")[1]
            s2 = [[float(x) for x in l.split()] for i, l in enumerate(t2.splitlines()) if i > 0 and t2.splitlines()[i - 1] == "SECTION"]
            self.assertAlmostEqual((s2[0][2] - s2[1][2]) * s2[0][3], 0.04 * self.w.area, places=8)
            self.assertAlmostEqual(s2[0][3], self.w.tip_chord, places=8)
            self.assertAlmostEqual(0.5 * (s2[0][2] + s2[1][2]), self.w.tip_z if mode == "sym" else self.w.tip_z + 0.5 * (s2[0][2] - s2[1][2]), places=8)
        self.assertEqual(V2.deck_full(self.w, "t"), A.lat_deck(self.w, "t", nchord=20, nspan=32))          # bare deck = lat_deck

    def test_model_key(self):
        self.assertEqual(V2.model_key("pure", "P07", None), "pure|P07")
        self.assertEqual(V2.model_key("lip", "P07", [0.2, "outer"]), "lip|P07|0.2|outer")
        self.assertEqual(V2.model_key("tail", "P07", [0.075, 2.0]), "tail|P07|0.075|2")

    def test_final_deck_np_close_to_planar_and_fin_small(self):
        w0 = m._wing(AR, SW, TP, 0)
        x0, _, _ = V2.xnp_of(w0, A.lat_deck(w0, "v", nchord=20, nspan=32), TMP / "np0")
        x3, _, _ = V2.xnp_of(self.w, V2.deck_full(self.w, "v", fin=dict(area_frac=0.06, x_qc=1.5 * self.w.root_chord)), TMP / "np3")
        self.assertLess(abs(x3 - x0) * 400, 1.5)                                    # dihedral + fin move the pure-wing NP by < 1.5 mm at b = 0.4 m
        self.assertGreater(x3, x0 - 1e-6)

    def test_lip_decomposition_algebra(self):
        M = lipM()
        d = V2.lip_decomposition(M)
        self.assertAlmostEqual(d["couple"] + d["redistribution"], d["cm0u_total"], places=12)
        self.assertAlmostEqual(d["frac_couple"] + d["frac_redistribution"], 1.0, places=12)
        self.assertGreater(d["frac_couple"], 0.3); self.assertLess(d["frac_couple"], 0.8)
        self.assertLess(abs(d["d_cl_check"]), 5e-4)                                  # sum of strip lifts reproduces AVL CL change
        self.assertLess(abs(d["closure_error"]), 0.2)                                # independent lift-arm estimate within 20 % of total


class MassHelpers(unittest.TestCase):
    def test_centroid(self):
        self.assertAlmostEqual(V2.planform_centroid_x(0.1, 0.1, 0.0, 0.2), 0.042, places=9)               # rectangle: 0.42 c
        self.assertAlmostEqual(V2.planform_centroid_x(0.1, 0.1, 0.04, 0.2), 0.042 + 0.02, places=6)       # swept rectangle: + X/2
        tri = V2.planform_centroid_x(0.1, 0.0, 0.0, 0.2, n=20000)                                         # root-to-point taper
        self.assertAlmostEqual(tri, 0.42 * 0.1 * (2 / 3), places=4)       # integral c^2/ integral c for linear c: 2/3 cr

    def test_ballast(self):
        mb = V2.ballast_for_target([(10.0, 100.0)], 80.0, 0.0)
        self.assertAlmostEqual(mb, 2.5)
        self.assertAlmostEqual((10.0 * 100.0 + mb * 0.0) / (10.0 + mb), 80.0)
        self.assertLess(V2.ballast_for_target([(10.0, 60.0)], 80.0, 0.0), 0)       # structure already ahead of target
        self.assertAlmostEqual(V2.ballast_for_target([(5.0, 100.0), (5.0, 60.0)], 80.0, -20.0), 0.0)

    def test_wing_mass(self):
        self.assertAlmostEqual(V2.wing_mass("shell", 0.04, 0.1, sigma=0.5), 0.02)
        self.assertAlmostEqual(V2.wing_mass("infill", 0.04, 0.1, rho_eff=250.0, tc=0.08), 250 * 0.685 * 0.08 * 0.04 * 0.1)


class StudyHelpers(unittest.TestCase):
    def test_forms_for(self):
        self.assertEqual(T.forms_for("lip"), ["avl_only", "coupled", "replace", "replace_k"])
        self.assertEqual(T.forms_for("tail"), ["avl_only", "coupled"])

    def test_tail_effectiveness_scaling_in_model(self):
        # scaling tail area scales the AVL tail authority and the NP shift ~ proportionally (used for the 0.5 / 0.75 effectiveness variants)
        w = m._wing(AR, SW, TP, 0)
        full = A.build_tail(AR, SW, TP, 0.10, 2.0, TMP / "t10")
        half = A.build_tail(AR, SW, TP, 0.05, 2.0, TMP / "t05")
        ratio = half["d_cl"] / full["d_cl"]
        self.assertGreater(ratio, 0.45); self.assertLess(ratio, 0.70)


if __name__ == "__main__":
    unittest.main()
