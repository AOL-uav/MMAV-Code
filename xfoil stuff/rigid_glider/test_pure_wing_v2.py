"""Unit tests for the pure_wing_v2_* helpers (synthetic data; one small AVL run for the fin decks)."""
import sys
sys.dont_write_bytecode = True
import math
import tempfile
import unittest
from pathlib import Path

import pure_wing_model as m
import pure_wing_study as st
import pure_wing_v2_model as v2
import pure_wing_v2_mass as mass
import pure_wing_v2_study as s2
from geometry import Wing

_TMP = Path(tempfile.mkdtemp(prefix="pure_wing_v2_test_"))
SHAPE = m.build_shape(4.5, 20, 0.7, 0, _TMP)
B, WS = 0.3, 4.0


def table(slope=0.2, xtr=lambda a: 1.0, cm=lambda a, cl: 0.0, res=(5000, 400000)):
    al = [i * 0.5 for i in range(11)]
    rows = [dict(alpha=a, cl=slope * a, cd=0.02, cm=cm(a, slope * a), xtr_top=xtr(a), xtr_bot=1.0) for a in al]
    inv = [dict(alpha=a, cl=0.11 * a) for a in (0, 1, 2, 3, 4, 5)]
    return m.SectionTable({r: rows for r in res}, inv)


class SweepSectionTests(unittest.TestCase):
    def test_zero_sweep_is_baseline(self):
        t = table(cm=lambda a, cl: 0.01 * cl)
        w = v2.SweepSection(t, 0.0)
        for re, cl in ((30000, 0.3), (60000, 0.5), (30000, -0.2)):
            self.assertEqual(w.lookup(re, cl), t.lookup(re, cl))
            self.assertAlmostEqual(w.alpha_inv(cl), t.alpha_inv(cl), places=12)

    def test_zero_sweep_curve_identical(self):
        t = table(cm=lambda a, cl: 0.01 * cl)
        a = st.build_curve(SHAPE, B, t, WS)
        b = st.build_curve(SHAPE, B, v2.SweepSection(t, 0.0), WS)
        for x, y in zip(a, b):
            self.assertEqual(x["cm"], y["cm"]); self.assertEqual(x["cov"], y["cov"])

    def test_scaling_laws(self):
        L = 35.0
        c = math.cos(math.radians(L))
        t = table(cm=lambda a, cl: 0.02 * cl)
        w = v2.SweepSection(t, L, re_exp=2)
        re_n, cl_n = w.transform(40000.0, 0.3)
        self.assertAlmostEqual(cl_n, 0.3 / c ** 2); self.assertAlmostEqual(re_n, 40000 * c ** 2)
        self.assertAlmostEqual(v2.SweepSection(t, L, re_exp=1).transform(40000.0, 0.3)[0], 40000 * c)
        base = t.lookup(re_n, cl_n)
        r = w.lookup(40000.0, 0.3)
        self.assertAlmostEqual(r["cm"], base["cm"] * c ** 4); self.assertAlmostEqual(r["cd"], base["cd"] * c ** 3)
        self.assertAlmostEqual(r["alpha"], base["alpha"] * c)
        self.assertAlmostEqual(w.alpha_inv(0.3), t.alpha_inv(cl_n) * c)

    def test_out_of_range_propagates(self):
        w = v2.SweepSection(table(res=(20000, 40000)), 35.0, re_exp=2)
        self.assertIsNone(w.lookup(21000.0, 0.3))      # Re_n = 21000*cos^2(35) = 14100 < 20000


class TransitionFraction(unittest.TestCase):
    def test_on_and_off(self):
        on = table(xtr=lambda a: 1.0 if a < 2.0 else 0.6)          # xtr departs 1.0 at alpha 2 -> cl 0.4
        off = table(xtr=lambda a: 1.0)
        # strips' cl = CL f ; with CL=0.4 and f~1.0 root strips sit near cl 0.4 -> some strips on the interval
        f_on = v2.ti_fraction_v2(SHAPE, B, on, 0.4, 5.0)
        self.assertGreater(f_on, 0.0)
        self.assertEqual(v2.ti_fraction_v2(SHAPE, B, off, 0.4, 5.0), 0.0)
        all_on = table(xtr=lambda a: 1.0 if a < 0.2 else 0.5)        # transition interval at alpha 0..0.5 -> cl 0..0.1
        self.assertEqual(v2.ti_fraction_v2(SHAPE, B, table(xtr=lambda a: 0.9 if a == 0.0 else 1.0), 0.0 + 1e-9, 5.0) >= 0.0, True)
        # a strip is 'on' when the bracketing segment has one end at 1.0 and the other below
        t = table(xtr=lambda a: 1.0 if a <= 1.0 else 0.5)
        self.assertTrue(v2.ra.on_transition({"xtr_top": 1.0}, {"xtr_top": 0.5}))
        self.assertFalse(v2.ra.on_transition({"xtr_top": 1.0}, {"xtr_top": 1.0}))
        self.assertFalse(v2.ra.on_transition({"xtr_top": 0.5}, {"xtr_top": 0.4}))

    def test_wrapped_table_uses_normal_section(self):
        t = table(xtr=lambda a: 1.0 if a < 2.0 else 0.6)
        w = v2.SweepSection(t, 0.0)
        self.assertEqual(v2.ti_fraction_v2(SHAPE, B, w, 0.4, 5.0), v2.ti_fraction_v2(SHAPE, B, t, 0.4, 5.0))


class GoodAndRoots(unittest.TestCase):
    base = dict(cl=0.3, dg=-0.02, df=-0.03, cov=0.95, covd=0.95, jag=0.05, ti=0.2)

    def test_matches_study_is_good_on_default_band(self):
        import itertools
        for cl, cov, covd, jag, dg, df in itertools.product((0.1, 0.15, 0.3, 0.6, 0.7), (0.89, 0.91), (0.89, 0.91), (0.05, 0.1, 0.11), (-0.1, 0.1), (-0.1, 0.1)):
            r = dict(self.base, cl=cl, cov=cov, covd=covd, jag=jag, dg=dg, df=df)
            self.assertEqual(v2.is_good_v2(r, (0.15, 0.6), "any"), st.is_good(r, False))
            self.assertEqual(v2.is_good_v2(r, (0.15, 0.6), "clean"), st.is_good(r, True))

    def test_band_and_ti_modes(self):
        g = lambda band=(0.2, 0.5), mode="any", **kw: v2.is_good_v2(dict(self.base, **kw), band, mode)
        self.assertFalse(g(cl=0.1999)); self.assertTrue(g(cl=0.2)); self.assertTrue(g(cl=0.5)); self.assertFalse(g(cl=0.5001))
        self.assertTrue(g(mode="offti", ti=0.49)); self.assertFalse(g(mode="offti", ti=0.5)); self.assertFalse(g(mode="offti", ti=None))
        self.assertFalse(g(mode="clean_offti", jag=0.11)); self.assertFalse(g(mode="clean_offti", ti=0.6)); self.assertTrue(g(mode="clean_offti"))
        self.assertTrue(g(mode="any", ti=None))

    def test_roots_from_curve_v2_matches_study(self):
        tb = table(cm=lambda a, cl: -0.01 + 0.3 * cl - 0.35 * cl * cl)
        curve = s2.v2.build_curve_v2(SHAPE, B, tb, WS)
        a = st.roots_from_curve(curve, 0.05)
        b = v2.roots_from_curve_v2(curve, 0.05)
        self.assertEqual(len(a), len(b))
        for x, y in zip(a, b):
            for k in ("cl", "dg", "df", "cov", "covd", "jag"):
                self.assertEqual(x[k], y[k])
        self.assertTrue(all("ti" in y for y in b))


class Windows(unittest.TestCase):
    def test_contiguous_definition(self):
        w = v2.contiguous_window([0.01, 0.015, 0.02, 0.03, 0.035], 0.005)
        self.assertEqual((w["n"], w["runs"], w["widest_n"]), (5, 2, 3))
        self.assertAlmostEqual(w["width"], 0.01)
        self.assertEqual((w["sm_lo"], w["sm_hi"]), (0.01, 0.02))
        one = v2.contiguous_window([0.02], 0.005)
        self.assertEqual((one["widest_n"], one["width"]), (1, 0.0))
        self.assertIsNone(v2.contiguous_window([], 0.005)["width"])
        self.assertAlmostEqual(v2.contiguous_window([0.0, 0.0050000001, 0.01], 0.005)["width"], 0.01)

    def test_compute_windows_uses_n_minus_1_and_tiers(self):
        key = s2.skey("P01")
        shapes = {key: dict(mac=0.2, xnp=0.5)}
        sm5 = [0.01, 0.015, 0.02, 0.025]
        sm9 = [0.015, 0.02, 0.025, 0.03]
        mk = lambda nc, sm: dict(id="P01", key=key, section="NACA_0006", ncrit=nc, span=0.4, ws=8,
                                 sets={md: list(sm) for md in v2.MODES},
                                 pts=[[x, 0.3, 30000.0, 0.2, 0.0] for x in sm])
        rows = s2.compute_windows([mk(5, sm5), mk(9, sm9)], shapes)
        r = [x for x in rows if x["mode"] == "any" and x["tier"] == "both" and x["sm_class"] == "all"][0]
        self.assertEqual(r["n_points"], 3)               # 0.015, 0.02, 0.025
        self.assertAlmostEqual(r["width_pct_mac"], 1.0)  # (3-1)*0.5 %
        self.assertAlmostEqual(r["width_mm"], 1000 * 0.01 * 0.2 * 0.4)
        self.assertAlmostEqual([x for x in rows if x["mode"] == "any" and x["tier"] == "N5" and x["sm_class"] == "all"][0]["width_pct_mac"], 1.5)

    def test_sm_ge_0_class_excludes_negative(self):
        key = s2.skey("P01")
        shapes = {key: dict(mac=0.2, xnp=0.5)}
        sm = [-0.01, -0.005, 0.0, 0.005]
        mk = lambda nc: dict(id="P01", key=key, section="NACA_0006", ncrit=nc, span=0.4, ws=8, sets={md: list(sm) for md in v2.MODES},
                             pts=[[x, 0.3, 30000.0, 0.2, 0.0] for x in sm])
        rows = s2.compute_windows([mk(5), mk(9)], shapes)
        get = lambda cls: [x for x in rows if x["mode"] == "any" and x["tier"] == "both" and x["sm_class"] == cls][0]
        self.assertEqual(get("all")["n_points"], 4); self.assertEqual(get("SM>=0")["n_points"], 2)


class FinDecks(unittest.TestCase):
    def setUp(self):
        self.w = m._wing(4.5, 20, 0.7, 0)

    def _sections(self, txt, surf):
        part = txt.split(f"SURFACE\n{surf}\n")[1].split("SURFACE")[0]
        lines = part.splitlines()
        out = []
        for i, ln in enumerate(lines):
            if ln == "SECTION":
                out.append([float(x) for x in lines[i + 1].split()])
        return part, out

    def test_tip_fin_area_and_modes(self):
        for mode in ("above", "sym"):
            txt = v2.deck_with_fins(self.w, "t", tip_fin=dict(area_frac=0.04, mode=mode))
            part, sec = self._sections(txt, "TipFin")
            self.assertIn("YDUPLICATE", part)
            (x1, y1, z1, c1, _), (x2, y2, z2, c2, _) = sec
            self.assertAlmostEqual(c1, self.w.tip_chord, places=6)          # chord = tip chord
            self.assertAlmostEqual((z1 - z2) * c1, 0.04 * self.w.area, places=7)   # area = 4 % of wing area (each fin)
            self.assertGreater(z1, z2)                                         # top to bottom
            if mode == "above":
                self.assertAlmostEqual(z2, self.w.tip_z, places=9)
            else:
                self.assertAlmostEqual(0.5 * (z1 + z2), self.w.tip_z, places=9)
            self.assertAlmostEqual(x1, self.w.tip_x_le, places=9); self.assertAlmostEqual(y1, 0.5, places=9)

    def test_centre_fin_position_and_area(self):
        for k in (1.0, 1.5):
            txt = v2.deck_with_fins(self.w, "t", centre_fin=dict(area_frac=0.03, x_qc_over_root_chord=k))
            part, sec = self._sections(txt, "CentreFin")
            self.assertNotIn("YDUPLICATE", part)
            (x1, y1, z1, c1, _), (x2, y2, z2, c2, _) = sec
            self.assertAlmostEqual(x1 + 0.25 * c1, k * self.w.root_chord, places=9)     # quarter chord position
            self.assertAlmostEqual((z1 - z2) * c1, 0.03 * self.w.area, places=7)
            self.assertEqual(y1, 0.0)

    def test_bare_deck_equals_geometry_text(self):
        from avl import geometry_text
        self.assertEqual(v2.deck_with_fins(self.w, "t"), geometry_text(self.w, "t", 8, 24))

    def test_avl_fin_adds_positive_cn_beta_and_keeps_xnp(self):
        xcg = SHAPE["xnp"] - 0.05 * SHAPE["mac"]
        bare = v2.lateral_case(4.5, 20, 0.7, 0, xcg, _TMP, "bare")
        fin = v2.lateral_case(4.5, 20, 0.7, 0, xcg, _TMP, "fin", centre_fin=dict(area_frac=0.06, x_qc_over_root_chord=1.5))
        self.assertGreater(fin["cn_beta"]["0.4"], bare["cn_beta"]["0.4"] + 0.005)
        self.assertAlmostEqual(fin["cl_beta"]["0.4"], bare["cl_beta"]["0.4"], delta=0.005)
        self.assertAlmostEqual(fin["xnp"], bare["xnp"], delta=0.002 * bare["mac"] * 10)
        self.assertAlmostEqual(bare["xnp"], SHAPE["xnp"], delta=3e-4)               # lateral deck reproduces the shape's Xnp
        dih = v2.lateral_case(4.5, 20, 0.7, 3, xcg, _TMP, "dih3")
        self.assertLess(dih["cl_beta"]["0.4"], bare["cl_beta"]["0.4"] - 0.02)      # dihedral makes Cl_beta more negative


class MassModels(unittest.TestCase):
    def test_planform_matches_geometry(self):
        pf = mass.planform(0.4, 4.5, 0.7)
        w = Wing(span_m=0.4, aspect_ratio=4.5, taper=0.7, quarter_chord_sweep_deg=0, washout_deg=0, dihedral_deg=0, section="NACA_0008", cg_x_m=0)
        self.assertAlmostEqual(pf["S"], w.area); self.assertAlmostEqual(pf["mac"], w.mac); self.assertAlmostEqual(pf["cr"], w.root_chord)

    def test_section_area_integral_equals_s_mac(self):
        pf = mass.planform(0.4, 4.5, 0.7)
        n = 4000
        integral = sum((pf["cr"] + (pf["ct"] - pf["cr"]) * (i + 0.5) / n) ** 2 * (0.4 / 2 / n) * 2 for i in range(n))
        self.assertAlmostEqual(integral, pf["S"] * pf["mac"], delta=1e-6 * pf["S"] * pf["mac"])

    def test_ws_scaling_with_span(self):
        # shell, no fixed mass: W/S constant = sigma g
        for b in (0.2, 0.4, 0.6):
            r = mass.row("shell", dict(sigma=0.5, rho_eff=None, tc=None, mfix_g=0), b, 4.5, 0.4)
            self.assertAlmostEqual(r["ws_n_m2"], 0.5 * mass.G, places=9)
        # infill, no fixed mass: W/S proportional to span at fixed AR
        r1 = mass.row("infill", dict(sigma=None, rho_eff=250, tc=0.08, mfix_g=0), 0.2, 4.5, 0.4)
        r2 = mass.row("infill", dict(sigma=None, rho_eff=250, tc=0.08, mfix_g=0), 0.4, 4.5, 0.4)
        self.assertAlmostEqual(r2["ws_n_m2"] / r1["ws_n_m2"], 2.0, places=9)
        # fixed mass adds g*m_fixed/S  (~1/b^2)
        a = mass.row("shell", dict(sigma=0.5, rho_eff=None, tc=None, mfix_g=10), 0.2, 4.5, 0.4)
        z = mass.row("shell", dict(sigma=0.5, rho_eff=None, tc=None, mfix_g=0), 0.2, 4.5, 0.4)
        self.assertAlmostEqual(a["ws_n_m2"] - z["ws_n_m2"], 0.010 * mass.G / a["area_m2"], places=9)

    def test_speed_re_and_bands(self):
        r = mass.row("shell", dict(sigma=0.8, rho_eff=None, tc=None, mfix_g=10), 0.4, 4.5, 0.4)
        self.assertAlmostEqual(r["v_m_s"], math.sqrt(2 * r["ws_n_m2"] / (1.225 * 0.4)))
        self.assertAlmostEqual(r["re_mac"], r["v_m_s"] * r["mac_m"] / 1.5e-5)
        self.assertAlmostEqual(r["frac_lt20k"] + r["frac_20_40k"] + r["frac_40_80k"] + r["frac_gt80k"], 1.0, places=9)
        tiny = mass.row("shell", dict(sigma=0.3, rho_eff=None, tc=None, mfix_g=0), 0.15, 6, 0.2)
        self.assertEqual(tiny["re_mac_below_db_floor"], 1)


class CurveWithTi(unittest.TestCase):
    def test_ti_only_in_band(self):
        t = table(xtr=lambda a: 1.0 if a < 2.0 else 0.6, cm=lambda a, cl: 0.0)
        c = v2.build_curve_v2(SHAPE, B, t, WS)
        for p in c:
            if 0.15 <= p["cl"] <= 0.6 and p["cov"] > 0:
                self.assertIsNotNone(p["ti"])
            else:
                self.assertIsNone(p["ti"])


if __name__ == "__main__":
    unittest.main()
