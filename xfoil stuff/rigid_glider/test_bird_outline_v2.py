"""Unit tests for bird_outline_v2.py.  Run from rigid_glider/:  py -B -m unittest test_bird_outline_v2 -v
Writes to temporary folders only; AVL runs serially.  The output tests run the whole pipeline in QUICK mode on a coarse AVL panelling (a smoke test of every
file, not a result)."""
import sys
sys.dont_write_bytecode = True
import csv
import hashlib
import math
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import bird_outline as bo
import bird_outline_v2 as v2
import flat_test as ft


def rect_params(ar=6.0):
    p = v2.base_params("BS")
    p.update(ar_target=ar, arm_sweep_deg=0.0, hand_sweep_deg=0.0, cw_over_croot=1.0, hand_stations=((0.0, 1.0), (1.0, 1.0)), tail_len_frac=0.05, tail_spread_frac=0.1,
             tail_root_frac=0.03, body_halfwidth_frac=0.01)
    return p


def turn_angles(poly):
    out = []
    n = len(poly)
    for i in range(n):
        a, b, c = poly[i - 1], poly[i], poly[(i + 1) % n]
        v1, v2_ = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
        out.append(abs(math.degrees(math.atan2(v1[0] * v2_[1] - v1[1] * v2_[0], v1[0] * v2_[0] + v1[1] * v2_[1]))))
    return out


class GeometryTests(unittest.TestCase):
    def test_features_off_reproduces_v1(self):
        for fam in bo.FAM_ORDER:
            g1, g2 = bo.family_geometry(fam), v2.build_geometry2(v2.base_params(fam))
            for k in ("S", "mac", "x_le_mac", "c_root", "ar", "sheet_area", "tail_area", "x_ref", "c_wrist", "x_wrist", "Lt", "St", "wt0", "x_h"):
                self.assertAlmostEqual(g1[k], g2[k], places=9, msg=(fam, k))
            self.assertAlmostEqual(abs(ft.area_signed(g1["full_outline"])), abs(ft.area_signed(g2["full_outline"])), places=10)
            self.assertAlmostEqual(g1["S"], g2["S_polygon"], places=10)

    def test_each_feature_changes_only_its_own_parameters(self):
        for fam in bo.FAM_ORDER:
            base = v2.base_params(fam)
            for f in v2.FEATURES:
                p = v2.feature_params(fam, {f: 1})
                changed = {k for k in p if p[k] != base[k]}
                self.assertEqual(changed, v2.feature_keys(f, fam), (fam, f))
                half = v2.feature_params(fam, {f: 0.5})
                for k in changed:
                    self.assertAlmostEqual(half[k], (base[k] + p[k]) / 2, places=12)            # strength interpolates linearly
                self.assertEqual(v2.feature_params(fam, {f: 0.0}), base)

    def test_each_feature_toggles_exactly_its_own_geometry(self):
        for fam in bo.FAM_ORDER:
            g0 = v2.build_geometry2(v2.base_params(fam))
            sec0, tail0 = bo.wing_sections(g0), bo.tail_sections(g0)
            for f in v2.FEATURES:
                g = v2.build_geometry2(v2.feature_params(fam, {f: 1}))
                same = lambda *ks: all(abs(g[k] - g0[k]) < 1e-12 for k in ks)
                if f == "nose":
                    self.assertTrue(same("x_h", "c_root", "S", "mac", "Lt", "St", "wt0", "x_wrist"))
                    self.assertEqual(bo.wing_sections(g), sec0)
                    self.assertEqual(bo.tail_sections(g), tail0)
                    self.assertNotEqual(g["nose_x"], g0["nose_x"])
                elif f == "root_blend":
                    self.assertTrue(same("x_h", "c_root", "S", "mac", "Lt", "St", "wt0", "nose_x"))
                    self.assertEqual(bo.wing_sections(g), sec0)
                    self.assertEqual(bo.tail_sections(g), tail0)
                    self.assertNotEqual(len(g["full_outline"]), len(g0["full_outline"]))
                elif f == "spread":
                    self.assertTrue(same("x_h", "c_root", "S", "mac", "Lt", "wt0", "x_wrist"))
                    self.assertEqual(bo.wing_sections(g), sec0)
                    self.assertGreater(g["St"], g0["St"])
                elif f == "neck":
                    self.assertTrue(same("Lt", "St", "x_wrist", "y_wrist", "nose_x"))
                    self.assertGreater(g["wt0"], g0["wt0"])
                elif f == "m_shape":
                    self.assertTrue(same("Lt", "St", "wt0", "y_wrist", "nose_x"))
                    self.assertLess(g["x_wrist"], g0["x_wrist"])                                  # wrist is further forward
                    self.assertEqual(bo.tail_sections(g)[0][0], tail0[0][0])
                elif f in ("curved_te", "tip_round"):
                    self.assertTrue(same("Lt", "St", "wt0", "y_wrist", "nose_x", "x_wrist"))
                    self.assertEqual(g["p"]["hand_sweep_deg"], g0["p"]["hand_sweep_deg"])
                    if f == "tip_round":
                        self.assertLess(g["te_nodes"][-1] - g["le_nodes"][-1], 0.05 * g["c_wrist"])   # rounded tip closes
                    else:
                        self.assertNotEqual(g["te_nodes"], g0["te_nodes"])
                self.assertAlmostEqual(g["ar"], bo.FAMILIES[fam]["params"]["ar_target"][0], places=6)   # the aspect ratio is held for every feature

    def test_all_outlines_simple_clean_and_ccw(self):
        for fam in bo.FAM_ORDER:
            combos = [{f: 1} for f in v2.FEATURES] + [{f: 1 for f in v2.FEATURES}, {f: 0.5 for f in v2.FEATURES}, {}]
            for on in combos:
                g = v2.build_geometry2(v2.feature_params(fam, on))
                poly = g["full_outline"]
                self.assertTrue(v2.polygon_is_simple(poly), (fam, on))
                self.assertGreater(ft.area_signed(poly), 0)
                self.assertEqual(v2.simplify(poly), poly)                                        # no collinear vertices left
                self.assertAlmostEqual(g["S"], g["S_polygon"], places=9)
                for i in range(len(poly)):
                    self.assertGreater(math.dist(poly[i], poly[(i + 1) % len(poly)]), 1e-5)
                ys = g["y_nodes"]
                self.assertTrue(all(ys[i + 1] - ys[i] >= v2.DY_MIN - 1e-12 or ys[i + 1] in (g["y_wrist"], g["half_span"], g["wt0"] / 2) or ys[i] in (0.0, g["wt0"] / 2, g["y_wrist"]) for i in range(len(ys) - 1)))

    def test_smoothing_reduces_the_sharpest_corners(self):
        for fam in bo.FAM_ORDER:
            g0 = v2.build_geometry2(v2.feature_params(fam, {"m_shape": 1}))
            p = v2.feature_params(fam, {"m_shape": 1})
            p["wrist_blend_frac"] = 0.0
            g_sharp = v2.build_geometry2(p)
            lead = lambda g: [a for (x, y), a in zip(g["full_outline"], turn_angles(g["full_outline"])) if 0 < y and abs(y - g["y_wrist"]) < 0.12 * g["half_span"] and x < g["x_wrist"] + 0.5 * g["c_wrist"]]
            self.assertLess(max(lead(g0)), max(lead(g_sharp)))                                    # rounded wrist has smaller turning angle than the sharp one
            gb = v2.build_geometry2(v2.feature_params(fam, {"root_blend": 1}))
            gv = v2.build_geometry2(v2.base_params(fam))
            near = lambda g: max(a for (x, y), a in zip(g["full_outline"], turn_angles(g["full_outline"])) if abs(y) < 0.2 * g["half_span"] and x < 0.5 * g["x_h"])
            self.assertLess(near(gb), near(gv))                                                    # no hard corner where the body meets the wing

    def test_rectangle_hand_case(self):
        g = v2.build_geometry2(rect_params(6.0))
        self.assertAlmostEqual(g["c_root"], 1 / 6, places=9)
        self.assertAlmostEqual(g["S"], 1 / 6, places=9)
        self.assertAlmostEqual(g["mac"], 1 / 6, places=9)
        self.assertAlmostEqual(g["x_le_mac"], 0.0, places=12)
        self.assertAlmostEqual(g["S"], g["S_polygon"], places=12)

    def test_mac_against_numeric_chord_all_on(self):
        for fam in bo.FAM_ORDER:
            g = v2.build_geometry2(v2.feature_params(fam, {f: 1 for f in v2.FEATURES}))
            ys, les, tes = g["y_nodes"], g["le_nodes"], g["te_nodes"]
            n = 20000
            h = g["half_span"] / n
            num = den = xl = 0.0
            for i in range(n):
                y = (i + 0.5) * h
                j = min(max(k for k in range(len(ys)) if ys[k] <= y), len(ys) - 2)
                t = (y - ys[j]) / (ys[j + 1] - ys[j])
                le = les[j] + t * (les[j + 1] - les[j])
                c = (tes[j] + t * (tes[j + 1] - tes[j])) - le
                num += c * c * h
                den += c * h
                xl += le * c * h
            self.assertAlmostEqual(g["mac"], num / den, places=5)
            self.assertAlmostEqual(g["x_le_mac"], xl / den, places=5)

    def test_tail_edge_fins_are_dihedral_panels(self):
        g = v2.build_geometry2(v2.base_params("BC"))
        self.assertEqual([(y, x, 0.0, c) for y, x, c in bo.tail_sections(g)], v2.tail_sections2(g, None))
        secs = v2.tail_sections2(g, (0.3, 60.0))
        self.assertGreater(secs[-1][2], 0)                                                     # outer edge is raised
        self.assertAlmostEqual(secs[-1][2], 0.3 * g["St"] / 2 * math.sin(math.radians(60.0)), places=9)
        self.assertEqual(len(secs), len(bo.tail_sections(g)) + 1)                             # one extra section at the fold line

    def test_parameters_and_provenance(self):
        for f in v2.FEATURES:
            self.assertIn(f, v2.FEATURE_LABEL)
        for fam in bo.FAM_ORDER:
            for k, (v, src, note) in bo.FAMILIES[fam]["params"].items():
                self.assertIn(src, ("CITED", "JUDGEMENT"))
        self.assertEqual(set(v2.AERO_FEATURES) | {"root_blend", "nose"}, set(v2.FEATURES))


class ArithmeticTests(unittest.TestCase):
    def fake_res(self, g, cm0=0.03):
        mac = g["mac"]
        npx = g["x_le_mac"] + 0.32 * mac
        return dict(np_span=npx, np_pct_mac=32.0, cm0=cm0, tail_lift_frac=-0.08, m_wrist_over_w_m=0.008, span_eff=0.97, cdi_cl03=0.003, tip_loading=1.2, cn_beta=0.005, cl_beta=-0.05,
                    trim={c: dict(x_cg_span=npx - mac * cm0 / c, static_margin=cm0 / c) for c in v2.TRIM_CLS})

    def test_ballast_puts_the_cg_on_target(self):
        g = v2.build_geometry2(v2.base_params("BS"))
        res = self.fake_res(g)
        for sk in ("printed", "foam"):
            n = v2.size_numbers(g, res, sk, 5.0)
            if n["reachable"]:
                cg = (n["M0"] * n["cg0"] + n["ballast_g"] * n["station"]) / (n["M0"] + n["ballast_g"])
                self.assertAlmostEqual(cg, n["cg_target"], places=6)
            self.assertAlmostEqual(n["M"], n["M0"] + n["ballast_g"], places=9)
            self.assertAlmostEqual(n["ws"], n["M"] * 1e-3 * 9.80665 / (g["S"] * n["span"] ** 2 * 1e-6), places=9)

    def test_rail_saddle_ends_ahead_of_the_hinge_and_ticks_cover_trim(self):
        for fam in bo.FAM_ORDER:
            g = v2.build_geometry2(v2.base_params(fam))
            res = self.fake_res(g, 0.04)
            for sk in ("printed", "foam"):
                n = v2.size_numbers(g, res, sk, 5.0)
                self.assertLessEqual(n["rail"]["saddle_end"], g["x_h"] * n["span"] - v2.RAIL_HINGE_GAP_MM + 1e-9)
                self.assertGreater(n["rail"]["saddle_end"], 0)
                self.assertLessEqual(n["rail"]["tick_range"][0], min(n["cgs"]))
                self.assertGreaterEqual(n["rail"]["tick_range"][1], max(n["cgs"]))
                self.assertGreater(n["rail"]["mass_g"], 0.3)
                self.assertEqual(ft.edge_report(n["rail"]["tris"], nd=4)[1:], (0, 0, 0))

    def test_structure_hand_case(self):
        g = dict(tail_centroid_x=0.30, x_h=0.20, wt0=0.05, chord_wrist_local=0.10, mac=0.12, x_ref=0.05)
        res = dict(cm0=0.03, m_wrist_over_w_m=0.01)
        sn = dict(W=1.0)
        s = v2.structure_numbers(g, res, sn, 250.0)
        P = 0.03 * 0.12 / (0.3 * (0.30 - 0.05)) * 1.0 * v2.N_LOAD
        self.assertAlmostEqual(s["tail_load_N"], P)
        self.assertAlmostEqual(s["arm_mm"], 25.0)
        self.assertAlmostEqual(s["sigma_neck_MPa"], 6 * P * 25.0 / (12.5 * v2.HINGE_V2_MM ** 2), places=9)
        self.assertAlmostEqual(v2.structure_numbers(g, res, sn, 250.0, 5.0)["hinge_strain_pct"], 100 * v2.HINGE_V2_MM * math.radians(5.0) / (2 * bo.HINGE_WIDTH_MM), places=9)
        self.assertAlmostEqual(s["wrist_moment_Nmm"], 0.01 * 1000 * 1.0 * v2.N_LOAD, places=9)
        self.assertAlmostEqual(s["sigma_wrist_MPa"], 6 * s["wrist_moment_Nmm"] / (25.0 * 0.6 ** 2), places=9)
        self.assertGreaterEqual(s["neck_min_mm"], v2.NECK_MIN_FRAC_SPAN * 250.0)
        wide = dict(g, wt0=0.20)
        self.assertLess(v2.structure_numbers(wide, res, sn, 250.0)["sigma_neck_MPa"], s["sigma_neck_MPa"])
        self.assertEqual(v2.structure_numbers(g, dict(res, cm0=0.0), sn, 250.0)["sigma_neck_MPa"], 0.0)               # no bend, no tail moment, no neck load      # a wider neck lowers the hinge stress

    def test_verdict_thresholds(self):
        self.assertEqual(v2.verdict("cn_beta", 0.004, 0.006), "helps")
        self.assertEqual(v2.verdict("cn_beta", 0.004, 0.0041), "neutral")
        self.assertEqual(v2.verdict("cn_beta", 0.004, 0.002), "hurts")
        self.assertEqual(v2.verdict("cl_beta", -0.05, -0.07), "helps")
        self.assertEqual(v2.verdict("sheet_area_mm2", 1000.0, 1100.0), "hurts")
        self.assertEqual(v2.verdict("np_pct_mac", 30.0, 35.0), "context")


class SelectionTests(unittest.TestCase):
    def row(self, **kw):
        r = dict(cn_beta=0.006, cl_beta=-0.05, sm03=0.11, x_cg03_span=0.05, nose_x_span=-0.05, ballast_frac_printed=0.1, ballast_frac_foam=0.08, reachable_all=True, tip_loading=0.5,
                 neck_ok=True, appearance_cost_frac=0.0, valid_outline=True, n_fins=0, n_features=2, n_justified=1, sheet_area_mm2=8000.0)
        r.update(kw)
        r["fails"] = v2.criteria_failures(r, 1.0)
        r["margin"] = v2.margin_of(r, 1.0)
        return r

    def test_each_criterion_fails_alone(self):
        self.assertEqual(self.row()["fails"], [])
        cases = {"Cn_beta": dict(cn_beta=0.001), "Cl_beta": dict(cl_beta=0.01), "static": dict(sm03=0.04), "static ": dict(sm03=0.2), "trim": dict(x_cg03_span=-0.06),
                 "ballast": dict(ballast_frac_foam=0.3), "tip": dict(tip_loading=1.5), "tail": dict(neck_ok=False), "appearance-only": dict(appearance_cost_frac=0.1), "outline": dict(valid_outline=False),
                 "trim CG not": dict(reachable_all=False)}
        for key, kw in cases.items():
            f = self.row(**kw)["fails"]
            self.assertEqual(len(f), 1, (key, f))
            self.assertTrue(f[0].startswith(key.strip()) or key.strip() in f[0], (key, f))

    def test_selection_ranking_on_a_synthetic_table(self):
        rows = [self.row(n_features=3, sheet_area_mm2=9000.0), self.row(n_features=5, sheet_area_mm2=9500.0), self.row(n_features=5, sheet_area_mm2=9400.0, ballast_frac_printed=0.12),
                self.row(n_features=7, n_fins=1, n_justified=4), self.row(n_features=6, cn_beta=0.001), self.row(n_features=4, sm03=0.2), self.row(n_features=7, n_justified=0)]
        ok, bad = v2.select(rows)
        self.assertEqual(len(ok), 5)
        self.assertEqual(len(bad), 2)
        self.assertEqual([r["n_features"] for r in ok], [5, 5, 3, 7, 7])                        # fewest fins first, then most justified features, then most features, then margin
        self.assertEqual([r["n_justified"] for r in ok], [1, 1, 1, 0, 4])
        self.assertGreater(ok[0]["margin"], ok[1]["margin"])
        self.assertEqual(ok[-1]["n_fins"], 1)
        none_ok, _ = v2.select([self.row(cn_beta=0.001), self.row(ballast_frac_printed=0.5)])
        self.assertEqual(none_ok, [])
        waived, rest = v2.select([self.row(cn_beta=0.001), self.row(ballast_frac_printed=0.5, n_features=6), self.row(ballast_frac_printed=0.3, n_features=2)], waive=("ballast",))
        self.assertEqual([r["n_features"] for r in waived], [2, 6])                              # waived quantity ranks first: smaller ballast wins
        self.assertEqual(len(rest), 1)

    def test_interp_bend_is_linear_and_exact_at_the_ends(self):
        r0 = dict(cm0=0.0, tail_lift_frac=0.02, m_wrist_over_w_m=0.1, span_eff=1.0, cdi_cl03=0.003, tip_loading=1.0, cn_beta=0.004, cl_beta=-0.05, trim={}, np_span=0.1)
        r5 = dict(r0, cm0=0.05, tail_lift_frac=-0.08, tip_loading=1.1)
        self.assertEqual(v2.interp_bend(r0, r5, 5.0)["cm0"], 0.05)
        self.assertAlmostEqual(v2.interp_bend(r0, r5, 2.0)["cm0"], 0.02)
        self.assertAlmostEqual(v2.interp_bend(r0, r5, 2.0)["tail_lift_frac"], 0.02 - 0.1 * 0.4)
        self.assertAlmostEqual(v2.interp_bend(r0, r5, 10.0)["tip_loading"], 1.2)


class AvlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="bird2_avl_")
        cls.dir = Path(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_avl_input_regenerates_identically(self):
        g1, g2 = v2.build_geometry2(v2.base_params("BS")), v2.build_geometry2(v2.base_params("BS"))
        a, b = v2.avl_deck2(g1, 250.0, 5.0), v2.avl_deck2(g2, 250.0, 5.0)
        self.assertEqual(hashlib.sha256(a.encode()).hexdigest(), hashlib.sha256(b.encode()).hexdigest())
        self.assertNotEqual(a, v2.avl_deck2(g1, 250.0, 7.5))
        self.assertNotEqual(a, v2.avl_deck2(g1, 250.0, 5.0, edge=(0.3, 30.0)))
        self.assertIn("VentralFin", v2.avl_deck2(g1, 250.0, 5.0, vfin=bo.vfin_for(g1, 0.03)))

    def test_tail_incidence_is_minus_the_bend(self):
        g = v2.build_geometry2(v2.base_params("BC"))
        deck = v2.avl_deck2(g, 250.0, 7.5)
        tail = deck[deck.index("SURFACE\nTail"):]
        incs = [float(l.split()[4]) for l in tail.splitlines() if re.match(r"^[-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+$", l)]
        self.assertTrue(incs)
        self.assertTrue(all(abs(i + 7.5) < 1e-9 for i in incs))

    def test_bend_zero_gives_zero_moment_and_bend_adds_nose_up(self):
        g = v2.build_geometry2(v2.base_params("BS"))
        r0 = v2.run_case2(g, 250.0, 0.0, self.dir / "b0", "b0")
        r5 = v2.run_case2(g, 250.0, 5.0, self.dir / "b5", "b5")
        self.assertLess(abs(r0["cm0"]), 1e-3)
        self.assertGreater(r5["cm0"], 0.01)
        for c, tr in r5["trim"].items():
            self.assertAlmostEqual(tr["x_cg_span"], r5["np_span"] - g["mac"] * r5["cm0"] / c)
        self.assertAlmostEqual(r5["np_from_cm_slope_span"], r5["np_span"], delta=2e-3)
        self.assertLess(r5["tail_lift_frac"], r0["tail_lift_frac"])                              # bending the tail up unloads / downloads it
        self.assertGreater(r5["span_eff"], 0.5)
        self.assertLess(r5["span_eff"], 1.5)

    def test_rectangular_wing_neutral_point_near_quarter_chord(self):
        g = v2.build_geometry2(rect_params(6.0))
        r = v2.run_case2(g, 250.0, 0.0, self.dir / "rect", "rect", with_tail=False)
        self.assertAlmostEqual((r["np_span"] - g["x_le_mac"]) / g["mac"], 0.25, delta=0.02)

    def test_strip_areas_rebuild_the_wing_area(self):
        g = v2.build_geometry2(v2.base_params("BS"))
        r = v2.run_case2(g, 250.0, 5.0, self.dir / "strips", "strips")
        import glob
        fs = sorted(glob.glob(str(self.dir / "strips" / "cl03" / "*.fs")))
        self.assertTrue(fs)
        strips = v2.parse_strips(Path(fs[0]))
        right = [s for s in strips["Wing"]]
        a = sum(s["a"] for s in right)
        self.assertAlmostEqual(a, g["S"] * 0.25 ** 2 / 2, delta=0.03 * g["S"] * 0.25 ** 2 / 2)

    def test_runner_cache_key_ignores_body_features(self):
        R = v2.Runner(self.dir / "cache", avl=False)
        self.assertEqual(R.key("BS", {"nose": 1, "root_blend": 1}, 5.0, None, None), R.key("BS", {}, 5.0, None, None))
        self.assertNotEqual(R.key("BS", {"neck": 1}, 5.0, None, None), R.key("BS", {}, 5.0, None, None))
        self.assertNotEqual(R.key("BS", {}, 5.0, 0.03, None), R.key("BS", {}, 5.0, None, None))


def text_boxes(txt):
    root = ET.fromstring(txt)
    out = []
    for el in root:
        if el.tag.split("}")[-1] != "text":
            continue
        s = float(el.get("font-size"))
        x, y = float(el.get("x")), float(el.get("y"))
        w = 0.5 * s * len(el.text or "")
        a = el.get("text-anchor", "start")
        x0 = x if a == "start" else (x - w if a == "end" else x - w / 2)
        out.append((x0, y - 0.8 * s, x0 + w, y + 0.2 * s, el.text))
    return out


class OutputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="bird2_out_")
        cls.out = Path(cls.tmp.name)
        cls._save = (v2.AVL_NCHORD, v2.AVL_NSPAN, v2.DY_MIN, v2.panelling_check)
        v2.AVL_NCHORD, v2.AVL_NSPAN, v2.DY_MIN = 6, 40, 0.013                       # coarse panelling: a smoke test of the files, not a result
        v2.panelling_check = lambda *a, **k: []
        cls.R = v2.run(cls.out, True, True, True, log=lambda *a: None)

    @classmethod
    def tearDownClass(cls):
        v2.AVL_NCHORD, v2.AVL_NSPAN, v2.DY_MIN, v2.panelling_check = cls._save
        cls.tmp.cleanup()

    def rows(self, name):
        with (self.out / name).open(encoding="utf-8") as f:
            return list(csv.DictReader(f))

    def test_row_counts(self):
        self.assertEqual(len(self.rows("features_single.csv")), len(bo.FAM_ORDER) * len(v2.FEATURES) * len(v2.QUANT))
        self.assertEqual(len(self.rows("features_strength.csv")), len(bo.FAM_ORDER) * len(v2.FEATURES) * 3)
        self.assertEqual(len(self.rows("bends.csv")), len(bo.FAM_ORDER) * 2 * len(v2.BENDS))
        self.assertEqual(len(self.rows("yaw.csv")), len(bo.FAM_ORDER) * (len(v2.BENDS) + 1) * len(v2.YAW_OPTIONS))
        scr = [r for r in self.rows("selection.csv") if r["stage"].startswith("screening")]
        self.assertEqual(len(scr), len(bo.FAM_ORDER) * len(v2.aero_combos(True)) * 4 * len(v2.SELECTION_BENDS))
        self.assertEqual({r["bend_deg"] for r in self.rows("bends.csv")}, {"0.0", "2.0", "3.0", "5.0", "7.5", "10.0"})
        for r in scr:
            self.assertTrue(r["passes"] in ("yes", "no"))
            self.assertEqual(r["passes"] == "no", bool(r["failures"]))

    def test_finals_meet_the_criteria_or_say_why_not(self):
        for fam in bo.FAM_ORDER:
            a = self.R["A"][fam]
            f = a["final"]
            left = v2.eff_fails(f, a["waive"])
            if a["flagged"]:                                                                      # nothing passed even with the waiver: the failures must be on the record
                self.assertTrue(left)
                continue
            self.assertEqual(left, [], (fam, left))
            if a["waive"]:
                self.assertTrue(f["fails"])                                                      # the waived failure is reported, not hidden
            else:
                self.assertEqual(f["fails"], [])
                self.assertGreaterEqual(f["cn_beta"] if not f.get("fin") else v2.CN_BETA_MIN, v2.CN_BETA_MIN)

    def test_stl_rails_dxf_and_drawings(self):
        for fam in bo.FAM_ORDER:
            for sk, k in (("printed", 250), ("foam", 400)):
                d = self.out / f"{fam}_{sk}_span{k}"
                for nm in ("rail.stl", "parts.dxf", "drawing.svg"):
                    self.assertTrue((d / nm).is_file(), (d, nm))
                svg = (d / "drawing.svg").read_text(encoding="utf-8")
                ET.fromstring(svg)
                self.assertIn("#14181d", svg)
                tris = ft.read_stl(d / "rail.stl")
                self.assertEqual(ft.edge_report(tris, nd=4)[1:], (0, 0, 0))
                st = ft.mesh_stats(tris)
                self.assertLess(st["centroid"][0], self.R["A"][fam]["final"]["g"]["x_h"] * k)
                self.assertTrue((d / "sheet.stl").is_file() == (sk == "printed"))
            tris = ft.read_stl(self.out / f"{fam}_printed_span250" / "sheet.stl")
            self.assertGreater(len(tris), 100)
            dxf = (self.out / f"{fam}_printed_span250" / "parts.dxf").read_text()
            for layer in ("OUTLINE", "HINGE_BEND_UP", "WRIST_MARKER", "CENTRELINE", "CG_MARKS"):
                self.assertIn(layer, dxf)

    def test_rails_saddle_ahead_of_hinge_and_cover_all_trim_cgs(self):
        n = 0
        for b in self.R["builds"]:
            n += 1
            self.assertTrue(b["rail_fits_ahead_of_hinge"], b["fam"])
            self.assertLessEqual(b["rail_union"]["saddle_end"], b["hinge"] - v2.RAIL_HINGE_GAP_MM + 1e-9)
            self.assertTrue(b["rail_covers_all_bends"], (b["fam"], b["size"], b["rail_union"]["tick_range"], b["cgs"]))
            self.assertGreater(b["rail_union"]["tick_range"][0], b["rail_union"]["x_lo"])
        self.assertGreaterEqual(n, 4)
        for r in self.rows("builds.csv"):
            self.assertEqual(r["rail_saddle_ends_ahead_of_hinge"], "yes")
            self.assertEqual(r["rail_ticks_cover_trim_CGs_all_bends"], "yes")

    def test_printed_mass_matches_screening_arithmetic(self):
        for fam in bo.FAM_ORDER:
            g = self.R["A"][fam]["final"]["g"]
            exact = v2.sheet_mass_g(g, "printed", exact=True)
            approx = v2.sheet_mass_g(g, "printed", exact=False)
            self.assertAlmostEqual(exact, approx, delta=0.03 * approx)

    def test_templates_supported_elements_scale_bar_and_no_overlaps(self):
        pages = [p for fam in bo.FAM_ORDER for p in self.R["info"][fam]["template_pages"]]
        self.assertGreaterEqual(len(pages), 8)
        for p in pages:
            txt = p.read_text(encoding="utf-8")
            root = ET.fromstring(txt)
            self.assertTrue({el.tag.split("}")[-1] for el in root} <= {"rect", "line", "polygon", "text"}, p)
            self.assertEqual(root.get("viewBox"), "0 0 215.9 279.4")
            m = re.search(r'<line id="scalebar" x1="([-0-9.]+)" y1="([-0-9.]+)" x2="([-0-9.]+)" y2="([-0-9.]+)"', txt)
            self.assertIsNotNone(m, p)
            x1, y1, x2, y2 = map(float, m.groups())
            self.assertEqual(y1, y2)
            self.assertAlmostEqual(x2 - x1, 100.0, places=9)
            self.assertNotIn("#14181d", txt)
            boxes = text_boxes(txt)
            for (a0, b0, a1, b1, s) in boxes:                                                    # every label lies inside the printable margins
                self.assertGreaterEqual(a0, ft.TEMPLATE_MARGIN - 0.01, (p, s))
                self.assertLessEqual(a1, ft.LETTER_W - ft.TEMPLATE_MARGIN + 0.01, (p, s))
                self.assertGreaterEqual(b0, 4.0, (p, s))
                self.assertLessEqual(b1, ft.LETTER_H - ft.TEMPLATE_MARGIN + 0.01, (p, s))
            for i in range(len(boxes)):                                                          # no two labels overlap (the v1 flaw)
                for j in range(i + 1, len(boxes)):
                    A_, B_ = boxes[i], boxes[j]
                    ov = A_[0] < B_[2] and B_[0] < A_[2] and A_[1] < B_[3] and B_[1] < A_[3]
                    self.assertFalse(ov, (p.name, A_[4], B_[4]))
            for (a0, b0, a1, b1, s) in boxes:                                                    # and none touches the scale bar
                self.assertFalse(a0 < x2 and x1 < a1 and b0 < y1 + 2.5 and y1 - 2.5 < b1 and "Scale bar" not in s and "If it does not" not in s, (p.name, s))

    def test_template_content_hinge_centreline_ticks(self):
        for fam in bo.FAM_ORDER:
            g = self.R["A"][fam]["final"]["g"]
            for sk, k in (("printed", 250.0), ("foam", 400.0)):
                pages = sorted((self.out / f"{fam}_{sk}_span{int(k)}").glob("template_page*.svg"))
                txts = [p.read_text(encoding="utf-8") for p in pages]
                for t in txts[:-1]:
                    for needle in ("centreline", "stroke-dasharray", "green ticks", "UP"):
                        self.assertIn(needle, t, (fam, needle))
                    self.assertGreaterEqual(t.count('stroke="#000" stroke-width="0.3"'), 5)
                self.assertTrue(any("hinge: bend tail UP" in t for t in txts[:-1]), fam)
                self.assertIn("gauge for a", txts[-1])
                m = re.search(r'<line [^>]*x1="([-0-9.]+)" y1="([-0-9.]+)" x2="([-0-9.]+)" y2="([-0-9.]+)" stroke="#c2621a" stroke-width="0.7"', txts[0])
                self.assertIsNotNone(m)
                ycl = v2.TOP_RESERVE + g["half_span"] * k + 2.0
                self.assertAlmostEqual(float(m.group(4)), ycl - g["wt0"] * k / 2, places=2)
                xs = [x * k for x, y in g["half_outline"]]
                if max(xs) - min(xs) + 8 > ft.LETTER_W - 20:
                    self.assertGreater(len(txts) - 1, 1)
                    self.assertIn("overlap", txts[0])

    def test_pdf_produced(self):
        pdf = self.out / self.R["pdf"]
        self.assertTrue(pdf.is_file())
        data = pdf.read_bytes()
        n_pages = sum(len(self.R["info"][fam]["template_pages"]) for fam in bo.FAM_ORDER)
        self.assertGreaterEqual(data.count(b"/Type /Page") - data.count(b"/Type /Pages"), n_pages)

    def test_page_leads_with_finals_and_is_honest(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        low = page.lower()
        h2 = re.findall(r"<h2>(.*?)</h2>", page)
        self.assertTrue(h2[0].startswith("1. The two proposed outlines"))
        self.assertTrue(h2[1].startswith("2. Feature / what the model says it does / kept because"))
        for needle in ("proposals for kai, not decisions", "nothing is traced from a bird image", "no planform data in the notes", "outline-only", "says nothing about flat-plate stall",
                       "not shown to stabilise a bird", "nothing here is a flight result", "unknown", "appearance"):
            self.assertIn(needle, low, needle)
        for banned in ("will pull out", "will glide", "glide ratio of", "predicted cg window", "the tail stabilises"):
            self.assertNotIn(banned, low, banned)
        for tg in ("CITED", "JUDGEMENT", "GEOMETRY", "AVL", "ARITHMETIC", "ASSUMED"):
            self.assertIn(tg, page)
        self.assertIn("#14181d", page)
        self.assertNotIn("background:#fff", low)
        for f in v2.FEATURES:
            self.assertIn(f"<b>{f}</b>", page)

    def test_every_feature_row_is_justified_or_labelled_appearance_only(self):
        rows = v2.feature_rows(self.R["A"])
        self.assertEqual(len(rows), len(bo.FAM_ORDER) * len(v2.FEATURES))
        for r in rows:
            self.assertTrue(r["text"])
            if r["on"]:
                self.assertTrue(r["kind"] in ("benefit", "appearance"))
                if r["kind"] == "appearance":
                    self.assertIn("APPEARANCE ONLY", r["text"])
                else:
                    self.assertIn("modelled benefit", r["text"])
            else:
                self.assertTrue(r["text"].startswith("NOT in the final"))

    def test_default_output_is_new_folder(self):
        self.assertEqual(v2.OUT_DEFAULT.name, "bird_outline_v2")
        self.assertNotEqual(v2.OUT_DEFAULT, bo.OUT_DEFAULT)
        import inspect
        self.assertIn("--out", inspect.getsource(v2.main))


if __name__ == "__main__":
    unittest.main()
