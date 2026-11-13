"""Tests for sheet_builder.py.   py -B rigid_glider/test_sheet_builder.py
Builds the three shipped configs plus synthetic cases in memory / a temp folder (nothing is written under results/)."""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import math
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sheet_builder as sb

CFG = HERE / "sheet_builder_configs"
_cache = {}


def kit(name):
    if name not in _cache:
        cfg = sb.load_config(CFG / (name + ".json"))
        _cache[name] = sb.build_kit(cfg, CFG)
    return _cache[name]


class TestStl(unittest.TestCase):
    def test_every_stl_manifold_oriented_positive_volume(self):
        for name in ("swiftA_400", "swiftA_250", "b45_400"):
            res = kit(name)
            with tempfile.TemporaryDirectory() as td:
                sb.write_kit(res, Path(td))
                n = 0
                for f in sorted((Path(td) / "parts").glob("*.stl")):
                    tris = sb.read_stl(f)
                    c = sb.mesh_check(tris, nd=3)
                    self.assertTrue(c["ok"], (name, f.name, c))
                    n += 1
                self.assertGreaterEqual(n, 3)

    def test_rail_shells_each_closed(self):
        for name in ("swiftA_400", "b45_400"):
            for k, v in kit(name)["rail_shells"].items():
                self.assertTrue(sb.mesh_check(v)["ok"], (name, k))

    def test_volume_vs_analytic(self):
        for name in ("swiftA_400", "swiftA_250", "b45_400"):
            res = kit(name)
            P, bead = res["P"], res["wing"]["bead"]
            H = sb.region_heights(P, bead)
            for pc in res["wing"]["pieces"]:
                exp = 0.0
                for r in pc["regions"]:
                    a = abs(sb.parea(r["outer"])) - sum(abs(sb.parea(h)) for h in r["holes"])
                    hv = H[r["cls"]]
                    if callable(hv):
                        cy = sb.centroid(r["outer"])
                        hv = hv(cy[0], cy[1])
                    exp += a * hv
                for k in pc["keys"]:
                    exp += abs(sb.parea(k["poly"])) * H["K"]
                got = sb.mesh_volume(pc["tris_halfsolid"])
                self.assertLess(abs(got - exp) / exp, 0.01, (name, got, exp))

    def test_plain_sheet_volume_is_area_times_thickness(self):
        o = sb.load_outline({"type": "builtin", "builtin": "swiftA"}, 400.0)
        tris = sb.mesh_regions([dict(outer=o["half"], holes=[], zb=0.0, zt=0.4)])
        self.assertLess(abs(sb.mesh_volume(tris) - abs(sb.parea(o["half"])) * 0.4) / (abs(sb.parea(o["half"])) * 0.4), 0.01)

    def test_mirror_symmetry_of_halves(self):
        for name in ("swiftA_400", "b45_400"):
            res = kit(name)
            parts = {p["name"]: p for p in res["printed"]}
            R = parts["wing_R"]["tris_raw"]
            L = parts["wing_L"]["tris_raw"]
            keyset = lambda tris: sorted({(round(p[0], 6), round(p[1], 6), round(p[2], 6)) for t in tris for p in t})
            vr = [(x, -y, z) for x, y, z in keyset(R)]
            vl = keyset(L)
            self.assertEqual(sorted((round(x, 6), round(y, 6) + 0.0, round(z, 6)) for x, y, z in vr), vl)
            self.assertLess(abs(parts["wing_R"]["volume"] - parts["wing_L"]["volume"]), 1e-6)

    def test_float32_roundtrip_deviation(self):
        res = kit("swiftA_400")
        with tempfile.TemporaryDirectory() as td:
            sb.write_kit(res, Path(td))
            pts = {p["name"]: p for p in res["printed"]}
            a = pts["wing_R"]["tris"]
            b = sb.read_stl(Path(td) / "parts/wing_R.stl")
            dev = max(abs(u - v) for ta, tb in zip(a, b) for pa, pb in zip(ta, tb) for u, v in zip(pa, pb))
            self.assertLess(dev, 0.01)


class TestJigsaw(unittest.TestCase):
    def test_key_socket_clearance_equals_setting(self):
        for name in ("swiftA_400", "swiftA_250", "b45_400"):
            res = kit(name)
            P = res["P"]
            S0 = P["S0"]
            ri = res["rail"]
            keys = res["wing"]["pieces"][0]["keys"]
            self.assertGreaterEqual(len(keys), 2)
            socks = ri["sockets_wing"] + [[(x + (ri["xh"] or 0.0), y) for x, y in s] for s in ri["sockets_tail"]]
            self.assertEqual(len(socks), len(keys))
            for k in keys:
                sk = min(socks, key=lambda s: abs(sum(q[0] for q in s) / len(s) - sum(q[0] for q in k["poly"]) / len(k["poly"])))
                sk = [(x, y - S0) for x, y in sk]
                d = sb.poly_min_dist(k["poly"], sk)
                self.assertLess(abs(d - P["key_clr"]) / P["key_clr"], 0.01, (name, d))
            self.assertAlmostEqual(ri["depth"] - P["kh"], P["key_clr"], places=9)

    def test_post_lands_in_pocket_in_flight_frame(self):
        cfg = sb.load_config({"name": "t", "span_mm": 400, "tail": {"bend_deg": 6.0}, "dihedral_deg": 3.0, "bead": {"height_root_mm": 1.0, "height_tip_mm": 0.5}, "root_strip": {"width_mm": 6}, "keys": {"height_mm": 0.6}})
        res = sb.build_kit(cfg)
        P, ri = res["P"], res["rail"]
        F = ri["frame"]
        for k in res["wing"]["pieces"][0]["keys"]:
            tail = ri["hinge"] and k["cx"] > ri["xh"]
            tip = sb.half_to_flight(P, (k["cx"], k["cy"], P["Hs"] + P["kh"]))        # post tip centre
            if tail:
                X = k["cx"] - ri["xh"]
                cols = (F["ut"], F["a"], F["nt"])
                base = (ri["xh"], 0.0, 0.0)
            else:
                X = k["cx"]
                cols = (F["ex"], F["a"], F["nw"])
                base = (0.0, 0.0, 0.0)
            S = k["cy"] + P["S0"]
            floor = tuple(base[i] + X * cols[0][i] + S * cols[1][i] - ri["depth"] * cols[2][i] for i in range(3))
            self.assertLess(abs(math.dist(tip, floor) - P["key_clr"]), 1e-9)

    def test_dihedral_and_tail_face_angles(self):
        P = dict(dihedral=3.0, bend=6.0, hinge_active=True)
        F = sb.wing_frame(P)
        self.assertAlmostEqual(math.degrees(math.atan2(F["a"][2], F["a"][1])), 3.0, places=9)
        ang = math.degrees(math.acos(sum(F["nw"][i] * F["nt"][i] for i in range(3))))
        self.assertAlmostEqual(ang, 6.0, places=9)


class TestIO(unittest.TestCase):
    def test_dxf_roundtrip(self):
        o = sb.load_outline({"type": "builtin", "builtin": "swiftA"}, 400.0)
        half = o["half"]
        full = [p for p in half] + [(x, -y) for x, y in reversed(half) if abs(y) > 1e-9]
        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "o.dxf"
            f.write_text(sb.dxf_text([("OUTLINE", half, True)]), encoding="ascii")
            o2 = sb.load_outline({"type": "dxf", "path": str(f), "kind": "half"}, 400.0)
            self.assertAlmostEqual(abs(sb.parea(o2["half"])), abs(sb.parea(half)), delta=abs(sb.parea(half)) * 1e-4)
            f2 = Path(td) / "full.dxf"
            sb_full = sb.ccw(full)
            f2.write_text(sb.dxf_text([("OUTLINE", sb_full, True)]), encoding="ascii")
            o3 = sb.load_outline({"type": "dxf", "path": str(f2)}, 400.0)
            self.assertEqual(o3["kind"], "full")
            self.assertLess(o3["asymmetry_max_mm"], 1e-3)
            self.assertLess(sb.poly_min_dist(o3["half"], half) , 1e-3)

    def test_svg_roundtrip_and_transform(self):
        o = sb.load_outline({"type": "builtin", "builtin": "b45"}, 400.0)
        half = o["half"]
        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "h.svg"
            f.write_text(sb.svg_outline_text(half), encoding="utf-8")
            o2 = sb.load_outline({"type": "svg", "path": str(f), "kind": "half"}, 400.0)
            self.assertAlmostEqual(abs(sb.parea(o2["half"])), abs(sb.parea(half)), delta=abs(sb.parea(half)) * 1e-4)
            # relative commands + transform
            svg = ('<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="100mm" viewBox="0 0 100 100"><g transform="translate(5,5)">'
                   '<path transform="scale(2)" d="m 0,0 h 10 v 10 h -10 z"/></g></svg>')
            p = sb.parse_svg(svg)
            pts = p["chains"][0][0]
            self.assertAlmostEqual(max(q[0] for q in pts) - min(q[0] for q in pts), 20.0, places=6)
            self.assertAlmostEqual(min(q[0] for q in pts), 5.0, places=6)

    def test_dxf_entities(self):
        txt = ("0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n8\n0\n90\n2\n70\n0\n10\n0\n20\n0\n42\n1.0\n10\n10\n20\n0\n"
               "0\nSPLINE\n8\n0\n70\n8\n71\n2\n72\n6\n73\n3\n40\n0\n40\n0\n40\n0\n40\n1\n40\n1\n40\n1\n10\n10\n20\n0\n10\n5\n20\n5\n10\n0\n20\n0\n"
               "0\nENDSEC\n0\nEOF\n")
        p = sb.parse_dxf(txt)
        self.assertEqual(p["counts"], {"LWPOLYLINE": 1, "SPLINE": 1})
        arc = p["chains"][0][0]
        self.assertAlmostEqual(min(q[1] for q in arc), -5.0, places=3)          # bulge +1 = CCW half circle of radius 5 below the chord, through (5, -5)
        sp = p["chains"][1][0]
        self.assertAlmostEqual(sp[0][0], 10.0, places=6)
        self.assertAlmostEqual(sp[-1][0], 0.0, places=6)
        self.assertLess(abs(sp[len(sp) // 2][1] - 2.5), 1e-6)                    # quadratic Bezier midpoint (10,0),(5,5),(0,0) -> (5, 2.5)
        loops, left, gap = sb.assemble_loops([([(0, 0), (1, 0)], False), ([(1.01, 0), (1, 1)], False), ([(1, 1), (0, 1), (0, 0.02)], False)], 0.05)
        self.assertEqual(len(loops), 1)

    def test_existing_foam_dxf_imports(self):
        f = HERE.parent / "results/runs/rigid_glider/bird_outline_v4_swift/A_foam_span400/parts.dxf"
        if not f.exists():
            self.skipTest("existing parts.dxf not present")
        o = sb.load_outline({"type": "dxf", "path": str(f), "layers": ["OUTLINE"]}, 400.0)
        ref = sb.load_outline({"type": "builtin", "builtin": "swiftA"}, 400.0)
        self.assertLess(abs(abs(sb.parea(o["half"])) - abs(sb.parea(ref["half"]))) / abs(sb.parea(ref["half"])), 1e-3)
        self.assertLess(o["asymmetry_max_mm"], 1e-3)

    def test_asymmetry_is_reported(self):
        o = sb.load_outline({"type": "builtin", "builtin": "b45"}, 400.0)
        half = o["half"]
        full = list(half) + [(x, -y) for x, y in reversed(half) if abs(y) > 1e-9]
        full = [(x + (0.8 if y < -150 else 0.0), y) for x, y in full]            # skew the left tip by 0.8 mm
        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "a.dxf"
            f.write_text(sb.dxf_text([("OUTLINE", sb.ccw(full), True)]), encoding="ascii")
            o2 = sb.load_outline({"type": "dxf", "path": str(f)}, 400.0)
            self.assertGreater(o2["asymmetry_max_mm"], 0.5)
            self.assertLess(o2["asymmetry_max_mm"], 1.0)


class TestFitAndSplit(unittest.TestCase):
    def test_swift400_half_fits(self):
        res = kit("swiftA_400")
        self.assertEqual(len(res["wing"]["pieces"]), 1)
        for p in res["printed"]:
            self.assertTrue(p["fit"]["fits"], p["name"])
            lo, hi = p["bbox_lo"], p["bbox_hi"]
            self.assertLessEqual(hi[0] - lo[0], 250.0)
            self.assertLessEqual(hi[1] - lo[1], 250.0)

    def test_all_shipped_configs_fit(self):
        for name in ("swiftA_250", "b45_400"):
            for p in kit(name)["printed"]:
                self.assertTrue(p["fit"]["fits"], (name, p["name"]))

    def test_synthetic_700_split(self):
        cfg = sb.load_config({"name": "big", "span_mm": 700, "source": {"type": "builtin", "builtin": "b45", "ar": 4.5}, "sheet": {"n_layers": 6},
                              "mass": {"target_cg_mm": 115.0, "cg_reference": "root_le"}, "rail": {"ballast_station_x_mm": -60.0}})
        res = sb.build_kit(cfg)
        self.assertFalse(res["wing"]["fit_unsplit"]["fits"])
        self.assertGreaterEqual(len(res["wing"]["pieces"]), 2)
        for p in res["printed"]:
            self.assertTrue(p["fit"]["fits"], p["name"])
            self.assertTrue(p["check"]["ok"], p["name"])
        # each cut carries matching tabs / notches: inboard piece reaches beyond the cut by the tab extent
        pcs = res["wing"]["pieces"]
        c = pcs[0]["cut_hi"]
        self.assertGreater(max(q[1] for q in pcs[0]["poly"]), c + 1.0)
        self.assertLess(min(q[1] for q in pcs[1]["poly"]), c + 1e-6)
        # no flag text about unfit pieces
        self.assertFalse([n for n in res["notes"] if "no spanwise cut" in n])
        # total volume split vs unsplit: joint strips add material, never remove it
        self.assertTrue(all(pc["volume"] > 0 for pc in pcs))

    def test_rotation_search_finds_diagonal(self):
        pts = [(0, 0), (300, 0), (300, 20), (0, 20)]
        r = sb.fit_search(pts, 250.0, 0.5)
        self.assertTrue(r["fits"])
        self.assertGreater(r["angle_deg"], 20.0)
        self.assertFalse(sb.fit_search([(0, 0), (400, 0), (400, 20), (0, 20)], 250.0, 0.5)["fits"])


class TestMass(unittest.TestCase):
    def test_budget_equation(self):
        b = sb.budget([("a", 10.0, 50.0)], [], 40.0, -10.0)
        self.assertAlmostEqual(b["ballast_g"], 10.0 * (40.0 - 50.0) / (-10.0 - 40.0) , places=9)
        cg = (10.0 * 50.0 + b["ballast_g"] * -10.0) / b["allup_g"]
        self.assertAlmostEqual(cg, 40.0, places=9)

    def test_cg_reached(self):
        for name in ("swiftA_400", "swiftA_250", "b45_400"):
            res = kit(name)
            b = res["budget"]
            cg = (sum(m * x for _, m, x in b["rows"]) + b["ballast_g"] * res["station"]) / b["allup_g"]
            self.assertAlmostEqual(cg, res["target"], places=6)
            self.assertGreater(b["allup_g"], b["M0"])
            self.assertGreaterEqual(res["win_len"], 2.0)

    def test_reproduces_options_study_within_2_percent(self):
        rows = sb.reference_check(0.02)
        if rows is None:
            self.skipTest("options.csv not present")
        for r in rows:
            self.assertTrue(r["ok"], r)
        self.assertGreaterEqual(len(rows), 10)


class TestFin(unittest.TestCase):
    def test_small_fin_flags_infeasible_tab(self):
        cfg = sb.load_config({"name": "tiny", "span_mm": 250, "tail": {"bend_deg": 6.0}, "dihedral_deg": 0.0, "sheet": {"n_layers": 2},
                              "bead": {"height_root_mm": 0.6, "height_tip_mm": 0.3}, "root_strip": {"width_mm": 4.4, "thickness_mm": 1.0},
                              "keys": {"head_r_mm": 1.0, "neck_w_mm": 0.8, "neck_len_mm": 1.0, "height_mm": 0.4}, "rail": {"window_height_mm": 4.5},
                              "fin": {"type": "centre", "height_mm": 13.1, "chord_mm": 10.5, "tab_chord_frac": 0.35}, "mass": {"target_cg_mm": 45.6}})
        res = sb.build_kit(cfg)
        self.assertTrue([n for n in res["notes"] if "trim tab cannot be made" in n])
        self.assertFalse(res["fin_geo"]["tab_feasible"])
        self.assertFalse([k for k in res["fin_parts"] if k.startswith("fin_tab")])

    def test_b45_tab_family(self):
        res = kit("b45_400")
        names = {p["name"] for p in res["printed"]}
        self.assertTrue({"fin_tab_0", "fin_tab_2", "fin_tab_4", "fin_tip_R", "fin_tip_L"} <= names)
        self.assertTrue(res["fin_geo"]["tab_feasible"])
        # tilt: tab_4 stands 4 deg proud of the bed at its far end
        p = {p["name"]: p for p in res["printed"]}
        z = p["fin_tab_4"]["bbox_hi"][2] - p["fin_tab_4"]["bbox_lo"][2]
        L = res["fin_geo"]["tilt_len"]
        self.assertAlmostEqual(z, res["fin_geo"]["tf"] + L * math.tan(math.radians(4.0)) * 1.0, delta=0.15)


class TestRail(unittest.TestCase):
    def test_dihedral_config_manifold_and_gap(self):
        cfg = sb.load_config({"name": "d3", "span_mm": 400, "tail": {"bend_deg": 6.0}, "dihedral_deg": 3.0, "bead": {"height_root_mm": 1.0, "height_tip_mm": 0.5},
                              "root_strip": {"width_mm": 6}, "keys": {"height_mm": 0.6}, "fin": {"type": "centre", "chord_mm": 16.76, "height_mm": 20.95, "tab_chord_frac": 0.5},
                              "mass": {"target_cg_mm": 73.0}})
        res = sb.build_kit(cfg)
        self.assertGreater(res["P"]["root_gap"], 0.4)
        for p in res["printed"]:
            self.assertTrue(p["check"]["ok"], p["name"])
        self.assertTrue(res["fin_geo"]["tab_feasible"])

    def test_rail_union_volume_below_sum_of_shells(self):
        ri = kit("swiftA_400")["rail"]
        self.assertLess(ri["volume_mm3"], sum(ri["shell_volumes"].values()))
        self.assertGreater(ri["overlap_mm3"], 0.0)

    def test_window_holds_the_lead(self):
        for name in ("swiftA_400", "b45_400"):
            res = kit(name)
            w = res["rail"]["window"]
            self.assertGreaterEqual(w["area"] * w["thickness"], res["budget"]["lead_volume_mm3"] * (1 + float(res["cfg"]["ballast"]["fill_margin"])) - 1.0)


class TestCli(unittest.TestCase):
    def test_refuses_to_overwrite(self):
        with tempfile.TemporaryDirectory() as td:
            (Path(td) / "x.txt").write_text("keep")
            self.assertEqual(sb.main([str(CFG / "swiftA_250.json"), "--out", td]), 2)
            self.assertEqual((Path(td) / "x.txt").read_text(), "keep")


if __name__ == "__main__":
    unittest.main(verbosity=1)
