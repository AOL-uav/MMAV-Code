"""Tests for known_good_zanonia.py.   py -B -m unittest test_known_good_zanonia -v   (from rigid_glider/; reads the saved trace and AVL results, no AVL runs)"""
import sys
sys.dont_write_bytecode = True
import json
import math
import re
import tempfile
import unittest
from pathlib import Path

import known_good_zanonia as kz

RES = kz.RES
OUT = kz.OUT


class Basics(unittest.TestCase):
    def test_rdp_keeps_endpoints_and_a_straight_line(self):
        pts = [(0, 0), (1, 0.001), (2, 0), (3, 0.0005), (4, 0)]
        self.assertEqual(kz.rdp(pts, 0.01), [(0, 0), (4, 0)])

    def test_ear_clip_with_a_hole_has_the_right_area(self):
        outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
        hole = kz.circle_pts(5, 5, 2, 32, cw=True)
        merged = kz.bridge_hole(outer, hole)
        area = sum(kz._cross(merged[i], merged[j], merged[k]) / 2 for i, j, k in kz.ear_clip(merged))
        self.assertAlmostEqual(area, 100 - 0.5 * 32 * 4 * math.sin(2 * math.pi / 32), places=6)

    def test_ascii_stl_round_trip_and_box_is_closed(self):
        tris = kz.ft.prism([(0, 0), (2, 0), (2, 1), (0, 1)], 0.0, 0.5)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.stl"
            kz.write_stl_ascii(p, tris, "t")
            self.assertTrue(p.read_text().startswith("solid t"))
            back = kz.read_stl_ascii(p)
        self.assertEqual(len(back), len(tris))
        chk = kz.mesh_ok(back)
        self.assertEqual(chk["edges_not_shared_by_two"], 0)
        self.assertAlmostEqual(chk["volume_mm3"], 1.0, places=6)

    def test_digit_mapping_is_counter_clockwise(self):
        tris = kz.digit_solids("8", 10.0, 0.0, 0.0, 5.0, 9.0, 1.3, 8.0, 8.8)
        self.assertGreater(kz.ft.mesh_stats(tris)["volume"], 0)


@unittest.skipUnless((RES / "zanonia_geometry.json").is_file(), "stage 1 not run")
class Trace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geo = json.loads((RES / "zanonia_geometry.json").read_text())
        cls.tab = kz.load_outline()

    def test_span_is_about_six_inches_and_dpi_is_300(self):
        self.assertAlmostEqual(self.geo["span_in"], 6.0, delta=0.15)
        self.assertEqual(self.geo["dpi"], 300.0)

    def test_outline_is_a_proper_wing(self):
        self.assertTrue(all(t[2] > t[1] for t in self.tab))
        self.assertTrue(all(b[0] > a[0] for a, b in zip(self.tab, self.tab[1:])))
        self.assertAlmostEqual(self.tab[0][1], 0.0, delta=0.01)
        self.assertLess(self.geo["asym_rms_in"], 0.03)

    def test_area_matches_the_table(self):
        s = sum(((a[2] - a[1]) + (b[2] - b[1])) / 2 * (b[0] - a[0]) for a, b in zip(self.tab, self.tab[1:])) * 2
        self.assertAlmostEqual(s, self.geo["area_in2"], delta=0.02 * self.geo["area_in2"])

    def test_marks_are_on_the_centre_line_and_in_the_expected_places(self):
        self.assertLess(abs(self.geo["cg_y_in"]), 0.03)
        self.assertGreater(self.geo["cg_x_in"], self.geo["hole_x_in"])
        self.assertAlmostEqual(self.geo["cg_x_in"], 0.68, delta=0.05)
        self.assertGreater(self.geo["cg_pct_mac"], 15)
        self.assertLess(self.geo["cg_pct_mac"], 30)


@unittest.skipUnless((RES / "avl_results.json").is_file(), "stage 2 not run")
class AvlRule(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ar = json.loads((RES / "avl_results.json").read_text())

    def test_sweep_is_complete(self):
        self.assertEqual(len(self.ar["results"]), 16)
        self.assertEqual(sum(1 for r in self.ar["results"] if r["theta"] == 0), 1)

    def test_cm0_rises_with_the_curl_angle(self):
        for f in kz.REGIONS:
            c = [r["Cm0"] for r in self.ar["results"] if r["region"] == f and r["theta"] > 0]
            self.assertTrue(all(b > a for a, b in zip(c, c[1:])), (f, c))
            self.assertGreater(c[0], 0)

    def test_verdict_follows_the_pre_written_rule(self):
        v, lines, rec = kz.evaluate(self.ar["results"])
        self.assertEqual(v, self.ar["verdict"])
        flat = next(r for r in self.ar["results"] if r["theta"] == 0)
        self.assertEqual(v == "AGREE", kz.SM_LO <= flat["SM"] <= kz.SM_HI and rec is not None)

    def test_static_margin_is_the_neutral_point_minus_cg_over_mac(self):
        r = self.ar["results"][0]
        geo = json.loads((RES / "zanonia_geometry.json").read_text())
        self.assertAlmostEqual(r["SM"], (r["Xnp_m"] - geo["cg_x_in"] * 0.0254) / (geo["mac_in"] * 0.0254), places=9)

    def test_readme_holds_the_rule_before_the_results(self):
        txt = (RES / "README.md").read_text(encoding="utf-8")
        self.assertIn("Pass rule (fixed before the AVL run)", txt)
        self.assertIn("3% and 25% of MAC", txt)


@unittest.skipUnless((OUT / "numbers.json").is_file(), "stage 3 not run")
class Print(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geo = json.loads((RES / "zanonia_geometry.json").read_text())
        cls.tab = kz.load_outline()
        cls.num = json.loads((OUT / "numbers.json").read_text())

    def test_plates_are_watertight_with_the_right_volume(self):
        for sp, hd in ((155.0, 4.0), (250.0, 6.0)):
            sc = kz.Scaled(self.tab, self.geo, sp)
            tris = kz.read_stl_ascii(OUT / "print" / f"zanonia_plate_{int(sp)}mm.stl")
            chk = kz.mesh_ok(tris)
            self.assertEqual(chk["edges_not_shared_by_two"], 0)
            self.assertEqual(chk["degenerate"], 0)
            z = [p[2] for t in tris for p in t]
            self.assertAlmostEqual(min(z), 0.0)
            self.assertAlmostEqual(max(z), kz.PLATE_T_MM + 0.2)
            hole = 0.5 * 32 * (hd / 2) ** 2 * math.sin(2 * math.pi / 32)
            plate = kz.plate_mesh(sc, kz.PLATE_T_MM, 8.0, 0.9, 0.2, hd)[0]
            self.assertAlmostEqual(kz.mesh_ok(plate)["volume_mm3"], (sc.area - hole) * kz.PLATE_T_MM, delta=0.002 * sc.area * kz.PLATE_T_MM)

    def test_span_of_the_plate_is_the_requested_span(self):
        for sp in (155.0, 250.0):
            tris = kz.read_stl_ascii(OUT / "print" / f"zanonia_plate_{int(sp)}mm.stl")
            ys = [p[1] for t in tris for p in t]
            self.assertAlmostEqual(max(ys) - min(ys), sp, delta=0.2)

    def test_jig_block_is_watertight_and_slopes_by_the_printed_angle(self):
        sc = kz.Scaled(self.tab, self.geo, 155.0)
        block, info = kz.jig_mesh(sc, 6.0, 0.3, "6", digits=False)
        chk = kz.mesh_ok(block)
        self.assertEqual(chk["edges_not_shared_by_two"], 0)
        self.assertGreater(chk["volume_mm3"], 0)
        z = {}
        for t in block:
            for p in t:
                key = (round(p[0], 4), round(p[1], 4))
                z[key] = max(z.get(key, -1), p[2])
        y = max(k[1] for k in z if k[1] <= sc.half)
        col = sorted((k[0], v) for k, v in z.items() if k[1] == y)
        xl = sc.xle(y)
        pts = [(x, v) for x, v in col if xl + 1 < x < xl + 6]
        self.assertGreater(len(pts), 2)
        slope = (pts[0][1] - pts[-1][1]) / (pts[-1][0] - pts[0][0])
        self.assertAlmostEqual(math.degrees(math.atan(slope)), 6.0, delta=0.2)

    def test_nose_weight_puts_the_cg_on_the_mark(self):
        sc = kz.Scaled(self.tab, self.geo, 155.0)
        tris = kz.read_stl_ascii(OUT / "print" / "zanonia_plate_155mm.stl")
        z = [t for t in tris if max(p[2] for p in t) <= kz.PLATE_T_MM + 1e-9]
        st = kz.ft.mesh_stats(z)
        row = kz.mass_rows(sc, st["volume"], st["centroid"][0])[0]
        cg = (row["plate_g"] * st["centroid"][0] + row["nose_g"] * sc.hole_x) / row["total_g"]
        self.assertAlmostEqual(cg, sc.cg_x, places=6)
        self.assertAlmostEqual(row["plate_g"], st["volume"] * 1e-3 * kz.RHO_PLA, places=9)

    def test_scale_up_is_scale_free_in_loading(self):
        s = self.num["scale"]
        self.assertAlmostEqual(s[0]["loading"], s[1]["loading"], places=6)
        self.assertAlmostEqual(s[1]["cg_mm"] / s[1]["span"], s[2]["cg_mm"] / s[2]["span"], places=9)

    def test_templates_have_a_100_mm_scale_bar(self):
        for i in (1, 2):
            t = (OUT / f"template_page{i}.svg").read_text(encoding="utf-8")
            m = re.search(r'<line id="scalebar" x1="([-0-9.]+)" y1="[-0-9.]+" x2="([-0-9.]+)"', t)
            self.assertAlmostEqual(float(m.group(2)) - float(m.group(1)), 100.0, places=9)
        self.assertTrue((OUT / "templates.pdf").is_file())

    def test_page_is_dark_and_tagged(self):
        t = (OUT / "index.html").read_text(encoding="utf-8")
        self.assertIn("color-scheme:dark", t)
        for w in ("CITED", "MEASURED-FROM-SCAN", "ASSUMED", "JUDGEMENT", "MODEL", "DISAGREE"):
            self.assertIn(w, t)


if __name__ == "__main__":
    unittest.main()
