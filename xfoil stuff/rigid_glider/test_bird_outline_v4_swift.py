"""Tests for bird_outline_v4_swift.py.   py -B -m unittest test_bird_outline_v4_swift -v   (from rigid_glider/; temporary folders; two short serial AVL evaluations)"""
import sys
sys.dont_write_bytecode = True
import csv
import math
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import bird_outline as bo
import bird_outline_v2 as v2
import bird_outline_v4_swift as v4
import flat_test as ft


def build(**kw):
    return v4.build_geometry4(dict(v4.GLIDE, **kw))


class GeometryTests(unittest.TestCase):
    def test_area_span_aspect_ratio(self):
        for p in (v4.GLIDE, v4.EXTENDED):
            g = v4.build_geometry4(p)
            self.assertAlmostEqual(v4.pair_ar(g), p["ar_target"], delta=0.01 * p["ar_target"])                 # the two wings ALONE (body strip excluded) have the data file's aspect ratio
            self.assertGreater(g["ar"], v4.pair_ar(g))                                                         # b^2/S with the body strip is higher
            self.assertAlmostEqual(g["ar"], 1.0 / g["S"], places=9)
            self.assertAlmostEqual(g["S"], g["S_polygon"], places=9)
            self.assertAlmostEqual(g["y_nodes"][-1], 0.5, places=12)                          # span 1
            xs = [x for x, y in g["full_outline"]]
            ys = [y for x, y in g["full_outline"]]
            self.assertAlmostEqual(max(ys) - min(ys), 1.0, places=9)

    def test_chord_law_reproduces_the_measured_fractions(self):
        gA, gC = v4.build_geometry4(v4.GLIDE), v4.build_geometry4(v4.EXTENDED)
        for e, f in v4.MEASURED_30:
            self.assertAlmostEqual(v4.chord_over_mean(gA, e), f, delta=0.06 * f + 0.02, msg=("30 deg", e))        # digitising error is 5-10 %
        for e, f in v4.MEASURED_EXT:
            if e < 0.95:                                                                                         # the last point is cut off by the minimum buildable chord (JUDGEMENT), a stated departure
                self.assertAlmostEqual(v4.chord_over_mean(gC, e), f, delta=0.06 * f + 0.02, msg=("extended", e))
        self.assertLess(v4.rms_vs(v4.MEASURED_30, lambda e: v4.chord_over_mean(gA, e)), 0.045)
        self.assertLess(v4.rms_vs(v4.MEASURED_EXT, lambda e: v4.chord_over_mean(gC, e)), 0.045)

    def test_minimum_chord_is_respected(self):
        g = build(min_chord=0.05)
        for y in g["y_nodes"]:
            self.assertGreaterEqual(g["chord_fn"](y), 0.05 - 1e-12)
        self.assertAlmostEqual(g["chord_fn"](0.5) , g["te_nodes"][-1] - g["le_nodes"][-1], places=12)

    def test_leading_edge_angles_follow_the_table(self):
        g = v4.build_geometry4(v4.GLIDE)
        yb, s = g["y_body"], g["half_span"]
        for eta, ang in ((0.6, 37.0), (0.7, 37.0), (0.05, 11.0)):
            h = 1e-3
            y = yb + eta * (s - yb)
            slope = (g["le_fn"](y + h) - g["le_fn"](y - h)) / (2 * h)
            self.assertAlmostEqual(math.degrees(math.atan(slope)), v4.le_angle_deg(eta, v4.GLIDE), delta=0.3)
        self.assertAlmostEqual(v4.le_angle_deg(0.6, v4.GLIDE), 37.0 + 0.0, delta=1e-9)
        self.assertAlmostEqual(v4.le_angle_deg(0.05, v4.EXTENDED), -9.0, delta=1e-9)             # extended: arm leading edge forward
        self.assertLess(v4.build_geometry4(v4.EXTENDED)["le_nodes"][3], 0.0)

    def test_each_degree_of_freedom_changes_only_its_own_geometry(self):
        base = v4.build_geometry4(v4.GLIDE)
        tail_same = lambda g: all(abs(g[k] - base[k]) < 1e-12 for k in ("Lt", "St", "wt0"))
        g = build(hinge_gap=0.03)
        self.assertAlmostEqual(g["x_h"] - base["x_h"], 0.03, places=9)
        self.assertEqual(g["y_nodes"], base["y_nodes"])
        self.assertTrue(tail_same(g))
        g = build(nose=0.15)
        self.assertAlmostEqual(g["nose_x"], -0.15)
        self.assertEqual(g["le_nodes"], base["le_nodes"])
        self.assertAlmostEqual(g["tail_area"], base["tail_area"], places=12)
        self.assertGreater(g["sheet_area"], base["sheet_area"])
        g = build(tail_spread=0.30)
        self.assertEqual(g["te_nodes"], base["te_nodes"])                                      # the tail never changes the wing
        self.assertGreater(g["tail_area"], base["tail_area"])
        g = build(wrist_eta=0.30)
        self.assertGreater(g["y_wrist"], base["y_wrist"])
        self.assertAlmostEqual(g["S"], base["S"], places=6)                                    # the wrist moves the sweep break and the fold line, not the area
        g = build(le_hand_knots=v4.hand_shift(v4.GLIDE, 10)["le_hand_knots"])
        self.assertGreater(g["le_nodes"][-1], base["le_nodes"][-1])
        self.assertTrue(tail_same(g))
        g = build(tail_len=v4.GLIDE["tail_len"] * 1.4)
        self.assertEqual(g["y_nodes"], base["y_nodes"])
        self.assertAlmostEqual(g["Lt"], 1.4 * base["Lt"], places=12)

    def test_tail_area_is_the_birds_fraction(self):
        for p in (v4.GLIDE, v4.EXTENDED):
            g = v4.build_geometry4(p)
            self.assertAlmostEqual(g["tail_area"] / g["S"], v4.TAIL_AREA_FRAC_BIRD, places=4)

    def test_outlines_are_simple_clean_and_ccw(self):
        for p in (v4.GLIDE, v4.EXTENDED, v4.hand_shift(v4.GLIDE, 8), dict(v4.GLIDE, wrist_eta=0.4), dict(v4.GLIDE, hinge_gap=0.05, nose=0.2)):
            g = v4.build_geometry4(p)
            poly = g["full_outline"]
            self.assertTrue(v2.polygon_is_simple(poly))
            self.assertGreater(ft.area_signed(poly), 0)
            self.assertEqual(v2.simplify(poly), poly)
            ys = g["y_nodes"]
            self.assertTrue(all(ys[i + 1] - ys[i] > 0 for i in range(len(ys) - 1)))
            for i in range(len(poly)):
                self.assertGreater(math.dist(poly[i], poly[(i + 1) % len(poly)]), 1e-6)

    def test_body_nose_and_tail_areas_are_reported_separately(self):
        g = v4.build_geometry4(v4.GLIDE)
        a = v4.area_parts(g)
        self.assertAlmostEqual(a["strip"] + a["arm"] + a["hand"], g["S"], delta=1e-3 * g["S"])                       # the nodes are piecewise linear, the smooth chord law is not: sub-0.1 % apart
        self.assertAlmostEqual(a["S"] + a["tail"] + a["body_extra"], a["sheet"], places=9)
        self.assertGreater(a["body_extra"], 0)

    def test_data_tags_are_present_for_every_parameter(self):
        for k in ("ar_target", "body_half", "wrist_eta", "chord_knots", "min_chord", "le_arm_deg", "le_hand_knots", "tail_len", "tail_spread", "tail_neck", "hinge_gap", "nose"):
            self.assertIn(k, v4.PARAM_SOURCE)
        self.assertEqual(v4.PARAM_SOURCE["min_chord"][0], "JUDGEMENT")
        self.assertEqual(v4.PARAM_SOURCE["wrist_eta"][0], "ESTIMATE")


class AvlAndMassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="v4_")
        cls.work = Path(cls.tmp.name)
        cls.o = v4.evaluate4("T_A", v4.GLIDE, cls.work, 1.288)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_deck_is_deterministic_and_sane(self):
        g = v4.build_geometry4(v4.GLIDE)
        v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
        d1, d2 = v2.avl_deck2(g, 250.0, 5.0), v2.avl_deck2(g, 250.0, 5.0)
        self.assertEqual(d1, d2)
        wing = d1[:d1.index("SURFACE\nTail")]
        secs = [l.split() for l in wing.splitlines() if re.match(r"^[-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+$", l)]
        self.assertEqual(len(secs), g["n_nodes"])
        ys = [float(r[1]) for r in secs]
        self.assertEqual(ys[0], 0.0)
        self.assertTrue(all(b > a for a, b in zip(ys, ys[1:])))
        self.assertTrue(all(float(r[3]) > 0 for r in secs))
        tail = d1[d1.index("SURFACE\nTail"):]
        incs = [float(l.split()[4]) for l in tail.splitlines() if re.match(r"^[-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+$", l)]
        self.assertTrue(incs and all(abs(i + 5.0) < 1e-9 for i in incs))                         # incidence = minus the bend
        self.assertAlmostEqual(float(d1.splitlines()[3].split()[0]), g["S"] * 0.25 ** 2, places=8)         # Sref

    def test_no_bend_no_moment_and_bend_adds_nose_up(self):
        r0 = self.o["r5"]
        self.assertGreater(r0["cm0"], 0.01)
        g = v4.build_geometry4(v4.GLIDE)
        v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
        z = v2.run_case2(g, 250.0, 0.0, self.work / "z", "z")
        self.assertLess(abs(z["cm0"]), 1e-3)
        self.assertGreater(self.o["np_pct"], 20.0)
        self.assertLess(self.o["np_pct"], 45.0)
        self.assertAlmostEqual(self.o["cm0_per_deg"], r0["cm0"] / 5.0)

    def test_bend_and_cg_bookkeeping(self):
        o = self.o
        mac = o["mac"]
        # margin s: CG = NP - s*MAC, bend = s*0.3/cm0_per_deg
        self.assertAlmostEqual(o["cg_mm_10"], (o["np_span"] - 0.10 * mac) * 250.0, places=9)
        self.assertAlmostEqual(o["bend_10"] * o["cm0_per_deg"] / 0.3, 0.10, places=9)
        self.assertAlmostEqual(o["bend_8"] * o["cm0_per_deg"] / 0.3, 0.08, places=9)
        self.assertGreater(o["bend_10"], o["bend_8"])

    def test_ballast_puts_the_cg_on_target_and_masses_add_up(self):
        g = o_g = self.o["g"]
        res = v4.fake_res(self.o["np_span"] - 0.10 * self.o["mac"], 0.10)
        for sk in ("printed", "foam"):
            n = v2.size_numbers(g, res, sk, 6.0)
            if n["reachable"]:
                cg = (n["M0"] * n["cg0"] + n["ballast_g"] * n["station"]) / (n["M0"] + n["ballast_g"])
                self.assertAlmostEqual(cg, n["cg_target"], places=6)
            self.assertAlmostEqual(n["M"], n["m_sheet"] + n["rail"]["mass_g"] + n["ballast_g"], places=9)
            self.assertAlmostEqual(n["ballast_frac"], n["ballast_g"] / n["M"], places=9)
            self.assertAlmostEqual(self.o[f"ballast_frac_{sk}_10"], n["ballast_frac"], places=9)

    def test_criteria_rows_and_logic(self):
        rows = v4.criteria4(self.o, v4.EXISTING)
        names = [r[0] for r in rows]
        for needle in ("static margin", "nose weight", "Cn_beta", "Cl_beta", "tip loading", "tail neck", "trim CG"):
            self.assertTrue(any(needle in n for n in names), needle)
        o = dict(self.o)
        o["cn_beta"] = 0.0035
        self.assertFalse(v4.passes(o, v4.EXISTING))
        self.assertTrue(dict((r[0], r[3]) for r in v4.criteria4(o, v4.PROPOSED))["Cn_beta with no fin, per rad"])       # the proposed floor 0.002 accepts it
        o["cn_beta"] = 0.0045
        o["ballast_frac_printed_8"] = 0.22
        self.assertFalse(v4.passes(o, v4.EXISTING))
        o["ballast_frac_printed_8"] = float("inf")
        self.assertFalse(v4.passes(o, v4.PROPOSED))

    def test_build_files_rail_stl_dxf_and_templates(self):
        out = Path(self.tmp.name) / "build"
        info = v4.build_files(self.o, 6.0, out, "T", make_pdf=True)
        g = self.o["g"]
        for sk, k in (("printed", 250), ("foam", 400)):
            d = out / f"T_{sk}_span{k}"
            for nm in ("rail.stl", "parts.dxf", "drawing.svg", "template_page1.svg"):
                self.assertTrue((d / nm).is_file(), (sk, nm))
            self.assertEqual((d / "sheet.stl").is_file(), sk == "printed")
            tris = ft.read_stl(d / "rail.stl")
            self.assertEqual(ft.edge_report(tris, nd=4)[1:], (0, 0, 0))
            b = next(x for x in info["builds"] if x["size"] == sk)
            rail = b["n"]["rail"]
            self.assertLessEqual(rail["saddle_end"], g["x_h"] * k - v2.RAIL_HINGE_GAP_MM + 1e-9)                # saddle ends ahead of the hinge
            cgs = b["n"]["cgs"]
            self.assertLessEqual(rail["tick_range"][0], min(cgs))
            self.assertGreaterEqual(rail["tick_range"][1], max(cgs))
            dxf = (d / "parts.dxf").read_text()
            for layer in ("OUTLINE", "HINGE_BEND_UP", "WRIST_MARKER", "BODY_EDGE_MARKER", "CENTRELINE", "CG_MARKS"):
                self.assertIn(layer, dxf)
            ET.fromstring((d / "drawing.svg").read_text(encoding="utf-8"))
            self.assertIn("body edge", (d / "drawing.svg").read_text(encoding="utf-8"))
        shells = ft.read_stl(out / "T_printed_span250" / "sheet.stl")
        self.assertGreater(len(shells), 100)
        sheet_g = v2.sheet_mass_g(g, "printed", exact=True)
        self.assertAlmostEqual(sheet_g, v2.sheet_mass_g(g, "printed", exact=False), delta=0.04 * sheet_g)
        pages = info["pages"]
        self.assertGreaterEqual(len(pages), 4)
        for p in pages:
            txt = p.read_text(encoding="utf-8")
            root = ET.fromstring(txt)
            self.assertTrue({el.tag.split("}")[-1] for el in root} <= {"rect", "line", "polygon", "text"})
            m = re.search(r'<line id="scalebar" x1="([-0-9.]+)" y1="([-0-9.]+)" x2="([-0-9.]+)" y2="([-0-9.]+)"', txt)
            self.assertIsNotNone(m)
            x1, _, x2, _ = map(float, m.groups())
            self.assertAlmostEqual(x2 - x1, 100.0, places=9)
            boxes = []
            for el in root:
                if el.tag.endswith("text"):
                    sz = float(el.get("font-size"))
                    x, y = float(el.get("x")), float(el.get("y"))
                    w = 0.5 * sz * len(el.text or "")
                    a = el.get("text-anchor", "start")
                    x0 = x if a == "start" else (x - w if a == "end" else x - w / 2)
                    boxes.append((x0, y - 0.8 * sz, x0 + w, y + 0.2 * sz, el.text))
            for (a0, b0, a1, b1, s) in boxes:
                self.assertGreaterEqual(a0, ft.TEMPLATE_MARGIN - 0.01, s)
                self.assertLessEqual(a1, ft.LETTER_W - ft.TEMPLATE_MARGIN + 0.01, s)
            for i in range(len(boxes)):
                for j in range(i + 1, len(boxes)):
                    A_, B_ = boxes[i], boxes[j]
                    self.assertFalse(A_[0] < B_[2] and B_[0] < A_[2] and A_[1] < B_[3] and B_[1] < A_[3], (p.name, A_[4], B_[4]))
        self.assertTrue(any("#7a3fb0" in p.read_text(encoding="utf-8") for p in pages))                           # the body-edge fold line is on the templates
        pdf = out / info["pdf"]
        self.assertTrue(pdf.is_file())
        self.assertGreaterEqual(pdf.read_bytes().count(b"/Type /Page") - pdf.read_bytes().count(b"/Type /Pages"), len(pages))


class OutputPageTests(unittest.TestCase):
    """Reads the finished output folder (skipped if the study has not been run)."""
    OUT = v4.OUT_DEFAULT

    def setUp(self):
        if not (self.OUT / "index.html").is_file():
            self.skipTest("output folder not generated")

    def rows(self, name):
        with (self.OUT / name).open(encoding="utf-8") as f:
            return list(csv.DictReader(f))

    def test_csvs_and_counts(self):
        self.assertEqual(len(self.rows("outlines.csv")), 3)
        crit = self.rows("criteria.csv")
        self.assertEqual({r["outline"] for r in crit}, {"A", "B", "C"})
        self.assertEqual(len({r["threshold set"] for r in crit}), 2)
        self.assertGreaterEqual(len(self.rows("wrist_sensitivity.csv")), 8)
        self.assertGreaterEqual(len(self.rows("panelling.csv")), 6)

    def test_page_is_dark_tagged_and_honest(self):
        page = (self.OUT / "index.html").read_text(encoding="utf-8")
        low = page.lower()
        self.assertIn("#14181d", page)
        for needle in ("digitised", "kai's decision", "proposed", "wrist", "body edge", "5% mac", "8-16%"):
            self.assertIn(needle, low, needle)
        for banned in ("will fly", "will pull out", "glide ratio of", "predicted cg window"):
            self.assertNotIn(banned, low, banned)
        for tag in ("DIGITISED", "CALCULATED", "MODEL", "JUDGEMENT", "ESTIMATE"):
            self.assertIn(tag, page)

    def test_two_candidates_not_a_recommendation(self):
        page = (self.OUT / "index.html").read_text(encoding="utf-8")
        low = page.lower()
        self.assertNotIn("recommendation (for the reviewer", low)
        self.assertNotIn("(recommended)", low)
        for needle in ("two candidates", "hybrid", "no evidence", "fallback", "first 5%, which gives +10", "pull-out", "10% static margin"):
            self.assertIn(needle.lower(), low, needle)
        for pdf in ("A_templates.pdf", "B_templates.pdf"):
            self.assertTrue((self.OUT / pdf).is_file(), pdf)
        for nm in ("A", "B"):
            for sk, k in (("printed", 250), ("foam", 400)):
                d = self.OUT / f"{nm}_{sk}_span{k}"
                for f in ("rail.stl", "parts.dxf", "drawing.svg", "template_page1.svg"):
                    self.assertTrue((d / f).is_file(), (d.name, f))
        hdr = self.rows("outlines.csv")[0]
        self.assertIn("aspect ratio of the two wings alone (body strip excluded)", hdr)
        for r in self.rows("outlines.csv"):
            want = 10.1 if r["outline"].startswith("C") else 8.5
            if not r["outline"].startswith("B"):
                self.assertAlmostEqual(float(r["aspect ratio of the two wings alone (body strip excluded)"]), want, delta=0.01 * want)

    def test_default_output_folder(self):
        self.assertEqual(v4.OUT_DEFAULT.name, "bird_outline_v4_swift")
        self.assertNotEqual(v4.OUT_DEFAULT, v2.OUT_DEFAULT)


if __name__ == "__main__":
    unittest.main()
