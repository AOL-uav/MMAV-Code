"""Unit tests for bird_outline.py.  Run from rigid_glider/:  py -B -m unittest test_bird_outline -v   (writes to a temporary folder only; AVL runs serially)"""
import sys
sys.dont_write_bytecode = True
import csv
import hashlib
import json
import math
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import bird_outline as bo
import flat_test as ft


def rect_params(ar=6.0):
    p = {k: v[0] for k, v in bo.FAMILIES["BS"]["params"].items()}
    p.update(ar_target=ar, arm_sweep_deg=0.0, hand_sweep_deg=0.0, cw_over_croot=1.0, hand_stations=((0.0, 1.0), (1.0, 1.0)), tail_len_frac=0.05, tail_spread_frac=0.1,
             tail_root_frac=0.03, body_halfwidth_frac=0.01)
    return p


class GeometryTests(unittest.TestCase):
    def test_rectangle_hand_case(self):
        g = bo.build_geometry(rect_params(6.0))
        self.assertAlmostEqual(g["c_root"], 1 / 6, places=9)
        self.assertAlmostEqual(g["S"], 1 / 6, places=9)                    # b * c
        self.assertAlmostEqual(g["S_polygon"], g["S"], places=12)           # polygon arithmetic and the chord integral agree
        self.assertAlmostEqual(g["mac"], 1 / 6, places=9)
        self.assertAlmostEqual(g["x_le_mac"], 0.0, places=12)
        self.assertAlmostEqual(g["ar"], 6.0, places=9)
        self.assertAlmostEqual(g["x_ref"], 0.25 / 6, places=9)

    def test_polygon_area_and_mac_against_numeric_chord(self):
        for fam in bo.FAM_ORDER:
            g = bo.family_geometry(fam)
            self.assertAlmostEqual(g["S"], g["S_polygon"], places=10)
            self.assertAlmostEqual(g["ar"], bo.pv(fam, "ar_target"), places=6)
            ys, les, tes = g["y_nodes"], g["le_nodes"], g["te_nodes"]
            n = 20000
            h = g["half_span"] / n
            num = den = xl = 0.0
            for i in range(n):
                y = (i + 0.5) * h
                j = max(k for k in range(len(ys)) if ys[k] <= y)
                j = min(j, len(ys) - 2)
                t = (y - ys[j]) / (ys[j + 1] - ys[j])
                le = les[j] + t * (les[j + 1] - les[j])
                c = (tes[j] + t * (tes[j + 1] - tes[j])) - le
                num += c * c * h
                den += c * h
                xl += le * c * h
            self.assertAlmostEqual(g["mac"], num / den, places=5)
            self.assertAlmostEqual(g["x_le_mac"], xl / den, places=5)

    def test_outline_is_a_closed_simple_ccw_polygon_with_tail(self):
        for fam in bo.FAM_ORDER:
            g = bo.family_geometry(fam)
            full = g["full_outline"]
            self.assertGreater(ft.area_signed(full), 0)
            self.assertAlmostEqual(abs(ft.area_signed(full)), g["sheet_area"])
            self.assertGreater(g["sheet_area"], g["S"])                       # tail and nose add area
            self.assertLess(g["tail_area"] + g["S"], g["sheet_area"] * 1.0 + 1e-12 + g["sheet_area"])   # tail + wing areas are parts of the sheet
            self.assertAlmostEqual(max(x for x, y in full), max(x for x, y in g["half_outline"]))
            # mirror symmetry
            pts = {(round(x, 9), round(y, 9)) for x, y in full}
            self.assertTrue(all((x, -y) in pts for x, y in pts))
            # hinge line is straight at x = c_root across the tail root
            self.assertAlmostEqual(g["x_h"], g["c_root"])
            self.assertIn((round(g["x_h"], 9), round(g["wt0"] / 2, 9)), {(round(x, 9), round(y, 9)) for x, y in g["half_outline"]})

    def test_family_provenance_tags(self):
        for fam in bo.FAM_ORDER:
            for name, (val, src, note) in bo.FAMILIES[fam]["params"].items():
                self.assertIn(src, bo.SOURCES, (fam, name))
                self.assertTrue(note)
        cited = [(fam, n) for fam in bo.FAM_ORDER for n, v in bo.FAMILIES[fam]["params"].items() if v[1] == "CITED"]
        self.assertEqual(sorted(cited), [("BC", "ar_target"), ("BS", "ar_target"), ("BS", "tail_spread_frac")])
        self.assertAlmostEqual(bo.pv("BS", "tail_spread_frac"), 0.09 / 0.39, places=2)           # cited swift tail span over wing span


class AvlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="bird_avl_")
        cls.dir = Path(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_avl_input_regenerates_identically(self):
        a = bo.avl_deck(bo.family_geometry("BS"), 250.0, 10.0)
        b = bo.avl_deck(bo.family_geometry("BS"), 250.0, 10.0)
        self.assertEqual(a, b)
        self.assertEqual(hashlib.sha256(a.encode()).hexdigest(), hashlib.sha256(b.encode()).hexdigest())
        self.assertNotEqual(a, bo.avl_deck(bo.family_geometry("BS"), 250.0, 5.0))

    def test_tail_incidence_is_minus_the_bend(self):
        g = bo.family_geometry("BC")
        deck = bo.avl_deck(g, 250.0, 10.0)
        tail = deck[deck.index("SURFACE\nTail"):]
        incs = [float(l.split()[4]) for l in tail.splitlines() if re.match(r"^[-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+$", l)]
        self.assertTrue(incs)
        self.assertTrue(all(abs(i + 10.0) < 1e-9 for i in incs))
        wing = deck[:deck.index("SURFACE\nTail")]
        self.assertTrue(all(float(l.split()[4]) == 0.0 for l in wing.splitlines() if re.match(r"^[-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+ [-0-9.]+$", l)))
        self.assertNotIn("Tail", bo.avl_deck(g, 250.0, 10.0, with_tail=False))

    def test_tail_bend_zero_gives_zero_zero_lift_moment(self):
        for fam in bo.FAM_ORDER:
            r = bo.run_case(bo.family_geometry(fam), 250.0, 0.0, self.dir / f"{fam}0", f"{fam}0")
            self.assertLess(abs(r["cm0"]), 5e-4)
            self.assertAlmostEqual(r["trim"][0.3]["static_margin"], r["cm0"] / 0.3)

    def test_bend_adds_nose_up_moment_and_trim_formula(self):
        g = bo.family_geometry("BS")
        r5 = bo.run_case(g, 250.0, 5.0, self.dir / "bs5", "bs5")
        r10 = bo.run_case(g, 250.0, 10.0, self.dir / "bs10", "bs10")
        self.assertGreater(r5["cm0"], 0.01)
        self.assertGreater(r10["cm0"], r5["cm0"])
        self.assertAlmostEqual(r10["cm0"] / r5["cm0"], 2.0, delta=0.15)                 # nearly linear in the bend (AVL is linear)
        for c, tr in r10["trim"].items():
            self.assertAlmostEqual(tr["x_cg_span"], r10["np_span"] - g["mac"] * r10["cm0"] / c)
            self.assertAlmostEqual(tr["static_margin"], r10["cm0"] / c)
        # the neutral point implied by dCM/dCL equals AVL's Xnp (sign-convention check)
        self.assertAlmostEqual(r10["np_from_cm_slope_span"], r10["np_span"], delta=2e-3)
        self.assertLess(r10["dcm_dcl"], 0)
        self.assertGreater(r10["trim"][0.2]["static_margin"], r10["trim"][0.5]["static_margin"])

    def test_rectangular_wing_reproduces_quarter_chord_neutral_point(self):
        g = bo.build_geometry(rect_params(6.0))
        r = bo.run_case(g, 250.0, 0.0, self.dir / "rect", "rect", with_tail=False)
        c = g["c_root"]
        self.assertAlmostEqual(r["np_span"] / c, 0.25, delta=0.02)                       # within AVL tolerance of the quarter chord
        self.assertAlmostEqual(abs(r["cm0"]), 0.0, delta=1e-4)

    def test_tip_bend_geometry(self):
        g = bo.family_geometry("BS")
        flat = bo.wing_sections(g)
        bent = bo.wing_sections(g, (bo.TIP_BEND_ETA, -30.0))
        self.assertEqual(len(bent), len(flat) + 1)                                      # one station inserted on the bend line
        zs = [z for y, x, z, c in bent]
        self.assertEqual(zs[0], 0.0)
        self.assertLess(min(zs), 0.0)                                                   # tip bent DOWN
        self.assertLess(max(y for y, x, z, c in bent), max(y for y, x, z, c in flat))   # span reduced by cos(angle) beyond the bend

    def test_pullout_arithmetic(self):
        self.assertAlmostEqual(bo.pullout_radius(10.0, 0.5), 2 * 10.0 / (1.225 * 9.80665 * 0.5), places=9)
        self.assertAlmostEqual(bo.pullout_radius(10.0, 0.3) / bo.pullout_radius(10.0, 0.5), 5 / 3, places=9)


class OutputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="bird_out_")
        cls.out = Path(cls.tmp.name)
        cls._levels, cls._deltas = bo.SCAN_LEVELS, bo.SCAN_SWEEP_DELTAS
        bo.SCAN_LEVELS, bo.SCAN_SWEEP_DELTAS = (0.8, 1.0), (0.0,)           # a reduced scan keeps the test short
        cls.R = bo.run(cls.out)

    @classmethod
    def tearDownClass(cls):
        bo.SCAN_LEVELS, bo.SCAN_SWEEP_DELTAS = cls._levels, cls._deltas
        cls.tmp.cleanup()

    def rows(self, name):
        with (self.out / name).open(encoding="utf-8") as f:
            return list(csv.DictReader(f))

    def test_row_counts(self):
        self.assertEqual(len(self.rows("avl_table.csv")), len(bo.FAM_ORDER) * len(bo.TAIL_BENDS) * len(bo.TRIM_CLS))
        self.assertEqual(len(self.rows("scan.csv")), len(bo.FAM_ORDER) * len(bo.SCAN_LEVELS) ** 2 * len(bo.SCAN_SWEEP_DELTAS))
        self.assertEqual(len(self.rows("yaw_fixes.csv")), len(bo.FAM_ORDER) * (2 + len(bo.TIP_BEND_ANGLES)))
        nb = sum(len(self.R["props"][f]) for f in bo.FAM_ORDER) * 2
        self.assertEqual(len(self.rows("builds.csv")), nb)
        self.assertEqual(len(self.rows("pullout.csv")), nb * len(bo.PULLOUT_CLS))
        self.assertEqual(len(self.rows("dimensions.csv")), sum(len(bo.FAMILIES[f]["params"]) for f in bo.FAM_ORDER))
        self.assertEqual(len(self.rows("scan_main_effects.csv")), len(bo.FAM_ORDER) * (len(bo.SCAN_LEVELS) * 2 + len(bo.SCAN_SWEEP_DELTAS)))
        for fam in bo.FAM_ORDER:
            self.assertTrue(1 <= len(self.R["props"][fam]) <= bo.N_BUILD_BENDS)
            for b in self.R["props"][fam]:
                self.assertIn(b, bo.TAIL_BENDS)
                self.assertGreater(b, 0)

    def test_proposal_rule_only_reachable_bends(self):
        for fam in bo.FAM_ORDER:
            reachable = [b for b in bo.TAIL_BENDS if b > 0 and self.R["res"][(fam, b)]["trim"][0.3]["x_cg_span"] >= 0]
            if reachable:
                self.assertTrue(set(self.R["props"][fam]) <= set(reachable))

    def test_stl_shells_watertight_and_mass(self):
        for fam in bo.FAM_ORDER:
            f = self.out / f"{fam}_printed_span250" / "sheet.stl"
            tris = ft.read_stl(f)
            g = self.R["geoms"][fam]
            shells, _ = bo.printed_meshes(g, 250.0, 0.6)
            for nm, s in shells.items():
                self.assertEqual(ft.edge_report(s, nd=4)[1:], (0, 0, 0), (fam, nm))
                self.assertGreater(ft.mesh_stats(s)["volume"], 0)
            self.assertEqual(len(tris), sum(len(s) for s in shells.values()))
            vol_main = ft.mesh_stats(shells["main"])["volume"]
            self.assertAlmostEqual(vol_main, abs(ft.area_signed(ft.orient_ccw([(x * 250, y * 250) for x, y in g["main_full"]]))) * 0.6, delta=0.01 * vol_main)
            lo, hi = ft.bbox(tris)
            self.assertAlmostEqual(hi[2], 0.6, places=4)
            self.assertAlmostEqual(hi[1] - lo[1], 250.0, delta=1.0)
            # hinge strip: one layer thick, overlapping both neighbours
            self.assertAlmostEqual(ft.bbox(shells["hinge"])[1][2], bo.HINGE_THICK_MM, places=4)

    def test_drawings_are_wellformed_dark_svg(self):
        for fam in bo.FAM_ORDER:
            for sz, k in (("printed", 250), ("foam", 400)):
                txt = (self.out / f"{fam}_{sz}_span{k}" / "drawing.svg").read_text(encoding="utf-8")
                root = ET.fromstring(txt)
                self.assertTrue(root.tag.endswith("svg"))
                self.assertIn('fill="#14181d"', txt)
                self.assertIn("hinge line", txt)
                self.assertIn("wrist joint line", txt)

    def test_dxf_layers(self):
        txt = (self.out / "BS_printed_span250" / "parts.dxf").read_text(encoding="ascii")
        for layer in ("OUTLINE", "HINGE_BEND_UP", "WRIST_MARKER", "CENTRELINE", "CG_MARKS"):
            self.assertIn(f"8\n{layer}\n", txt)

    def test_templates_supported_elements_and_scale_bar(self):
        pages = self.R["pdf_pages"]
        self.assertGreaterEqual(len(pages), 4)
        for p in pages:
            txt = p.read_text(encoding="utf-8")
            root = ET.fromstring(txt)
            tags = {el.tag.split("}")[-1] for el in root}
            self.assertTrue(tags <= {"rect", "line", "polygon", "text"}, (p.name, tags))
            self.assertEqual(root.get("viewBox"), "0 0 215.9 279.4")
            self.assertEqual((root.get("width"), root.get("height")), ("215.9mm", "279.4mm"))
            m = re.search(r'<line id="scalebar" x1="([-0-9.]+)" y1="([-0-9.]+)" x2="([-0-9.]+)" y2="([-0-9.]+)"', txt)
            self.assertIsNotNone(m, p)
            x1, y1, x2, y2 = map(float, m.groups())
            self.assertEqual(y1, y2)
            self.assertAlmostEqual(x2 - x1, 100.0, places=9)                           # exactly 100 mm
            self.assertIn('fill="white"', txt)
            self.assertNotIn("#14181d", txt)                                           # print sheets stay white

    def test_template_content_and_tiling(self):
        by = {}
        for p in self.R["pdf_pages"]:
            by.setdefault(p.parent.name, []).append(p)
        self.assertEqual(set(by), {"BS_foam_span400", "BC_foam_span400"})
        for name, pages in by.items():
            txts = [p.read_text(encoding="utf-8") for p in pages]
            half = txts[:-1]                                                           # last page is the notes page
            for t in half:
                for needle in ("hinge: bend tail UP", "centreline", "0 = wing root leading edge", "wrist line", "stroke-dasharray"):
                    self.assertIn(needle, t, (name, needle))
                self.assertGreaterEqual(t.count('stroke="#000" stroke-width="0.3"'), 8)           # CG tick marks
            self.assertIn("gauge for a", txts[-1])
            fam = name.split("_")[0]
            g = self.R["geoms"][fam]
            xs = [x * 400 for x, y in g["half_outline"]]
            need = max(xs) - min(xs) + 8
            pw = ft.LETTER_W - 20
            if need > pw:
                self.assertGreater(len(half), 1)                                          # tiled with overlap marks
                self.assertIn("overlap", half[0])
                self.assertIn("overlap", half[1])
            else:
                self.assertEqual(len(half), 1)
            # the hinge line is where the geometry says: x_h mm aft of the root leading edge
            m = re.search(r'<line [^>]*x1="([-0-9.]+)" y1="([-0-9.]+)" x2="([-0-9.]+)" y2="([-0-9.]+)" stroke="#c2621a" stroke-width="0.7"', half[0])
            self.assertIsNotNone(m)
            self.assertAlmostEqual(float(m.group(1)), float(m.group(3)))
            self.assertAlmostEqual(float(m.group(4)), 240.0 - g["wt0"] * 400 / 2, places=2)

    def test_pdf_produced(self):
        pdf = self.out / self.R["pdf"]
        data = pdf.read_bytes()
        self.assertTrue(data.startswith(b"%PDF"))
        pages = len(re.findall(rb"/Type\s*/Page\b(?!s)", data))
        self.assertEqual(pages, len(self.R["pdf_pages"]))

    def test_ballast_and_pullout_numbers(self):
        for b in self.R["builds"]:
            self.assertAlmostEqual(b["M"], b["M0"] + b["ballast_g"])
            self.assertAlmostEqual(b["ws"], b["M"] * 1e-3 * 9.80665 / (b["S_w_mm2"] * 1e-6))
            for cl in bo.PULLOUT_CLS:
                self.assertAlmostEqual(b["pull"][cl], bo.pullout_radius(b["ws"], cl))
            self.assertAlmostEqual((b["m_sheet"] * b["cx_sheet"] + b["rail"]["mass_g"] * b["rail"]["cx"]) / b["M0"], b["cg0"])
            if b["ballast_reachable"]:
                m = b["M0"] + b["ballast_g"]
                self.assertAlmostEqual((b["M0"] * b["cg0"] + b["ballast_g"] * b["rail"]["station"]) / m, b["cg_target"], places=6)
        # printed mass from the mesh volume x the PLA density read from flat_test
        b = next(x for x in self.R["builds"] if x["size"] == "printed")
        shells, _ = bo.printed_meshes(self.R["geoms"][b["fam"]], 250.0, 0.6)
        self.assertAlmostEqual(b["m_sheet"], ft.mass_g(sum(ft.mesh_stats(s)["volume"] for s in shells.values())))
        f = next(x for x in self.R["builds"] if x["size"] == "foam")
        self.assertAlmostEqual(f["m_sheet"], f["area_sheet_mm2"] * 1e-6 * ft.FOAM_AREAL_MASS_G_M2)

    def test_rail_reuse_flags_are_computed_not_assumed(self):
        for b in self.R["builds"]:
            self.assertEqual(b["rail_fits_under_hinge"], b["rail"]["saddle_end"] <= b["hinge_x_mm"])
            lo, hi = b["rail"]["tick_range"]
            self.assertEqual(b["rail_covers_trim"], lo <= min(b["trim_cgs_mm"]) and hi >= max(b["trim_cgs_mm"]))
        self.assertEqual(self.R["builds"][0]["rail"]["tick_range"], (30, 55))                  # the existing F45 rail, read from the kit

    def test_page_honesty(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        low = page.lower()
        for needle in ("not passively stable", "outline-only", "ignored in the avl model", "a proposal for kai, not a decision", "from the bird notes", "engineering judgement",
                       "does not rely on either claim", "not a window", "says nothing about", "static pitch stability is not the same as a steady glide"):
            self.assertIn(needle, low, needle)
        for banned in ("will pull out", "will glide", "glide ratio of", "passively stable bird", "predicted cg window", "the tail stabilises"):
            self.assertNotIn(banned, low, banned)
        self.assertNotIn("<td class='n'>glide ratio", low)
        for tg in ("CITED", "JUDGEMENT", "GEOMETRY", "AVL", "ARITHMETIC", "ASSUMED"):
            self.assertIn(tg, page)
        self.assertIn("#14181d", page)                                                           # always-dark palette from flat_test
        self.assertIn("0 mark", page)

    def test_default_output_is_new_folder(self):
        self.assertEqual(bo.OUT_DEFAULT.name, "bird_outline")
        self.assertNotIn("flat_test", bo.OUT_DEFAULT.as_posix())


if __name__ == "__main__":
    unittest.main()
