"""Tests for fin_kit_v4.py.   py -B -m unittest test_fin_kit_v4 -v   (from rigid_glider/; no AVL runs, reads the saved results folder)"""
import sys
sys.dont_write_bytecode = True
import csv
import json
import math
import re
import unittest
import xml.etree.ElementTree as ET

import fin_kit_v4 as fk

OUT = fk.OUT


class UnitTests(unittest.TestCase):
    def test_eigenvalues_known_matrix(self):
        a = [[-2.0, 0, 0, 0], [0, 0, 3.0, 0], [0, -3.0, 0, 0], [0, 0, 0, 0.5]]
        z = sorted(fk.eig_roots(a), key=lambda c: (c.real, c.imag))
        want = sorted([complex(-2, 0), complex(0, 3), complex(0, -3), complex(0.5, 0)], key=lambda c: (c.real, c.imag))
        for u, v in zip(z, want):
            self.assertAlmostEqual(abs(u - v), 0.0, places=6)

    def test_fin_area_and_aspect_ratio(self):
        g = fk.v4.build_geometry4(fk.v4.GLIDE)
        for opt in fk.OPTIONS:
            f = fk.fin_geometry(g, opt, 0.01)
            self.assertAlmostEqual(f["n"] * f["c"] * f["h"], 0.01 * g["S"], places=9)
            self.assertAlmostEqual(f["ar"], fk.FIN_AR, places=6)
            self.assertFalse(f["capped"])
        tip = fk.fin_geometry(g, "tip", 0.01)
        self.assertAlmostEqual(tip["x_te"], g["le_fn"](tip["y"]) + g["chord_fn"](tip["y"]), places=9)          # trailing edge level with the wing's
        ctr = fk.fin_geometry(g, "centre", 0.01)
        self.assertLess(ctr["x_te"], g["x_h"])                                                                  # ahead of the hinge line
        tail = fk.fin_geometry(g, "tail", 0.01)
        self.assertGreater(tail["x_le"], g["x_h"])                                                              # aft of the hinge line

    def test_deck_surfaces(self):
        g = fk.v4.build_geometry4(fk.v4.GLIDE)
        d = fk.deck_for(g, 6.0, fk.fin_geometry(g, "tip", 0.02), 0.023)
        self.assertIn("TipFin", d)
        self.assertEqual(d.split("TipFin")[1].split("SURFACE")[0].count("YDUPLICATE"), 1)
        d = fk.deck_for(g, 6.0, fk.fin_geometry(g, "centre", 0.02), 0.023)
        self.assertNotIn("YDUPLICATE", d.split("CentreFin")[1])

    def test_fin_mass_scales_with_material(self):
        g = fk.v4.build_geometry4(fk.v4.GLIDE)
        f = fk.fin_geometry(g, "tip", 0.02)
        self.assertGreater(fk.fin_mass(f, "foam"), 0)
        self.assertGreater(fk.fin_mass(f, "printed"), 0)


@unittest.skipUnless((OUT / "runs.json").is_file(), "fin kit not generated")
class ResultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.R = json.loads((OUT / "runs.json").read_text(encoding="utf-8"))

    def test_cn_beta_rises_monotonically_for_tip_and_tail_fins(self):
        for k in "AB":
            for opt in ("tip", "tail"):
                cn = [self.R[k]["base"]["cnb"]] + [r["cnb"] for r in self.R[k]["sweep"][opt]]
                self.assertTrue(all(b > a for a, b in zip(cn, cn[1:])), (k, opt, cn))

    def test_centre_fin_ahead_of_hinge_does_not_reach_the_floor_gain(self):
        for k in "AB":
            cn = [r["cnb"] for r in self.R[k]["sweep"]["centre"]]
            self.assertEqual(cn.index(max(cn)) < len(cn) - 1, True)                    # peaks inside the sweep, then falls
            self.assertLess(cn[-1], self.R[k]["base"]["cnb"])

    def test_chosen_sizes_reach_their_targets(self):
        for k in "AB":
            for opt in ("tip", "tail"):
                f, x = self.R[k]["chosen"][f"{opt}_floor"], self.R[k]["chosen"][f"{opt}_x15"]
                cn0 = self.R[k]["base"]["cnb"]
                self.assertAlmostEqual(f["per_size"]["foam"]["r"]["cnb"], cn0 + self.R["delta"]["floor"], delta=0.0003)
                self.assertAlmostEqual(x["per_size"]["foam"]["r"]["cnb"], cn0 + self.R["delta"]["x15"], delta=0.0003)
                self.assertLess(f["f_pct"], x["f_pct"])
        a = self.R["A"]["chosen"]["tip_floor"]["per_size"]["foam"]["r"]["cnb"]
        self.assertGreaterEqual(a, 0.0039)

    def test_scale_free_cn_same_for_both_builds(self):
        for k in "AB":
            for key, e in self.R[k]["chosen"].items():
                self.assertAlmostEqual(e["per_size"]["foam"]["r"]["cnb"], e["per_size"]["printed"]["r"]["cnb"], delta=2e-5, msg=(k, key))

    def test_trim_cg_matches_v4_table(self):
        with (fk.ROOT / "results/runs/rigid_glider/bird_outline_v4_swift/outlines.csv").open(encoding="utf-8") as f:
            rows = {r["outline"][0]: r for r in csv.DictReader(f)}
        for k in "AB":
            want = float(rows[k]["trim CG mm at 10% margin (250)"])
            self.assertAlmostEqual(self.R[k]["base"]["cg_span"] * 250, want, delta=1.0)

    def test_eigenvalues_are_a_roll_a_spiral_and_a_dutch_roll(self):
        for k in "AB":
            for key, e in self.R[k]["chosen"].items():
                m = e["per_size"]["foam"]["r"]["modes"]
                self.assertLess(m["roll"], -5.0)
                self.assertLess(abs(m["spiral"]), 1.0)
                self.assertGreater(m["dr_wn"], 1.0)

    def test_fin_mass_added_aft_needs_more_ballast_than_shift_alone(self):
        e = self.R["A"]["chosen"]["tip_build"]["per_size"]["foam"]["cgc"]
        self.assertGreater(e["extra_ballast_g"], 0)
        self.assertGreater(e["cg_mass_only_mm"], 0)

    def test_templates(self):
        for i in (1, 2):
            txt = (OUT / f"fin_template_page{i}.svg").read_text(encoding="utf-8")
            root = ET.fromstring(txt)
            self.assertTrue({el.tag.split("}")[-1] for el in root} <= {"rect", "line", "polygon", "text"})
            m = re.search(r'<line id="scalebar" x1="([-0-9.]+)" y1="[-0-9.]+" x2="([-0-9.]+)"', txt)
            self.assertAlmostEqual(float(m.group(2)) - float(m.group(1)), 100.0, places=9)
            for p in re.findall(r'points="([^"]+)"', txt):
                for pt in p.split():
                    x, y = map(float, pt.split(","))
                    self.assertTrue(10 <= x <= 205.9 and 10 <= y <= 269.4, pt)
        pdf = (OUT / "fin_templates.pdf").read_bytes()
        self.assertEqual(len(re.findall(rb"/Type\s*/Page(?![s\w])", pdf)), 2)

    def test_page_is_dark_and_tagged(self):
        t = (OUT / "index.html").read_text(encoding="utf-8")
        self.assertIn("color-scheme:dark", t)
        for w in ("MODEL", "ASSUMED", "JUDGEMENT", "uncertain"):
            self.assertIn(w, t)


if __name__ == "__main__":
    unittest.main()
