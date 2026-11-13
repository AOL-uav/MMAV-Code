"""Unit tests for flat_test.py (generator) and flat_test_report.py. Run from rigid_glider/:  py -B -m unittest test_flat_test -v
The generator is run once into a temporary folder (AVL included); nothing is written under results/."""
import sys
sys.dont_write_bytecode = True
import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

import flat_test as ft
import flat_test_report as fr


def fr_outcomes():
    return list(fr.OUTCOMES)


def tagged_ok(node):
    """Every number must sit inside a dict that has a 'source' tag from the allowed set."""
    if isinstance(node, dict):
        if "source" in node:
            return node["source"] in ft.SOURCES and "value" in node
        return all(tagged_ok(v) for v in node.values())
    if isinstance(node, list):
        return all(tagged_ok(v) for v in node)
    if isinstance(node, bool) or not isinstance(node, (int, float)):
        return True
    return False


class Built(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="flat_test_")
        cls.out = Path(cls.tmp.name)
        cls.res = ft.build(cls.out)
        cls.by = {r["name"]: r for r in cls.res}
        cls.printed = [r for r in cls.res if r["kind"] == "printed" and not r.get("optional")]
        cls.optional = [r for r in cls.res if r.get("optional")]
        cls.foam = [r for r in cls.res if r["kind"] == "foam"]

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()


class GeometryTests(Built):
    def test_area_taper_sweep(self):
        for ar in (4.5, 6.0):
            g = ft.wing_geometry(250.0, ar)
            self.assertAlmostEqual(g["area"], 250.0 ** 2 / ar)
            self.assertAlmostEqual(g["ct"] / g["cr"], 0.5)
            self.assertAlmostEqual(g["span"] * (g["cr"] + g["ct"]) / 2, g["area"])
            self.assertAlmostEqual(math.degrees(math.atan2(g["tipx"] + g["ct"] / 4 - g["cr"] / 4, g["span"] / 2)), 25.0, places=9)

    def test_matches_geometry_module(self):
        from geometry import Wing
        w = Wing(span_m=0.25, aspect_ratio=4.5, taper=0.5, quarter_chord_sweep_deg=25, washout_deg=0, dihedral_deg=0, section="NACA_0000", cg_x_m=0)
        g = ft.wing_geometry(250.0, 4.5)
        for a, b in ((g["cr"], w.root_chord), (g["ct"], w.tip_chord), (g["mac"], w.mac), (g["tipx"], w.tip_x_le), (g["mac_y"], w.mac_y)):
            self.assertAlmostEqual(a / 1000, b, places=9)

    def test_p07_outline_scaled(self):
        r = self.by["F45"]["g"]
        self.assertAlmostEqual(r["ar"], 4.5)
        self.assertAlmostEqual(r["cr"], 74.07, places=2)

    def test_names_and_two_wings(self):
        self.assertEqual(sorted(self.by), ["B45", "F45", "F45-plain", "F60"])
        self.assertEqual(ft.DIHEDRAL_DEG, 0.0)

    def test_fin_area_about_four_percent(self):
        for r in self.printed:
            self.assertAlmostEqual(r["fd"]["height"], 15.0)
            self.assertTrue(3.5 < r["fin_pct"] < 4.5, r["fin_pct"])
            self.assertAlmostEqual(r["fin_pct"], 100 * r["g"]["ct"] * 15.0 / r["g"]["area"])


class MeshTests(Built):
    def load(self, r, part):
        return ft.read_stl(r["folder"] / f"{part}.stl")

    def test_stl_watertight_and_consistent(self):
        for r in self.printed:
            for part in ("wing", "fin", "stick"):
                tris = self.load(r, part)
                n_edges, bad, same, degenerate = ft.edge_report(tris, nd=4)
                self.assertEqual(bad, 0, (r["name"], part))
                self.assertEqual(same, 0, (r["name"], part))
                self.assertEqual(degenerate, 0, (r["name"], part))
                self.assertGreater(ft.mesh_stats(tris)["volume"], 0.0, (r["name"], part))

    def test_wing_volume_within_2_percent(self):
        for r in self.printed:
            poly = ft.plate_polygon(r["g"], r["t"], tabs=True)
            R = r["rib"]
            self.assertIsNotNone(R)                                       # the rib is the default
            expect = abs(ft.area_signed(poly)) * r["t"] + 2 * R["area_each"] * (R["height"] + ft.RIB_OVERLAP_MM)
            vol = ft.mesh_stats(self.load(r, "wing"))["volume"]
            self.assertLess(abs(vol - expect) / expect, 0.02)
            self.assertAlmostEqual(R["volume"], 2 * R["area_each"] * R["height"])

    def test_bounding_boxes(self):
        for r in self.printed:
            g, t = r["g"], r["t"]
            lo, hi = ft.bbox(self.load(r, "wing"))
            self.assertAlmostEqual(hi[0] - lo[0], g["tipx"] + g["ct"], places=3)
            self.assertAlmostEqual(hi[1] - lo[1], g["span"] + 2 * ft.tab_dims(g, t)["out"], places=3)
            self.assertAlmostEqual(hi[2] - lo[2], t + ft.RIB_HEIGHT_MM, places=4)
            flo, fhi = ft.bbox(self.load(r, "fin"))
            self.assertAlmostEqual(fhi[0] - flo[0], g["ct"], places=3)
            self.assertAlmostEqual(fhi[1] - flo[1], 15.0, places=3)
            slo, shi = ft.bbox(self.load(r, "stick"))
            self.assertAlmostEqual(shi[0] - slo[0], g["cr"] * 1.4, places=3)
            self.assertAlmostEqual(shi[2] - slo[2], ft.STICK_WIDTH_MM, places=4)
            self.assertAlmostEqual(shi[1] - slo[1], ft.STICK_DEPTH_MM, places=4)

    def test_fin_slots_match_wing_tabs(self):
        for r in self.printed:
            G = r["geometry"]
            tl, sl = G["tab_length_mm"]["value"], G["slot_length_mm"]["value"]
            self.assertAlmostEqual(sl - tl, ft.SLOT_LEN_CLEARANCE_MM)
            self.assertAlmostEqual(G["slot_width_mm"]["value"], G["thickness_mm"]["value"] + 0.1)
            self.assertEqual(len(G["tab_centres_from_tip_LE_mm"]["value"]), 2)
            fvol = ft.mesh_stats(self.load(r, "fin"))["volume"]
            expect = (r["g"]["ct"] * r["fd"]["height"] - 2 * sl * G["slot_width_mm"]["value"]) * r["fd"]["thickness"]
            self.assertAlmostEqual(fvol, expect, delta=1e-3 * expect)
            xs = {round(p[0], 6) for p in ft.plate_polygon(r["g"], r["t"]) if abs(p[1]) > r["g"]["span"] / 2 + 1e-9}
            for c in G["tab_centres_from_tip_LE_mm"]["value"]:
                for s in (-1, 1):
                    self.assertIn(round(r["g"]["tipx"] + c + s * tl / 2, 6), xs)

    def test_rib_is_closed_shells_and_plain_is_available(self):
        g = ft.wing_geometry(250.0, 4.5)
        tris, _ = ft.wing_mesh(g, 0.6, rib=True)
        _, bad, same, degenerate = ft.edge_report(tris, nd=5)
        self.assertEqual((bad, same, degenerate), (0, 0, 0))             # every edge shared by exactly two triangles
        lo, hi = ft.bbox(tris)
        self.assertAlmostEqual(hi[2], 0.6 + ft.RIB_HEIGHT_MM)
        plain, _ = ft.wing_mesh(g, 0.6, rib=False)
        self.assertAlmostEqual(ft.bbox(plain)[1][2], 0.6)
        self.assertEqual(ft.edge_report(plain, nd=5)[1:], (0, 0, 0))

    def test_rib_geometry(self):
        for r in self.printed:
            g, R = r["g"], r["rib"]
            self.assertEqual((R["width"], R["height"]), (1.2, 2.5))
            self.assertAlmostEqual(R["gap"], ft.STICK_WIDTH_MM + 0.4)       # centre gap = stick width + 0.4
            ys = [abs(p[1]) for poly in R["polys"] for p in poly]
            self.assertAlmostEqual(min(ys), R["gap"] / 2)
            self.assertAlmostEqual(max(ys), g["span"] / 2 - ft.RIB_TIP_STOP_MM)   # stops 3 mm short of the tip
            for poly in R["polys"]:                                           # centre line of each strip = the quarter-chord line
                for x, y in poly:
                    xq = g["cr"] / 4 + (g["tipx"] + g["ct"] / 4 - g["cr"] / 4) * abs(y) / (g["span"] / 2)
                    self.assertAlmostEqual(abs(x - xq), R["width"] / 2)
            self.assertGreater(len(R["polys"]), 1)
            # no rib in the centre gap: the stick glues flat to the plate
            self.assertTrue(all(abs(y) >= R["gap"] / 2 - 1e-9 for poly in R["polys"] for _, y in poly))
            self.assertTrue(r["geometry"]["rib_included"]["value"])

    def test_rib_down_text_in_drawing(self):
        txt = (self.printed[0]["folder"] / "drawing.svg").read_text(encoding="utf-8")
        self.assertIn("FLOWN RIB-DOWN", txt)

    def test_plain_flag_not_default(self):
        self.assertTrue(all(r["rib"] and not r["plain"] for r in self.printed))
        import inspect
        self.assertIn("plain", inspect.signature(ft.build).parameters)
        self.assertFalse(inspect.signature(ft.build).parameters["plain"].default)

    def test_fins_centred_on_plate_midplane(self):
        for r in self.printed:                                                # fin slot centred on the fin height = plate mid-plane
            fd, sd = r["fd"], r["sd"]
            xs, ys, keep = ft.fin_mesh(r["g"], r["t"])[1]
            self.assertAlmostEqual((ys[1] + ys[2]) / 2, fd["height"] / 2)

    def test_stick_features(self):
        for r in self.printed:
            L = r["stick"]["layout"]
            self.assertAlmostEqual(L["nose"], 0.4 * r["g"]["cr"])
            self.assertAlmostEqual(L["x_hi"], r["g"]["cr"])
            xs = [t["x"] for t in r["stick"]["ticks"]]
            self.assertTrue(min(xs) <= r["settings"][0] and max(xs) >= r["settings"][-1])
            self.assertEqual(xs, list(range(xs[0], xs[-1] + 1)))
            self.assertEqual({h[0] for h in r["stick"]["holes"]}, {"ballast", "release"})

    def test_ballast_hole_row(self):
        for r in self.printed:
            b = sorted(h[1] for h in r["stick"]["holes"] if h[0] == "ballast")
            self.assertTrue(all(abs((y - x) - 5.0) < 1e-9 for x, y in zip(b, b[1:])))        # 5 mm pitch
            self.assertTrue(all(h[3] == 3.2 for h in r["stick"]["holes"] if h[0] == "ballast"))   # M3 clearance
            self.assertTrue(any(x < 0 for x in b) and any(x > 0 for x in b))                  # along the nose AND under the wing
            self.assertEqual(r["default_station"] in b, True)

    def test_loop_notches_every_5mm_through_cg_range(self):
        for r in self.printed:
            five = [t for t in r["stick"]["ticks"] if t["x"] % 5 == 0]
            xs = [t["x"] for t in five]
            self.assertTrue(all(b - a == 5 for a, b in zip(xs, xs[1:])))
            self.assertLessEqual(min(xs), r["settings"][0])
            self.assertGreaterEqual(max(xs), r["settings"][-1])
            self.assertTrue(all(t["width"] >= 1.0 for t in five))


class ArithmeticTests(Built):
    def test_ballast_arithmetic(self):
        self.assertAlmostEqual(ft.ballast_g(10.0, 50.0, 40.0, -20.0), 100.0 / 60.0)
        self.assertAlmostEqual(ft.ballast_g(10.0, 40.0, 40.0, -20.0), 0.0)
        self.assertLess(ft.ballast_g(10.0, 30.0, 40.0, -20.0), 0.0)
        m, cg0, x, st = 13.46, 54.7, 40.0, -22.0
        mb = ft.ballast_g(m, cg0, x, st)
        self.assertAlmostEqual((m * cg0 + mb * st) / (m + mb), x)

    def test_masses(self):
        r = self.by["F45"]
        x = r["thick"][0.6]
        self.assertAlmostEqual(x["plate_mass"], r["plate_area"] * 0.6 * 1e-3 * 1.24)
        self.assertAlmostEqual(x["rib_mass"], r["rib"]["volume"] * 1e-3 * 1.24)
        self.assertAlmostEqual(x["wing_mass"], x["plate_mass"] + x["rib_mass"])
        self.assertAlmostEqual(r["thick"][0.4]["plate_mass"] * 1.5, x["plate_mass"])
        self.assertAlmostEqual(x["allup_noballast"], x["wing_mass"] + x["fins_mass"] + x["stick_mass"])
        self.assertAlmostEqual(x["allup_ballast"], x["allup_noballast"] + x["ballast_mid"])

    def test_speed_and_re(self):
        r = self.by["F45"]
        x = r["thick"][0.6]
        W = x["allup_ballast"] * 1e-3 * 9.80665
        v = math.sqrt(2 * W / (1.225 * r["g"]["area"] * 1e-6 * 0.3))
        self.assertAlmostEqual(x["V_cl0.3"], v)
        self.assertAlmostEqual(x["Re_tip_cl0.3"], v * r["g"]["ct"] * 1e-3 / 1.5e-5)

    def test_bend_scales_with_t_cubed(self):
        a, b = ft.bend_mm(0.1, 250, 55, 0.4), ft.bend_mm(0.1, 250, 55, 0.8)
        self.assertAlmostEqual(a / b, 8.0)

    def test_rib_stiffens_by_t_section(self):
        c, t, w, H = 55.0, 0.6, 1.2, 2.5
        I, yn = ft.second_moment_T(c, t, w, H)
        self.assertGreater(I, c * t ** 3 / 12 * 5)
        self.assertAlmostEqual(ft.second_moment_T(c, t, w, 1e-12)[0], c * t ** 3 / 12, places=6)   # no rib height -> plate only
        self.assertAlmostEqual(yn, 0.42917, places=4)
        self.assertAlmostEqual(I, 9.159, places=2)                      # worked by hand: A1 = 33 at 0.3, A2 = 3.0 at 1.85
        W = 0.15
        self.assertLess(ft.bend_mm(W, 250, c, t, (w, H)), ft.bend_mm(W, 250, c, t) / 5)

        s = ft.sweep_settings(43.9, 57.6)
        self.assertEqual(s[0], math.ceil(43.9 - 0.15 * 57.6))
        self.assertGreaterEqual(s[-1], 43.9 + 0.08 * 57.6)                  # at least 8 % MAC behind the neutral point
        self.assertLess(s[-1], 43.9 + 0.08 * 57.6 + 1.0)
        self.assertTrue(all(abs((b - a) - 1.0) < 1e-9 for a, b in zip(s, s[1:])))

    def test_avl_present_and_sane(self):
        for r in self.printed:
            self.assertIsNotNone(r["A"])
            self.assertTrue(20 < r["A"]["bare"]["xnp_mm"] < r["g"]["cr"])
            self.assertAlmostEqual(r["A"]["fins"]["cl_at"], 0.3, delta=0.01)
            self.assertGreater(r["A"]["fins"]["cn_beta"], r["A"]["bare"]["cn_beta"])


class TagTests(Built):
    def test_every_number_tagged(self):
        for r in self.res:
            G = json.loads((r["folder"] / "geometry.json").read_text())
            self.assertTrue(tagged_ok(G))
            n = [0]

            def count(x):
                if isinstance(x, dict):
                    if "source" in x:
                        n[0] += 1
                        return
                    for v in x.values():
                        count(v)
            count(G)
            self.assertGreater(n[0], 60)

    def test_untagged_would_fail(self):
        self.assertFalse(tagged_ok({"a": 1.0}))
        self.assertTrue(tagged_ok({"a": ft.T(1.0, "GEOMETRY")}))

    def test_output_files_exist(self):
        for r in self.foam:
            for f in ("stick.stl", "parts.dxf", "drawing.svg", "geometry.json", "template_half_wing_letter.svg", "template_fins_letter.svg"):
                self.assertTrue((r["folder"] / f).is_file(), f)
            self.assertFalse((r["folder"] / "wing.stl").exists())
        for r in self.printed + self.optional:
            for f in ("wing.stl", "fin.stl", "stick.stl", "parts.dxf", "drawing.svg", "geometry.json"):
                self.assertTrue((r["folder"] / f).is_file(), f)
        for f in ("index.html", "scale_table.csv", "log_template.csv", "assumptions.csv"):
            self.assertTrue((self.out / f).is_file(), f)
        page = (self.out / "index.html").read_text(encoding="utf-8")
        self.assertNotIn("http", page.replace("http://www.w3.org", ""))

    def test_assumptions_have_tags(self):
        with (self.out / "assumptions.csv").open(encoding="utf-8") as f:
            for row in csv.DictReader(f):
                self.assertIn(row["source_tag"], ft.SOURCES)


class FoamTests(Built):
    def test_b45_geometry(self):
        r = self.by["B45"]
        g = r["g"]
        self.assertEqual(r["folder"].name, "B45_span400")
        self.assertAlmostEqual(g["span"], 400.0)
        self.assertAlmostEqual(g["area"], 400.0 ** 2 / 4.5)
        self.assertAlmostEqual(g["cr"], 118.5, delta=0.1)               # P07 outline
        self.assertAlmostEqual(g["ct"], 59.3, delta=0.1)
        self.assertAlmostEqual(g["ct"] / g["cr"], 0.5)
        self.assertAlmostEqual(r["fd"]["height"], 24.0)
        self.assertAlmostEqual(r["fin_pct"], 4.0, delta=0.05)
        self.assertEqual(ft.DIHEDRAL_DEG, 0.0)
        self.assertIsNone(r["rib"])

    def test_b45_material_and_thickness(self):
        r = self.by["B45"]
        x = list(r["thick"].values())[0]
        G = r["geometry"]
        self.assertEqual(G["thickness_mm"]["source"], "ASSUMED")
        self.assertEqual(G["material_areal_mass_g_m2"]["source"], "ASSUMED")
        self.assertAlmostEqual(x["wing_mass"], r["g"]["area"] * 1e-6 * 300.0)
        self.assertAlmostEqual(x["t_pct_mean"], 100 * 5.0 / r["g"]["mean_chord"])
        self.assertTrue(x["t_pct_mean"] > 4.3 and x["t_pct_tip"] > 4.3)   # above the published 2.6-4.3 % range
        self.assertIsNone(x["tip_bend"])                                   # foam stiffness unknown: no estimate
        self.assertEqual(G["thickness_table"]["5"]["tip_bend_1g_mm"]["source"], "UNKNOWN")

    def test_b45_stick_scaled_and_length(self):
        r = self.by["B45"]
        self.assertAlmostEqual(r["stick_len"], 1.4 * r["g"]["cr"], places=2)
        self.assertLess(r["stick_len"], 180.0)
        self.assertFalse(r["geometry"]["stick_longer_than_180mm"]["value"])
        self.assertEqual(r["settings"][0], math.ceil(r["A"]["fins"]["xnp_mm"] - 0.15 * r["g"]["mac"]))
        self.assertAlmostEqual(r["stick"]["layout"]["nose"], 0.4 * r["g"]["cr"])

    def test_b45_avl(self):
        r = self.by["B45"]
        self.assertGreater(r["A"]["fins"]["cn_beta"], r["A"]["bare"]["cn_beta"])
        self.assertAlmostEqual(r["A"]["fins"]["xnp_mm"], self.by["F45"]["A"]["fins"]["xnp_mm"] * 1.6, delta=0.5)   # same outline scaled 1.6

    def test_html_mentions(self):
        page = (self.out / "index.html").read_text(encoding="utf-8").lower()
        for needle in ("two printed wings", "foamboard wing", "own test", "open wire hook", "not from a hover", "dummy drop", "above the 2.6 to 4.3", "flown rib-down"):
            self.assertIn(needle, page, needle)
        self.assertNotIn("not by shaking", page)

    @staticmethod
    def svg_points(text):
        import re
        pts = []
        for m in re.finditer(r'<(?:line|rect|text)[^>]*?\bx1?="([-0-9.]+)"[^>]*?\by1?="([-0-9.]+)"', text):
            pts.append((float(m.group(1)), float(m.group(2))))
        for m in re.finditer(r'<line[^>]*x2="([-0-9.]+)" y2="([-0-9.]+)"', text):
            pts.append((float(m.group(1)), float(m.group(2))))
        for m in re.finditer(r'points="([^"]+)"', text):
            for pair in m.group(1).split():
                x, y = pair.split(",")
                pts.append((float(x), float(y)))
        return pts

    def test_template_fits_letter_with_margins_and_scale_bar(self):
        import re
        r = self.by["B45"]
        for fname in ("template_half_wing_letter.svg", "template_fins_letter.svg"):
            txt = (r["folder"] / fname).read_text(encoding="utf-8")
            self.assertIn('width="215.9mm" height="279.4mm"', txt)
            self.assertIn('viewBox="0 0 215.9 279.4"', txt)
            pts = self.svg_points(txt)
            self.assertGreater(len(pts), 10)
            for x, y in pts:
                self.assertGreaterEqual(x, 10.0 - 1e-9, (fname, x, y))
                self.assertLessEqual(x, 215.9 - 10.0 + 1e-9, (fname, x, y))
                self.assertLessEqual(y, 279.4 - 8.0, (fname, x, y))
            for m in re.finditer(r'<text x="([-0-9.]+)" y="([-0-9.]+)"[^>]*font-size="([0-9.]+)" text-anchor="(\w+)"[^>]*>([^<]*)</text>', txt):
                x, size, anchor, body = float(m.group(1)), float(m.group(3)), m.group(4), m.group(5)
                w = 0.55 * size * len(body)                                # conservative text width
                lo, hi = (x, x + w) if anchor == "start" else (x - w, x)
                self.assertGreaterEqual(lo, 10.0 - 1e-9, body)
                self.assertLessEqual(hi, 215.9 - 10.0 + 1e-9, body)
            m = re.search(r'<line id="scalebar" x1="([-0-9.]+)" y1="([-0-9.]+)" x2="([-0-9.]+)" y2="([-0-9.]+)"', txt)
            self.assertIsNotNone(m)
            x1, y1, x2, y2 = map(float, m.groups())
            self.assertEqual(y1, y2)
            self.assertAlmostEqual(x2 - x1, 100.0, places=9)               # exactly 100 mm in SVG units of mm

    def test_half_wing_template_content(self):
        import re
        r = self.by["B45"]
        g = r["g"]
        txt = (r["folder"] / "template_half_wing_letter.svg").read_text(encoding="utf-8")
        for needle in ("flip here for the other half", "CG test range", "AVL neutral point"):
            self.assertIn(needle, txt)
        m = re.search(r'<polygon points="([^"]+)" fill="#e6eef6"', txt)
        pts = [tuple(map(float, p.split(","))) for p in m.group(1).split()]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        self.assertAlmostEqual(max(xs) - min(xs), g["tipx"] + g["ct"], places=2)     # chordwise extent 1:1
        self.assertAlmostEqual(max(ys) - min(ys), g["span"] / 2, places=2)           # half span 1:1
        fin = (r["folder"] / "template_fins_letter.svg").read_text(encoding="utf-8")
        self.assertIn(f'width="{g["ct"]:.3f}" height="24.000"', fin)

    def test_foam_dxf_fin_has_no_slots(self):
        txt = (self.by["B45"]["folder"] / "parts.dxf").read_text(encoding="ascii")
        self.assertIn("FIN_OUTLINE", txt)
        self.assertNotIn("8\nFIN_SLOT\n10", txt)                             # no slot entities (the layer-table entry is allowed)


class ExtendedSweepTests(Built):
    def test_table_extends_behind_neutral_point(self):
        for r in self.res:
            np_, mac = r["np_ref"], r["g"]["mac"]
            self.assertEqual(r["settings"][0], math.ceil(np_ - 0.15 * mac))             # forward start unchanged
            self.assertGreaterEqual(r["settings"][-1], np_ + 0.08 * mac)
            self.assertTrue(all(abs(b - a - 1.0) < 1e-9 for a, b in zip(r["settings"], r["settings"][1:])))   # 1 mm steps
            self.assertTrue(any(c > np_ for c in r["settings"]) and any(c < np_ for c in r["settings"]))

    def test_stick_ticks_and_ballast_table_cover_extended_table(self):
        for r in self.res:
            xs = [t["x"] for t in r["stick"]["ticks"]]
            self.assertGreaterEqual(max(xs), r["settings"][-1])
            self.assertLessEqual(min(xs), r["settings"][0])
            cgs = {b["cg_mm_aft_of_root_LE"] for b in r["ballast_rows"]}
            self.assertEqual(cgs, set(r["settings"]))

    def test_page_marks_ahead_and_behind_and_bare_balance(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        self.assertIn("side of the neutral point", page)
        self.assertIn("class='n behind'", page)
        self.assertIn("class='n ahead'", page)
        for r in self.res:
            if r["kind"] == "foam" or r["A"]:
                x = r["thick"].get(0.6) or list(r["thick"].values())[0]
                self.assertIn(f"<b>{x['cg_noballast']:.1f} mm</b>", page)         # bare-aircraft balance point printed
        self.assertIn("Bare-aircraft balance point", page)
        for r in self.res:
            bare = (r["thick"].get(0.6) or list(r["thick"].values())[0])["cg_noballast"]
            self.assertGreater(bare, r["settings"][-1] - 6)                        # the printed table ends near or ahead of the bare balance point
        G = self.by["F45"]["geometry"]["cg_sweep"]
        self.assertIn("bare_aircraft_balance_mm", G)
        self.assertEqual(G["bare_aircraft_balance_mm"]["source"], "ARITHMETIC")

    def test_stop_rule_and_li_paragraph_on_page(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        low = page.lower()
        for needle in ("stop rule", "while any setting still glides or still dives", "two settings in a row give no steady glide",
                       "bare-aircraft balance point", "holes under the wing instead of the nose", "gets lighter across the sweep", "weigh it at each setting",
                       "10.1017/jfm.2022.89", "j. fluid mech. 937, a6", "0 and 0.08 tumbled", "0.14 was unsteady", "0.24 glided", "0.31 dived",
                       "50 %, 42 %, 36 %, 26 % and 19 % of chord", "0.22 to 0.29 ahead of mid-chord", "21 % to 28 % of chord from the leading edge",
                       "20.3 cm span x 2.54 cm chord x 0.15 cm thick", "reynolds number about 1,000 to 10,000", "centre of pressure on the plate moves with angle of attack",
                       "read from the first 13 of 15 pages of the paper", "our model has no plate result", "unswept rectangles",
                       "tips may let go first", "spar rib; the study", "not a prediction"):
            self.assertIn(needle, low, needle)

    def test_no_window_claim_for_our_wings(self):
        page = (self.out / "index.html").read_text(encoding="utf-8").lower()
        for banned in ("expected window", "predicted window of", "% mac window", "window is about", "will glide", "should glide", "glide window of"):
            self.assertNotIn(banned, page, banned)
        md = (Path(__file__).resolve().parent / "FLAT_TEST.md").read_text(encoding="utf-8")
        self.assertIn("Our model has no plate result.", md)
        self.assertIn("Lips, tails, dihedral joints and other spans are out of scope. No CG window, glide ratio or \"will fly\" claim is made for any plate.", md)

    def test_outcome_vocabulary(self):
        self.assertEqual(fr_outcomes(), ["steady glide", "dive", "pitch-up or stall", "pitch oscillation", "tumble", "roll-off left", "roll-off right", "yaw wander"])
        self.assertEqual(ft.OUTCOMES, fr_outcomes())
        self.assertNotIn("oscillation", fr_outcomes())
        page = (self.out / "index.html").read_text(encoding="utf-8")
        self.assertIn("repeated swooping or bounding, nose up and down", page)
        self.assertIn("flips end over end", page)

    def test_outcome_meaning_entries(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        sec = page[page.index("10. What each outcome would mean"):page.index("11. All assumptions")].lower()
        self.assertIn("only after the stop rule has been followed past the avl neutral point", sec)
        self.assertIn("dive, then glide, then pitch oscillation, then tumble", sec)
        self.assertIn("do not blame the fins", sec)
        self.assertNotIn("yaw wander or oscillation", sec)
        self.assertIn("f45-plain", sec)

    def test_optional_plain_wing(self):
        (r,) = self.optional
        self.assertEqual(r["folder"].name, "F45_plain_span250")
        self.assertIsNone(r["rib"])
        self.assertTrue(r["plain"])
        F45 = self.by["F45"]
        x, y = r["thick"][0.6], F45["thick"][0.6]
        self.assertGreater(x["tip_bend"], 5 * y["tip_bend"])                      # floppier
        self.assertAlmostEqual(x["rib_mass"], 0.0)
        self.assertAlmostEqual(r["g"]["area"], F45["g"]["area"])
        for part in ("wing", "fin", "stick"):
            tris = ft.read_stl(r["folder"] / f"{part}.stl")
            self.assertEqual(ft.edge_report(tris, nd=4)[1:], (0, 0, 0))
        lo, hi = ft.bbox(ft.read_stl(r["folder"] / "wing.stl"))
        self.assertAlmostEqual(hi[2], 0.6, places=4)
        page = (self.out / "index.html").read_text(encoding="utf-8")
        self.assertIn("Optional:</b> F45-plain", page)
        self.assertIn("rough tip bend", page)
        self.assertIn("one plain wing shows whether the rib changes the behaviour", page)
        # not part of the main comparison: it is not in the printed list used for the Reynolds-number statement
        self.assertNotIn(r, self.printed)


class ReportTests(Built):
    def test_log_template_columns_and_examples_ignored(self):
        with (self.out / "log_template.csv").open(encoding="utf-8") as f:
            self.assertEqual(next(csv.reader(f)), fr.COLUMNS)
        valid, errors, ignored = fr.read_log(self.out / "log_template.csv")
        self.assertEqual((len(valid), len(errors)), (0, 0))
        self.assertGreaterEqual(ignored, 2)

    def test_empty_log_says_no_flights(self):
        valid, errors, ignored = fr.read_log(self.out / "log_template.csv")
        page = fr.render(fr.analyse(valid), valid, errors, ignored, "template")
        self.assertIn("No flights logged yet", page)
        self.assertNotIn("<svg", page)

    def synthetic(self, rows):
        p = self.out / "synthetic.csv"
        with p.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(fr.COLUMNS)
            w.writerows(rows)
        return p

    def row(self, cg, k, outcome, wing="F45", launch="drone drop"):
        return ["2026-10-12", wing, 0.6, "tip fins", 16.0, cg, launch, k, outcome, 10, 7, 4.0 if outcome == "steady glide" else "", 3.0 if outcome == "steady glide" else "", ""]

    def test_synthetic_log(self):
        rows = []
        for k in range(10):
            rows.append(self.row(40.2, k + 1, "steady glide" if k < 9 else "dive"))
        for k in range(5):
            rows.append(self.row(39.0, k + 1, "steady glide" if k < 3 else "dive"))
        for k in range(5):
            rows.append(self.row(41.0, k + 1, "steady glide" if k < 4 else "pitch-up or stall"))
        rows.append(self.row(42.0, 1, "banana"))
        rows.append(["EXAMPLE", "EXAMPLE-F45", 0.6, "x", "", 40, "", 1, "steady glide", "", "", "", "", ""])
        valid, errors, ignored = fr.read_log(self.synthetic(rows))
        self.assertEqual((len(valid), len(errors), ignored), (20, 1, 1))
        self.assertIn("banana", errors[0]["problems"])
        a = fr.analyse(valid)
        self.assertEqual(len(a), 1)
        self.assertEqual(a[0]["settings"][40]["steady"], 9)
        self.assertEqual(a[0]["passing"], [40])
        page = fr.render(a, valid, errors, ignored, "synthetic")
        self.assertIn("<svg", page)
        self.assertIn("Rows not counted", page)
        self.assertNotIn("No flights logged yet", page)

    def test_neighbour_must_glide(self):
        rows = ([self.row(40, k + 1, "steady glide") for k in range(10)] + [self.row(39, k + 1, "dive") for k in range(5)]
                + [self.row(41, k + 1, "steady glide") for k in range(5)])
        valid, _, _ = fr.read_log(self.synthetic(rows))
        self.assertEqual(fr.analyse(valid)[0]["passing"], [])

    def test_hand_and_drop_never_pooled(self):
        rows = [self.row(40, k + 1, "steady glide", launch="drone drop") for k in range(10)] + [self.row(40, k + 1, "dive", launch="hand") for k in range(10)]
        valid, _, _ = fr.read_log(self.synthetic(rows))
        a = fr.analyse(valid)
        self.assertEqual(sorted(x["launch"] for x in a), ["drone drop", "hand"])
        by = {x["launch"]: x for x in a}
        self.assertEqual(by["drone drop"]["settings"][40]["steady"], 10)
        self.assertEqual(by["hand"]["settings"][40]["steady"], 0)

    def test_new_log_columns(self):
        self.assertIn("release_height_m", fr.COLUMNS)
        self.assertIn("release_speed_mps", fr.COLUMNS)
        self.assertNotIn("height_m", fr.COLUMNS)
        self.assertEqual(fr.COLUMNS, ft.LOG_COLUMNS)

    def test_three_wings(self):
        rows = []
        for wing, cg in (("F45", 40), ("F60", 38), ("B45", 62)):
            rows += [self.row(cg, k + 1, "steady glide" if k < 6 else "dive", wing=wing) for k in range(10)]
        valid, errors, ignored = fr.read_log(self.synthetic(rows))
        a = fr.analyse(valid)
        self.assertEqual(sorted(x["wing"] for x in a), ["B45", "F45", "F60"])
        page = fr.render(a, valid, errors, ignored, "three")
        for w in ("F45", "F60", "B45"):
            self.assertIn(w, page)
        self.assertEqual(page.count("<svg"), 3)

    def test_oscillation_flagged_not_guessed(self):
        rows = [self.row(40, 1, "oscillation"), self.row(40, 2, "pitch oscillation"), self.row(40, 3, "tumble"), self.row(40, 4, "steady glide")]
        valid, errors, ignored = fr.read_log(self.synthetic(rows))
        self.assertEqual(len(valid), 3)
        self.assertEqual(len(errors), 1)
        self.assertIn("pitch oscillation", errors[0]["problems"])
        self.assertIn("yaw wander", errors[0]["problems"])
        page = fr.render(fr.analyse(valid), valid, errors, ignored, "x")
        self.assertIn("Rows not counted", page)
        self.assertEqual(fr.analyse(valid)[0]["settings"][40]["counts"]["tumble"], 1)
        self.assertEqual(fr.analyse(valid)[0]["settings"][40]["counts"]["pitch oscillation"], 1)

    def test_chart_legend_has_all_outcomes(self):
        rows = [self.row(40, k + 1, o) for k, o in enumerate(fr.OUTCOMES)]
        valid, errors, ignored = fr.read_log(self.synthetic(rows))
        page = fr.render(fr.analyse(valid), valid, errors, ignored, "x")
        for o in fr.OUTCOMES:
            self.assertIn(f">{o}</text>", page)
        self.assertEqual(len(fr.COLORS), len(fr.OUTCOMES))
        self.assertEqual(len(set(fr.COLORS)), len(fr.COLORS))

    def test_missing_column_rejected(self):
        p = self.out / "bad.csv"
        p.write_text("date,wing\n1,2\n")
        with self.assertRaises(ValueError):
            fr.read_log(p)


if __name__ == "__main__":
    unittest.main()
