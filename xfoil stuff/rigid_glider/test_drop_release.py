"""Tests for drop_release.py.   Run:  py -B rigid_glider/test_drop_release.py
Writes only to a temporary folder."""
import sys
sys.dont_write_bytecode = True
import csv
import math
import re
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import drop_release as dr          # noqa: E402
import flat_test as ft             # noqa: E402

_TMP = tempfile.TemporaryDirectory()
OUT = Path(_TMP.name) / "default"
D0, A0, INFO0 = dr.main(["--out", str(OUT)])
NAMES = ["A_tip_latch", "A2_servo_pad", "B_frame_saddle", "C_rx_plate"]


def stl_of(over=None):
    P = dr.default_params()
    P.update(over or {})
    D, parts, info = dr.build(P, mass=False)
    return P, D, parts, info


def bore_radii(tris, Ro):
    r = set()
    for t in tris:
        for p in t:
            if abs(p[0]) < 1e-9:
                rr = math.hypot(p[1], p[2])
                if 1.0 < rr < Ro - 1.0:
                    r.add(round(rr, 3))
    return r


def hole_radii(tris, D):
    r = set()
    for t in tris:
        for p in t:
            if abs(p[0]) < 1e-9:
                rr = math.hypot(p[1] - D["yp"], p[2] - D["zpin"])
                if rr < 1.5:
                    r.add(round(rr, 3))
    return r


class TestMeshes(unittest.TestCase):
    def test_files_watertight_positive_volume(self):
        for n in NAMES:
            tris = ft.read_stl(OUT / f"{n}.stl")
            n_edges, bad, same, degenerate = ft.edge_report(tris)
            self.assertEqual(bad, 0, n)            # every edge shared by exactly two triangles
            self.assertEqual(same, 0, n)           # consistent orientation
            self.assertEqual(degenerate, 0, n)
            self.assertGreater(ft.mesh_stats(tris)["volume"], 100.0, n)
            self.assertGreater(INFO0[n]["mass_g"], 0.2)

    def test_each_shell_watertight(self):
        _, _, parts, _ = stl_of()
        for p in parts.values():
            for s in p.shells:
                tris = s.tris()
                self.assertEqual(ft.edge_report(tris)[1], 0, s.name)
                self.assertGreater(ft.mesh_stats(tris)["volume"], 0, s.name)


class TestDiameters(unittest.TestCase):
    def test_default_bore_and_pin_hole(self):
        tris = ft.read_stl(OUT / "A_tip_latch.stl")
        P = dr.default_params()
        self.assertEqual(bore_radii(tris, D0["Ro"]), {round((P["rod_d_mm"] + P["bore_clear_mm"]) / 2, 3)})
        self.assertEqual(hole_radii(tris, D0), {round((P["pin_d_mm"] + P["pin_clear_mm"]) / 2, 3)})

    def test_rod_changes_bore_not_pin_hole(self):
        P1, D1, _, I1 = stl_of({"rod_d_mm": 6.0})
        self.assertEqual(bore_radii(I1["A_tip_latch"]["tris"], D1["Ro"]), {round((6.0 + P1["bore_clear_mm"]) / 2, 3)})
        self.assertEqual(hole_radii(I1["A_tip_latch"]["tris"], D1), {round((P1["pin_d_mm"] + P1["pin_clear_mm"]) / 2, 3)})
        self.assertEqual(I1["C_rx_plate"]["tris"], INFO0["C_rx_plate"]["tris"])          # receiver plate independent of the rod
        self.assertNotEqual(I1["B_frame_saddle"]["tris"], INFO0["B_frame_saddle"]["tris"])   # saddle groove follows the rod

    def test_pin_changes_hole_not_bore(self):
        P1, D1, _, I1 = stl_of({"pin_d_mm": 1.5})
        self.assertEqual(hole_radii(I1["A_tip_latch"]["tris"], D1), {round((1.5 + P1["pin_clear_mm"]) / 2, 3)})
        self.assertEqual(bore_radii(I1["A_tip_latch"]["tris"], D1["Ro"]), {round(D0["Rb"], 3)})
        self.assertEqual(I1["B_frame_saddle"]["tris"], INFO0["B_frame_saddle"]["tris"])
        self.assertEqual(I1["C_rx_plate"]["tris"], INFO0["C_rx_plate"]["tris"])
        self.assertGreaterEqual(D1["Ro"], D0["Ro"])        # the socket outer grows only if a wider rail needs it; the bore does not change

    def test_servo_changes_only_servo_features(self):
        P1, D1, parts1, I1 = stl_of({"servo_l_mm": 18.0, "servo_w_mm": 10.0, "servo_h_mm": 16.0})
        self.assertEqual(bore_radii(I1["A_tip_latch"]["tris"], D1["Ro"]), {round(D0["Rb"], 3)})
        self.assertAlmostEqual(D1["hole_d"], D0["hole_d"])
        self.assertEqual(I1["B_frame_saddle"]["tris"], INFO0["B_frame_saddle"]["tris"])
        self.assertEqual(I1["C_rx_plate"]["tris"], INFO0["C_rx_plate"]["tris"])
        self.assertNotEqual(I1["A2_servo_pad"]["tris"], INFO0["A2_servo_pad"]["tris"])
        self.assertLess(D1["pad_z0"], D0["pad_z0"])
        # servo height only moves the pad and lengthens/shortens the vertical post: every socket / rail / post shell is identical
        _, _, partsn, _ = stl_of()
        _, _, partsm, _ = stl_of({"servo_h_mm": 16.0})
        for sn, sm in zip(partsn["A_tip_latch"].shells, partsm["A_tip_latch"].shells):
            if sn.name != "side_post":
                self.assertEqual(sn.tris(), sm.tris(), sn.name)


class TestArithmetic(unittest.TestCase):
    def test_pin_travel_hand_case(self):
        # gap 5 + engage 3 + retract-inside 1.5 = 9.5 mm.  Arm radius 10, throw 90 deg -> 2*10*sin(45 deg) = 14.142 mm
        self.assertAlmostEqual(dr.pin_travel(5.0, 3.0, 1.5), 9.5)
        self.assertAlmostEqual(dr.arm_throw(10.0, 90.0), 14.1421356, places=5)
        self.assertAlmostEqual(dr.arm_throw(12.0, 60.0), 12.0, places=9)       # chord of 60 degrees at r = 12 is 12
        self.assertAlmostEqual(A0["travel"], 9.5)
        self.assertGreater(A0["arm_throw"], A0["travel"])
        self.assertAlmostEqual(A0["arm_angle_req"], math.degrees(2 * math.asin(9.5 / 20)), places=6)

    def test_friction_force_hand_case(self):
        r30 = [r for r in A0["force_rows"] if r["m"] == 30.0][0]
        self.assertAlmostEqual(r30["W"], 0.03 * 9.80665, places=6)
        self.assertAlmostEqual(r30["F_lo"], 0.2 * 0.03 * 9.80665, places=6)
        self.assertAlmostEqual(r30["F_hi"], 0.6 * 0.03 * 9.80665 * 2.0, places=6)

    def test_prop_geometry_hand_case(self):
        self.assertAlmostEqual(A0["prop_edge"], 225 / 2 * math.cos(math.pi / 4) + 63.5, places=3)
        self.assertGreaterEqual(A0["hang_margin"], dr.default_params()["prop_margin_mm"])


class TestPage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = (OUT / "index.html").read_text(encoding="utf-8")
        cls.svg = (OUT / "drawing.svg").read_text(encoding="utf-8")

    @staticmethod
    def _lum(hexc):
        h = hexc.lstrip("#")
        if len(h) == 3:
            h = "".join(c * 2 for c in h)
        r, g, b = (int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))
        return 0.2126 * r + 0.7152 * g + 0.0722 * b

    def test_outputs_exist(self):
        for f in ["index.html", "drawing.svg", "parameters.csv"] + [f"{n}.stl" for n in NAMES]:
            self.assertTrue((OUT / f).exists(), f)

    def test_no_light_theme_colours(self):
        for text in (self.html, self.svg):
            low = text.lower()
            self.assertNotRegex(low, r"(?<![-\w])white(?![-\w])")
            self.assertNotRegex(low, r"#fff(fff)?\b")
            self.assertNotRegex(low, r"rgb\(\s*2[0-9][0-9]")
            for m in re.finditer(r"background(?:-color)?\s*:\s*(#[0-9a-f]{3,6})", low):
                self.assertLess(self._lum(m.group(1)), 0.2, m.group(0))
            for m in re.finditer(r"<rect[^>]*fill=\"(#[0-9a-f]{3,6})\"", low):
                self.assertLess(self._lum(m.group(1)), 0.2, m.group(0))
        self.assertIn("color-scheme:dark", self.html)
        self.assertIn("--bg:#14181d", self.html)

    def test_assumed_hardware_is_labelled(self):
        hw = [p for p in dr.PARAMS if p[5]]
        self.assertGreaterEqual(len(hw), 15)
        for p in hw:
            self.assertIn(p[3], ("ASSUMED", "JUDGEMENT"), p[0])
            # a table row for this parameter exists and carries a tag span
            m = re.search(r"<tr><td><code>" + p[0] + r"</code></td><td class='n'>[^<]*</td><td>[^<]*</td><td><span class='tag'>\[(\w+)\]</span>", self.html)
            self.assertIsNotNone(m, p[0])
            self.assertEqual(m.group(1), p[3])
            # and the "measure yours" table lists it
            self.assertRegex(self.html, r"<tr><td><code>" + p[0] + r"</code></td><td class='n'>[\d.eE+-]+</td><td>[^<]*</td><td>" + p[3] + r"</td>")
        self.assertIn("measure yours and re-run", self.html.lower())
        self.assertIn("Nothing here has been printed, fitted or flown", self.html)
        # every parameter-table row has a valid tag
        for m in re.finditer(r"<span class='tag'>\[(\w+)\]</span>", self.html):
            self.assertIn(m.group(1), ft.SOURCES)
        # prose: assumed hardware values (rod, pin, servo, quad, motor diagonal, prop) are followed by a tag
        P = dr.default_params()
        for txt in (f"{P['rod_d_mm']:.2f} mm rod", f"{P['pin_d_mm']:.2f} mm wire", f"{P['motor_diag_mm']:.0f} mm", f"{P['quad_mass_g']:.0f} g"):
            for m in re.finditer(re.escape(txt), self.html):
                tail = self.html[m.end():m.end() + 80]
                self.assertIn("tag", tail, txt + " | " + tail)

    def test_csv_tags(self):
        with (OUT / "parameters.csv").open(encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertTrue(rows)
        for r in rows:
            self.assertIn(r["tag"], ft.SOURCES)
        names = {r["name"] for r in rows}
        for k in dr.PARAM_KEYS:
            self.assertIn(k, names)

    def test_svg_text_inside_page_and_not_overlapping(self):
        vb = re.search(r'viewBox="0 0 (\d+) (\d+)"', self.svg)
        W, H = float(vb.group(1)), float(vb.group(2))
        boxes = []
        for m in re.finditer(r'<text x="([\d.]+)" y="([\d.]+)"[^>]*font-size="(\d+)"[^>]*>(.*?)</text>', self.svg):
            x, y, size, txt = float(m.group(1)), float(m.group(2)), float(m.group(3)), m.group(4)
            anchor = re.search(r'text-anchor="(\w+)"', m.group(0)).group(1)
            w = len(txt) * size * 0.56
            x0 = x if anchor == "start" else x - w / 2
            self.assertGreaterEqual(x0, 0, txt)
            self.assertLessEqual(x0 + w, W, txt)
            self.assertLessEqual(y, H, txt)
            self.assertGreaterEqual(y - size, 0, txt)
            boxes.append((x0, y - size * 0.8, x0 + w, y + size * 0.2, txt))
        for i, a in enumerate(boxes):
            for b in boxes[i + 1:]:
                overlap = a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]
                self.assertFalse(overlap, f"text overlap: {a[4]!r} / {b[4]!r}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
