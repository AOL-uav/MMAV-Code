"""Tests for shape_sweep_v1.   py -B rigid_glider/test_shape_sweep_v1.py
Standard library only.  The AVL tests run AVL 3.52 (about 1.5 minutes in all); the output tests use the files in results/runs/rigid_glider/shape_sweep_v1/ if the sweep has been run."""
import hashlib
import itertools
import json
import math
import random
import sys
import tempfile
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import shape_sweep_v1 as sw          # noqa: E402
import straight_smooth_v1 as ss      # noqa: E402
import flat_test as ft               # noqa: E402
import bird_outline_v4_swift as s4   # noqa: E402
import sheet_builder as sb           # noqa: E402

OUT = sw.OUT


# ------------------------------------------------------------------------------------------------ geometry helpers
def seg_intersect(p, q, a, b):
    def o(x, y, z):
        return (y[0] - x[0]) * (z[1] - x[1]) - (y[1] - x[1]) * (z[0] - x[0])
    d1, d2, d3, d4 = o(a, b, p), o(a, b, q), o(p, q, a), o(p, q, b)
    return (d1 * d2 < -1e-12) and (d3 * d4 < -1e-12)


def self_intersections(poly):
    n = len(poly)
    cnt = 0
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if seg_intersect(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]):
                cnt += 1
    return cnt


def mirror_error(poly):
    pts = {(round(x, 5), round(y, 5)) for x, y in poly}
    mir = {(round(x, 5), round(-y, 5)) for x, y in poly}
    miss = [p for p in mir if min(abs(p[0] - q[0]) + abs(p[1] - q[1]) for q in pts) > 1e-3]
    return len(miss)


class TestOutlines(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.specs = sw.outline_specs()

    def test_counts_and_names(self):
        self.assertEqual(len(self.specs), 54)
        self.assertEqual(sum(1 for s in self.specs if s["family"] == "T"), 27)
        self.assertEqual(sum(1 for s in self.specs if s["family"] == "B"), 27)
        self.assertEqual(len({s["name"] for s in self.specs}), 54)

    def test_closed_simple_symmetric(self):
        for s in self.specs + [sp for sp, st in sw.ref_specs()]:
            poly = s["full_outline"]
            self.assertGreater(len(poly), 4, s["name"])
            self.assertGreater(abs(sw.poly_area(poly)), 1000.0, s["name"])                    # closed polygon with a real area
            self.assertNotEqual(poly[0], poly[-1], s["name"])                                  # closed implicitly, no duplicated end point
            self.assertEqual(self_intersections(poly), 0, s["name"])
            self.assertEqual(mirror_error(poly), 0, s["name"])
            xs = [p[1] for p in poly]
            self.assertAlmostEqual(max(xs) - min(xs), 400.0, delta=0.5, msg=s["name"])         # span 400 mm

    def test_swift_a_exact_and_levels(self):
        lv = sw.b_levels_all()
        self.assertEqual(len(set(lv)), 27)
        self.assertIn((0, 0, 0, 0), lv)
        self.assertIs(sw.b_params((0, 0, 0, 0)), s4.GLIDE)
        A = sw.spec_B((0, 0, 0, 0))
        g = s4.build_geometry4(s4.GLIDE)
        self.assertEqual(len(A["full_outline"]), len(g["full_outline"]))
        for (x, y), (u, v) in zip(A["full_outline"], g["full_outline"]):
            self.assertAlmostEqual(x, u * 400.0, places=6)
            self.assertAlmostEqual(y, v * 400.0, places=6)
        for a, b in itertools.combinations((-1, 0, 1), 2):                                      # every pair of factors is balanced (fractional factorial)
            for i, j in itertools.combinations(range(4), 2):
                counts = {}
                for t in lv:
                    counts[(t[i], t[j])] = counts.get((t[i], t[j]), 0) + 1
                self.assertEqual(set(counts.values()), {3}, (i, j))

    def test_b45_outline_in_t_family(self):
        sp = sw.spec_T(4.5, 25.0, 0.5)
        G = ft.wing_geometry(400.0, 4.5)
        hp = sp["half_outline"]
        self.assertAlmostEqual(hp[1][0], G["tipx"], places=9)
        self.assertAlmostEqual(hp[3][0], G["cr"], places=9)
        self.assertAlmostEqual(sp["S_mm2"], G["area"], places=6)

    def test_bed_fit_check(self):
        fits = [sb.fit_search([tuple(p) for p in s["half_outline"]], 250.0, 0.5)["fits"] for s in self.specs]
        self.assertTrue(all(fits))
        big = sb.fit_search([(0, 0), (400, 0), (400, 10), (0, 10)], 250.0, 0.5)                  # a 400 x 10 mm strip is 290 mm along the bed diagonal: it does not fit
        self.assertFalse(big["fits"])


# ------------------------------------------------------------------------------------------------ pure logic
class TestLogic(unittest.TestCase):
    def test_ranks_spearman(self):
        self.assertEqual(sw.avg_ranks([3, 1, 1, 2]), [4.0, 1.5, 1.5, 3.0])
        self.assertAlmostEqual(sw.spearman([1, 2, 3, 4], [10, 20, 30, 40]), 1.0)
        self.assertAlmostEqual(sw.spearman([1, 2, 3, 4], [4, 3, 2, 1]), -1.0)
        self.assertIsNone(sw.spearman([0, 0, 0], [1, 2, 3]))

    def test_pareto(self):
        pts = [(1, 5, "a"), (2, 4, "b"), (2, 3, "c"), (0.5, 6, "d"), (3, 1, "e")]
        self.assertEqual(sorted(sw.pareto_front(pts)), ["a", "b", "d", "e"])

    @staticmethod
    def fake(spiral=-0.05, zeta=0.2, cnb=0.01, tw=0.5, sm=12.0, alpha=4.0, feasible=True, fits=True, fixed=False):
        return dict(modes=dict(spiral=spiral, dr_zeta=zeta, roll=-30.0), deriv=dict(Cnb=cnb), tw30=dict(b=tw), sm_pct=sm, alpha=alpha, trim_feasible=feasible, bed_fits=fits, fixed=fixed)

    def test_judge(self):
        j = sw.judge(self.fake(), 0.035)
        self.assertTrue(j["candidate"] and j["n_pass"] == 5)
        self.assertFalse(sw.judge(self.fake(spiral=0.05), 0.035)["r1"])                         # doubles in 13.9 s: fails the 30 s rule
        self.assertTrue(sw.judge(self.fake(spiral=0.02), 0.035)["r1"])                          # doubles in 34.7 s: passes
        self.assertFalse(sw.judge(self.fake(cnb=-0.001), 0.035)["r2"])                          # oscillation with Cnb < 0 is not credited
        self.assertFalse(sw.judge(self.fake(zeta=0.09), 0.035)["r2"])
        self.assertFalse(sw.judge(self.fake(tw=0.1), 0.035)["r3"])                              # 0.1 < 3 x 0.035 = 0.105
        self.assertTrue(sw.judge(self.fake(tw=0.106), 0.035)["r3"])
        self.assertFalse(sw.judge(self.fake(alpha=5.1), 0.035)["r4"])
        self.assertFalse(sw.judge(self.fake(sm=7.0), 0.035)["r4"])
        self.assertFalse(sw.judge(self.fake(feasible=False), 0.035)["r4"])
        self.assertFalse(sw.judge(self.fake(fits=False), 0.035)["r5"])
        self.assertFalse(sw.judge(self.fake(fixed=True), 0.035)["candidate"])                   # references are never candidates

    def test_stage2_selection(self):
        def mk(name, fam, tw, alpha, ok=True):
            r = self.fake(tw=tw, alpha=alpha, feasible=ok)
            r.update(name=name, family=fam, spec_name=name)
            return r
        res = [mk(f"T{i}", "T", 0.01 * (i + 1), 5.0 + 0.1 * i) for i in range(8)] + [mk(f"B{i}", "B", 0.01 * (i + 1), 4.0 + 0.1 * i) for i in range(7)] + [mk("Bx", "B", 9.0, 0.0, ok=False)]
        chosen, notes = sw.select_stage2(res)
        names = [r["name"] for r in chosen]
        self.assertEqual(len(names), 10)
        self.assertNotIn("Bx", names)                                                         # infeasible trim is outside the filter
        self.assertEqual(sum(1 for r in chosen if r["family"] == "T"), 5)
        self.assertEqual(notes, [])

    def test_top3_rule(self):
        def mk(name, outline, tw, n_pass, cand):
            r = self.fake(tw=tw)
            r.update(name=name, outline=outline, spec_name=outline, rule=dict(candidate=cand, n_pass=n_pass))
            return r
        rows = [mk("a", "o1", 0.5, 5, True), mk("b", "o1", 0.9, 5, True), mk("c", "o2", 0.2, 5, True), mk("d", "o3", 0.8, 4, False), mk("e", "o4", 0.3, 3, False)]
        top = sw.pick_top3(rows)
        self.assertEqual([(l, r["name"]) for l, r in top], [("CANDIDATE", "b"), ("CANDIDATE", "c"), ("NEAR-MISS", "d")])

    def test_newton_step(self):
        L = dict(c0=0.0, ca=0.06, cd=-0.01, m0=0.0, ma=-0.002, md=-0.001)
        a, d = sw.newton_step(L, 0.05, 0.0, 0.0, 0.3)
        cl = L["c0"] + L["ca"] * a + L["cd"] * d
        cm = L["m0"] + L["ma"] * a + L["md"] * d + cl * 0.05
        self.assertAlmostEqual(cl, 0.3, places=9)
        self.assertAlmostEqual(cm, 0.0, places=9)

    def test_rawdon_and_twist_scale(self):
        sp = sw.spec_T(4.5, 25.0, 0.5)
        r0 = sw.rawdon(sp, dict(dih=0.0, fin=("tip", 0.0)), 60.0, 0.3)
        r20 = sw.rawdon(sp, dict(dih=20.0, fin=("tip", 0.0)), 60.0, 0.3)
        self.assertEqual(r0["ss"], 0.0)
        self.assertGreater(r20["eda"], 0.2 * 20.0 * 0.5)                                       # outer 40 % of a tapered wing carries a share of the roll moment above its area share
        self.assertAlmostEqual(r20["ss"], r20["eda"] * (r20["lv"] / 400.0) / 0.3, places=9)
        self.assertAlmostEqual(sw.Z_B, 0.4 / 60.0)


# ------------------------------------------------------------------------------------------------ GIF
class TestGif(unittest.TestCase):
    def test_roundtrip(self):
        random.seed(3)
        fr = []
        c = sw.Canvas(120, 80)
        c.fill_poly([(5, 5), (100, 20), (60, 70)], sw.FILL)
        c.poly_outline([(5, 5), (100, 20), (60, 70)], sw.INK)
        c.text(4, 4, "TWIST 0.123 DEG R1 P", sw.GOOD, 2)
        fr.append(bytes(c.buf))
        n = sw.Canvas(120, 80)
        for _ in range(9000):
            n.px(random.randrange(120), random.randrange(80), random.randrange(16))             # noise: forces dictionary growth and the table-full reset
        fr.append(bytes(n.buf))
        fr.append(bytes(sw.Canvas(120, 80).buf))
        g = sw.gif_bytes(fr, 120, 80, delay_cs=60)
        self.assertEqual(g[:6], b"GIF89a")
        self.assertEqual(g[-1:], b"\x3B")
        w, h, nf, dec, delays = sw.gif_decode(g)
        self.assertEqual((w, h, nf), (120, 80, 3))
        self.assertEqual(delays, [60, 60, 60])
        for a, b in zip(fr, dec):
            self.assertEqual(a, b)

    def test_long_runs_table_reset(self):
        data = bytes((i * 7 + i // 13) % 16 for i in range(60000))
        w, h, nf, dec, _ = sw.gif_decode(sw.gif_bytes([data], 300, 200))
        self.assertEqual(dec[0], data)

    def test_font_covers_frame_text(self):
        for ch in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,:-+=/%()_<>~^*!?|# ":
            self.assertIn(ch, sw._G)
            self.assertEqual(len(sw._G[ch]), 7)
            self.assertTrue(all(len(row) == 5 for row in sw._G[ch]), ch)

    def test_written_gif(self):
        p = OUT / "sweep.gif"
        if not p.is_file():
            self.skipTest("sweep.gif not written yet")
        d = p.read_bytes()
        self.assertEqual(d[:6], b"GIF89a")
        w, h, nf, dec, delays = sw.gif_decode(d)
        runs = json.loads((OUT / "runs.json").read_text(encoding="utf-8"))
        n_ref_s1 = sum(1 for r in runs["runs"] if r["stage"] in ("ref", "1"))
        groups = {(r["outline"], r["feat"]["dih"], str(r["feat"]["fin"])) for r in runs["runs"] if r["stage"] == "2"}
        self.assertEqual(nf, n_ref_s1 + len(groups))
        self.assertEqual(nf, len(delays))
        self.assertTrue(all(abs(x - 60) <= 1 for x in delays))
        self.assertTrue(all(len(f) == w * h for f in dec))
        self.assertGreater(sum(1 for b in dec[0] if b == sw.BG) / (w * h), 0.5)                  # dark background
        self.assertNotEqual(dec[0], dec[-1])
        self.assertEqual(tuple(d[13:16]), sw.PALETTE[0])                                          # palette entry 0 is the dark background


# ------------------------------------------------------------------------------------------------ AVL tests
class TestAvl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.work = Path(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def close(self, a, b, tol, what):
        self.assertLessEqual(abs(a - b), tol * abs(b), f"{what}: {a} vs {b}")

    def reproduce(self, spec, st, cfg):
        mine = sw.evaluate(spec, st, self.work / "mine")
        res = ss.evaluate(cfg, None, self.work / "ss")
        m = ss.mode_run(cfg, res, ss.GLIDE_REF, True, self.work / "ss", "x")
        self.close(mine["deriv"]["Cnb"], res["deriv"]["Cnb"], 0.02, "Cnb")
        self.close(mine["deriv"]["Clb"], res["deriv"]["Clb"], 0.02, "Clb")
        self.close(mine["modes"]["spiral"], m["modes"]["spiral"], 0.02, "spiral eigenvalue")
        self.close(mine["modes"]["dr_zeta"], m["modes"]["dr_zeta"], 0.02, "oscillation damping")
        self.close(mine["alpha"], res["alpha"], 0.02, "trim alpha")
        self.close(mine["V"], res["V"], 0.01, "trim speed")
        return mine

    def test_b45_reference_reproduces_straight_smooth(self):
        mine = self.reproduce(sw.spec_B45_ref(), dict(name="b45", fin=("tip", 0.0), dih=0.0), ss.make_cfg("b45", 0.0, 1.0))
        self.assertAlmostEqual(mine["tw30"]["b"], 0.035, delta=0.002)                           # README of straight_smooth_v1: 0.033 deg at R = 31.7 m

    def test_swift_a_reference_reproduces_straight_smooth(self):
        self.reproduce(sw.spec_SW_ref(), dict(name="sw", fin=None, dih=0.0), ss.make_cfg("sw", 0.0, False))

    def test_trim_targeting_converges(self):
        for spec, st in ((sw.spec_T(4.5, 25.0, 0.5), dict(name="tt", fin=("tip", 0.0), dih=0.0, cl=0.30)),
                         (sw.spec_B((0, 0, 0, 0)), dict(name="tb", fin=None, dih=10.0, cl=0.35))):
            r = sw.evaluate(spec, st, self.work / "trim")
            self.assertTrue(r["converged"], st["name"])
            self.assertAlmostEqual(r["sm_pct"], 12.0, delta=0.05)
            self.assertAlmostEqual(r["cl_trim"], st["cl"], delta=0.002)
            self.assertLess(abs(r["cm_trim"]), 1.5e-4)
            self.assertLess(r["iterations"], 6)
            self.assertTrue(-12.0 < r["gamma"] < -4.0)

    def test_dihedral_and_fin_sign_sanity(self):
        sp = sw.spec_T(4.5, 25.0, 0.5)
        r0 = sw.evaluate(sp, dict(name="d0", fin=None, dih=0.0, cl=0.30), self.work / "feat")
        r2 = sw.evaluate(sp, dict(name="d20", fin=None, dih=20.0, cl=0.30), self.work / "feat")
        self.assertLess(r2["deriv"]["Clb"], r0["deriv"]["Clb"])                                  # tip dihedral raises the dihedral effect (more negative Clb)
        self.assertGreater(r2["tw30"]["b"], 3.0 * r0["tw30"]["b"])
        rf = sw.evaluate(sp, dict(name="f0", fin=("tip", 0.0), dih=0.0, cl=0.30), self.work / "feat")
        self.assertGreater(rf["deriv"]["Cnb"], r0["deriv"]["Cnb"])                               # tip fins add weathercock stability
        self.assertIn("c", rf["asym"])                                                           # fin toe asymmetry exists only with a fin


# ------------------------------------------------------------------------------------------------ frozen section and outputs
class TestOutputs(unittest.TestCase):
    def test_prestated_section_unchanged(self):
        raw = (OUT / "README.md").read_bytes()
        head = raw.split(b"\nRESULTS\n")[0]
        want = (OUT / "prestated_section.sha256").read_text().split()[0]
        self.assertEqual(hashlib.sha256(head).hexdigest(), want)
        self.assertIn(b"BEFORE any grid configuration of the sweep was run", head)

    def test_csv_and_candidates(self):
        p = OUT / "configs.csv"
        if not p.is_file():
            self.skipTest("configs.csv not written yet")
        import csv
        with p.open(encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        self.assertLessEqual(len(rows), sw.CAP_CONFIGS)
        for r in rows:
            parts = sum(int(r[k]) for k in ("rule1_spiral", "rule2_oscillation", "rule3_twist", "rule4_pitch", "rule5_bed"))
            self.assertEqual(parts, int(r["parts_passed"]))
            if r["stage"] != "ref":
                self.assertEqual(int(r["CANDIDATE"]), int(parts == 5))
            else:
                self.assertEqual(int(r["CANDIDATE"]), 0)
            if r["stage"] != "ref" and r["trim_converged"] == "1":
                self.assertAlmostEqual(float(r["static_margin_pct_mac"]), 12.0, delta=0.06)
        self.assertEqual(sum(1 for r in rows if r["stage"] == "1"), 54)

    def test_top3_files(self):
        d = OUT / "sheet_builder_top3"
        if not d.is_dir():
            self.skipTest("no top-3 folder yet")
        for k in (1, 2, 3):
            cfgp = d / f"top{k}.json"
            if cfgp.is_file():
                cfg = sb.load_config(cfgp)
                self.assertEqual(cfg["span_mm"], 400)
                self.assertTrue((d / cfg["source"]["path"]).is_file())


class TestReview(unittest.TestCase):
    def test_gamma_and_band_files(self):
        import csv
        g = OUT / "gamma11_check.csv"
        if not g.is_file():
            self.skipTest("not written yet")
        with g.open(encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        self.assertGreaterEqual(len(rows), 7)
        for r in rows:                                                                         # the rerun at the own gamma reproduces the stored spiral eigenvalue
            self.assertAlmostEqual(float(r["spiral_rerun_own_gamma_per_s"]), float(r["spiral_stored_per_s"]), delta=0.02 * abs(float(r["spiral_stored_per_s"])) + 1e-6)
            self.assertEqual(float(r["gamma_alt_deg"]), -11.0)
        with (OUT / "clb_band.csv").open(encoding="utf-8") as fh:
            band = list(csv.DictReader(fh))
        self.assertTrue(any(b["role"].startswith("robust") for b in band))
        base = float(next(x for x in band if x["config"] == "B45_as_flown")["tw30_b_clb_x1"])
        for b in band:
            self.assertAlmostEqual(float(b["ratio_vs_B45_same_factor_x1"]), float(b["tw30_b_clb_x1"]) / base, delta=0.01 * float(b["ratio_vs_B45_same_factor_x1"]) + 0.06)

    def test_closed_form_matches_stored_twist(self):
        import pickle
        import shape_sweep_v1 as m
        for name in ("B45_as_flown", "SwiftA_as_flown"):
            runs = json.loads((OUT / "runs.json").read_text(encoding="utf-8"))["runs"]
            r = next(x for x in runs if x["name"] == name)
            for k in ("a", "b"):
                self.assertAlmostEqual(m.tw_at(r, r["gamma"])[k], r["tw30"][k], delta=1e-6 * r["tw30"][k] + 1e-9)
            self.assertGreater(m.tw_at(r, r["gamma"], 0.7)["b"], 0.0)

    def test_cancellation_markers(self):
        import csv
        with (OUT / "configs.csv").open(encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        flagged = [r for r in rows if float(r["twist_tol_cancellation_ratio"]) < 0.25]
        self.assertTrue(flagged)
        self.assertTrue(all(r["twist_flag"] == sw.FLAG_TEXT for r in flagged))
        self.assertTrue(all(r["twist_flag"] == "" for r in rows if float(r["twist_tol_cancellation_ratio"]) >= 0.25))
        svg = (OUT / "contact_sheet.svg").read_text(encoding="utf-8")
        n_cells = sum(1 for r in rows if r["stage"] in ("ref", "1") and float(r["twist_tol_cancellation_ratio"]) < 0.25)
        self.assertEqual(svg.count(sw.FLAG_TEXT + "</text>"), n_cells)
        html = (OUT / "index.html").read_text(encoding="utf-8")
        self.assertGreaterEqual(html.count("class='cancel'"), len(flagged))
        for f in ("top1", "top2", "top3", "posthoc"):
            c = json.loads((OUT / "sheet_builder_top3" / f"{f}.json").read_text(encoding="utf-8"))
            self.assertTrue(c["_header"].startswith("NOT a build candidate"))
        self.assertTrue((OUT / "sheet_builder_top3" / "kits" / "README.md").read_text(encoding="utf-8").startswith("NOT a build candidate"))

    def test_marked_gif_frames_differ(self):
        d = (OUT / "sweep.gif").read_bytes()
        w, h, nf, dec, _ = sw.gif_decode(d)
        border = [sum(1 for b in f[:w] if b == sw.BAD) for f in dec]                           # top row of the frame: red border pixels on flagged frames
        self.assertTrue(any(x == w for x in border) and any(x == 0 for x in border))

    def test_no_personal_names_in_generated_text(self):
        for f in ("configs.csv", "index.html", "contact_sheet.svg", "gamma11_check.csv", "clb_band.csv"):
            self.assertNotIn("Kai", (OUT / f).read_text(encoding="utf-8"), f)
        res = (OUT / "README.md").read_bytes().split(b"\nRESULTS\n")[1].decode("utf-8")
        self.assertNotIn("Kai", res)


if __name__ == "__main__":
    unittest.main(verbosity=2)
