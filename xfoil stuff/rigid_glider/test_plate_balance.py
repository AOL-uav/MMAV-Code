"""Unit tests for plate_balance.py.  Run from rigid_glider/:  py -B -m unittest test_plate_balance -v   (writes to a temporary folder only)"""
import sys
sys.dont_write_bytecode = True
import csv
import math
import tempfile
import unittest
from pathlib import Path

import plate_balance as pb


def synthetic(dip=False, step=0.25, cm0=0.0):
    """Increasing sweep with exactly linear CP travel x_cp = 0.15 + 0.02 alpha (optionally with a forward dip and a zero-lift moment cm0), CD = 0.01 + 0.05 CL^2."""
    pts = []
    n = int(round(16 / step))
    for i in range(n + 1):
        a = i * step
        cl = 0.08 * a if a <= 10 else 0.8 - 0.04 * (a - 10)
        cd = 0.01 + 0.05 * cl * cl
        xcp = 0.15 + 0.02 * a
        if dip and 4.0 <= a <= 4.5:
            xcp -= 0.03 * (a - 4.0) / 0.5
        if dip and 4.5 < a < 4.75:
            xcp -= 0.03
        cn = cl * math.cos(math.radians(a)) + cd * math.sin(math.radians(a))
        cm = (pb.X_REF - xcp) * cn + cm0
        pts.append(dict(alpha=a, CL=cl, CD=cd, CM=cm, Re=60000.0, AR=3, taper=1.0, run="syn", sweep="increasing", seq=i))
    return pb.annotate(pts)


class FormulaTests(unittest.TestCase):
    def test_cp_formula_hand_case(self):
        cn, xcp = pb.cn_xcp(0.0, 0.3, 0.02, 0.01)
        self.assertAlmostEqual(cn, 0.3)
        self.assertAlmostEqual(xcp, 0.25 - 0.01 / 0.3)                    # 0.21667
        a = 10.0
        cn, xcp = pb.cn_xcp(a, 0.5, 0.1, -0.02)
        self.assertAlmostEqual(cn, 0.5 * math.cos(math.radians(a)) + 0.1 * math.sin(math.radians(a)))
        self.assertAlmostEqual(xcp, 0.25 + 0.02 / cn)

    def test_small_normal_force_skipped(self):
        self.assertIsNone(pb.cn_xcp(0.0, 0.04, 0.02, 0.01)[1])
        self.assertIsNone(pb.cn_xcp(0.0, -0.049, 0.02, 0.01)[1])
        self.assertIsNotNone(pb.cn_xcp(0.0, 0.051, 0.02, 0.01)[1])

    def test_lsq(self):
        s, i, rms, mx = pb.lsq([0, 1, 2, 3], [1, 3, 5, 7])
        self.assertAlmostEqual(s, 2.0)
        self.assertAlmostEqual(i, 1.0)
        self.assertAlmostEqual(rms, 0.0)

    def test_zero_lift_offset_worked_sizes(self):
        self.assertAlmostEqual(pb.cp_shift(-0.008, 0.2), -0.04)           # 4% of chord at CN 0.2
        self.assertAlmostEqual(pb.cp_shift(-0.008, 0.4), -0.02)
        self.assertAlmostEqual(pb.cp_shift(0.002, 0.2), 0.01)
        # the shift is exactly what removing the offset does to the computed x_cp
        cm0, cl, cd, a = -0.008, 0.2, 0.02, 0.0
        raw = pb.cn_xcp(a, cl, cd, 0.01 + cm0)
        rem = pb.cn_xcp(a, cl, cd, 0.01)
        self.assertAlmostEqual(rem[1] - raw[1], pb.cp_shift(cm0, raw[0]))


class SyntheticTests(unittest.TestCase):
    def test_linear_cp_recovers_best_and_band(self):
        t = pb.trim_reading(synthetic())
        self.assertIsNotNone(t)
        cl_star = math.sqrt(0.01 / 0.05)
        ld_max = cl_star / (0.01 + 0.05 * cl_star ** 2)
        self.assertAlmostEqual(t["best_ld"], ld_max, delta=0.1)
        self.assertAlmostEqual(t["best_xcg"], 0.15 + 0.02 * (cl_star / 0.08), delta=0.006)
        self.assertEqual(t["nonmono"], [])
        for frac, key in ((0.9, "band90"), (0.75, "band75")):
            target = frac * ld_max
            disc = 1 - 4 * 0.05 * 0.01 * target * target
            lo_cl = (1 - math.sqrt(disc)) / (2 * 0.05 * target)
            hi_cl = (1 + math.sqrt(disc)) / (2 * 0.05 * target)
            lo, hi = 0.15 + 0.02 * lo_cl / 0.08, 0.15 + 0.02 * min(hi_cl, 0.8) / 0.08
            self.assertAlmostEqual(t[key][0], lo, delta=0.006)
            self.assertAlmostEqual(t[key][1], hi, delta=0.006)
        self.assertAlmostEqual(t["alpha_clmax"], 10.0, delta=0.01)
        self.assertAlmostEqual(t["xcg_stalled_trim"], 0.15 + 0.02 * 10, delta=0.006)

    def test_offsets_from_fitted_ac(self):
        pts = synthetic()
        fit = pb.fit_low_angle(pts)
        tr = pb.trim_reading(pts)
        o = pb.offsets(tr, fit)
        self.assertAlmostEqual(o["best"], 100 * (tr["best_xcg"] - fit["ac"]))
        self.assertAlmostEqual(o["b90_lo"], 100 * (tr["band90"][0] - fit["ac"]))

    def test_unstable_stretch_flagged_and_band_not_across_it(self):
        t = pb.trim_reading(synthetic(dip=True))
        self.assertTrue(t["major_nonmono"])
        for key in ("band90", "band75"):
            lo, hi = t[key]
            for x, tr in t["tmap"]:
                if lo - 1e-9 <= x <= hi + 1e-9:
                    self.assertIsNotNone(tr, (key, x))
                    self.assertFalse(tr["stalled"])

    def test_fit_range_and_residual(self):
        f = pb.fit_low_angle(synthetic())
        self.assertGreater(f["n"], 10)
        self.assertLess(f["rms_cl"], 1e-9)
        self.assertAlmostEqual(f["cl_alpha_per_deg"], 0.08)
        self.assertAlmostEqual(f["alpha0_deg"], 0.0, places=6)

    def test_zero_lift_moment_fit_and_removal(self):
        pts = []
        for i in range(0, 61):                                                  # exactly linear: CL = 0.08 alpha, CM = 0.03 CL - 0.006
            a = -4 + i * 0.25
            cl = 0.08 * a
            pts.append(dict(alpha=a, CL=cl, CD=0.02, CM=0.03 * cl - 0.006, Re=1.0, AR=3, taper=1.0, run="syn", sweep="increasing", seq=i))
        pts_raw = pb.annotate(pts)
        f = pb.fit_low_angle(pts_raw)
        self.assertAlmostEqual(f["cm_at_zero_lift"], -0.006, places=9)         # the intercept recovers the planted offset
        self.assertAlmostEqual(f["ac"], 0.25 - 0.03, places=9)
        rem = pb.cm0_shift(pts, f["cm_at_zero_lift"])
        f2 = pb.fit_low_angle(rem)
        self.assertAlmostEqual(f2["cm_at_zero_lift"], 0.0, places=9)
        self.assertAlmostEqual(f2["ac"], f["ac"], places=9)                  # a slope-based quantity does not move when an offset is removed

    def test_resolved_quantities_synthetic(self):
        pts = synthetic()
        q = pb.resolved(pts, pb.fit_low_angle(pts))
        self.assertAlmostEqual(q["onset_alpha"], 5.0, delta=0.05)           # baseline x_cp 0.23 (median over 2-6 deg) + 0.02
        cl9 = 0.9 * 0.8
        self.assertAlmostEqual(q["near_ld"], cl9 / (0.01 + 0.05 * cl9 ** 2), delta=0.2)
        self.assertAlmostEqual(q["clmax"], 0.8)
        self.assertAlmostEqual(q["xcp_clmax"], 0.35, delta=0.002)
        self.assertGreaterEqual(q["plateau_alpha_span"], 0.0)


class RealDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="plate_balance_")
        cls.out = Path(cls.tmp.name)
        cls.R = pb.run(cls.out)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_counts_match_csv(self):
        self.assertEqual(len(self.R["rows"]), 3482)
        self.assertEqual(len(self.R["runs"]), 40)
        with (self.out / "step1_cp_table.csv").open(encoding="utf-8") as f:
            n = sum(1 for _ in csv.DictReader(f))
        self.assertEqual(n, 3482)
        self.assertEqual(len({(r["AR"], r["taper"]) for r in self.R["rows"]}), 10)
        self.assertEqual(sum(len(r["raw"]["points"]) for r in self.R["results"]), 3482)
        self.assertEqual(sum(len(r["rem"]["points"]) for r in self.R["results"]), 3482)       # both versions kept
        with (self.out / "step2_fits.csv").open(encoding="utf-8") as f:
            self.assertGreaterEqual(sum(1 for _ in csv.DictReader(f)), 40)

    def test_hand_check_ar3_taper1_re60k(self):
        r = next(x for x in self.R["results"] if x["wing"] == "AR3_taper1" and abs(x["Re"] - 60000) < 100 and x["sweep"] == "increasing")
        p = min(r["raw"]["points"], key=lambda q: abs(q["alpha"] - 4.54))
        self.assertAlmostEqual(p["alpha"], 4.54)
        self.assertAlmostEqual(p["xcp"], 0.2055, delta=0.001)               # README reference: quarter chord, normal force
        self.assertAlmostEqual(p["LD"], 8.39, delta=0.02)
        self.assertAlmostEqual(r["raw"]["trim"]["best_ld"], 8.4, delta=0.1)
        self.assertAlmostEqual(r["fit"]["ac"], 0.2017, delta=0.01)
        self.assertAlmostEqual(r["raw"]["res"]["near_ld"], 5.5, delta=0.2)  # L/D at 90% of maximum lift, quoted on the page

    def test_zero_lift_moment_reported_and_removed(self):
        missing = [r for r in self.R["results"] if r["fit"] is None]
        self.assertEqual(len(missing), 1)                                       # AR4 taper 1 Re 120k decreasing leg: 17 points, 13-29 deg, nothing in the fit range
        self.assertEqual((missing[0]["wing"], missing[0]["sweep"]), ("AR4_taper1", "decreasing"))
        for r in self.R["results"]:
            if r["fit"] is None:
                continue
            f2 = pb.fit_low_angle(r["rem"]["points"])
            self.assertAlmostEqual(f2["cm_at_zero_lift"], 0.0, places=8)
            self.assertAlmostEqual(f2["ac"], r["fit"]["ac"], places=8)
        cm0 = [r["cm0"] for r in self.R["results"]]
        self.assertLess(min(cm0), -0.005)                                      # the AR 2 runs show roughly -0.004 to -0.008
        with (self.out / "step2_fits.csv").open(encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertIn("CM_at_zero_lift (should be 0)", rows[0])
        self.assertIn("x_cp_change_if_removed_at_CN0.2", rows[0])
        page = (self.out / "index.html").read_text(encoding="utf-8")
        for needle in ("symmetric flat plate should give <b>zero moment at zero lift</b>", "CM0 / CN", "4% of chord", "CN 0.2", "CN 0.4"):
            self.assertIn(needle, page)

    def test_nonmonotonic_cp_is_flagged_on_aspect_ratio_2(self):
        rs = [r for r in self.R["results"] if r["wing"] == "AR2_taper0.5" and r["sweep"] == "increasing"]
        self.assertTrue(any(r["raw"]["trim"]["major_nonmono"] for r in rs))

    def test_bands_only_where_trim_exists(self):
        for r in self.R["results"]:
            for ver in ("raw", "rem"):
                t = r[ver]["trim"]
                if not t:
                    continue
                for key in ("band90", "band75"):
                    lo, hi = t[key]
                    self.assertLessEqual(lo, t["best_xcg"] + 1e-9)
                    self.assertGreaterEqual(hi, t["best_xcg"] - 1e-9)
                    for x, tr in t["tmap"]:
                        if lo - 1e-9 <= x <= hi + 1e-9:
                            self.assertIsNotNone(tr)
                            self.assertFalse(tr["stalled"])
                self.assertLessEqual(t["band90"][1] - t["band90"][0], t["band75"][1] - t["band75"][0] + 1e-9)

    def test_decreasing_sweep_reported_separately(self):
        self.assertEqual({r["sweep"] for r in self.R["results"]}, {"increasing", "decreasing"})
        with (self.out / "step3_summary.csv").open(encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual({r["sweep"] for r in rows}, {"increasing", "decreasing"})
        self.assertEqual({r["version"] for r in rows}, {"raw", "offset removed"})          # both versions kept

    def test_all_csvs_and_page_exist(self):
        for f in ("step1_cp_table.csv", "step2_fits.csv", "step2b_resolvability.csv", "step3_summary.csv", "step3_NOT_RESOLVED.csv", "step4_offsets_from_ac.csv", "step5_trends.csv",
                  "step6_if_it_carries_over.csv", "index.html"):
            self.assertTrue((self.out / f).is_file(), f)
        self.assertFalse((self.out / "step3_trim.csv").exists())

    def test_summary_no_longer_exposes_band_rows(self):
        for row in self.R["summary"]:
            for k in row:
                self.assertNotIn("band", k.lower())
                self.assertNotIn("best", k.lower())
        with (self.out / "step3_summary.csv").open(encoding="utf-8") as f:
            header = next(csv.reader(f))
        self.assertFalse([h for h in header if "band" in h.lower() or "best" in h.lower()])
        with (self.out / "step3_NOT_RESOLVED.csv").open(encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertTrue(all("NOT RESOLVED" in r["note"] for r in rows))
        with (self.out / "step5_trends.csv").open(encoding="utf-8") as f:
            self.assertFalse([h for h in next(csv.reader(f)) if "band" in h.lower() or "best" in h.lower()])
        page = (self.out / "index.html").read_text(encoding="utf-8")
        first = {k: page.index(k) for k in ("90% band", "75% band", "best-L/D balance point") if k in page}
        marker = page.index("7. How well resolved")
        for k, i in first.items():
            self.assertGreater(i, marker, k)                                    # bands and the best-L/D point only appear in the resolvability / not-resolved section
        self.assertIn("NOT RESOLVED BY THIS DATA", page)
        # headline text leads with the resolved quantities
        lead = page[:page.index("1. What this is")]
        self.assertIn("aerodynamic centre", lead)
        self.assertNotIn("band", lead.lower().replace("nothing", ""))

    def test_resolvability_evidence(self):
        rs, st = self.R["resrows"], self.R["stats"]
        self.assertTrue(rs)
        for qn in ("ac_pct", "xcp_clmax_offset", "onset_alpha", "near_ld", "best", "b90_lo", "b90_hi", "b75_lo", "b75_hi", "front"):
            self.assertIn(qn, st)
        self.assertAlmostEqual(st["ac_pct"]["move_max"], 0.0, places=6)         # the slope-based centre does not move when the offset is removed
        self.assertAlmostEqual(st["near_ld"]["move_max"], 0.0, places=6)         # L/D does not depend on the moment
        self.assertGreater(st["best"]["move_max"], 1.0)                          # the balance-point quantities do move
        self.assertGreater(st["b90_lo"]["move_max"], 1.0)
        # the AR 2 taper 0.5 Re 120k example named in the review: best offset differs between up and down legs
        r = [x for x in rs if x["wing"] == "AR2_taper0.5" and abs(x["Re"] - 120000) < 100 and x["quantity"] == "best"]
        self.assertTrue(r)
        self.assertGreater(r[0]["updown_raw"], 1.0)

    def test_physical_reading_paragraph_numbers_are_from_tables(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        i = page.index("What the data supports")
        para = page[i:page.index("1. What this is")]
        self.assertIn("unswept, 4.3%-thick plates at Reynolds number 60,000 and above", para)
        r = next(x for x in self.R["results"] if x["wing"] == "AR3_taper1" and abs(x["Re"] - 60000) < 100 and x["sweep"] == "increasing")
        self.assertIn(f"L/D is {r['raw']['res']['near_ld']:.1f}", para)
        self.assertIn(f"aft travel starts at {r['raw']['res']['onset_alpha']:.1f} degrees", para)

    def test_nearest_wings(self):
        tested = sorted({(r["AR"], r["taper"]) for r in self.R["trends"]})
        n45 = pb.nearest_wings(4.5, 0.5, tested)
        self.assertEqual(n45[0], (4, 0.5))
        n60 = pb.nearest_wings(6.0, 0.5, tested)
        self.assertIn((5, 1.0), n60)
        self.assertEqual(len(n60), pb.N_NEAREST)

    def test_carry_over_reduced_to_two_rows_per_specimen(self):
        c = self.R["carry"]
        self.assertEqual(len(c), 2 * len(pb.SPECIMENS))
        for spec in pb.SPECIMENS:
            rs = [r for r in c if r["spec"] == spec]
            self.assertEqual(len(rs), 2)
            self.assertIn("aerodynamic centre", rs[0]["quantity"])
            self.assertIn("near-stall", rs[1]["quantity"])
            for r in rs:
                self.assertIsInstance(r["mm_lo"], int)                          # whole millimetres
                self.assertIsInstance(r["mm_hi"], int)
                self.assertLessEqual(r["mm_lo"], r["mm_hi"])
                for must in ("25 deg sweep", "tip fins", "spar rib", "tested minimum", "versus 4.3% tested"):
                    self.assertIn(must, r["not_tested"])
            # row 1 is the neutral point widened forward by the measured bias (0 to ~5 % MAC)
            r1 = rs[0]
            self.assertLessEqual(r1["mm_lo"], r1["np_mm"])
            self.assertGreaterEqual(r1["mm_lo"], r1["np_mm"] - 0.06 * r1["mac_mm"])
            self.assertGreater(rs[1]["mm_hi"], r1["mm_hi"])                      # near-stall trim ends aft of the low-angle aerodynamic centre
        page = (self.out / "index.html").read_text(encoding="utf-8")
        a, b = page.index("8. IF IT CARRIES OVER"), page.index("9. Method notes")
        self.assertIn("not a CG setting", page[a:b])
        self.assertIn("does not replace the CG sweep", page[a:b])
        self.assertGreater(page[a:b].count("EXTRAPOLATED"), 6)
        self.assertNotIn("EXTRAPOLATED</span>", page[:a])
        for tagname in pb.TAGS:
            self.assertIn(tagname.split(" (")[0], page)

    def test_step6_csv_tags(self):
        with (self.out / "step6_if_it_carries_over.csv").open(encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 6)
        self.assertTrue(all("EXTRAPOLATED" in r["tag"] and r["NOT_TESTED"] and "NOT a CG setting" in r["NOTE"] for r in rows))

    def test_wording_requirements(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        s1 = page[page.index("1. What this is"):page.index("2. The zero-lift moment")]
        self.assertIn("static moment balance only", s1)
        self.assertIn("it does not show which balance points glide", s1)
        s4 = page[page.index("4. Where the centre of pressure goes"):page.index("5. Offsets from each")]
        self.assertIn("sit inside the zero-lift-offset error and the up/down repeatability error", s4)
        self.assertIn("no positive-lift trim exists ahead of x", s4)
        self.assertIn("depends on the zero-lift offset", s4)
        s5 = page[page.index("5. Offsets from each"):page.index("6. Trends")]
        self.assertIn("applies only to the slope-based aerodynamic centre and to the first-lift-peak point", s5)
        s9 = page[page.index("9. Method notes"):]
        self.assertIn("4.6%", s9)
        self.assertIn("zero-lift moment", s9)

    def test_default_output_is_new_folder(self):
        self.assertEqual(pb.OUT_DEFAULT.name, "plate_balance")
        self.assertNotIn("flat_test", pb.OUT_DEFAULT.as_posix())


def curve(f, step=0.5, amax=24.0):
    """Annotated increasing sweep with CL = f(alpha), CD small, x_cp = 0.25 (CM = 0)."""
    pts = []
    for i in range(int(round(amax / step)) + 1):
        a = i * step
        cl = f(a)
        pts.append(dict(alpha=a, CL=cl, CD=0.02 + 0.05 * cl * cl, CM=0.0, Re=60000.0, AR=4, taper=1.0, run="syn", sweep="increasing", seq=i))
    return pb.annotate(pts)


class FirstPeakTests(unittest.TestCase):
    def peak(self, f):
        br, i = pb.trim_branch(curve(f))
        return br[i]["alpha"], pb.first_peak(br)[1]

    def test_true_peak_with_fall(self):
        a, kind = self.peak(lambda a: 0.07 * a if a <= 11 else 0.77 - 0.03 * (a - 11))
        self.assertAlmostEqual(a, 11.0, delta=0.6)
        self.assertEqual(kind, "peak")

    def test_plateau_that_creeps_up_is_flattening_and_not_the_late_maximum(self):
        f = lambda a: 0.07 * a if a <= 9 else (0.63 + 0.0004 * (a - 9) if a <= 13 else 0.632 + 0.006 * (a - 13))
        a, kind = self.peak(f)
        self.assertLess(a, 11.0)                                                  # the knee, not the 19-20 deg maximum
        self.assertEqual(kind, "flattening")
        br, _ = pb.trim_branch(curve(f))
        self.assertGreater(br[pb.global_max(br)]["alpha"], 18.0)                  # the old definition lands on the plateau

    def test_no_knee_falls_back_to_global_maximum(self):
        a, kind = self.peak(lambda a: 0.05 * a)
        self.assertEqual(kind, "none (global maximum)")
        self.assertAlmostEqual(a, pb.CLMAX_ALPHA_LIMIT, delta=0.6)

    def test_second_higher_hump_is_ignored(self):
        f = lambda a: 0.07 * a if a <= 10 else (0.7 - 0.02 * (a - 10) if a <= 15 else 0.6 + 0.04 * (a - 15))
        a, kind = self.peak(f)
        self.assertAlmostEqual(a, 10.0, delta=0.6)


class ReviewDirectiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="plate_balance_rev_")
        cls.out = Path(cls.tmp.name)
        cls.R = pb.run(cls.out)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def rows(self, name):
        with (self.out / name).open(encoding="utf-8") as f:
            return list(csv.DictReader(f))

    def test_summary_has_alpha_first_peak_column(self):
        rows = self.rows("step3_summary.csv")
        for col in ("alpha_first_peak_deg", "first_peak_kind", "alpha_global_max_to_20deg", "AC_ahead_of_quarter_chord_pct", "Re", "wing", "AR"):
            self.assertIn(col, rows[0])                                                  # names other pages read
        for r in rows:
            for old in ("CLmax", "alpha_CLmax", "LD_at_CLmax", "xcp_at_CLmax_pct_chord", "xcp_at_CLmax_offset_from_AC_pct_MAC"):
                self.assertNotIn(old, r)                                                  # renamed to first-peak names, no duplicate columns
            self.assertEqual(r["CL_first_peak"], r["CL_first_peak"])
            self.assertIn(r["first_peak_kind"], ("peak", "flattening", "none (global maximum)"))
            self.assertLessEqual(float(r["alpha_first_peak_deg"]), float(r["alpha_global_max_to_20deg"]) + 1e-9)
        ar4 = [r for r in rows if r["wing"] == "AR4_taper1" and abs(float(r["Re"]) - 60000) < 100 and r["sweep"] == "increasing" and r["version"] == "raw"]
        self.assertTrue(ar4)
        self.assertLess(float(ar4[0]["alpha_first_peak_deg"]), 14.0)
        self.assertGreater(float(ar4[0]["alpha_global_max_to_20deg"]), 18.0)

    def test_aft_travel_onset_resolved_only_for_ar3_and_up(self):
        rr = [r for r in self.R["resrows"] if r["quantity"] == "onset_alpha"]
        self.assertTrue(rr)
        for r in rr:
            ar = int(r["wing"].split("_")[0][2:])
            self.assertEqual(r["resolved"], ar >= 3, r["wing"])
        st = self.R["stats"]
        self.assertGreater(st["onset_alpha_AR2"]["move_max"], 3.0)                 # AR 2 moves by several degrees when the offset is removed
        self.assertLess(st["onset_alpha"]["move_max"], 1.5)                        # AR 3 and up by about half a degree or less
        for r in self.rows("step2b_resolvability.csv"):
            if r["quantity"] == "onset_alpha" and r["wing"].startswith("AR2"):
                self.assertEqual(r["resolved"], "NO")
        for r in self.rows("step3_summary.csv"):
            self.assertEqual(r["aft_travel_onset_resolved"] == "yes", r["wing"][2] != "2", r["wing"])

    def test_lead_paragraph_uses_resolved_wings_and_gives_angles(self):
        page = (self.out / "index.html").read_text(encoding="utf-8")
        lead = page[page.index("What the data supports"):page.index("1. What this is")]
        ons = sorted(r["raw"]["res"]["onset_alpha"] for r in self.R["results"] if r["sweep"] == "increasing" and r["AR"] >= 3 and r["raw"]["res"]["onset_alpha"] is not None)
        pk = sorted(r["raw"]["res"]["alpha_clmax"] for r in self.R["results"] if r["sweep"] == "increasing" and r["AR"] >= 3 and r["raw"]["res"])
        self.assertIn(f"median angle of {ons[len(ons) // 2]:.1f} degrees", lead)
        self.assertIn(f"median {pk[len(pk) // 2]:.1f} degrees", lead)
        self.assertIn("offset-dependent and not resolved", lead)
        self.assertNotIn("close to the angle of maximum lift", lead)

    def test_carry_over_second_row_follows_first_peak(self):
        old_hi = {"F45_span250": 50, "F60_span250": 46, "B45_span400": 80}                  # upper ends before the first-peak definition (global maximum up to 20 deg)
        n = 0
        for c in self.R["carry"]:
            if c["quantity"].startswith("where near-stall"):
                n += 1
                self.assertLessEqual(c["mm_hi"], old_hi[c["spec"]])
                if c["spec"] == "F60_span250":
                    self.assertLess(c["mm_hi"], 46)                                          # the AR 5 point near 19.8 deg no longer sets the limit
        self.assertEqual(n, 3)


if __name__ == "__main__":
    unittest.main()
