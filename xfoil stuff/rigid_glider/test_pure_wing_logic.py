"""Logic tests for the pure-wing model with synthetic section data (no XFOIL database needed)."""
import sys
sys.dont_write_bytecode = True
import math
import tempfile
import unittest
from pathlib import Path

import pure_wing_model as m
import pure_wing_study as st

_TMP = tempfile.mkdtemp(prefix="pure_wing_logic_")
SHAPE = m.build_shape(4.5, 20, 0.7, 0, Path(_TMP))
_L2 = None
B, WS = 0.3, 4.0
GRID = [round(0.02 + 0.01 * i, 3) for i in range(119)]


def l2():
    global _L2
    if _L2 is None:
        _L2 = m.build_l2(4.5, 20, 0.7, 0, Path(_TMP))
    return _L2


def weights():
    """K1 = 2 sum(f c A)/(S mac) (cm ~ cl weighting), Kc = 2 sum(c A)/(S mac) (cm constant weighting)."""
    S, mac = SHAPE["area"], SHAPE["mac"]
    k1 = 2 * sum(s["f"] * s["c"] * s["a"] for s in SHAPE["strips"]) / (S * mac)
    kc = 2 * sum(s["c"] * s["a"] for s in SHAPE["strips"]) / (S * mac)
    return k1, kc


class Fn:
    """Synthetic section: cm = fn(re, cl); inviscid lift curve; optional steep flag."""
    inv_deg_per_cl = 9.0
    def __init__(self, fn, dalpha=lambda re, cl: 0.0):
        self.fn, self.dalpha = fn, dalpha
    def alpha_inv(self, cl): return cl * self.inv_deg_per_cl
    def lookup(self, re, cl):
        return dict(alpha=cl * self.inv_deg_per_cl + self.dalpha(re, cl), cd=0.0, cm=self.fn(re, cl), nbranch=1, steep=False)


class LinearCm(unittest.TestCase):
    def roots(self, cm0, k, sm):
        return m.trim_roots(SHAPE, B, Fn(lambda re, cl: cm0 + k * cl), WS, sm, GRID)

    def test_slope_equals_weighted_k_minus_sm_and_flag_flips(self):
        k1, kc = weights()
        sm = 0.05
        # cm0 > 0: root exists only while k_w < SM (then stable); slope = k_w - SM
        for kw, stable in ((0.03, True), (0.04, True)):
            r = self.roots(0.01, kw / k1, sm)
            self.assertEqual(len(r), 1)
            self.assertAlmostEqual(r[0]["dcm_dcl_glide"], kw - sm, places=6)
            self.assertAlmostEqual(r[0]["dcm_dcl_fixed_speed"], kw - sm, places=6)
            self.assertAlmostEqual(r[0]["CL"], 0.01 * kc / (sm - kw), places=5)
            self.assertEqual(r[0]["stable_glide"], stable)
        self.assertEqual(self.roots(0.01, 0.0501 / k1, sm), [])        # k_w just above SM: no positive-CL equilibrium
        # cm0 < 0: root exists only while k_w > SM (then UNSTABLE); flag flips relative to above
        r = self.roots(-0.01, 0.07 / k1, sm)
        self.assertEqual(len(r), 1)
        self.assertAlmostEqual(r[0]["dcm_dcl_glide"], 0.07 - sm, places=6)
        self.assertFalse(r[0]["stable_glide"])
        self.assertFalse(r[0]["stable_fixed"])
        self.assertEqual(self.roots(-0.01, 0.04 / k1, sm), [])

    def test_flag_flips_as_k_crosses_sm_at_fixed_root(self):
        # choose cm0 so the root sits at CL=0.4 for both k: cm0 = (SM - k_w) * 0.4 / Kc
        k1, kc = weights()
        sm = 0.05
        out = []
        for kw in (0.04, 0.06):
            cm0 = (sm - kw) * 0.4 / kc
            r = self.roots(cm0, kw / k1, sm)
            self.assertEqual(len(r), 1)
            self.assertAlmostEqual(r[0]["CL"], 0.4, places=5)
            out.append(r[0]["stable_glide"])
        self.assertEqual(out, [True, False])


class ReDependentCm(unittest.TestCase):
    def test_glide_vs_fixed_derivative(self):
        a, sm = 0.002, 0.03
        k1, kc = weights()
        tb = Fn(lambda re, cl: a * math.log(re))
        r = m.trim_roots(SHAPE, B, tb, WS, sm, GRID)
        self.assertEqual(len(r), 1)
        r = r[0]
        h = 0.02
        # fixed speed: cm independent of CL at frozen V => slope exactly -SM
        self.assertAlmostEqual(r["dcm_dcl_fixed_speed"], -sm, places=9)
        # glide: Re ~ CL^-1/2 so ln Re = const - 0.5 ln CL; Cm_extra = Kc a (const - 0.5 ln CL)
        expected = -sm - 0.5 * a * kc * (math.log(r["CL"] + h) - math.log(r["CL"] - h)) / (2 * h)
        self.assertAlmostEqual(r["dcm_dcl_glide"], expected, places=7)
        diff = r["dcm_dcl_glide"] - r["dcm_dcl_fixed_speed"]
        self.assertAlmostEqual(diff, -0.5 * a * kc / r["CL"], delta=0.02 * abs(diff))   # ~ analytic -a Kc/(2 CL)
        self.assertLess(diff, 0)


class FlagsAndIsGood(unittest.TestCase):
    @staticmethod
    def table(slope, nonmono=False):
        al = [i * 0.5 for i in range(11)]
        cl = [slope * a for a in al]
        if nonmono:
            cl = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.45, 0.4, 0.5, 0.6, 0.7]
        rows = [dict(alpha=a, cl=c, cd=0.02, cm=0.0, xtr_top=1.0) for a, c in zip(al, cl)]
        inv = [dict(alpha=a, cl=0.11 * a) for a in (0, 1, 2, 3, 4, 5)]
        return m.SectionTable({5000: rows, 400000: rows}, inv)   # wide Re range so every strip is covered

    def test_steep_threshold(self):
        # slope per degree just below / above 0.25
        self.assertFalse(self.table(0.249).lookup(30000, 0.3)["steep"])
        self.assertTrue(self.table(0.251).lookup(30000, 0.3)["steep"])

    def test_multibranch_flag_and_branch_choice(self):
        t = self.table(0.2, nonmono=True)
        r = t.lookup(30000, 0.47)
        self.assertEqual(r["nbranch"], 3)                         # 3 crossings of cl=0.47
        hint = t.alpha_inv(0.47)
        self.assertLess(abs(r["alpha"] - hint), 0.6)               # nearest crossing to the inviscid alpha
        self.assertEqual(self.table(0.2).lookup(30000, 0.3)["nbranch"], 1)

    def test_level1_jagged_fraction_counts_steep_area(self):
        self.assertEqual(m.level1(SHAPE, B, self.table(0.249), 0.3, ws=WS)["jagged_fraction"], 0.0)
        r = m.level1(SHAPE, B, self.table(0.251), 0.3, ws=WS)
        self.assertAlmostEqual(r["jagged_fraction"], 1.0, places=9)

    def test_is_good(self):
        base = dict(cl=0.3, dg=-0.02, df=-0.03, cov=0.95, covd=0.95, jag=0.05)
        g = lambda **kw: st.is_good({**base, **kw}, True)
        self.assertTrue(g())
        self.assertFalse(g(cov=0.89)); self.assertTrue(g(cov=0.91))
        self.assertFalse(g(covd=0.89)); self.assertTrue(g(covd=0.91))
        self.assertFalse(g(cl=0.1499)); self.assertTrue(g(cl=0.15))
        self.assertTrue(g(cl=0.6)); self.assertFalse(g(cl=0.6001))
        self.assertFalse(g(jag=0.11)); self.assertTrue(g(jag=0.10))
        self.assertTrue(st.is_good({**base, "jag": 0.9}, False))      # non-clean criterion ignores jaggedness
        self.assertFalse(g(dg=0.001)); self.assertFalse(g(df=0.0))
        self.assertFalse(g(dg=None))


class TwoRoots(unittest.TestCase):
    def test_scan_and_direct_agree(self):
        tb = Fn(lambda re, cl: -0.01 + 0.3 * cl - 0.35 * cl * cl)
        sm = 0.05
        curve = st.build_curve(SHAPE, B, tb, WS)
        scan = st.roots_from_curve(curve, sm)
        direct = m.trim_roots(SHAPE, B, tb, WS, sm, GRID, curve=[(p["cl"], p["cm"], p["cov"]) for p in curve])
        self.assertEqual(len(scan), 2)
        self.assertEqual(len(direct), 2)
        scan.sort(key=lambda r: r["cl"]); direct.sort(key=lambda r: r["CL"])
        for s, d in zip(scan, direct):
            self.assertAlmostEqual(s["cl"], d["CL"], delta=2e-3)
        self.assertGreater(direct[0]["dcm_dcl_glide"], 0); self.assertFalse(direct[0]["stable_glide"])
        self.assertLess(direct[1]["dcm_dcl_glide"], 0); self.assertTrue(direct[1]["stable_glide"])
        self.assertGreater(scan[0]["dg"], 0); self.assertLess(scan[1]["dg"], 0)
        self.assertGreater(scan[0]["df"], 0); self.assertLess(scan[1]["df"], 0)
        for s, d in zip(scan, direct):
            self.assertAlmostEqual(s["dg"], d["dcm_dcl_glide"], delta=0.02)
            self.assertAlmostEqual(s["df"], d["dcm_dcl_fixed_speed"], delta=0.02)


class Level2Synthetic(unittest.TestCase):
    CL = 0.4
    def run_l2(self, dalpha):
        return m.level2(SHAPE, l2(), B, Fn(lambda re, cl: 0.0, dalpha), self.CL, WS)

    def test_uniform_slope_reduction_changes_alpha_not_cm(self):
        base = self.run_l2(lambda re, cl: 0.0)
        red = self.run_l2(lambda re, cl: 0.2 * 9.0 * cl)          # alpha needed 20% larger per cl (uniform lift-slope reduction)
        self.assertTrue(base["converged"] and red["converged"])
        # alpha rises by about the area-weighted section offset (Level 1 value); NOT by 20 % of total alpha,
        # because the induced-angle part of alpha is unaffected by the section slope
        l1 = m.level1(SHAPE, B, Fn(lambda re, cl: 0.0, lambda re, cl: 0.2 * 9.0 * cl), self.CL, ws=WS)
        self.assertAlmostEqual(red["alpha_deg"] - base["alpha_deg"], l1["dalpha_deg"], delta=0.05 * l1["dalpha_deg"])
        # Cm vs CL nearly unchanged (AVL influence operator is not a scalar, so a small residual shift is allowed:
        # measured -3.5e-4, i.e. <2 % of SM*CL=0.02 scale)
        self.assertAlmostEqual(red["cm_extra"], base["cm_extra"], delta=5e-4)

    def test_spanwise_varying_slope_shifts_cm_in_expected_direction(self):
        # slope penalty grows as Re falls (tip, small chord): tip unloads, load moves inboard / forward on a
        # swept-back wing => cm_extra becomes more nose-up (more positive) relative to the uniform case.
        base = self.run_l2(lambda re, cl: 0.0)
        var = self.run_l2(lambda re, cl: 0.2 * 9.0 * cl * (30000.0 / re))
        uni = self.run_l2(lambda re, cl: 0.2 * 9.0 * cl)
        self.assertTrue(var["converged"])
        self.assertGreater(var["cm_extra"] - base["cm_extra"], 0.0)
        self.assertGreater(var["cm_extra"], uni["cm_extra"])


if __name__ == "__main__":
    unittest.main()
