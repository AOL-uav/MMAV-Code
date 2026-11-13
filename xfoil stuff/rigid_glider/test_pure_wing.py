import sys
sys.dont_write_bytecode = True
import json
import tempfile
import unittest
from pathlib import Path

import pure_wing_model as m

_TMP = tempfile.mkdtemp(prefix="pure_wing_test_")
_SHAPE = m.build_shape(4.5, 20, 0.7, 0, Path(_TMP))
_L2 = None


def l2():
    global _L2
    if _L2 is None:
        _L2 = m.build_l2(4.5, 20, 0.7, 0, Path(_TMP))
    return _L2


GRID = [round(0.02 + 0.01 * i, 3) for i in range(110)]


def rows(alphas, clfun, cmfun=lambda a: 0.0):
    return [dict(alpha=a, cl=clfun(a), cd=0.02, cm=cmfun(a)) for a in alphas]


class AvlShape(unittest.TestCase):
    def test_linear_and_consistent(self):
        s = _SHAPE
        self.assertLess(s["cla_lin_err"], 0.02)
        self.assertLess(s["strip_f_dev"], 0.02)
        self.assertAlmostEqual(s["cm0"], 0.0, places=4)
        # Cm/CL about xref must equal (xref - xnp)/MAC
        self.assertAlmostEqual(s["cm_over_cl_xref"], s["cma_over_cla_check"], delta=2e-3)
        # strip loading integrates to CL: sum(f A)*2 / S = 1
        tot = 2 * sum(t["f"] * t["a"] for t in s["strips"]) / s["area"]
        self.assertAlmostEqual(tot, 1.0, delta=0.02)


class ZeroSectionReducesToAvl(unittest.TestCase):
    def test_level1_zero_cm_gives_cl_zero_trim(self):
        tb = m.ZeroSection()
        for sm in (0.025, 0.1, 0.2):
            self.assertEqual(m.trim_roots(_SHAPE, 0.3, tb, 4.0, sm, GRID), [])   # no positive-CL root
            self.assertAlmostEqual(m.avl_only_trim(_SHAPE, sm), 0.0, places=4)
        r = m.level1(_SHAPE, 0.3, tb, 0.4, ws=4.0)
        self.assertEqual(r["cm_visc"], 0.0)
        self.assertEqual(r["dalpha_deg"], 0.0 * r["dalpha_deg"])

    def test_level2_zero_section_equals_avl(self):
        tb = m.ZeroSection()
        r = m.level2(_SHAPE, l2(), 0.3, tb, 0.4, 4.0)
        self.assertTrue(r["converged"])
        self.assertAlmostEqual(r["cm_extra"], 0.0, delta=2e-4)
        self.assertAlmostEqual(r["alpha_deg"], 0.4 / (l2()["a_cl"]), delta=1e-6)


class ConstantCm(unittest.TestCase):
    def test_trim_cl_is_cm_integral_over_sm(self):
        cm0, sm, b = 0.01, 0.05, 0.3
        tb = m.ConstCmSection(cm0)
        integral = 2 * sum(cm0 * s["c"] * s["a"] for s in _SHAPE["strips"]) / (_SHAPE["area"] * _SHAPE["mac"])
        roots = m.trim_roots(_SHAPE, b, tb, 4.0, sm, GRID)
        self.assertEqual(len(roots), 1)
        self.assertAlmostEqual(roots[0]["CL"], integral / sm, places=5)
        self.assertAlmostEqual(integral, cm0, delta=0.01 * cm0)         # MAC definition: sum c^2 dy = S MAC
        self.assertAlmostEqual(roots[0]["dcm_dcl_glide"], -sm, places=5)  # constant cm adds no slope
        self.assertTrue(roots[0]["stable_glide"] and roots[0]["valid"])

    def test_negative_cm_gives_no_positive_trim(self):
        self.assertEqual(m.trim_roots(_SHAPE, 0.3, m.ConstCmSection(-0.01), 4.0, 0.05, GRID), [])


class InterpolationRefuses(unittest.TestCase):
    def setUp(self):
        al = [0, 0.5, 1, 1.5, 2, 3.5, 4, 4.5, 5]              # gap 2 -> 3.5 (1.5 deg)
        inv = rows([0, 1, 2, 3, 4, 5], lambda a: 0.11 * a)
        mk = lambda k: rows(al, lambda a: 0.1 * a * k, lambda a: 0.002 * a)
        self.t = m.SectionTable({20000: mk(1.0), 40000: mk(1.0)}, inv)
        self.t_gap = m.SectionTable({20000: mk(1.0), 40000: rows([0, 0.5, 1, 1.5, 2], lambda a: 0.1 * a)}, inv)

    def test_in_range(self):
        r = self.t.lookup(30000, 0.1)
        self.assertAlmostEqual(r["alpha"], 1.0, places=6)
        self.assertAlmostEqual(r["cm"], 0.002, places=6)

    def test_refuses_gap_over_1deg(self):
        self.assertIsNone(self.t.lookup(30000, 0.27))     # cl in the 2..3.5 deg gap (0.2..0.35)

    def test_refuses_re_out_of_range(self):
        self.assertIsNone(self.t.lookup(15000, 0.1))
        self.assertIsNone(self.t.lookup(50000, 0.1))

    def test_refuses_when_one_bracketing_re_lacks_the_cl(self):
        self.assertIsNone(self.t_gap.lookup(30000, 0.4))  # 40k polar has no cl>=0.2

    def test_odd_symmetry(self):
        a, b = self.t.lookup(30000, 0.1), self.t.lookup(30000, -0.1)
        self.assertAlmostEqual(a["alpha"], -b["alpha"], places=9)
        self.assertAlmostEqual(a["cm"], -b["cm"], places=9)
        self.assertEqual(a["cd"], b["cd"])


class ReproGate(unittest.TestCase):
    def test_stored_vs_regenerated(self):
        p = m.OUT / "repro_gate.json"
        if not p.is_file():
            self.skipTest("run pure_wing_polars.py repro first")
        d = json.loads(p.read_text())
        self.assertEqual(d["common_points"], d["stored_points"])
        self.assertLessEqual(max(d["max_abs_diff"].values()), 1e-9)


if __name__ == "__main__":
    unittest.main()
