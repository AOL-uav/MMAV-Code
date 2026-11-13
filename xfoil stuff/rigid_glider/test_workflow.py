"""Small geometry, polar, and actual AVL smoke checks for the baseline workflow."""

import json
import math
import tempfile
import unittest
from pathlib import Path

from avl import read_strips, run_avl
from geometry import FlightCondition, Wing
from polars import PolarCoverageError, SectionPolars
from study_geometry import deck as study_deck


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((Path(__file__).with_name("baseline.json")).read_text(encoding="utf-8"))


class BaselineChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wing = Wing(**CONFIG["wing"])
        cls.flight = FlightCondition(**CONFIG["flight"])

    def test_geometry_and_loading(self):
        wing = self.wing
        self.assertAlmostEqual(wing.area, wing.span_m**2 / wing.ar)
        self.assertAlmostEqual(wing.area, wing.span_m * (wing.root_chord + wing.tip_chord) / 2)
        self.assertAlmostEqual(wing.tip_chord, wing.taper * wing.root_chord)
        self.assertAlmostEqual(self.flight.reference_mass(wing) * self.flight.g_m_s2 / wing.area,
                               self.flight.wing_loading(wing))

    def test_polar_coverage_is_explicit(self):
        polars = SectionPolars(ROOT / "data/polars/Currentmegapolar.dat", self.wing.section)
        alpha, cd = polars.drag_at_local_cl(100000, 0.1, 1)
        self.assertTrue(-8 <= alpha <= 10)
        self.assertGreater(cd, 0)
        with self.assertRaisesRegex(PolarCoverageError, "outside stored"):
            polars.drag_at_local_cl(200000, 0.1, 1)

    def test_avl_baseline_static_point(self):
        exe = ROOT / "tools/avl/avl352.exe"
        if not exe.is_file():
            self.skipTest("official AVL executable not installed")
        with tempfile.TemporaryDirectory() as directory:
            point = run_avl(self.wing, "smoke", [2.689], Path(directory), exe)[0]
            strips = read_strips(point.strip_file)
            self.assertAlmostEqual(point.cl, 0.1228, delta=0.002)
            self.assertAlmostEqual(point.cm, 0, delta=0.001)
            self.assertGreater(point.xnp_m, self.wing.cg_x_m)
            self.assertAlmostEqual(sum(s.area_m2 for s in strips), self.wing.area,
                                   delta=0.02 * self.wing.area)
            self.assertAlmostEqual((point.xnp_m - self.wing.cg_x_m) / self.wing.mac,
                                   -point.cma_per_rad / point.cla_per_rad, delta=0.015)
            self.assertTrue(math.isfinite(point.cnb_per_rad))

    def test_simple_wing_symmetry_and_incidence_control(self):
        exe = ROOT / "tools/avl/avl352.exe"
        if not exe.is_file():
            self.skipTest("official AVL executable not installed")
        from dataclasses import replace
        wing = replace(self.wing, washout_deg=0, dihedral_deg=0)
        with tempfile.TemporaryDirectory() as directory:
            case = Path(directory)
            plain = run_avl(wing, "plain", [0, 4], case / "plain", exe,
                            deck_text=study_deck(wing, "plain"))
            self.assertAlmostEqual(plain[0].cm, 0, delta=1e-5)
            self.assertGreater(plain[1].cl, 0)
            left = run_avl(wing, "left", [4], case / "left", exe,
                           deck_text=study_deck(wing, "left", incidence_control=True),
                           oper_prefix=["D1 D1 -0.5"])[0]
            right = run_avl(wing, "right", [4], case / "right", exe,
                            deck_text=study_deck(wing, "right", incidence_control=True),
                            oper_prefix=["D1 D1 0.5"])[0]
            self.assertAlmostEqual(left.cl_roll, -right.cl_roll, delta=0.0002)
            self.assertGreater(abs(left.cl_roll), 0.001)


if __name__ == "__main__":
    unittest.main()
