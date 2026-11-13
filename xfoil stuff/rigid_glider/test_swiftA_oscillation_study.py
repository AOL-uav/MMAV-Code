"""Plumbing checks for swiftA_oscillation_study.py (not checks of the answer).  Runs AVL twice (serial, a few seconds) in a scratch directory.
    py -B rigid_glider/test_swiftA_oscillation_study.py
"""
import sys
sys.dont_write_bytecode = True

import csv
import math
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import swiftA_oscillation_study as st


class Study(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        avl = st.build_avl_numbers(Path(cls.tmp.name), log=lambda *a: None)
        cls.nm = st.numbers_from_avl(avl)
        cls.mb = st.mass_basis()
        cls.spec = st.make_spec(cls.nm, cls.mb)
        cls.plate = st.ps.PlateTable(cls.spec["ar"], cls.spec["taper"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_np_and_trim_cg_match_v4_outputs(self):
        rows = list(csv.DictReader(open(st.ROOT / "results/runs/rigid_glider/bird_outline_v4_swift" / "outlines.csv", encoding="utf8")))
        A = [r for r in rows if r["outline"] == "A_as_measured"][0]
        g = st.s4.build_geometry4(st.s4.GLIDE)
        k = st.SPAN_MM / st.s4.SPAN
        np_v4 = (g["x_le_mac"] * st.s4.SPAN + float(A["NP % MAC"]) / 100 * float(A["MAC mm"])) * k            # outlines.csv NP, scaled 250 -> 400 mm
        self.assertAlmostEqual(self.nm["xnp_mm"], np_v4, delta=0.5)
        cg_v4 = float(A["trim CG mm at 10% margin (250)"]) * k                                                 # 6 deg bend (5.99) trim CG at CL 0.3
        self.assertAlmostEqual(self.nm["trim_cg_6deg_mm"], cg_v4, delta=0.5)
        self.assertAlmostEqual(self.nm["dcm_ac_per_deg"], float(A["CM0 per deg of bend"]), delta=2e-4)

    def test_flown_cg_is_the_6deg_trim_cg(self):
        self.assertAlmostEqual(self.nm["trim_cg_6deg_mm"] + st.NOSE_AHEAD_MM, 73.0, delta=0.5)

    def test_mass_numbers_come_from_flight_day_pages(self):
        rows, M, cg, inertia, ball = st.mass_table(self.mb, 36.96)
        self.assertAlmostEqual(M, 9.77, delta=0.01)
        self.assertAlmostEqual(ball, 1.21, delta=0.01)
        self.assertAlmostEqual(cg, 36.96, delta=0.15)          # MassModel centroid (rail as a uniform bar) differs from size_numbers by about 0.07 mm; negligible (0.15 % MAC)
        self.assertGreater(inertia, 3e3)            # g mm2: a 10 g glider whose mass is spread over about 140 mm of fore-aft length is of order m L^2 / 12 = 1e4
        self.assertLess(inertia, 1e5)

    def test_zero_bend_at_73_is_not_a_glide(self):
        # plumbing: with no bend the trim angle of attack is 0 (symmetric table), i.e. no lift, so the model has no glide.  This tests what the model gives, not what is wanted.
        a, rec, veh = st.run_one(self.spec, self.plate, 73.0, 0.0)
        self.assertFalse(a["glide"])
        self.assertEqual(a["class"], "dive")

    def test_trim_is_an_equilibrium_and_modes_match_the_simulation(self):
        veh = st.vehicle(self.spec, self.plate, 73.0, 6.0)
        tr = st.find_trim(veh)
        self.assertIsNotNone(tr)
        d = veh.deriv([0.0, 0.0, tr["state"][2], tr["state"][0], tr["state"][1], 0.0])
        self.assertLess(abs(d[3]), 1e-6)
        self.assertLess(abs(d[4]), 1e-6)
        self.assertLess(abs(d[5]), 1e-6)
        m = st.jacobian_modes(veh, tr)
        self.assertLess(m["max_re"], 0.0)                                  # stable trim
        a, rec, _ = st.run_one(self.spec, self.plate, 73.0, 6.0)[:3]
        self.assertAlmostEqual(m["phugoid"]["period"], a["period_mean_s"], delta=0.15 * m["phugoid"]["period"])
        self.assertGreater(m["short"]["wn"], 5 * m["phugoid"]["wn"])      # modes well separated

    def test_period_does_not_depend_on_step(self):
        veh = st.vehicle(self.spec, self.plate, 73.0, 6.0)
        p = [st.analyze(st.simulate_full(veh, dt=dt), veh)["period_first_s"] for dt in (0.002, 0.004)]
        self.assertAlmostEqual(p[0], p[1], delta=0.04)

    def test_zigzag_period_and_decay(self):
        # synthetic decaying sine: period 3 s, 0.6 per cycle
        t = [0.02 * i for i in range(1500)]
        y = [40 * math.exp(-0.17 * ti) * math.sin(2 * math.pi * ti / 3.0) for ti in t]
        ext = st.zigzag(y, 0.5)
        mx = [i for i, ty in ext if ty == 1]
        self.assertAlmostEqual(t[mx[1]] - t[mx[0]], 3.0, delta=0.1)
        self.assertAlmostEqual(y[mx[1]] / y[mx[0]], math.exp(-0.17 * 3.0), delta=0.03)

    def test_pullout_sim_unchanged_interface(self):
        veh = st.vehicle(self.spec, self.plate, 73.0, 6.0)
        self.assertAlmostEqual(veh.cg_calc_mm, 36.96, delta=0.15)
        self.assertAlmostEqual(100 * (veh.xnp_frac - veh.xcg_frac), 9.9, delta=0.5)         # static margin at the flown setting, % MAC


if __name__ == "__main__":
    unittest.main(verbosity=2)
