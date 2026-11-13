"""Tests for pullout_sim.py.   py -B -m unittest test_pullout_sim -v   (from rigid_glider/; writes to a temporary folder; three light serial AVL sessions)"""
import sys
sys.dont_write_bytecode = True
import math
import tempfile
import unittest
from pathlib import Path

import pullout_sim as ps


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="pullout_")
        cls.spec = ps.flat_specimen("F45_span250", Path(cls.tmp.name), log=lambda *a: None)
        cls.plate = ps.PlateTable(cls.spec["ar"], cls.spec["taper"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def veh(self, cg_i=1, up=0.0, **kw):
        return ps.Vehicle(self.spec, self.plate, self.spec["cgs"][cg_i], up, **kw)


class PhysicsTests(Base):
    def test_energy_conserved_with_aero_off(self):
        v = self.veh(no_aero=True)
        rec = ps.simulate(v, -90.0, (0.3, 0.0), dt=0.002, t_max=2.0)
        e0 = None
        for r in rec:
            e = v.mass * ps.G * r[2] + 0.5 * v.mass * r[6] ** 2
            e0 = e if e0 is None else e0
            self.assertAlmostEqual(e, e0, delta=1e-6 * v.mass * ps.G * 20)
        self.assertAlmostEqual(rec[-1][5], -ps.G * rec[-1][0], delta=1e-6)                  # free fall: vz = -g t

    def test_steady_glide_reproduces_the_table_lift_to_drag(self):
        v = self.veh(2, 6.0)
        rec = ps.simulate(v, t_max=120.0, tol=0.02)                                         # run until really steady
        m = ps.metrics(rec, v)
        self.assertTrue(m["settled"], m["verdict"])
        al = m["alpha_final_deg"]
        cn, ca, cm4 = self.plate.coeffs(al - v.tau * v.up)
        a = math.radians(al)
        cl, cd = cn * math.cos(a) - ca * math.sin(a), ca * math.cos(a) + cn * math.sin(a)       # inverse of CN = CL cos + CD sin, CA = CD cos - CL sin
        g_table = -math.degrees(math.atan2(cd, cl))
        self.assertAlmostEqual(m["gamma_final_deg"], g_table, delta=0.02 * abs(g_table) + 0.05)
        self.assertAlmostEqual(m["glide_ratio_final"], cl / cd, delta=0.03 * cl / cd)           # L/D = 1 / tan(glide angle)
        # speed from lift = weight cos(gamma)
        vs = math.sqrt(2 * v.mass * ps.G * math.cos(math.radians(m["gamma_final_deg"])) / (ps.RHO * v.S * cl))
        self.assertAlmostEqual(m["speed_final"], vs, delta=0.03 * vs)

    def test_time_step_halving(self):
        v = self.veh(1, 4.0)
        a = ps.metrics(ps.simulate(v, dt=0.002), v)
        b = ps.metrics(ps.simulate(v, dt=0.001), v)
        for k in ("peak_speed", "swoop_depth_m", "h_first_level_m", "peak_n"):
            self.assertAlmostEqual(a[k], b[k], delta=0.015 * abs(b[k]), msg=k)
        self.assertEqual(a["verdict"], b["verdict"])

    def test_negative_angles_mirror(self):
        for al in (3.0, 12.0, 35.0, 80.0):
            p, n = self.plate.coeffs(al), self.plate.coeffs(-al)
            self.assertAlmostEqual(p[0], -n[0], places=12)
            self.assertAlmostEqual(p[1], n[1], places=12)
            self.assertAlmostEqual(p[2], -n[2], places=12)
        v = self.veh(1, 0.0)
        for al in (4.0, 20.0, 60.0, 120.0):                                                  # including flow from behind
            c, s = math.cos(math.radians(al)), math.sin(math.radians(al))
            f1 = v.aero(5 * c, -5 * s, 0.0)
            f2 = v.aero(5 * c, 5 * s, 0.0)
            self.assertAlmostEqual(f1[0], f2[0], places=9)
            self.assertAlmostEqual(f1[1], -f2[1], places=9)
            self.assertAlmostEqual(f1[2], -f2[2], places=9)

    def test_blend_is_continuous_at_the_end_of_the_table(self):
        e = self.plate.a_end
        a, b = self.plate.coeffs(e - 1e-6), self.plate.coeffs(e + 1e-6)
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, delta=2e-4)
        prev = self.plate.coeffs(0.0)
        for al in [0.5 * i for i in range(1, 181)]:
            cur = self.plate.coeffs(al)
            self.assertLess(abs(cur[0] - prev[0]), 0.12)                                       # no jumps in the normal force
            self.assertLess(abs(cur[2] - prev[2]), 0.12)
            prev = cur
        self.assertAlmostEqual(self.plate.coeffs(90.0)[0], ps.CN90, delta=1e-9)

    def test_no_input_never_pitches_and_dives(self):
        v = self.veh(1, 0.0)
        rec = ps.simulate(v)
        m = ps.metrics(rec, v)
        self.assertEqual(m["verdict"], "DIVES")
        self.assertAlmostEqual(m["theta_range_deg"], 0.0, places=6)
        self.assertLess(m["gamma_final_deg"], -89.0)

    def test_nose_up_input_produces_pitch_up(self):
        v = self.veh(2, 6.0)
        rec = ps.simulate(v)
        self.assertGreater(max(r[3] for r in rec), -90.0 + 30.0)
        self.assertGreater(max(r[8] for r in rec if r[8] is not None), -30.0)


class InputTests(Base):
    def test_kit_mass_and_cg_agree(self):
        for i, cg in enumerate(self.spec["cgs"]):
            m, ball = self.spec["masses"](cg)
            self.assertAlmostEqual(m.cg_mm(), cg, delta=0.6)                                    # the components reproduce the kit CG setting
            self.assertGreater(m.inertia_kg_m2(), 0)
        self.assertGreater(self.spec["masses"](self.spec["cgs"][0])[0].mass_g(), self.spec["masses"](self.spec["cgs"][-1])[0].mass_g())

    def test_poly_moments_rectangle(self):
        A, cx, i2 = ps.poly_moments([(0, -5), (20, -5), (20, 5), (0, 5)])
        self.assertAlmostEqual(A, 200.0)
        self.assertAlmostEqual(cx, 10.0)
        self.assertAlmostEqual(i2, 200.0 * 20 ** 2 / 12)

    def test_avl_tab_signs(self):
        a = self.spec["avl"][sorted(self.spec["avl"])[1]]
        self.assertGreater(a["CLd_per_deg"], 0)                                                  # control derivative is per degree trailing edge DOWN: more lift
        v = self.veh(2, 4.0)
        self.assertGreater(v.cm_tab, 0)                                                          # trailing edge up = nose-up couple
        self.assertGreater(v.tau, 0)                                                             # and less lift
        self.assertLess(a["Cmq"], 0)
        self.assertAlmostEqual(v.xnp_frac, (self.spec["avl"][self.spec["cgs"][0]]["Xnp"] * 1000 - self.spec["mac_le_mm"]) / (self.spec["mac_m"] * 1000), places=9)

    def test_table_choice_and_symmetry_of_data(self):
        self.assertEqual(self.plate.wing, "AR4_taper0.5")
        self.assertAlmostEqual(self.plate.coeffs(0.0)[0], 0.0, places=9)
        self.assertAlmostEqual(self.plate.coeffs(0.0)[2], 0.0, places=9)
        self.assertGreater(self.plate.cl_alpha_deg, 0.05)
        self.assertLess(self.plate.Re, 65000)

    def test_tab_deck_has_two_control_sections(self):
        deck = (ps.KIT / "avl_work" / "F45_span250" / "fins" / "cl03" / "fins.avl").read_text()
        new = ps.inject_tab(deck)
        self.assertEqual(new.count("CONTROL"), 2)
        self.assertEqual(new.count("SECTION"), deck.count("SECTION") + 1)


class ReviewTests(Base):
    def test_constant_cl_zero_drag_bottoms_at_three_trim_speed_heights(self):
        for cl, ws in ((0.3, 13.0), (0.12, 13.0), (0.5, 5.0)):
            d = ps.point_mass_const_cl(cl, ws)
            v_trim_sq = 2 * ws / (ps.RHO * cl)                                               # trim speed squared
            self.assertAlmostEqual(d, 3 * v_trim_sq / (2 * ps.G), delta=0.01 * d)             # three speed-heights V^2 / 2g
            self.assertAlmostEqual(d, 3 * ws / (ps.RHO * ps.G * cl), delta=0.01 * d)
            self.assertAlmostEqual(d, 1.5 * (2 * ws / (ps.RHO * ps.G * cl)), delta=0.01 * d)   # 1.5 x the constant-CL radius 2 (W/S) / (rho g CL)

    def test_swoop_depth_is_the_first_local_minimum_of_height(self):
        v = self.veh(2, 6.0)
        rec = ps.simulate(v)
        m = ps.metrics(rec, v)
        k = next(i for i, r in enumerate(rec) if r[0] > 0.05 and r[6] > 0.3 and r[5] >= 0)
        self.assertAlmostEqual(m["swoop_depth_m"], -rec[k][2])
        self.assertLessEqual(m["swoop_depth_m"], m["height_lost_at_end_of_run_m"] + 1e-9)
        self.assertNotIn("lowest_point_m", m)                                                 # the end-of-run quantity is named for what it is
        self.assertIsNotNone(m["depth_formula_m"])
        self.assertAlmostEqual(m["depth_over_formula"], m["swoop_depth_m"] / m["depth_formula_m"])
        ws = v.mass * ps.G / v.S
        self.assertAlmostEqual(m["depth_formula_m"], 3 * ws / (ps.RHO * ps.G * m["cl_final"]))

    def test_dive_has_no_swoop_depth(self):
        v = self.veh(1, 0.0)
        m = ps.metrics(ps.simulate(v), v)
        self.assertEqual(m["verdict"], "DIVES")
        self.assertIsNone(m["swoop_depth_m"])

    def test_effectiveness_scales_tab_effect(self):
        a, b = self.veh(2, 4.0), ps.Vehicle(self.spec, self.plate, self.spec["cgs"][2], 4.0, eff=0.5)
        self.assertAlmostEqual(b.cm_tab, 0.5 * a.cm_tab, places=12)
        self.assertAlmostEqual(b.tau, 0.5 * a.tau, places=12)

    def test_deep_stall_needs_a_clear_margin(self):
        class V:  # minimal stand-in for the vehicle
            alpha_stall = 11.0
            c, S, mass, tau, up = 0.05, 0.01, 0.01, 0.0, 0.0
            plate = self.plate
        def rec_at(alpha, gamma=-8.0, T=12.0):
            return [(i * 0.02, 0.0, -1.0 * i * 0.001, -5.0, 3.0, -1.0, 5.0, alpha, gamma, 1.0, 0.0) for i in range(int(T / 0.02))]
        m1 = ps.metrics(rec_at(11.9), V)
        m2 = ps.metrics(rec_at(29.8), V)
        self.assertEqual(m1["verdict"], "SETTLES TO A GLIDE")                                 # 0.9 deg above a soft peak is not a deep stall
        self.assertEqual(m2["verdict"], "DEEP STALL")
        self.assertAlmostEqual(m1["margin_to_first_peak_deg"], -0.9, places=6)

    def test_cannot_say_flags(self):
        mk = lambda **kw: dict(dict(spec="x", cg_mm=1.0, up_deg=4.0, tag="base", static_margin_pct=10.0, verdict="SETTLES TO A GLIDE", is_bird=False), **kw)
        cases = [mk(), mk(static_margin_pct=1.0, cg_mm=2.0), mk(is_bird=True, spec="BS", cg_mm=3.0), mk(up_deg=0.0, is_bird=True, spec="BS2", verdict="DIVES"),
                 mk(tag="effectiveness 50%", verdict="DIVES")]
        ps.flag_cannot_say(cases)
        self.assertTrue(cases[0]["cannot_say"])                                               # its own sensitivity run flips the verdict
        self.assertIn("flips", cases[0]["cannot_say_why"])
        self.assertIn("neutral point", cases[1]["cannot_say_why"])
        self.assertIn("bird", cases[2]["cannot_say_why"])
        self.assertFalse(cases[3]["cannot_say"])                                              # the exact zero-input symmetry result is claimed
        kit = [mk(tag=ps.KIT_TAGS[0], up_deg=0.0, verdict="DIVES"), mk(tag=ps.KIT_TAGS[1], up_deg=0.0, verdict="SETTLES TO A GLIDE")]
        ps.flag_cannot_say(kit)
        self.assertTrue(all(c["cannot_say"] for c in kit))
        self.assertIn("disagree", kit[0]["cannot_say_why"])


class KitBandTests(Base):
    def test_kit_variants_and_flag_logic(self):
        mk = lambda tag, cg=1.0, v="SETTLES TO A GLIDE": dict(spec="x", cg_mm=cg, up_deg=0.0, tag=tag, static_margin_pct=-5.0, verdict=v, is_bird=False)
        # no variant runs: band position not sensitivity-tested
        c = [mk(ps.KIT_TAGS[0]), mk(ps.KIT_TAGS[1])]
        ps.flag_cannot_say(c)
        self.assertTrue(all("not sensitivity-tested" in x["cannot_say_why"] for x in c))
        # variants that agree: not flagged; a flip in either start flags both
        agree = [mk(ps.KIT_TAGS[0]), mk(ps.KIT_TAGS[1])] + [mk(t) for t in ps.KIT_VAR_TAGS]
        ps.flag_cannot_say(agree)
        self.assertFalse(agree[0]["cannot_say"])
        self.assertFalse(agree[1]["cannot_say"])
        flip = [mk(ps.KIT_TAGS[0]), mk(ps.KIT_TAGS[1])] + [mk(t, v=("DIVES" if t == ps.KIT_VAR_TAGS[3] else "SETTLES TO A GLIDE")) for t in ps.KIT_VAR_TAGS]
        ps.flag_cannot_say(flip)
        self.assertTrue(flip[0]["cannot_say"] and flip[1]["cannot_say"])
        self.assertIn("flips", flip[0]["cannot_say_why"] + flip[1]["cannot_say_why"])

    def test_band_offsets_are_aft_of_the_neutral_point(self):
        spec = dict(self.spec)
        cases = []
        for cg in spec["cgs_all"]:
            for st, _ in ps.KIT_STARTS:
                cases.append(dict(spec=spec["name"], cg_mm=cg, tag=f"kit as built: {st}", verdict="SETTLES TO A GLIDE" if cg in (46.0, 47.0) else "DIVES"))
        b = ps.kit_bands([spec], cases)[0]
        xnp = spec["avl"][spec["cgs"][0]]["Xnp"] * 1000
        self.assertEqual(b["cgs"], [46.0, 47.0])
        self.assertAlmostEqual(b["off_mm"][0], 46.0 - xnp, places=9)

    def test_inertia_scale_and_drag_off_options(self):
        a = self.veh(1, 4.0)
        b = ps.Vehicle(self.spec, self.plate, self.spec["cgs"][1], 4.0, inertia_scale=0.5)
        self.assertAlmostEqual(b.inertia, 0.5 * a.inertia, places=15)
        d = ps.Vehicle(self.spec, self.plate, self.spec["cgs"][1], 4.0, drag_off=True)
        ca_on = a.aero(5 * math.cos(0.1), -5 * math.sin(0.1), 0.0)[0]
        ca_off = d.aero(5 * math.cos(0.1), -5 * math.sin(0.1), 0.0)[0]
        self.assertNotEqual(ca_on, ca_off)


class OutputTests(unittest.TestCase):
    def test_default_output_folder_is_new(self):
        self.assertEqual(ps.OUT_DEFAULT.name, "pullout")
        self.assertNotIn("flat_test", ps.OUT_DEFAULT.as_posix())

    def test_svg_plot_is_dark_and_wellformed(self):
        import xml.etree.ElementTree as ET
        s = ps.svg_traj("t", [("a", "#fff", [(0, 0), (1, -2), (3, -3)])])
        ET.fromstring(s)
        self.assertIn("#14181d", s)


if __name__ == "__main__":
    unittest.main()
