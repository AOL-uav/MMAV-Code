"""Tests for straight_smooth_v1.  py -B rigid_glider/test_straight_smooth_v1.py   (about 1-2 minutes: it runs AVL serially on a few configurations in a temporary folder)."""
import hashlib
import json
import math
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import straight_smooth_v1 as ss

TMP = Path(tempfile.mkdtemp(prefix="ss_test_"))
_CACHE = {}


def cfg_result(kind, gam, fin, base=None):
    key = (kind, gam, fin)
    if key not in _CACHE:
        c = ss.make_cfg(kind, gam, fin)
        _CACHE[key] = ss.evaluate(c, base, TMP)
    return _CACHE[key]


def baseline(kind):
    r = cfg_result(kind, 0.0, 1.0 if kind == "b45" else False)
    return dict(sm_pct=r["sm_pct"], cl_trim=r["cl_trim"])


class TestFormula(unittest.TestCase):
    def test_equations_satisfied_and_kai_form(self):
        clb, cnb, clr, cnr = -0.12, 0.012, 0.08, -0.009
        dcl, dcn = 0.0007, -0.0001
        rh, den = ss.rhat_closed(clb, cnb, clr, cnr, dcl, dcn)
        beta = ss.beta_steady(clb, clr, dcl, rh)
        self.assertAlmostEqual(clb * beta + clr * rh + dcl, 0.0, places=12)
        self.assertAlmostEqual(cnb * beta + cnr * rh + dcn, 0.0, places=12)
        kai = (clb * dcn - cnb * dcl) / (clr * cnb - clb * cnr)
        self.assertAlmostEqual(rh, kai, places=12)
        self.assertGreater(den, 0)                                # spiral-stable combination used here

    def test_sign_right_wing_down_turns_right_and_mirror(self):
        clb, cnb, clr, cnr = -0.12, 0.012, 0.08, -0.009           # D > 0
        r_pos, d = ss.rhat_closed(clb, cnb, clr, cnr, +0.001, 0.0)
        r_neg, _ = ss.rhat_closed(clb, cnb, clr, cnr, -0.001, 0.0)
        self.assertGreater(d, 0)
        self.assertGreater(r_pos, 0)                              # right wing down (dCl > 0) -> right turn (r > 0)
        self.assertLess(r_neg, 0)                                 # left wing down -> left turn
        self.assertAlmostEqual(r_pos, -r_neg, places=15)

    def test_synthetic_system_integration_matches_formula(self):
        """A hand-built dimensional lateral system with the same derivatives: steady r from RK4 equals the closed form (p = 0 frame, phi_dot = p)."""
        V, b, S, rho, g = 5.0, 0.4, 0.03, 1.225, 9.81
        q = 0.5 * rho * V * V
        Ixx, Izz = 1.5e-4, 2.0e-4
        clb, cnb, clr, cnr, clp, cnp = -0.12, 0.03, 0.08, -0.05, -0.34, -0.02
        K1, K2 = q * S * b / Ixx, q * S * b / Izz
        k = b / (2 * V)
        A = [[-0.5, 0.0, -V, g],                                       # mild side-force damping, -V r, gravity bank term
             [K1 * clb / V, K1 * clp * k, K1 * clr * k, 0.0],
             [K2 * cnb / V, K2 * cnp * k, K2 * cnr * k, 0.0],
             [0.0, 1.0, 0.0, 0.0]]
        dcl, dcn = 0.0006, -0.0002
        eig = ss.fk.eig_roots(A)
        self.assertTrue(all(z.real < 0 for z in eig), eig)               # the synthetic system is stable, so a steady state exists
        B = [[0.0], [K1 * dcl], [K2 * dcn], [0.0]]
        traj, x = ss.integrate_lateral(A, B, [1.0], 400.0)
        rh_int = x[2] * b / (2 * V)
        rh_cf, den = ss.rhat_closed(clb, cnb, clr, cnr, dcl, dcn)
        self.assertGreater(den, 0)
        self.assertLess(abs(rh_int - rh_cf) / abs(rh_cf), 0.05)
        self.assertEqual(math.copysign(1, x[3]), math.copysign(1, rh_cf))   # turn direction goes with bank direction (right turn, right wing down)

    def test_radius_heading_scaling(self):
        rh = 0.002
        self.assertAlmostEqual(ss.radius_m(rh, 400.0), 0.4 / (2 * rh))
        self.assertAlmostEqual(ss.radius_m(rh, 250.0) / ss.radius_m(rh, 400.0), 250 / 400)
        self.assertAlmostEqual(ss.heading_deg(rh, 400.0), math.degrees(2 * rh * 100 / 0.4))
        self.assertAlmostEqual(ss.heading_deg(rh, 250.0) / ss.heading_deg(rh, 400.0), 400 / 250)
        self.assertLess(ss.heading_deg(math.pi / 1000 * 0.99, 400.0), 90.0)         # r_hat limit 0.00314

    def test_body_axis_form_reduces_to_prestated_at_zero_alpha(self):
        a = ss.rhat_closed(-0.1, 0.01, 0.09, -0.008, 0.0005, 0.0001)[0]
        b_ = ss.rhat_body(-0.1, 0.01, 0.09, -0.008, -0.3, -0.02, 0.0005, 0.0001, 0.0)
        self.assertAlmostEqual(a, b_[0], places=12)
        self.assertAlmostEqual(a, b_[1], places=12)


class TestMassAndDecks(unittest.TestCase):
    def test_b45_mass_reproduces_geometry_json(self):
        a = ss.b45_mass(68.0, 0.0, 1.0)
        self.assertAlmostEqual(a["cg0"], 81.0234, delta=0.1)
        self.assertAlmostEqual(a["ballast_g"], 1.9227, delta=0.01)
        self.assertAlmostEqual(a["M_g"], 17.498, delta=0.01)

    def test_swift_mass_close_to_flight_day_numbers(self):
        a = ss.sw_mass(37.0, 0.0, False)
        self.assertAlmostEqual(a["M_g"], 9.765, delta=0.05)
        self.assertAlmostEqual(a["ballast_g"], 1.2018, delta=0.05)
        self.assertAlmostEqual(a["cg"][0], 37.0, delta=0.01)

    def test_mirror_keeps_interval_nspan_and_flips_y(self):
        wr, _ = ss.b45_halves(0.0)
        wl = ss.mirror(wr)
        self.assertEqual([s["y"] for s in wl], [-s["y"] for s in reversed(wr)])
        self.assertEqual([s["nspan"] for s in wl][:3], [wr[2]["nspan"], wr[1]["nspan"], wr[0]["nspan"]])
        self.assertTrue(all(c[0].endswith("L") for s in wl for c in s["controls"]))

    def test_dihedral_rotation(self):
        y, z = ss.rot(200.0, 0.0, 12.0)
        self.assertAlmostEqual(math.hypot(y, z), 200.0)
        self.assertAlmostEqual(z, 200 * math.sin(math.radians(12)))


class TestAvl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.b = cfg_result("b45", 0.0, 1.0)
        cls.s = cfg_result("sw", 0.0, False)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(TMP, ignore_errors=True)

    def test_b45_baseline_reproduces_existing_xnp_and_margin(self):
        self.assertAlmostEqual(self.b["xnp"], 70.30, delta=0.5)                # b45_centre_flaps README, nominal location, 10 deg
        self.assertAlmostEqual(self.b["sm_pct"], 4.65, delta=0.5 / self.b["mac"] * 100)   # same table at CG 66: 4.65 % MAC
        self.assertEqual(self.b["cg"], 66.0)
        self.assertAlmostEqual(self.b["cl_trim"], 0.401, delta=0.03)           # README CG sweep, trim CL at 66 mm (0.401)

    def test_swift_baseline_reproduces_existing_xnp(self):
        self.assertAlmostEqual(self.s["xnp"], 41.70, delta=0.5)                # swiftA_oscillation README: 41.70 mm behind root LE
        self.assertAlmostEqual(self.s["cg"] + 36.0, 73.0)
        self.assertAlmostEqual(self.s["sm_pct"], 100 * (41.70 - 37.0) / (0.1185156 * 400), delta=1.0)

    def test_symmetric_decks_have_no_asymmetric_totals(self):
        for r in (self.b, self.s):
            self.assertLess(abs(r["deriv"]["Cl"]), 3e-4)
            self.assertLess(abs(r["deriv"]["Cn"]), 3e-4)
            self.assertLess(abs(r["deriv"]["CY"]), 3e-4)
            self.assertLess(abs(r["cm_trim"]), 5e-4)

    def test_asymmetry_signs(self):
        # (a) more trailing-edge-up on the right = less lift on the right = right wing down (Cl' > 0)
        self.assertGreater(self.b["asym"]["a"]["dCl"], 0)
        self.assertGreater(self.s["asym"]["a"]["dCl"], 0)
        # (b) right half leading edge up = more lift on the right = right wing up (Cl' < 0)
        self.assertLess(self.b["asym"]["b"]["dCl"], 0)
        self.assertLess(self.s["asym"]["b"]["dCl"], 0)
        # (c) positive toe (TE toward +y) makes a side force toward -y and, with the fin behind the CG, a nose-right yaw (Cn > 0)
        self.assertLess(self.b["asym"]["c"]["dCY"], 0)
        self.assertGreater(self.b["asym"]["c"]["dCn"], 0)

    def test_derivative_signs_and_flap_gain(self):
        for r in (self.b, self.s):
            d = r["deriv"]
            self.assertLess(d["Clb"], 0)
            self.assertGreater(d["Cnb"], 0)
            self.assertLess(d["Cnr"], 0)
            self.assertLess(d["Clp"], 0)
        self.assertGreater(self.b["deriv"]["Cmd1"], 0)               # flap up (input in degrees UP) gives nose-up moment (README GAIN check)

    def test_fin_toe_equals_sideslip_response_of_that_fin(self):
        """Calibration of the toe sign and size on the Swift centre fin: a toe of +1 deg (TE toward +y) must give the side-force change a 1 deg sideslip gives for the fin."""
        c1 = ss.make_cfg("sw", 0.0, True)
        c0 = ss.make_cfg("sw", 0.0, False)
        res = {}
        for tag, c in (("fin", c1), ("nofin", c0)):
            r = cfg_result("sw", 0.0, bool(c["fin"]))
            deck = ss.deck_for(c, r["cg"], r["zcg"])
            a = r["alpha"]
            out = ss.avl_session(TMP / f"toe_{tag}", "ss", deck, [([f"A A {a:.5f}"], "b0"), ([f"A A {a:.5f}", "B B 1.0"], "b1")])
            res[tag] = out["b1"]["CY"] - out["b0"]["CY"]
        fin_beta_response = res["fin"] - res["nofin"]                  # CY change from the fin for beta = +1 deg
        toe = cfg_result("sw", 0.0, True)["asym"]["c"]["dCY"]          # sign-corrected toe response, 1 deg
        self.assertLess(fin_beta_response, 0)
        self.assertLess(toe, 0)
        self.assertLess(abs(toe - fin_beta_response) / abs(fin_beta_response), 0.25)

    def test_control_columns_are_per_degree_and_system_consistent(self):
        """B column for the flap vs the stability-file control derivative: dCl per deg times q S b / Ixx_stab is of the right size (within 30 %, axes rotation / Ixz make it inexact)."""
        r = self.b
        kc = ss.kind_consts("b45")
        q = 0.5 * ss.RHO * r["V"] ** 2
        k = q * kc["S_m2"] * 0.4 / r["inertia"][0]
        est = k * r["deriv"]["Cld1"]
        got = r["B4"][1][0]
        self.assertGreater(got, 0)
        self.assertLess(abs(got - est) / est, 0.3)

    def test_formula_vs_time_integration(self):
        """Pre-stated cross-check: steady turn rate from RK4 of AVL's lateral system with its control columns vs the closed form; >= 3 all-stable configurations.
        The body-axis form must agree within 5 %; the stability-axis form as originally written is reported and is NOT required to (it ignores the roll rate of the descending turn)."""
        b45b, swb = baseline("b45"), baseline("sw")
        cases = [cfg_result("b45", 3.0, 1.0, b45b), cfg_result("b45", 3.0, 1.5, b45b), cfg_result("sw", 3.0, False, swb), cfg_result("sw", 3.0, True, swb)]
        n_ok, pre_err = 0, []
        for r in cases:
            c = ss.crosscheck(r)
            if not c["integ"]:
                continue
            pre_err.append(abs(c["err_int_pre_pct"]))
            if abs(c["err_int_body_pct"]) < 5.0:
                n_ok += 1
                self.assertLess(abs(c["err_eq_body_pct"]), 5.0, (r["cfg"]["name"], c))
            else:                                                          # allowed only when the numerator is at the print resolution of the input run
                self.assertLess(c["num_over_unc"], 5.0, (r["cfg"]["name"], c))
        self.assertGreaterEqual(n_ok, 3)
        self.assertGreater(max(pre_err), 5.0)                          # documents that the as-written formula does not match the AVL frame

    def test_left_down_roll_gives_left_turn_in_time_integration(self):
        b45b = baseline("b45")
        r = cfg_result("b45", 3.0, 1.0, b45b)
        A, B = r["A4"], r["B4"]
        up = ss.integrate_lateral(A, B, ss.u_vec(len(B[0])), 300.0)[1]
        um = ss.integrate_lateral(A, B, [-v for v in ss.u_vec(len(B[0]))], 300.0)[1]
        self.assertAlmostEqual(up[2], -um[2], delta=1e-9 + 1e-6 * abs(up[2]))
        self.assertEqual(math.copysign(1, up[2]), math.copysign(1, up[3]))   # turn direction = bank direction
        self.assertEqual(math.copysign(1, um[2]), math.copysign(1, um[3]))
        self.assertNotEqual(math.copysign(1, up[2]), math.copysign(1, um[2]))

    def test_rule_thresholds(self):
        base = dict(sm_pct=5.0, cl_trim=0.4)
        mk = lambda sp, zeta=0.2, roll=-10.0, hd=50.0, sm=5.0, cl=0.4: dict(modes=dict(spiral=sp, dr_zeta=zeta, roll=roll), sm_pct=sm, cl_trim=cl,
                                                                          asym={"a": dict(hdg400=hd, hdg400_body=hd)})
        self.assertTrue(ss.judge(mk(-0.01), base)["passed"])
        self.assertTrue(ss.judge(mk(0.04), base)["r1"])                # t2 = 17 s > 15 s
        self.assertFalse(ss.judge(mk(0.05), base)["r1"])               # t2 = 13.9 s
        self.assertFalse(ss.judge(mk(-0.01, zeta=0.09), base)["r2"])
        self.assertFalse(ss.judge(mk(-0.01, hd=95.0), base)["r3"])
        self.assertFalse(ss.judge(mk(-0.01, cl=0.43), base)["r4"])
        self.assertTrue(ss.judge(mk(-0.01, cl=0.419, sm=5.9), base)["r4"])


class TestRevision(unittest.TestCase):
    """Reviewer REVISE: base-state elevation, corrected closed form, product of inertia, mode shape."""

    def test_rhat_gamma_limits(self):
        a = ss.rhat_closed(-0.1, 0.01, 0.09, -0.008, 0.0005, 0.0001)[0]
        g0 = ss.rhat_gamma(-0.1, 0.01, 0.09, -0.008, -0.3, -0.02, 0.0005, 0.0001, 0.0)
        self.assertAlmostEqual(a, g0[0], places=12)                     # level flight: p' = 0, the pre-stated form
        b = ss.rhat_body(-0.1, 0.01, 0.09, -0.008, -0.3, -0.02, 0.0005, 0.0001, 7.0)
        g7 = ss.rhat_gamma(-0.1, 0.01, 0.09, -0.008, -0.3, -0.02, 0.0005, 0.0001, -7.0)
        self.assertAlmostEqual(b[0], g7[0], places=12)                  # theta0 = 0 means gamma = -alpha
        self.assertAlmostEqual(g7[1], g7[0] / math.cos(math.radians(7.0)))

    def test_numerator_independent_of_frame(self):
        n0 = ss.rhat_gamma(-0.1, 0.01, 0.09, -0.008, -0.3, -0.02, 0.0, 0.0, -6.61)[0]
        self.assertEqual(n0, 0.0)                                      # zero asymmetry gives no turn in any frame

    def test_mode_shape_on_synthetic_oscillation(self):
        # v and phi oscillate 90 deg apart in a damped 2-state toy embedded in 4 states: shape ratio must be finite and positive
        A = [[-0.5, 0.0, -5.0, 9.81], [-20.0, -15.0, 3.0, 0.0], [2.0, -1.0, -0.4, 0.0], [0.0, 1.0, 0.0, 0.0]]
        eig = ss.fk.eig_roots(A)
        lam = [z for z in eig if z.imag > 1e-6][0]
        sh = ss.mode_shape(A, 5.0, lam)
        self.assertGreater(sh["beta_over_phi"], 0)
        # residual of (A - lam I) x = 0 with the returned scaling is zero: rebuild x from the same solver and check
        M = [[A[i][j] - (lam if i == j else 0) for j in range(4)] for i in range(4)]
        x = ss._csolve([r[:3] for r in M[:3]], [-M[i][3] for i in range(3)]) + [1.0 + 0j]
        res4 = sum(abs(sum(M[i][j] * x[j] for j in range(4))) for i in range(4))
        self.assertLess(res4, 1e-6)

    def test_avl_elevation_ixz_and_corrected_crosscheck(self):
        b45b, swb = baseline("b45"), baseline("sw")
        r = cfg_result("b45", 3.0, 1.0, b45b)
        work = TMP / "post"
        m0 = ss.mode_run(r["cfg"], r, 0.0 - r["alpha"], False, work, "t_elev0")        # theta0 = 0
        m1 = ss.mode_run(r["cfg"], r, ss.GLIDE_REF, False, work, "t_g")
        self.assertAlmostEqual(m0["A4"][3][2], 0.0, places=6)                           # phi_dot = p + r tan(theta0)
        self.assertAlmostEqual(m1["A4"][3][2], math.tan(math.radians(r["alpha"] + ss.GLIDE_REF)), delta=2e-4)
        m2 = ss.mode_run(r["cfg"], r, ss.GLIDE_REF, True, work, "t_gi")
        self.assertGreater(m2["jxz"], 0)                                               # dihedral tips are up and aft: integral(x z dm) > 0
        self.assertLess(m2["modes"]["dr_zeta"], m1["modes"]["dr_zeta"])                 # positive Ixz lowers the damping here (found in the first test of the IZX sign)
        self.assertGreater(m1["shape"]["beta_over_phi"], 0)
        cases = [(r, base_) for r, base_ in ((r, b45b), (cfg_result("b45", 3.0, 1.5, b45b), b45b), (cfg_result("sw", 3.0, True, swb), swb), (cfg_result("sw", 0.0, True, swb), swb))]
        n_ok = 0
        for rr, _ in cases:
            post = ss.post_analysis(rr["cfg"], rr, work)
            for tag, e in post.items():
                cc = e["crosscheck"]
                if cc["rhat_rk4"] is None or cc["num_over_unc"] < 5.0 or abs(cc["rhat_closed"]) < 1e-4:
                    continue
                if abs(cc["rhat_closed"]) > 1.0:                                       # denominator ~ 0: hypersensitive, not a test
                    continue
                self.assertLess(abs(cc["err_rk4_pct"]), 5.0, (rr["cfg"]["name"], tag, cc))
                n_ok += 1
        self.assertGreaterEqual(n_ok, 3)

    def test_roll_trim_zeroes_numerator(self):
        r = cfg_result("b45", 0.0, 1.0)
        rt = ss.roll_trims(r, {})
        d, c = r["deriv"], r["ctl"]
        for k, v in r["asym"].items():
            num = d["Cnb"] * (v["dCl"] + rt[k]["diff_deg"] * c["dCl"]) - d["Clb"] * (v["dCn"] + rt[k]["diff_deg"] * c["dCn"])
            self.assertAlmostEqual(num, 0.0, places=12)


class TestPrestated(unittest.TestCase):
    def test_prestated_section_unchanged(self):
        out = ss.OUT
        txt = (out / "README.md").read_bytes()
        head = txt.split(b"\nRESULTS\n")[0]
        want = (out / "prestated_section.sha256").read_text().split()[0]
        self.assertEqual(hashlib.sha256(head).hexdigest(), want)


if __name__ == "__main__":
    unittest.main(verbosity=2)
