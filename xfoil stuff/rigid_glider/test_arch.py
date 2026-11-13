"""Unit tests for the architecture-comparison helpers (arch_model / arch_study / arch_report)."""
import sys
sys.dont_write_bytecode = True
import json
import math
import tempfile
import unittest
from pathlib import Path

import arch_model as A
import arch_report as R
import arch_study as S
import pure_wing_model as m
import pure_wing_v2_model as v2

TMP = Path(tempfile.mkdtemp(prefix="arch_test_"))
AR, SW, TP = 6.0, 15, 0.5
_cache = {}


def lip_model(cf=0.2, pl="outer"):
    k = ("lip", cf, pl)
    if k not in _cache:
        _cache[k] = A.build_lip(AR, SW, TP, cf, pl, TMP / f"lip_{cf}_{pl}")
    return _cache[k]


def pure_model():
    if "pure" not in _cache:
        _cache["pure"] = A.build_pure(AR, SW, TP, TMP / "pure")
    return _cache["pure"]


def tail_model():
    if "tail" not in _cache:
        _cache["tail"] = A.build_tail(AR, SW, TP, 0.10, 2.0, TMP / "tail")
    return _cache["tail"]


class Tab:
    """Synthetic section table: lookup(re, cl) -> cm = fn(re, cl); inviscid lift line cl = k*alpha (+cl0)."""
    def __init__(self, fn, cl0=0.0, k=0.11):
        self.fn, self.cl0, self.k = fn, cl0, k
    def alpha_inv(self, cl): return (cl - self.cl0) / self.k
    def lookup(self, re, cl):
        return dict(alpha=self.alpha_inv(cl), cd=0.02, cm=self.fn(re, cl), nbranch=1, steep=False)


class Inv:
    def __init__(self, fn): self.fn = fn
    def cm_at(self, cl): return self.fn(cl)


class StripParsing(unittest.TestCase):
    TEXT = """ Surface # 1     Right
 Strip Forces referred to Strip Area, Chord
    j     Xle      Yle      Zle      Chord    Area     c_cl     ai     cl_norm    cl       cd       cdv    cm_c/4     cm_LE   C.P.x/c
     1   0.0100   0.0200   0.0000   0.2000   0.0100  -0.0600  -0.0066  -0.3776  -0.3100   0.0095   0.0000   0.0108   0.0795    0.285
 Surface # 2     FixedTail
 Strip Forces referred to Strip Area, Chord
     1   0.5000   0.0300   0.0000   0.0200   0.0010  -0.0600  -0.0066  -0.3776  -0.9000   0.0095   0.0000   0.0200   0.0795    0.285
 Surface # 3     MainWing (YDUP)
 Strip Forces referred to Strip Area, Chord
     1   0.0100  -0.0200   0.0000   0.2000   0.0100  -0.0600  -0.0066  -0.3776  -0.3100   0.0095   0.0000   0.0108   0.0795    0.285
"""

    def test_by_surface_and_wing_filter(self):
        p = TMP / "fake.fs"
        p.write_text(self.TEXT)
        d = A.parse_strips(p)
        self.assertEqual(list(d), ["Right", "FixedTail", "MainWing (YDUP)"])
        self.assertAlmostEqual(d["Right"][0]["cm"], 0.0108); self.assertAlmostEqual(d["Right"][0]["cl"], -0.31)
        w = A.wing_right_strips(p)
        self.assertEqual(len(w), 1)                      # tail strip and the mirrored (y<0) strip excluded
        self.assertAlmostEqual(w[0]["y"], 0.02)


class SectionClasses(unittest.TestCase):
    def rows(self, cl0=0.2, k=0.1, n=11):
        return [dict(alpha=a, cl=cl0 + k * a, cd=0.02, cm=0.01, xtr_top=1.0) for a in [i * 0.5 for i in range(n)]]

    def test_bent_table_affine_hint_and_no_symmetry(self):
        rows = self.rows()
        inv = [dict(alpha=a, cl=0.2 + 0.11 * a, cm=0.03) for a in range(-4, 11)]
        t = A.BentSectionTable({20000: rows, 40000: rows}, inv)
        self.assertAlmostEqual(t.k, 0.11, places=9); self.assertAlmostEqual(t.cl0, 0.2, places=9)
        self.assertAlmostEqual(t.alpha_inv(0.2), 0.0, places=9)
        self.assertIsNone(t.lookup(30000, -0.1))          # no odd symmetry for a bent section
        r = t.lookup(30000, 0.4)
        self.assertAlmostEqual(r["alpha"], 2.0, places=6)
        self.assertIsNone(t.lookup(50000, 0.4))           # Re outside stored range

    def test_inv_curve(self):
        c = A.InvCurve([dict(cl=0.0, cm=0.01), dict(cl=0.5, cm=0.03), dict(cl=1.0, cm=0.03)])
        self.assertAlmostEqual(c.cm_at(0.25), 0.02); self.assertAlmostEqual(c.cm_at(0.75), 0.03)
        self.assertIsNone(c.cm_at(-0.1)); self.assertIsNone(c.cm_at(1.1))


class LinearModels(unittest.TestCase):
    def test_lip_signs_and_linearity(self):
        M = lip_model()
        self.assertLess(M["d_cl"], 0)                                   # TE-up lip lowers lift
        self.assertGreater(M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"], 0)    # and gives a nose-up moment about the NP
        self.assertAlmostEqual(M["lin_check"]["cl_avl"], M["lin_check"]["cl_pred"], delta=5e-4)
        ls = [s for s in M["strips"] if s["inlip"]]
        self.assertTrue(all(s["eta"] >= 0.4 - 1e-9 for s in ls)); self.assertTrue(all(s["K"] > 0 for s in ls))
        self.assertTrue(all(not s["inlip"] for s in M["strips"] if s["eta"] < 0.39))
        self.assertLess(M["xnp_spread"], 1e-4)                          # NP independent of alpha and lip angle

    def test_lip_prefix_sign(self):
        self.assertEqual(A.lip_prefix(5.0), ["D1 D1 -5.000000", "D2 D2 -5.000000"])

    def test_tail_model(self):
        T, P = tail_model(), pure_model()
        self.assertGreater(T["xnp"], P["xnp"])                          # tail moves the NP aft
        cm0u = T["d_cm"] - T["a_cm"] / T["a_cl"] * T["d_cl"]
        self.assertLess(cm0u, 0)                                        # positive incidence pitches nose-down; negative incidence nose-up
        self.assertAlmostEqual(T["lin_check"]["cl_avl"], T["lin_check"]["cl_pred"], delta=5e-4)
        g = A.tail_geometry(T, 0.4, V=7.0)
        S_ = T["area"] * 0.16
        self.assertAlmostEqual(g["span_mm"], 140.0); self.assertAlmostEqual(g["area_cm2"], 1e4 * 0.10 * S_, places=6)
        self.assertAlmostEqual(g["chord_mm"] * g["span_mm"], g["area_cm2"] * 100, places=5)
        self.assertAlmostEqual(g["re_tail"], 7.0 * g["chord_mm"] / 1000 / 1.5e-5)

    def test_cm_np_independent_of_cl_for_fixed_u(self):
        M = lip_model()
        a, b = A.cm_np_avl(M, 8.0, 0.2), A.cm_np_avl(M, 8.0, 0.5)
        self.assertAlmostEqual(a, b, delta=1e-4)
        P = pure_model()
        self.assertAlmostEqual(A.cm_np_avl(P, 0.0, 0.4), 0.0, places=5)


class Reductions(unittest.TestCase):
    """The two reductions required of the coupled lip model."""

    def cfg_inviscid(self, M, cm_clean=0.0):
        inv_fn = lambda cl: 0.03 + 0.01 * cl                             # arbitrary inviscid bent cm(cl)
        bent = Tab(lambda re, cl: inv_fn(cl), cl0=0.2)                   # viscous := inviscid
        clean = Tab(lambda re, cl: cm_clean)
        return A.Cfg(clean, bent, Inv(inv_fn)), inv_fn

    def test_viscous_equals_inviscid_gives_avl_only(self):
        M = lip_model()
        cfg, _ = self.cfg_inviscid(M)
        for CL in (0.2, 0.35, 0.5):
            a = A.cm_extra_arch(M, 0.4, 8.0, cfg, CL, ws=8.0, form="coupled")
            b = A.cm_extra_arch(M, 0.4, 8.0, cfg, CL, ws=8.0, form="avl_only")
            self.assertAlmostEqual(a["cm_extra"], b["cm_extra"], places=12)
            self.assertEqual(a["coverage"], 1.0)

    def test_all_clean_strips_reduce_to_pure_wing_model(self):
        P = pure_model()
        shape = m.build_shape(AR, SW, TP, 0, TMP / "shape")
        fn = lambda re, cl: 0.01 * cl + 0.002 * math.log(re / 20000)
        tab = Tab(fn)
        for CL in (0.2, 0.4):
            a = A.cm_extra_arch(P, 0.4, 0.0, A.Cfg(tab), CL, ws=8.0, form="coupled")
            ref = m.level1(shape, 0.4, tab, CL, ws=8.0)
            self.assertAlmostEqual(a["cm_extra"], shape["cm0"] + ref["cm_visc"], delta=0.02 * abs(ref["cm_visc"]) + 1e-5)   # 8x24 decks differ by ~0.5 %
        # lip model with the lip UNdeflected (u = 0) is the same wing: clean strips only
        M = lip_model()
        cfg = A.Cfg(tab, Tab(lambda re, cl: 123.0), Inv(lambda cl: 0.0))
        a = A.cm_extra_arch(M, 0.4, 0.0, cfg, 0.3, ws=8.0, form="coupled")
        ref = m.level1(shape, 0.4, tab, 0.3, ws=8.0)
        self.assertAlmostEqual(a["cm_extra"], shape["cm0"] + ref["cm_visc"], delta=0.03 * abs(ref["cm_visc"]) + 1e-5)

    def test_replace_form_formula(self):
        M = lip_model()
        cfg, inv_fn = self.cfg_inviscid(M, cm_clean=0.004)
        CL, u, b = 0.3, 8.0, 0.4
        r_c = A.cm_extra_arch(M, b, u, cfg, CL, ws=8.0, form="coupled")
        r_r = A.cm_extra_arch(M, b, u, cfg, CL, ws=8.0, form="replace")
        S_, mac = M["area"] * b * b, M["mac"] * b
        k = 2.0 / (S_ * mac)
        al = A.alpha_avl(M, u, CL)
        sc = couple = 0.0
        for s in M["strips"]:
            c, a = s["c"] * b, s["a"] * b * b
            cl = s["B"] + s["A"] * al + s["D"] * u
            if s["inlip"]:
                sc += inv_fn(cl) * c * a
                couple += s["K"] * u * c * a
            else:
                sc += 0.004 * c * a
        self.assertAlmostEqual(r_r["cm_extra"], A.cm_np_avl(M, u, CL) + k * sc - k * couple, places=12)
        # coupled form with viscous = inviscid on the lip strips and 0.004 on clean strips: AVL + clean-strip term only
        sc_clean = sum(0.004 * s["c"] * b * s["a"] * b * b for s in M["strips"] if not s["inlip"])
        self.assertAlmostEqual(r_c["cm_extra"], A.cm_np_avl(M, u, CL) + k * sc_clean + k * sum(0.0 for _ in M["strips"]), places=10)
        self.assertNotAlmostEqual(r_c["cm_extra"], r_r["cm_extra"], places=6)    # AVL lip couple != XFOIL inviscid -> the two forms differ

    def test_uncovered_strip_lowers_coverage_and_is_excluded(self):
        M = lip_model()
        class Partial(Tab):
            def lookup(self, re, cl):
                return None if re < 20000 else super().lookup(re, cl)
        cfg = A.Cfg(Partial(lambda re, cl: 0.0), Partial(lambda re, cl: 0.0, cl0=0.2), Inv(lambda cl: 0.0))
        r = A.cm_extra_arch(M, 0.4, 5.0, cfg, 0.3, V=5.0, form="coupled")          # V = 5 m/s: tip strips Re < 20k
        self.assertLess(r["coverage"], 1.0); self.assertGreater(r["coverage"], 0.0)


class Windows(unittest.TestCase):
    def test_window_rows_width_convention_and_avl_only(self):
        key = "lip|P07|0.2|outer"
        models = {key: dict(mac=0.23, xnp=0.175, area=0.22)}
        S_sets = lambda sm: {md: list(sm) for md in v2.MODES}
        mk = lambda form, nc, pan, sm: dict(kind="lip", wing="P07", section="NACA_0006", cfg=[0.2, "outer"], u=10.0, ws=8, form=form, ncrit=nc, panels=pan,
                                             sets=S_sets(sm), pts=[[x, 0.3, -0.1, -0.1, 0.0, 0.0] for x in sm])
        recs = [mk("avl_only", None, None, [0.01, 0.015, 0.02, 0.03]), mk("coupled", 5, 160, [0.01, 0.015, 0.02, 0.025]),
                mk("coupled", 9, 160, [0.015, 0.02, 0.025, 0.03]), mk("coupled", 5, 240, [0.01]), mk("coupled", 9, 240, [0.01])]
        rows = S.window_rows(recs, models)
        get = lambda form, pan, tier, mode="any": [r for r in rows if r["form"] == form and r["panels"] == pan and r["tier"] == tier and r["mode"] == mode][0]
        mac_mm = 0.23 * 0.4 * 1000
        self.assertEqual(get("avl_only", None, "avl")["widest_n"], 3)                        # 0.01..0.02 contiguous; 0.03 isolated
        self.assertAlmostEqual(get("avl_only", None, "avl")["width_mm"], 2 * 0.005 * mac_mm)
        b = get("coupled", 160, "both")
        self.assertEqual(b["n_points"], 3); self.assertAlmostEqual(b["width_mm"], 2 * 0.005 * mac_mm)
        self.assertAlmostEqual(b["cg_mm_hi"] - b["cg_mm_lo"], b["width_mm"], places=9)
        self.assertEqual(get("coupled", 240, "both")["n_points"], 1); self.assertAlmostEqual(get("coupled", 240, "both")["width_mm"], 0.0)

    def test_nominal_helpers(self):
        pts = [[0.10, 0.30, -0.1, -0.1, 0, 0], [0.105, 0.28, -0.1, -0.1, 0, 0], [0.11, 0.26, -0.1, -0.1, 0, 0]]
        self.assertIsNotNone(S.pt_at(pts, 0.105)); self.assertIsNone(S.pt_at(pts, 0.2))
        self.assertAlmostEqual(S.cl_slope_per_mm(pts, 0.105, 100.0), abs((0.26 - 0.30) / 0.01) / 100.0)
        self.assertAlmostEqual(S.cl_slope_per_mm(pts, 0.10, 100.0), abs((0.28 - 0.30) / 0.005) / 100.0)


class LateralDecks(unittest.TestCase):
    def setUp(self):
        self.w = m._wing(AR, SW, TP, 0)

    def test_control_numbering_and_fin_geometry(self):
        fin = dict(area_frac=0.06, x_qc=1.5 * self.w.root_chord)
        txt = A.lat_deck(self.w, "t", lip=(0.2, "outer"), fin=fin)
        names = []
        for ln in txt.splitlines():
            if ln.startswith("refl") or ln.startswith("finyaw"):
                n = ln.split()[0]
                if n not in names:
                    names.append(n)
        self.assertEqual(names, ["reflL", "reflR", "finyaw"])           # fin control is D3 when a lip is present
        self.assertEqual(A.fin_ctrl_index((0.2, "outer")), 3); self.assertEqual(A.fin_ctrl_index(None), 1)
        txt2 = A.lat_deck(self.w, "t", fin=fin)
        self.assertNotIn("reflL", txt2)
        part = txt2.split("SURFACE\nFin\n")[1].splitlines()
        secs = [[float(x) for x in part[i + 1].split()] for i, l in enumerate(part) if l == "SECTION"]
        (x1, y1, z1, c1, _), (x2, y2, z2, c2, _) = secs
        self.assertAlmostEqual(c1, 0.5 * self.w.root_chord); self.assertAlmostEqual((z1 - z2) * c1, 0.06 * self.w.area, places=8)
        self.assertAlmostEqual(x1 + 0.25 * c1, 1.5 * self.w.root_chord); self.assertEqual(y1, 0.0)

    def test_outer_lip_controls_only_outboard(self):
        txt = A.lat_deck(self.w, "t", lip=(0.2, "outer"))
        right = txt.split("SURFACE\nRight\n")[1]
        sections = right.split("SECTION\n")[1:]
        self.assertEqual(len(sections), 3)
        self.assertNotIn("CONTROL", sections[0])                         # root section: no lip
        self.assertIn("reflR 1.0 0.800000", sections[1]); self.assertIn("reflR 1.0 0.800000", sections[2])
        full = A.lat_deck(self.w, "t", lip=(0.2, "full")).split("SURFACE\nRight\n")[1].split("SECTION\n")[1:]
        self.assertTrue(all("CONTROL" in s for s in full))

    def test_dihedral_z_and_tail_surface(self):
        w3 = m._wing(AR, SW, TP, 3)
        txt = A.lat_deck(w3, "t", tail=dict(area_frac=0.1, x_qc=2 * w3.root_chord, incidence=-3.0))
        right = txt.split("SURFACE\nRight\n")[1].split("SURFACE")[0].split("SECTION\n")[2].split()
        self.assertAlmostEqual(float(right[2]), w3.tip_z, places=8)       # tip z = b/2 tan(dihedral)
        tail = txt.split("SURFACE\nTail\n")[1]
        self.assertIn("YDUPLICATE", tail); self.assertIn("-3.000000000", tail)

    def test_fin_adds_weathervane_stiffness(self):
        P = pure_model()
        xcg = P["xnp"] - 0.05 * P["mac"]
        bare = A.lateral_arch(AR, SW, TP, 0, xcg, TMP / "lat_bare", "b", P, "pure", 0.0)
        fin = A.lateral_arch(AR, SW, TP, 0, xcg, TMP / "lat_fin", "f", P, "pure", 0.0, fin=dict(area_frac=0.06, x_qc=1.5 * P["root_chord"]))
        self.assertGreater(fin["cn_beta"]["0.3"], bare["cn_beta"]["0.3"] + 0.01)
        self.assertGreater(abs(fin["yaw_per_deg_fin_misalignment"]), 1e-5)
        self.assertLess(bare["roll_per_deg_incidence_mismatch"] * 1.0, 0.0 + 1e-9)    # sign convention fixed by AVL: documented in the report


class BuildNumbers(unittest.TestCase):
    def test_lip_rise_and_geometry(self):
        models = {S.mkey("pure", "P07"): dict(root_chord=0.2963, tip_chord=0.1481, mac=0.2305, x_lemac=0.12, xnp=0.175, area=0.2222, tip_x_le=0.27)}
        rows = R.lip_rise_table(models, "P07")
        r = [x for x in rows if x["lip"] == "20/10" and x["placement"] == "outer"][0]
        c04 = 0.4 * (0.2963 + (0.1481 - 0.2963) * 0.4)               # chord (m) at 40 % semispan for b = 0.4 m
        self.assertAlmostEqual(r["lip_chord_inboard_mm"], 1000 * 0.2 * c04, places=6)
        self.assertAlmostEqual(r["te_rise_tip_mm"], 1000 * 0.2 * 0.4 * 0.1481 * math.sin(math.radians(10)), places=6)
        g = R.geometry_sheet(models, "P07")
        self.assertAlmostEqual(g["le_sweep_deg"], math.degrees(math.atan(0.27 * 0.4 / 0.2)), places=6)


if __name__ == "__main__":
    unittest.main()
