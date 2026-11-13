"""Stage-3 additions to the architecture model (wraps arch_model / arch_study / pure_wing_*; edits none of them).

* Cfg2 / cm_extra_c       : lip form (c) -- "replace" form with the AVL lift redistribution scaled by a retained fraction k
                            (single area-weighted k per case, iterated with CL; weights |D_i| A_i of the lip strips).
* build_curve_ev          : curve builder for any evaluation closure (same dict format as pure_wing_study.build_curve).
* scan_curve2 / window_rows2 : uncensored SM scan (-0.15 .. +0.60) and window rows with edge flags and window centre.
* deck_full / build_*_final : ONE deck definition (20x32 panels, dihedral 3 deg, 6 % centre fin; tail case: tail + boom fin).
* lip_decomposition       : AVL lip Cm0 split into lift-redistribution part and section-couple part.
* mass budget helpers.
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import json
import math
import statistics
from pathlib import Path

import arch_model as A
import pure_wing_model as m
import pure_wing_v2_model as v2
from avl import run_avl, geometry_text
from geometry import Wing

ROOT = A.ROOT
OUT = ROOT / "results/runs/rigid_glider/arch_compare_v2"
NU = A.NU
G = 9.80665
BAND = (0.2, 0.5)
SM_MIN, SM_MAX, STEP = -0.15, 0.60, 0.005
SM_GRID = [round(SM_MIN + STEP * i, 4) for i in range(int(round((SM_MAX - SM_MIN) / STEP)) + 1)]
MODES = v2.MODES


# ------------------------------------------------------------------ form (c)
class Cfg2(A.Cfg):
    def __init__(self, clean, bent=None, inv=None, inv_clean=None):
        super().__init__(clean, bent, inv)
        self.inv_clean = inv_clean


def cm_extra_c(M, b, u, cfg, CL, ws=None, V=None, nu=NU, maxit=40, relax=0.6, tol=2e-3):
    """Form (c): replace-form couple with lift redistribution scaled by k = viscous/inviscid lip moment at matched cl (self-consistent in CL).
    The lip AVL deflection is u_eff = k*u in the lift/moment of the loading; the AVL strip couple at u_eff is removed and the XFOIL viscous
    section cm (bent section for lip strips, clean elsewhere) is used for every strip."""
    if V is None:
        V = m.glide_speed(ws, CL)
    S, mac = M["area"] * b * b, M["mac"] * b
    lips = [s for s in M["strips"] if s["inlip"]]
    k, converged, it = 1.0, False, 0
    for it in range(1, maxit + 1):
        ue = k * u
        al = A.alpha_avl(M, ue, CL)
        num = den = 0.0
        for s in lips:
            c = s["c"] * b
            cl = s["B"] + s["A"] * al + s["D"] * ue
            re = V * c / nu
            rb, rc = cfg.bent.lookup(re, cl), cfg.clean.lookup(re, cl)
            ib, ic = cfg.inv.cm_at(cl), cfg.inv_clean.cm_at(cl)
            if rb is None or rc is None or ib is None or ic is None or abs(ib - ic) < 1e-6:
                continue
            w = abs(s["D"]) * s["a"]
            num += w * (rb["cm"] - rc["cm"]) / (ib - ic)
            den += w
        knew = k if den == 0 else min(1.0, max(0.0, num / den))
        kn = k + relax * (knew - k)
        if abs(kn - k) < tol:
            k, converged = kn, True
            break
        k = kn
    ue = k * u
    al = A.alpha_avl(M, ue, CL)
    cmn = A.cm_np_avl(M, ue, CL)
    sc = couple = ca = at = cd_s = jag = da_s = 0.0
    for s in M["strips"]:
        c, a = s["c"] * b, s["a"] * b * b
        at += a
        cl = s["B"] + s["A"] * al + s["D"] * ue
        tb = cfg.bent if (s["inlip"] and ue != 0) else cfg.clean
        r = tb.lookup(V * c / nu, cl)
        if r is None:
            continue
        ca += a
        sc += r["cm"] * c * a
        if s["inlip"]:
            couple += s["K"] * ue * c * a
        cd_s += r["cd"] * a
        da_s += (r["alpha"] - tb.alpha_inv(cl)) * a
        if r["steep"] or r["nbranch"] > 1:
            jag += a
    kk = 2.0 / (S * mac)
    cm_visc = kk * sc - kk * couple
    return dict(cm_extra=cmn + cm_visc, cm_np=cmn, cm_visc=cm_visc, coverage=ca / at, jag=jag / at, V=V, cdp=cd_s / ca if ca else None,
                dalpha=da_s / ca if ca else None, alpha_avl=al, k=k, converged=converged, iterations=it)


# ------------------------------------------------------------------ curves and scans
def build_curve_ev(ev, ti_fn=None, ti_band=(0.15, 0.6), dcl=0.02, avl_only=False):
    """ev(CL, V) -> dict(cm_extra, coverage, jag, V, ...).  Same curve dicts as pure_wing_study.build_curve (+ti, +k flags)."""
    rs = [ev(CL, None) for CL in A.GRID]
    n = len(rs)
    curve = []
    nonconv = 0
    for i, (CL, r) in enumerate(zip(A.GRID, rs)):
        cov, V = r["coverage"], r["V"]
        if 2 <= i < n - 2:
            dg = (rs[i + 2]["cm_extra"] - rs[i - 2]["cm_extra"]) / 0.04
            covd = min(rs[i + 2]["coverage"], rs[i - 2]["coverage"], cov)
        else:
            dg, covd = None, 0.0
        df = None
        if cov > 0 and 0.02 <= CL <= 1.15:
            fp, fm = ev(CL + dcl, V), ev(CL - dcl, V)
            df = (fp["cm_extra"] - fm["cm_extra"]) / (2 * dcl)
            covd = min(covd, fp["coverage"], fm["coverage"])
        ti = 0.0 if avl_only else None
        if ti_fn is not None and ti_band[0] <= CL <= ti_band[1] and cov > 0:
            ti = ti_fn(CL, V, r)
        if r.get("converged") is False:
            nonconv += 1
        curve.append(dict(cl=CL, cm=r["cm_extra"], cov=cov, jag=r["jag"], dg=dg, df=df, covd=covd, ti=ti, V=V, k=r.get("k")))
    return curve, nonconv


def scan_curve2(curve, sm_grid=SM_GRID, band=BAND):
    sets = {md: [] for md in MODES}
    pts = []
    for sm in sm_grid:
        rs = v2.roots_from_curve_v2(curve, sm)
        for md in MODES:
            if any(v2.is_good_v2(r, band, md) for r in rs):
                sets[md].append(sm)
        good = [r for r in rs if v2.is_good_v2(r, band, "any")]
        if good:
            r0 = min(good, key=lambda r: r["cl"])
            pts.append([sm, r0["cl"], r0["dg"], r0["df"], r0["jag"], r0["ti"]])
    return sets, pts


def evaluators(M, b, u, kind, tabs, sec, form, nc, pan, cfgkey, ws):
    """-> (ev closure, ti_fn, avl_only flag).  form: avl_only | coupled | replace | replace_k."""
    if form == "avl_only":
        return (lambda CL, V: A.cm_extra_arch(M, b, u, A.Cfg(None), CL, ws=ws, V=V, form="avl_only")), None, True
    clean = tabs.clean(sec, nc, pan)
    if kind == "lip":
        cf = cfgkey[0]
        cfg = Cfg2(clean, tabs.bent(sec, cf, u, nc, pan), tabs.inv_curve(sec, cf, u, pan), tabs.inv_curve(sec, 0.0, 0.0, pan))
    else:
        cfg = Cfg2(clean)
    uu = u if kind != "pure" else 0.0
    if form == "replace_k":
        return (lambda CL, V: cm_extra_c(M, b, uu, cfg, CL, ws=ws, V=V)), (lambda CL, V, r: A.ti_fraction_arch(M, b, r["k"] * uu, cfg, CL, V)), False
    f = "coupled" if form == "coupled" else "replace"
    return (lambda CL, V: A.cm_extra_arch(M, b, uu, cfg, CL, ws=ws, V=V, form=f)), (lambda CL, V, r: A.ti_fraction_arch(M, b, uu, cfg, CL, V)), False


# ------------------------------------------------------------------ windows
def window_rows2(recs, models_by_deck, b=0.40):
    """One row per (case, form, panels, tier, mode).  Edge flags: the window touches the scan edge (-0.15 / +0.60) => censored."""
    from collections import defaultdict
    grp = defaultdict(list)
    for r in recs:
        grp[(r["deck"], r["kind"], r["wing"], r["section"], json.dumps(r["cfg"]), r["u"], r["ws"])].append(r)
    rows = []
    for (deck, kind, wing, sec, cfgj, u, ws), rs in grp.items():
        cfg = json.loads(cfgj)
        M = models_by_deck[deck][model_key(kind, wing, cfg)]
        mac_mm, xnp_mm = M["mac"] * b * 1000, M["xnp"] * b * 1000
        base = dict(deck=deck, kind=kind, wing=wing, section=sec, cfg1=(cfg[0] if cfg else None), cfg2=(cfg[1] if cfg else None), u=u, ws=ws)

        def emit(form, pan, tier, mode, sm_set, ptsets):
            w = v2.contiguous_window(sm_set, STEP)
            row = dict(base, form=form, panels=pan, tier=tier, mode=mode, n_points=w["n"], widest_n=w["widest_n"], runs=w["runs"],
                       width_mm=None if w["width"] is None else 1000 * w["width"] * M["mac"] * b)
            if w["n"]:
                lo, hi = w["sm_lo"], w["sm_hi"]
                inw = [p for pts in ptsets for p in pts if lo - 1e-9 <= p[0] <= hi + 1e-9]
                row.update(sm_lo=lo, sm_hi=hi, sm_centre=0.5 * (lo + hi), cg_mm_lo=xnp_mm - hi * mac_mm, cg_mm_hi=xnp_mm - lo * mac_mm,
                           cg_mm_centre=xnp_mm - 0.5 * (lo + hi) * mac_mm, cl_lo=min(p[1] for p in inw), cl_hi=max(p[1] for p in inw),
                           touches_lo_edge=int(lo <= SM_MIN + 1e-9), touches_hi_edge=int(hi >= SM_MAX - 1e-9))
            rows.append(row)

        av = [r for r in rs if r["form"] == "avl_only"][0]
        emit("avl_only", None, "avl", "any", av["sets"]["any"], [av["pts"]])
        for form in ("coupled", "replace", "replace_k"):
            for pan in (160, 240):
                by = {r["ncrit"]: r for r in rs if r["form"] == form and r["panels"] == pan}
                if 5 not in by or 9 not in by:
                    continue
                for mode in MODES:
                    s5, s9 = set(by[5]["sets"][mode]), set(by[9]["sets"][mode])
                    emit(form, pan, "N5", mode, s5, [by[5]["pts"]])
                    emit(form, pan, "N9", mode, s9, [by[9]["pts"]])
                    emit(form, pan, "both", mode, s5 & s9, [by[5]["pts"], by[9]["pts"]])
    return rows


def model_key(kind, wing, cfg):
    f = lambda c: f"{c:g}" if isinstance(c, float) else str(c)
    if kind == "pure":
        return f"pure|{wing}"
    return "|".join([kind, wing, f(cfg[0]), f(cfg[1])])


# ------------------------------------------------------------------ one consistent deck
def deck_full(w: Wing, name: str, lip=None, tail=None, fin=None, tip_fin=None, nchord=20, nspan=32, tail_z=0.0) -> str:
    """Left/Right wing surfaces (dihedral from w), optional lip controls (reflL=D1, reflR=D2), horizontal tail (dict(area_frac, x_qc, incidence,
    span_frac=0.35), raised tail_z above the root plane), vertical centre fin (dict(area_frac, x_qc); chord 0.5 root chord, above the plane, finyaw
    control last), tip fins (dict(area_frac, mode 'above'|'sym'); each fin: chord = tip chord, area_frac of wing area)."""
    txt = A.lat_deck(w, name, lip=lip, tail=None, fin=None, nchord=nchord, nspan=nspan)
    if tail:
        ts = tail.get("span_frac", 0.35) * w.span_m
        tc = tail["area_frac"] * w.area / ts
        x = tail["x_qc"] - tc / 4
        txt += "SURFACE\nTail\n6 1.0 10 -2.0\nYDUPLICATE\n0.0\n"
        txt += f"SECTION\n{x:.9f} 0.0 {tail_z:.9f} {tc:.9f} {tail['incidence']:.9f}\nNACA\n0006\n"
        txt += f"SECTION\n{x:.9f} {ts/2:.9f} {tail_z:.9f} {tc:.9f} {tail['incidence']:.9f}\nNACA\n0006\n"
    if fin:
        fc = 0.5 * w.root_chord
        h = fin["area_frac"] * w.area / fc
        x = fin["x_qc"] - 0.25 * fc
        ctl = "CONTROL\nfinyaw 1.0 0.0 0.0 0.0 1.0 1.0\n"
        txt += "SURFACE\nFin\n5 1.0 8 -2.0\n"
        txt += f"SECTION\n{x:.9f} 0.0 {h:.9f} {fc:.9f} 0.0\nNACA\n0008\n{ctl}"
        txt += f"SECTION\n{x:.9f} 0.0 0.0 {fc:.9f} 0.0\nNACA\n0008\n{ctl}"
    if tip_fin:
        ct = w.tip_chord
        h = tip_fin["area_frac"] * w.area / ct
        top, bot = (w.tip_z + h, w.tip_z) if tip_fin["mode"] == "above" else (w.tip_z + h / 2, w.tip_z - h / 2)
        txt += v2.fin_surface_text("TipFin", w.tip_x_le, w.span_m / 2, top, bot, ct, True)
    return txt


FIN_STD = 0.06
FIN_STD_X = 1.5


def final_wing(ar, sw, tp, dih=3, xref=None):
    return m._wing(ar, sw, tp, dih, xref=xref)


def build_pure_final(ar, sw, tp, workdir, nchord=20, nspan=32, dih=3, fin=True):
    w = final_wing(ar, sw, tp, dih)
    d = deck_full(w, "pure", fin=dict(area_frac=FIN_STD, x_qc=FIN_STD_X * w.root_chord) if fin else None, nchord=nchord, nspan=nspan)
    return A.build_linear(w, "pure", workdir, lambda u: d, lambda u: [], 0.0, None, "pure", dict(ar=ar, sweep=sw, taper=tp, deck=f"{nchord}x{nspan} dih{dih} fin{int(fin)}"))


def build_lip_final(ar, sw, tp, cf, workdir, nchord=20, nspan=32, dih=3, fin=True):
    w = final_wing(ar, sw, tp, dih)
    d = deck_full(w, "lip", lip=(cf, "outer"), fin=dict(area_frac=FIN_STD, x_qc=FIN_STD_X * w.root_chord) if fin else None, nchord=nchord, nspan=nspan)
    return A.build_linear(w, "lip", workdir, lambda u: d, A.lip_prefix, 8.0, A.OUTER_SPLIT, "lip",
                          dict(ar=ar, sweep=sw, taper=tp, cf=cf, placement="outer", deck=f"{nchord}x{nspan} dih{dih} fin{int(fin)}"))


def build_tail_final(ar, sw, tp, area_frac, workdir, x_qc_cr=2.0, nchord=20, nspan=32, dih=3, boom_fin=True, tail_z_cr=0.0, u_check=-4.0):
    w = final_wing(ar, sw, tp, dih)
    cr = w.root_chord
    deck = lambda u: deck_full(w, "tail", tail=dict(area_frac=area_frac, x_qc=x_qc_cr * cr, incidence=u),
                               fin=dict(area_frac=FIN_STD, x_qc=x_qc_cr * cr) if boom_fin else None, nchord=nchord, nspan=nspan, tail_z=tail_z_cr * cr)
    return A.build_linear(w, "tail", workdir, deck, lambda u: [], u_check, None, "tail",
                          dict(ar=ar, sweep=sw, taper=tp, area_frac=area_frac, x_qc_over_cr=x_qc_cr, x_qc=x_qc_cr * cr, deck=f"{nchord}x{nspan} dih{dih} fin{int(boom_fin)} z{tail_z_cr}"))


def xnp_of(w: Wing, deck_text: str, workdir: Path, name="x", oper=None):
    p = run_avl(w, name, [3.0], workdir, A.AVL_EXE, deck_text=deck_text, oper_prefix=oper or [])[0]
    return p.xnp_m, p.cl / 3.0, p


# ------------------------------------------------------------------ lip Cm0 decomposition
def lip_decomposition(M: dict) -> dict:
    """Per degree of lip angle (b = 1 m units, about the neutral point): AVL total Cm0u, section-couple part, lift-redistribution part,
    and an independent estimate of the lift-arm part from the strip lifts (-2 sum D_i A_i (x_qc,i - x_np)/(S MAC))."""
    cm0u = M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"]
    S, mac = M["area"], M["mac"]
    lips = [s for s in M["strips"] if s["inlip"]]
    couple = 2.0 * sum(s["K"] * s["c"] * s["a"] for s in lips) / (S * mac)
    lift_arm = -2.0 * sum(s["D"] * s["a"] * (M["tip_x_le"] * s["eta"] + 0.25 * s["c"] - M["xnp"]) for s in M["strips"]) / (S * mac)
    return dict(cm0u_total=cm0u, couple=couple, redistribution=cm0u - couple, redistribution_independent=lift_arm,
                frac_couple=couple / cm0u, frac_redistribution=(cm0u - couple) / cm0u,
                closure_error=(cm0u - couple - lift_arm) / cm0u, d_cl_check=2.0 * sum(s["D"] * s["a"] for s in M["strips"]) / S - M["d_cl"])


# ------------------------------------------------------------------ mass budget (analytic, exploratory)
WING_CG_CHORD_FRACTION = 0.42


def planform_centroid_x(cr, ct, tip_x_le, half_span, frac=WING_CG_CHORD_FRACTION, n=2000):
    """x of the area centroid of the trapezoid with mass centre at `frac` of the local chord (LE x = tip_x_le*eta)."""
    num = den = 0.0
    for i in range(n):
        eta = (i + 0.5) / n
        c = cr + (ct - cr) * eta
        num += (tip_x_le * eta + frac * c) * c
        den += c
    return num / den


def wing_mass(model: str, S, mac, sigma=0.5, rho_eff=250.0, tc=0.06, mfix=0.0):
    return sigma * S if model == "shell" else rho_eff * 0.685 * tc * S * mac


def ballast_for_target(parts, x_target, x_ballast):
    """parts = [(mass, x)].  Mass at x_ballast to bring the total CG to x_target (negative => structure CG already ahead of target)."""
    mt = sum(p[0] for p in parts)
    num = sum(p[0] * (p[1] - x_target) for p in parts)
    return num / (x_target - x_ballast) if x_target != x_ballast else float("inf")
