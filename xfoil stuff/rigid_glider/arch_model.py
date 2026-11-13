"""Architecture comparison model: pure wing / reflex lip / small tail on the same planform, coupled to low-Re section data.

Reuses (never edits) pure_wing_model, pure_wing_v2_model, reflex_geometry, study_geometry, avl.

AVL is linear in (alpha, feature) for planar wings:   CL = cl0 + a_cl*alpha + d_cl*u ;  Cm_ref = cm0 + a_cm*alpha + d_cm*u
strip cl_i = B_i + A_i*alpha + D_i*u ;  strip cm_c/4 (AVL) = K_i*u.   u = lip angle (deg, TE-UP positive) or tail incidence (deg).
All AVL data are for a span-normalised (b = 1 m) planform and scaled.

Moment about the CG:   Cm = Cm_np(CL,u) - SM*CL + viscous terms,  SM = (Xnp - Xcg)/MAC, positive nose-up.
Viscous forms (kept separate):
  avl_only : no viscous term
  coupled  : Cm_np + sum_i w_i [cm_visc,i - cm_inv,i]  (lip strips: bent XFOIL viscous minus bent XFOIL inviscid at the same cl;
             clean strips: cm_visc only, i.e. the existing pure-wing model, clean XFOIL inviscid cm taken as 0)
  replace  : (2b) the AVL lip couple (sum_lip w_i K_i u) is removed and the XFOIL viscous section moment is used for every strip:
             Cm_np - u sum_lip w_i K_i + sum_i w_i cm_visc,i
with w_i = 2 c_i A_i /(S MAC) over the right-half strips.  Section cm is about the local quarter chord (streamwise strips, no sweep
correction) and acts as a couple.
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
from pathlib import Path

import pure_wing_model as m
import pure_wing_v2_model as v2
import pure_wing_reanalysis as ra
import reflex_geometry as rg
import study_geometry as sg
from avl import run_avl
from geometry import Wing

ROOT = m.ROOT
OUT = ROOT / "results/runs/rigid_glider/arch_compare"
LIP_DB = ROOT / "results/runs/rigid_glider/reflex_small/polars/lip_polar_database.csv"
AVL_EXE = m.AVL_EXE
NU = m.NU
G = 9.80665
GRID = [round(0.02 + 0.01 * i, 3) for i in range(119)]
LIPS = [(0.10, 3.0), (0.10, 6.0), (0.15, 5.0), (0.15, 8.0), (0.20, 5.0), (0.20, 10.0)]
OUTER_SPLIT = 0.4


# ------------------------------------------------------------------ AVL strips (per surface, with cm_c/4)
def parse_strips(path: Path) -> dict[str, list[dict]]:
    out, cur, inside = {}, None, False
    for line in Path(path).read_text(errors="replace").splitlines():
        s = line.strip()
        if s.startswith("Surface #"):
            cur = " ".join(s.split()[3:])
            out[cur] = []
            inside = False
        elif "Strip Forces referred to Strip Area, Chord" in line:
            inside = True
        elif inside and cur is not None:
            p = line.split()
            if len(p) >= 15 and p[0].isdigit():
                out[cur].append(dict(y=float(p[2]), c=float(p[4]), a=float(p[5]), cl=float(p[9]), cm=float(p[12])))
    return out


def wing_right_strips(path: Path, names=("MainWing", "Right", "Left")) -> list[dict]:
    res = []
    for nm, rows in parse_strips(path).items():
        if nm.split(" (")[0] in names:
            res += [r for r in rows if r["y"] > 0]
    res.sort(key=lambda r: r["y"])
    return res


# ------------------------------------------------------------------ linear AVL model
def build_linear(w: Wing, name: str, workdir: Path, deck_for_u, prefix_for_u, u_check: float, split_eta: float | None,
                 kind: str, meta: dict) -> dict:
    """deck_for_u(u) -> deck text ; prefix_for_u(u) -> list of AVL oper commands.  Runs: (alpha 0,3 | u=0), (alpha 0 | u=1), (alpha 0 | u_check)."""
    pb = run_avl(w, name, [0.0, 3.0], workdir / "base", AVL_EXE, deck_text=deck_for_u(0.0), oper_prefix=prefix_for_u(0.0))
    pu = run_avl(w, name, [0.0], workdir / "unit", AVL_EXE, deck_text=deck_for_u(1.0), oper_prefix=prefix_for_u(1.0))[0]
    pc = run_avl(w, name, [0.0], workdir / "check", AVL_EXE, deck_text=deck_for_u(u_check), oper_prefix=prefix_for_u(u_check))[0]
    p0, p3 = pb
    s0, s3 = wing_right_strips(p0.strip_file), wing_right_strips(p3.strip_file)
    su, sc = wing_right_strips(pu.strip_file), wing_right_strips(pc.strip_file)
    a_cl = (p3.cl - p0.cl) / 3.0
    a_cm = (p3.cm - p0.cm) / 3.0
    d_cl, d_cm = pu.cl - p0.cl, pu.cm - p0.cm
    strips = []
    for i, (a, b3, c1) in enumerate(zip(s0, s3, su)):
        eta = a["y"] / (w.span_m / 2)
        strips.append(dict(eta=eta, c=a["c"], a=a["a"], B=a["cl"], A=(b3["cl"] - a["cl"]) / 3.0, D=c1["cl"] - a["cl"],
                           K=c1["cm"] - a["cm"], inlip=bool(split_eta is not None and eta >= split_eta - 1e-9)))
    # linearity check at u_check
    pred_cl = p0.cl + d_cl * u_check
    pred_cm = p0.cm + d_cm * u_check
    lin = dict(u_check=u_check, cl_pred=pred_cl, cl_avl=pc.cl, cm_pred=pred_cm, cm_avl=pc.cm,
               max_strip_cl_err=max(abs(s["cl"] - (a["cl"] + u_check * x["D"])) for s, a, x in zip(sc, s0, strips)) if sc else None)
    return dict(kind=kind, name=name, meta=meta, a_cl=a_cl, a_cm=a_cm, d_cl=d_cl, d_cm=d_cm, cl0=p0.cl, cm0=p0.cm,
                xnp=sum(p.xnp_m for p in (p0, p3, pu)) / 3, xnp_spread=max(p.xnp_m for p in (p0, p3, pu)) - min(p.xnp_m for p in (p0, p3, pu)),
                xref=w.cg_x_m, mac=w.mac, area=w.area, x_lemac=w.x_le(w.mac_y), k_cdi=p3.cdi / p3.cl ** 2,
                cla_per_rad=a_cl * 180 / math.pi, strips=strips, lin_check=lin, root_chord=w.root_chord,
                tip_chord=w.tip_chord, tip_x_le=w.tip_x_le)


def build_pure(ar, sweep, taper, workdir: Path) -> dict:
    w = m._wing(ar, sweep, taper, 0)
    d = sg.deck(w, "pure")
    return build_linear(w, "pure", workdir, lambda u: d, lambda u: [], 0.0, None, "pure", dict(ar=ar, sweep=sweep, taper=taper))


def lip_prefix(u: float) -> list[str]:
    return [f"D1 D1 {-u:.6f}", f"D2 D2 {-u:.6f}"]          # AVL: negative control value = trailing edge UP (as reflex_ext_avl._controls)


def build_lip(ar, sweep, taper, cf, placement, workdir: Path, u_check=8.0) -> dict:
    w = m._wing(ar, sweep, taper, 0)
    refl = rg.Reflex(cf, 1.0, placement, split_fraction=OUTER_SPLIT if placement == "outer" else 0.5)
    d = rg.deck(w, "lip", refl)
    return build_linear(w, "lip", workdir, lambda u: d, lip_prefix, u_check, OUTER_SPLIT if placement == "outer" else 0.0, "lip",
                        dict(ar=ar, sweep=sweep, taper=taper, cf=cf, placement=placement))


def build_tail(ar, sweep, taper, area_frac, x_qc_over_cr, workdir: Path, u_check=-4.0) -> dict:
    w = m._wing(ar, sweep, taper, 0)
    x_qc = x_qc_over_cr * w.root_chord
    deck = lambda u: sg.deck(w, "tail", tail_area_fraction=area_frac, tail_incidence_deg=u, tail_ac_x_m=x_qc, tail_span_fraction=0.35)
    return build_linear(w, "tail", workdir, deck, lambda u: [], u_check, None, "tail",
                        dict(ar=ar, sweep=sweep, taper=taper, area_frac=area_frac, x_qc_over_cr=x_qc_over_cr, x_qc=x_qc))


def tail_geometry(M: dict, b: float, V: float | None = None) -> dict:
    mt = M["meta"]
    S = M["area"] * b * b
    span = 0.35 * b
    chord = mt["area_frac"] * S / span
    g = dict(area_cm2=1e4 * mt["area_frac"] * S, span_mm=1000 * span, chord_mm=1000 * chord, x_qc_mm=1000 * mt["x_qc"] * b,
             arm_from_np_mm=1000 * (mt["x_qc"] - M["xnp"]) * b)
    if V:
        g["re_tail"] = V * chord / NU
    return g


# ------------------------------------------------------------------ section tables
class BentSectionTable(m.SectionTable):
    """Attached-branch lookup for a section with a lip: the inviscid lift line is affine (cl = cl0 + k alpha), no odd symmetry."""

    def __init__(self, visc_rows, inv_rows, **kw):
        super().__init__(visc_rows, inv_rows, **kw)
        n = len(inv_rows)
        sa = sum(r["alpha"] for r in inv_rows); sc = sum(r["cl"] for r in inv_rows)
        saa = sum(r["alpha"] ** 2 for r in inv_rows); sac = sum(r["alpha"] * r["cl"] for r in inv_rows)
        self.k = (n * sac - sa * sc) / (n * saa - sa * sa)
        self.cl0 = (sc - self.k * sa) / n

    def alpha_inv(self, cl):
        return (cl - self.cl0) / self.k

    def lookup(self, re, cl):
        if cl < 0:
            return None
        return super().lookup(re, cl)


class InvCurve:
    """XFOIL inviscid cm(cl) by linear interpolation between adjacent alphas."""

    def __init__(self, rows):
        self.pts = sorted(((r["cl"], r["cm"]) for r in rows))

    def cm_at(self, cl):
        p = self.pts
        if not p or cl < p[0][0] or cl > p[-1][0]:
            return None
        for (c0, m0), (c1, m1) in zip(p, p[1:]):
            if c0 <= cl <= c1 and c1 > c0:
                return m0 + (cl - c0) / (c1 - c0) * (m1 - m0)
        return None


def read_lip_db(path: Path = LIP_DB):
    rows = []
    with path.open(encoding="utf-8") as s:
        for r in csv.DictReader(s):
            rows.append(dict(section=r["section"], ncrit=int(float(r["ncrit"])), re=int(float(r["re"])), alpha=float(r["alpha"]),
                             cl=float(r["cl"]), cd=float(r["cd"]), cdp=float(r["cdp"]), cm=float(r["cm"]), xtr_top=float(r["xtr_top"]),
                             xtr_bot=float(r["xtr_bot"]), pass_=r["pass"], cf=float(r["lip_chord_frac"]), ang=float(r["lip_angle_deg"]),
                             panels=int(r["panels"])))
    return rows


class ArchTables:
    """clean[(section, ncrit, panels)], bent[(section, cf, ang, ncrit, panels)], inv[(section, cf, ang, panels)]."""

    def __init__(self, db_path: Path = LIP_DB, pure_tables=None):
        rows = read_lip_db(db_path)
        pure = pure_tables or m.load_tables()
        self.clean_cache = {}
        self.pure160 = pure
        grp = {}
        for r in rows:
            grp.setdefault((r["section"], r["cf"], r["ang"], r["panels"], r["ncrit"]), {}).setdefault(r["re"], []).append(r)
        self.grp = grp
        self.inv = {}
        for (sec, cf, ang, pan, nc), byre in grp.items():
            if nc == 0:
                rr = sorted(byre[0], key=lambda x: x["alpha"])
                self.inv[(sec, cf, ang, pan)] = rr
        self.bent_cache = {}

    def clean(self, section, ncrit, panels):
        if panels == 160:
            return self.pure160[(section, ncrit)]
        k = (section, ncrit, panels)
        if k not in self.clean_cache:
            byre = {re: sorted(rs, key=lambda x: x["alpha"]) for re, rs in self.grp[(section, 0.0, 0.0, panels, ncrit)].items()}
            self.clean_cache[k] = m.SectionTable(byre, self.inv[(section, 0.0, 0.0, panels)])
        return self.clean_cache[k]

    def bent(self, section, cf, ang, ncrit, panels):
        k = (section, cf, ang, ncrit, panels)
        if k not in self.bent_cache:
            byre = {re: sorted(rs, key=lambda x: x["alpha"]) for re, rs in self.grp[(section, cf, ang, panels, ncrit)].items()}
            self.bent_cache[k] = BentSectionTable(byre, self.inv[(section, cf, ang, panels)])
        return self.bent_cache[k]

    def inv_curve(self, section, cf, ang, panels):
        return InvCurve(self.inv[(section, cf, ang, panels)])


class Cfg:
    """Section data for one evaluation: clean table, optional bent table + its inviscid cm curve, lip angle u."""

    def __init__(self, clean, bent=None, inv=None):
        self.clean, self.bent, self.inv = clean, bent, inv


# ------------------------------------------------------------------ evaluation
def alpha_avl(M, u, CL):
    return (CL - M["cl0"] - M["d_cl"] * u) / M["a_cl"]


def cm_np_avl(M, u, CL):
    """AVL pitching moment about the (configuration) neutral point at (CL, u), no CG term (nose-up positive)."""
    al = alpha_avl(M, u, CL)
    return M["cm0"] + M["a_cm"] * al + M["d_cm"] * u + CL * (M["xnp"] - M["xref"]) / M["mac"]


def _strip_table(M, s, u, cfg):
    return cfg.bent if (s["inlip"] and u != 0 and cfg.bent is not None) else cfg.clean


def cm_extra_arch(M, b, u, cfg, CL, ws=None, V=None, form="coupled", nu=NU):
    if V is None:
        V = m.glide_speed(ws, CL)
    al = alpha_avl(M, u, CL)
    cmn = cm_np_avl(M, u, CL)
    S, mac = M["area"] * b * b, M["mac"] * b
    if form == "avl_only":
        return dict(cm_extra=cmn, cm_np=cmn, cm_visc=0.0, coverage=1.0, jag=0.0, V=V, cdp=None, dalpha=0.0, alpha_avl=al)
    sc = couple = ca = at = cd_s = jag = da_s = 0.0
    for s in M["strips"]:
        c, A = s["c"] * b, s["a"] * b * b
        at += A
        cl = s["B"] + s["A"] * al + s["D"] * u
        tb = _strip_table(M, s, u, cfg)
        r = tb.lookup(V * c / nu, cl)
        if r is None:
            continue
        term = r["cm"]
        lipstrip = tb is cfg.bent and tb is not None
        if form == "coupled" and lipstrip:
            ci = cfg.inv.cm_at(cl)
            if ci is None:
                continue
            term = r["cm"] - ci
        ca += A
        sc += term * c * A
        if lipstrip:
            couple += s["K"] * u * c * A
        cd_s += r["cd"] * A
        da_s += (r["alpha"] - tb.alpha_inv(cl)) * A
        if r["steep"] or r["nbranch"] > 1:
            jag += A
    k = 2.0 / (S * mac)
    cm_visc = k * sc - (k * couple if form == "replace" else 0.0)
    return dict(cm_extra=cmn + cm_visc, cm_np=cmn, cm_visc=cm_visc, coverage=ca / at, jag=jag / at, V=V,
                cdp=cd_s / ca if ca else None, dalpha=da_s / ca if ca else None, alpha_avl=al)


def ti_fraction_arch(M, b, u, cfg, CL, V, nu=NU):
    al = alpha_avl(M, u, CL)
    w = t = 0.0
    for s in M["strips"]:
        c, A = s["c"] * b, s["a"] * b * b
        cl = s["B"] + s["A"] * al + s["D"] * u
        tb = _strip_table(M, s, u, cfg)
        re = V * c / nu
        if cl < 0 or tb.lookup(re, cl) is None:
            continue
        lo = max(r for r in tb.res if r <= re)
        hi = min(r for r in tb.res if r >= re)
        hit = False
        for rs in {lo, hi}:
            seg = ra.segment(tb, rs, cl)
            if seg is not None and ra.on_transition(*seg):
                hit = True
        w += A
        t += A * hit
    return t / w if w else None


def build_curve_arch(M, b, u, cfg, ws, form, ti_band=(0.15, 0.6), dcl=0.02):
    pts = []
    for CL in GRID:
        r = cm_extra_arch(M, b, u, cfg, CL, ws=ws, form=form)
        pts.append((CL, r["cm_extra"], r["coverage"], r["jag"], r["V"]))
    n = len(pts)
    curve = []
    for i, (CL, cm, cov, jag, V) in enumerate(pts):
        if 2 <= i < n - 2:
            dg = (pts[i + 2][1] - pts[i - 2][1]) / 0.04
            covd = min(pts[i + 2][2], pts[i - 2][2], cov)
        else:
            dg, covd = None, 0.0
        df = None
        if cov > 0 and 0.02 <= CL <= 1.15:
            fp = cm_extra_arch(M, b, u, cfg, CL + dcl, V=V, form=form)
            fm = cm_extra_arch(M, b, u, cfg, CL - dcl, V=V, form=form)
            df = (fp["cm_extra"] - fm["cm_extra"]) / (2 * dcl)
            covd = min(covd, fp["coverage"], fm["coverage"])
        ti = None
        if form != "avl_only" and ti_band[0] <= CL <= ti_band[1] and cov > 0:
            ti = ti_fraction_arch(M, b, u, cfg, CL, V)
        elif form == "avl_only":
            ti = 0.0
        curve.append(dict(cl=CL, cm=cm, cov=cov, jag=jag, dg=dg, df=df, covd=covd, ti=ti, V=V))
    return curve


def trim_state(M, b, u, cfg, CL, ws, form="coupled"):
    """Alpha (AVL + viscous correction), V, Re range, L/D estimate at a trim point."""
    r = cm_extra_arch(M, b, u, cfg, CL, ws=ws, form=form)
    V = r["V"]
    mac = M["mac"] * b
    st = M["strips"]
    cdi = M["k_cdi"] * CL ** 2
    alpha = r["alpha_avl"] + (r["dalpha"] or 0.0)
    return dict(V=V, alpha_deg=alpha, alpha_avl_deg=r["alpha_avl"], coverage=r["coverage"], jag=r["jag"], cdp=r["cdp"], cdi=cdi,
                ld=(CL / (cdi + r["cdp"]) if r["cdp"] else None),
                re_root=V * st[0]["c"] * b / NU, re_tip=V * st[-1]["c"] * b / NU, re_mac=V * mac / NU)


# ------------------------------------------------------------------ lateral decks (left/right surfaces, optional lip, tail, fin)
def lat_deck(w: Wing, name: str, lip: tuple | None = None, left_inc: float = 0.0, right_inc: float = 0.0, tail: dict | None = None,
             fin: dict | None = None, nchord: int = 20, nspan: int = 32) -> str:
    """lip = (chord_fraction, placement).  tail = dict(area_frac, x_qc, incidence, span_frac=0.35).
    fin = dict(area_frac, x_qc, control=True): vertical, chord = 0.5 root chord, above the wing plane, at y = 0 (boom not modelled).
    Controls are numbered in declaration order: reflL=D1, reflR=D2 (if a lip is present), then the fin control."""
    txt = (f"{name}\n0.0\n0 0 0.0\n{w.area:.9f} {w.mac:.9f} {w.span_m:.9f}\n{w.cg_x_m:.9f} 0.0 {w.cg_z_m:.9f}\n")
    if lip:
        cf, pl = lip
        split = OUTER_SPLIT if pl == "outer" else 0.5
        stations = [0.0, split, 1.0]
    else:
        stations = [0.0, 1.0]
    for side, inc in (("Left", left_inc), ("Right", right_inc)):
        order = list(reversed(stations)) if side == "Left" else stations
        txt += f"SURFACE\n{side}\n{nchord} 1.0 {nspan} -2.0\nCOMPONENT\n1\n"
        for f in order:
            y = f * w.span_m / 2 * (-1 if side == "Left" else 1)
            txt += (f"SECTION\n{w.x_le(abs(y)):.9f} {y:.9f} {w.tip_z * f:.9f} {w.chord(abs(y)):.9f} {inc:.9f}\nNACA\n{w.section[-4:]}\n")
            if lip:
                active = (lip[1] == "full") or f >= split
                if active:
                    txt += f"CONTROL\nrefl{side[0]} 1.0 {1 - lip[0]:.6f} 0.0 1.0 0.0 1.0\n"
    if tail:
        ts = tail.get("span_frac", 0.35) * w.span_m
        tc = tail["area_frac"] * w.area / ts
        x = tail["x_qc"] - tc / 4
        txt += "SURFACE\nTail\n6 1.0 10 -2.0\nYDUPLICATE\n0.0\n"
        txt += f"SECTION\n{x:.9f} 0.0 0.0 {tc:.9f} {tail['incidence']:.9f}\nNACA\n0006\n"
        txt += f"SECTION\n{x:.9f} {ts/2:.9f} 0.0 {tc:.9f} {tail['incidence']:.9f}\nNACA\n0006\n"
    if fin:
        fc = 0.5 * w.root_chord
        h = fin["area_frac"] * w.area / fc
        x = fin["x_qc"] - 0.25 * fc
        ctl = "CONTROL\nfinyaw 1.0 0.0 0.0 0.0 1.0 1.0\n"
        txt += "SURFACE\nFin\n5 1.0 8 -2.0\n"
        txt += f"SECTION\n{x:.9f} 0.0 {h:.9f} {fc:.9f} 0.0\nNACA\n0008\n{ctl}"
        txt += f"SECTION\n{x:.9f} 0.0 0.0 {fc:.9f} 0.0\nNACA\n0008\n{ctl}"
    return txt


def fin_ctrl_index(lip) -> int:
    return 3 if lip else 1


def lateral_arch(ar, sweep, taper, dih, xcg, workdir: Path, tag: str, M: dict, kind: str, u: float, lip=None, tail=None, fin=None,
                 cls=(0.2, 0.3, 0.4), cl_nom=0.3) -> dict:
    """Cl_beta, Cn_beta (stability axes, per rad) at CL in cls with the architecture's feature set, plus roll per degree of
    left/right wing-incidence mismatch and lip-angle mismatch, and yaw per degree of fin misalignment, at CL = cl_nom."""
    w = m._wing(ar, sweep, taper, dih, xref=xcg)
    pre = lambda ul, ur: [f"D1 D1 {-ul:.6f}", f"D2 D2 {-ur:.6f}"] if lip else []
    deck = lambda **kw: lat_deck(w, tag, lip=lip, tail=tail, fin=fin, **kw)
    # alphas that give CL ~ 0.15..0.6 under the pitch model (u = lip angle (TE-up) or tail incidence: baked into the deck for the tail)
    uu = u if kind == "lip" else 0.0
    alphas = [round((cl - M["cl0"] - M["d_cl"] * uu) / M["a_cl"], 3) for cl in (0.15, 0.3, 0.45, 0.6)]
    if kind == "tail":
        alphas = [round((cl - M["cl0"] - M["d_cl"] * u) / M["a_cl"], 3) for cl in (0.15, 0.3, 0.45, 0.6)]
    pts = run_avl(w, tag, alphas, workdir / "sym", AVL_EXE, deck_text=deck(), oper_prefix=pre(uu, uu) + ([f"D{fin_ctrl_index(lip)} D{fin_ctrl_index(lip)} 0.0"] if fin else []))
    c = [p.cl for p in pts]
    def fit(v):
        n = len(c); mx, my = sum(c) / n, sum(v) / n
        sl = sum((x - mx) * (y - my) for x, y in zip(c, v)) / sum((x - mx) ** 2 for x in c)
        return my - sl * mx, sl
    a0, a1 = fit([p.clb_per_rad for p in pts]); n0, n1 = fit([p.cnb_per_rad for p in pts])
    p_nom = min(pts, key=lambda p: abs(p.cl - cl_nom))
    out = dict(tag=tag, cl_beta={str(k): a0 + a1 * k for k in cls}, cn_beta={str(k): n0 + n1 * k for k in cls}, xnp=sum(p.xnp_m for p in pts) / len(pts),
               cl_nom_actual=p_nom.cl, alpha_nom_deg=p_nom.alpha_deg, roll0=p_nom.cl_roll, yaw0=p_nom.cn_yaw)
    ctl = ([f"D{fin_ctrl_index(lip)} D{fin_ctrl_index(lip)} 0.0"] if fin else [])
    # wing incidence mismatch: right +0.5, left -0.5 (right minus left = 1 deg)
    pm = run_avl(w, tag, [p_nom.alpha_deg], workdir / "incmis", AVL_EXE, deck_text=deck(left_inc=-0.5, right_inc=0.5), oper_prefix=pre(uu, uu) + ctl)[0]
    out["roll_per_deg_incidence_mismatch"] = pm.cl_roll - p_nom.cl_roll
    out["yaw_per_deg_incidence_mismatch"] = pm.cn_yaw - p_nom.cn_yaw
    if lip:
        pl = run_avl(w, tag, [p_nom.alpha_deg], workdir / "lipmis", AVL_EXE, deck_text=deck(), oper_prefix=pre(uu - 0.5, uu + 0.5) + ctl)[0]
        out["roll_per_deg_lip_mismatch"] = pl.cl_roll - p_nom.cl_roll
        out["yaw_per_deg_lip_mismatch"] = pl.cn_yaw - p_nom.cn_yaw
    if fin:
        k = fin_ctrl_index(lip)
        pf = run_avl(w, tag, [p_nom.alpha_deg], workdir / "finmis", AVL_EXE, deck_text=deck(), oper_prefix=pre(uu, uu) + [f"D{k} D{k} 1.0"])[0]
        out["yaw_per_deg_fin_misalignment"] = pf.cn_yaw - p_nom.cn_yaw
        out["roll_per_deg_fin_misalignment"] = pf.cl_roll - p_nom.cl_roll
    return out
