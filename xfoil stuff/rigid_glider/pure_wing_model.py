"""Part B: coupled AVL + low-Re XFOIL section model for small tailless symmetric flying wings.

AVL (inviscid, linear) is run once per geometry SHAPE (span-normalised, b = 1 m):
CL_alpha, strip cl(y)/CL, strip chords/areas, Xnp/MAC, CDi = k CL^2, Cl_beta/Cn_beta slopes.
Level 1 (one-way): section cm_c/4, cd, alpha_2D from the XFOIL database at the matched
local cl and strip Re are added to the AVL result.  Level 2 (two-way, 'viscous
decambering'): per-node incidence offsets are iterated through a precomputed AVL linear
influence matrix so every strip sits on the XFOIL viscous lift curve at its Re.

Moment sign convention: Cm positive nose-up.  Static margin SM = (Xnp - Xcg)/MAC (AVL),
AVL-only Cm(CL) = Cm0 - SM*CL.  Total Cm(CL) = Cm_extra(CL) - SM*CL, with Cm_extra the
viscous part (Level 1: sum of section cm couples; Level 2 also the loading change).
"""

from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import json
import math
import tempfile
from math import log, pi, sqrt, radians
from pathlib import Path

from avl import run_avl, read_strips
from geometry import Wing

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/pure_wing"
AVL_EXE = ROOT / "tools/avl/avl352.exe"
RHO, NU = 1.225, 1.5e-5

ASSUMPTIONS = [
    "AVL is inviscid and linear in alpha for planar/dihedral untwisted symmetric-section wings; it is run once per span-normalised shape (b=1 m) and scaled.",
    "2D XFOIL section data (cm about local quarter chord, cd, alpha_2D at matched local cl and Re) are applied STREAMWISE on the swept wing with no sweep (cos Lambda) correction.",
    "No spanwise boundary-layer flow, no tip/centre-section viscous effects, no interference, no 3D separation; each strip is independent 2D.",
    "Strip Re = V * local chord / nu with V = sqrt(2 (W/S)/(rho CL)); rho=1.225, nu=1.5e-5. L = W (small glide angle, cos(gamma)~1).",
    "Section cm acts as a pure couple (streamwise sections) added to the AVL moment about the CG: Cm_visc = sum(cm c^2 dy)/(S MAC) over both halves.",
    "Level 1: the AVL loading shape (cl(y)/CL) is unchanged by viscosity; alpha_visc correction = area-weighted (alpha_2D,visc(cl) - alpha_2D,inviscid XFOIL(cl)).",
    "Level 2: viscous decambering via per-node AINC offsets = -(alpha_visc - alpha_inv_XFOIL)(cl, Re) with under-relaxation; AVL slope (thin-plate 2 pi) differs from XFOIL inviscid thick-section slope by ~ 2 pi*0.77 t, so the AVL strip lift curve is not identical to XFOIL inviscid.",
    "Sections symmetric: negative-cl strips use odd symmetry (cm, alpha odd; cd even).",
    "Attached-flow branch = the cl-alpha crossing nearest the inviscid alpha; crossings only between adjacent stored alphas <= 1 deg apart; Re interpolation is linear in log Re between adjacent stored Re only when BOTH are valid; no Re extrapolation; no gap filling. Otherwise the strip is UNCOVERED.",
    "Uncovered strips contribute zero viscous cm, are excluded from CDp and the alpha correction (reported through the coverage fraction); trims with coverage < 0.9 are flagged, not results.",
    "XFOIL has no free-stream turbulence model other than Ncrit; Ncrit 5 vs 9 is treated as an uncertainty band, not a measured quantity. Laminar separation bubbles at Re<40k make polars jagged; neither AVL nor XFOIL is ground truth.",
    "Weight/mass/CG are exploratory parameters (W/S, SM), not measured.  No twist, washout, reflex, tail, fuselage, or fins.",
    "Cm_alpha and static margin of the AVL-only wing are Re-independent; 'coupled static margin' = -dCm/dCL, along the constant-W/S glide line and at fixed speed.",
]


# ------------------------------------------------------------------ AVL shapes
def shape_key(ar, sweep, taper, dih) -> str:
    return f"AR{ar:g}_sw{sweep:g}_tp{taper:g}_di{dih:g}"


def _wing(ar, sweep, taper, dih, xref=None) -> Wing:
    w = Wing(span_m=1.0, aspect_ratio=ar, taper=taper, quarter_chord_sweep_deg=sweep, washout_deg=0.0,
             dihedral_deg=dih, section="NACA_0008", cg_x_m=0.0)
    xr = w.x_le(w.mac_y) + 0.25 * w.mac if xref is None else xref
    return Wing(span_m=1.0, aspect_ratio=ar, taper=taper, quarter_chord_sweep_deg=sweep, washout_deg=0.0,
                dihedral_deg=dih, section="NACA_0008", cg_x_m=xr)


def build_shape(ar, sweep, taper, dih, workdir: Path, nchord=8, nspan=24, alphas=(0.0, 3.0, 6.0, 9.0)) -> dict:
    """Run AVL for one shape (b=1 m) and distil span-normalised linear data."""
    w = _wing(ar, sweep, taper, dih)
    pts = run_avl(w, "shape", list(alphas), workdir / shape_key(ar, sweep, taper, dih), AVL_EXE, nchord, nspan)
    p0 = pts[0]
    # linear fits through the non-zero alphas
    cl_per_deg = [p.cl / p.alpha_deg for p in pts if p.alpha_deg]
    cla_deg = sum(cl_per_deg) / len(cl_per_deg)
    lin_err = max(abs(c - cla_deg) / cla_deg for c in cl_per_deg)
    ref = pts[2] if len(pts) > 2 else pts[-1]
    s_ref = [s for s in read_strips(ref.strip_file) if s.y_m > 0]
    s_hi = [s for s in read_strips(pts[-1].strip_file) if s.y_m > 0]
    strips = [dict(eta=s.y_m / (w.span_m / 2), c=s.chord_m, a=s.area_m2, f=s.cl / ref.cl) for s in s_ref]
    f_dev = max(abs(a.cl / pts[-1].cl - b["f"]) for a, b in zip(s_hi, strips))
    k_cdi = sum(p.cdi / p.cl ** 2 for p in pts if p.alpha_deg) / len([p for p in pts if p.alpha_deg])
    # lateral: Cl_beta, Cn_beta linear in CL (fit through alphas)
    cls = [p.cl for p in pts]
    def fit(v):
        n = len(cls); mx, my = sum(cls) / n, sum(v) / n
        sxx = sum((x - mx) ** 2 for x in cls)
        sl = sum((x - mx) * (y - my) for x, y in zip(cls, v)) / sxx
        return my - sl * mx, sl
    clb0, clb1 = fit([p.clb_per_rad for p in pts])
    cnb0, cnb1 = fit([p.cnb_per_rad for p in pts])
    xnp = [p.xnp_m for p in pts]
    # Cm about xref: slope check against Xnp
    cm_slope = [p.cm / p.cl for p in pts if p.alpha_deg]
    return dict(key=shape_key(ar, sweep, taper, dih), ar=ar, sweep=sweep, taper=taper, dih=dih,
                span=1.0, area=w.area, mac=w.mac, mac_y=w.mac_y, x_lemac=w.x_le(w.mac_y), xref=w.cg_x_m,
                root_chord=w.root_chord, tip_chord=w.tip_chord,
                xnp=sum(xnp) / len(xnp), xnp_spread=max(xnp) - min(xnp),
                cla_per_rad=cla_deg * 180 / pi, cla_lin_err=lin_err, strip_f_dev=f_dev,
                cm0=p0.cm, cm_over_cl_xref=sum(cm_slope) / len(cm_slope),
                cma_over_cla_check=(w.cg_x_m - sum(xnp) / len(xnp)) / w.mac,
                k_cdi=k_cdi, e_oswald=1.0 / (pi * ar * k_cdi),
                clb0=clb0, clb_slope=clb1, cnb0=cnb0, cnb_slope=cnb1, strips=strips,
                avl_alphas=list(alphas), avl_cl=cls)


def get_shape(ar, sweep, taper, dih, cache: dict, workdir: Path | None = None) -> dict:
    k = shape_key(ar, sweep, taper, dih)
    if k not in cache:
        wd = workdir or (OUT / "avl")
        cache[k] = build_shape(ar, sweep, taper, dih, wd)
    return cache[k]


# ------------------------------------------------------------------ Level 2 influence matrices
def l2_deck(ar, sweep, taper, dih, ainc_deg: list[float], etas: list[float], nchord=8, nspan=48) -> str:
    w = _wing(ar, sweep, taper, dih)
    txt = (f"l2\n0.0\n0 0 0.0\n{w.area:.9f} {w.mac:.9f} {w.span_m:.9f}\n{w.cg_x_m:.9f} 0.0 0.0\n"
           f"SURFACE\nMainWing\n{nchord} 1.0 {nspan} -2.0\nYDUPLICATE\n0.0\n")
    for eta, ai in zip(etas, ainc_deg):
        y = eta * w.span_m / 2
        c = w.root_chord + (w.tip_chord - w.root_chord) * eta
        x = w.tip_x_le * eta
        z = w.tip_z * eta
        txt += f"SECTION\n{x:.9f} {y:.9f} {z:.9f} {c:.9f} {ai:.9f}\nNACA\n0008\n"
    return txt


def build_l2(ar, sweep, taper, dih, workdir: Path, nodes=11, nspan=48) -> dict:
    """AVL response matrices: strip cl, CL, Cm(about xref) per deg of alpha and per deg of node AINC."""
    etas = [math.sin(0.5 * pi * k / nodes) for k in range(nodes + 1)]   # denser toward the tip
    w = _wing(ar, sweep, taper, dih)
    base = workdir / (shape_key(ar, sweep, taper, dih) + "_l2")
    def run(ainc, alpha, tag):
        deck = l2_deck(ar, sweep, taper, dih, ainc, etas, nspan=nspan)
        p = run_avl(w, "l2", [alpha], base / tag, AVL_EXE, deck_text=deck)[0]
        st = [s for s in read_strips(p.strip_file) if s.y_m > 0]
        return p, st
    zero = [0.0] * (nodes + 1)
    p1, s1 = run(zero, 1.0, "a1")
    p2, s2 = run(zero, 2.0, "a2")
    lin_err = abs(p2.cl / p1.cl - 2.0)
    n = nodes + 1
    cols = []
    for k in range(n):
        a = [0.0] * n
        a[k] = 1.0
        pk, sk = run(a, 0.0, f"n{k:02d}")
        cols.append((pk, sk))
    strips = [dict(eta=s.y_m / 0.5, c=s.chord_m, a=s.area_m2) for s in s1]
    return dict(key=shape_key(ar, sweep, taper, dih), etas=etas, nstrips=len(s1), nspan=nspan,
                strips=strips, xref=w.cg_x_m, mac=w.mac,
                A=[s.cl for s in s1], a_cl=p1.cl, a_cm=p1.cm,
                B=[[cols[k][1][i].cl for k in range(n)] for i in range(len(s1))],
                b_cl=[cols[k][0].cl for k in range(n)], b_cm=[cols[k][0].cm for k in range(n)],
                xnp=p1.xnp_m, cla_per_rad=p1.cl / radians(1.0), alpha_linearity_err=lin_err)


# ------------------------------------------------------------------ section data
class SectionTable:
    """Attached-branch lookup of (alpha_2D, cd, cm) at matched cl and Re from the XFOIL database."""

    def __init__(self, visc_rows: dict[int, list[dict]], inv_rows: list[dict], cl_step=0.005, max_gap=1.0):
        self.res = sorted(r for r, rows in visc_rows.items() if len(rows) >= 2)
        self.rows = {r: visc_rows[r] for r in self.res}
        # inviscid slope (deg per unit cl) through the origin
        num = sum(r["cl"] * r["alpha"] for r in inv_rows)
        den = sum(r["cl"] ** 2 for r in inv_rows)
        self.inv_deg_per_cl = num / den
        self.step, self.max_gap = cl_step, max_gap
        self.steep_slope = 0.25      # dcl/dalpha per deg above which a segment is a 'jump' (bubble burst), flagged
        self._cache: dict = {}
        self.cl_max = {}
        for r in self.res:
            cls = [x["cl"] for x in self.rows[r]]
            self.cl_max[r] = max(cls)

    def alpha_inv(self, cl: float) -> float:
        return cl * self.inv_deg_per_cl

    def _branch(self, re: int, cl: float):
        hint = self.alpha_inv(cl)
        best, n = None, 0
        rows = self.rows[re]
        for a, b in zip(rows, rows[1:]):
            if b["alpha"] - a["alpha"] > self.max_gap + 1e-9:
                continue
            lo, hi = min(a["cl"], b["cl"]), max(a["cl"], b["cl"])
            if not lo <= cl <= hi or abs(b["cl"] - a["cl"]) < 1e-9:
                continue
            t = (cl - a["cl"]) / (b["cl"] - a["cl"])
            slope = abs(b["cl"] - a["cl"]) / (b["alpha"] - a["alpha"])
            cand = (a["alpha"] + t * (b["alpha"] - a["alpha"]), a["cd"] + t * (b["cd"] - a["cd"]),
                    a["cm"] + t * (b["cm"] - a["cm"]), slope)
            n += 1
            if best is None or abs(cand[0] - hint) < abs(best[0] - hint):
                best = cand
        return None if best is None else (best[0], best[1], best[2], n, 1.0 if best[3] > self.steep_slope else 0.0)

    def _node(self, re: int, idx: int):
        key = (re, idx)
        if key not in self._cache:
            self._cache[key] = self._branch(re, idx * self.step)
        return self._cache[key]

    def _at_re(self, re: int, cl: float):
        x = cl / self.step
        i = int(x)
        t = x - i
        n0, n1 = self._node(re, i), self._node(re, i + 1)
        if n0 is None or n1 is None:
            return None
        return tuple(n0[k] + t * (n1[k] - n0[k]) for k in range(3)) + (max(n0[3], n1[3]), max(n0[4], n1[4]))

    def lookup(self, re: float, cl: float):
        """-> dict(alpha, cd, cm, nbranch) or None (uncovered: Re out of range / cl unreachable / gap)."""
        if not self.res or re < self.res[0] or re > self.res[-1]:
            return None
        sgn = 1.0 if cl >= 0 else -1.0
        c = abs(cl)
        # bracket stored Re
        lo = max((r for r in self.res if r <= re), default=None)
        hi = min((r for r in self.res if r >= re), default=None)
        if lo is None or hi is None:
            return None
        v0 = self._at_re(lo, c)
        if v0 is None:
            return None
        if lo == hi:
            v = v0
        else:
            v1 = self._at_re(hi, c)
            if v1 is None:
                return None
            w = (log(re) - log(lo)) / (log(hi) - log(lo))
            v = tuple(v0[k] + w * (v1[k] - v0[k]) for k in range(3)) + (max(v0[3], v1[3]), max(v0[4], v1[4]))
        return dict(alpha=sgn * v[0], cd=v[1], cm=sgn * v[2], nbranch=int(v[3]), steep=v[4] > 0)


class ZeroSection:
    """Synthetic: inviscid lift curve, zero cm, zero cd (reduces coupled model to AVL-only)."""
    inv_deg_per_cl = 9.0
    def lookup(self, re, cl):
        return dict(alpha=cl * self.inv_deg_per_cl, cd=0.0, cm=0.0, nbranch=1, steep=False)
    def alpha_inv(self, cl): return cl * self.inv_deg_per_cl


class ConstCmSection(ZeroSection):
    """Synthetic: constant cm0 at all cl and Re, inviscid lift."""
    def __init__(self, cm0): self.cm0 = cm0
    def lookup(self, re, cl):
        d = ZeroSection.lookup(self, re, cl); d["cm"] = self.cm0; return d


def load_tables(db_path: Path | None = None) -> dict:
    from pure_wing_polars import read_database
    visc, inv = read_database(db_path) if db_path else read_database()
    tables = {}
    for (sec, nc, re), rows in visc.items():
        tables.setdefault((sec, nc), {})[re] = rows
    return {k: SectionTable(v, inv[k[0]]) for k, v in tables.items()}


# ------------------------------------------------------------------ Level 1
def glide_speed(ws: float, cl: float, rho=RHO) -> float:
    return sqrt(2.0 * ws / (rho * cl))


def level1(shape: dict, b: float, table, CL: float, ws: float | None = None, V: float | None = None,
           nu: float = NU) -> dict:
    """Viscous section contribution at trial CL. V (fixed speed) overrides the glide-line speed."""
    if V is None:
        V = glide_speed(ws, CL)
    S, mac = shape["area"] * b * b, shape["mac"] * b
    sc = ca = a_tot = cd_s = da_s = jag = 0.0
    nbr = 0
    for s in shape["strips"]:
        c, A = s["c"] * b, s["a"] * b * b
        a_tot += A
        r = table.lookup(V * c / nu, CL * s["f"])
        if r is None:
            continue
        ca += A
        sc += r["cm"] * c * A
        cd_s += r["cd"] * A
        da_s += (r["alpha"] - table.alpha_inv(CL * s["f"])) * A
        nbr = max(nbr, r["nbranch"])
        if r["steep"] or r["nbranch"] > 1:
            jag += A
    cov = ca / a_tot
    return dict(CL=CL, V=V, cm_visc=2.0 * sc / (S * mac), coverage=cov, jagged_fraction=jag / a_tot,
                cdp=cd_s / ca if ca else None, dalpha_deg=da_s / ca if ca else None, nbranch=nbr,
                re_root=V * shape["strips"][0]["c"] * b / nu, re_tip=V * shape["strips"][-1]["c"] * b / nu,
                re_mac=V * mac / nu)


def re_strip_limits(shape, b, V, nu=NU):
    return [V * s["c"] * b / nu for s in shape["strips"]]


# ------------------------------------------------------------------ Level 2
def level2(shape: dict, l2: dict, b: float, table, CL: float, ws: float, nu: float = NU,
           relax=0.5, maxit=60, tol=0.01, ainc0=None, V: float | None = None) -> dict:
    """Viscous decambering at trial CL (glide-line speed unless V is given). Returns converged flag/iterations."""
    if V is None:
        V = glide_speed(ws, CL)
    S, mac = l2["strips"], l2["mac"] * b
    Sref = shape["area"] * b * b
    n = len(l2["etas"])
    yst = [s["eta"] for s in S]
    ainc = list(ainc0) if ainc0 else [0.0] * n
    converged, it = False, 0
    last = None
    for it in range(1, maxit + 1):
        alpha = (CL - sum(bk * ak for bk, ak in zip(l2["b_cl"], ainc))) / l2["a_cl"]   # deg (a_cl per deg)
        cls = [Ai * alpha + sum(Bik * ak for Bik, ak in zip(Bi, ainc)) for Ai, Bi in zip(l2["A"], l2["B"])]
        dal, lk = [], []
        for s, cl in zip(S, cls):
            r = table.lookup(V * s["c"] * b / nu, cl)
            lk.append(r)
            dal.append(0.0 if r is None else r["alpha"] - table.alpha_inv(cl))
        # node targets by linear interpolation over strip eta
        tgt = []
        for e in l2["etas"]:
            if e <= yst[0]:
                v = dal[0]
            elif e >= yst[-1]:
                v = dal[-1]
            else:
                j = max(k for k in range(len(yst)) if yst[k] <= e)
                t = (e - yst[j]) / (yst[j + 1] - yst[j])
                v = dal[j] + t * (dal[j + 1] - dal[j])
            tgt.append(-v)
        new = [(1 - relax) * a + relax * t for a, t in zip(ainc, tgt)]
        delta = max(abs(x - y) for x, y in zip(new, ainc))
        ainc = new
        last = (alpha, cls, lk)
        if delta < tol:
            converged = True
            break
    # final consistent evaluation with converged ainc
    alpha = (CL - sum(bk * ak for bk, ak in zip(l2["b_cl"], ainc))) / l2["a_cl"]
    cls = [Ai * alpha + sum(Bik * ak for Bik, ak in zip(Bi, ainc)) for Ai, Bi in zip(l2["A"], l2["B"])]
    cm_ref = l2["a_cm"] * alpha + sum(bk * ak for bk, ak in zip(l2["b_cm"], ainc))
    cm_xnp = cm_ref + CL * (l2["xnp"] - l2["xref"]) / l2["mac"]
    sc = ca = at = cd_s = jag = 0.0
    for s, cl in zip(S, cls):
        c, A = s["c"] * b, s["a"] * b * b
        at += A
        r = table.lookup(V * c / nu, cl)
        if r is None:
            continue
        ca += A
        if r["steep"] or r["nbranch"] > 1:
            jag += A
        sc += r["cm"] * c * A
        cd_s += r["cd"] * A
    cm_visc = 2.0 * sc / (Sref * mac)
    return dict(CL=CL, V=V, alpha_deg=alpha, ainc=ainc, converged=converged, iterations=it,
                cm_xnp_inviscid_part=cm_xnp, cm_visc=cm_visc, cm_extra=cm_xnp + cm_visc,
                coverage=ca / at, jagged_fraction=jag / at, cdp=cd_s / ca if ca else None, strip_cl_max=max(cls), strip_cl_min=min(cls))


# ------------------------------------------------------------------ trim
def cm_extra_L1(shape, b, table, CL, ws=None, V=None):
    r = level1(shape, b, table, CL, ws=ws, V=V)
    return shape["cm0"] + r["cm_visc"], r


def trim_roots(shape: dict, b: float, table, ws: float, sm: float, cl_grid: list[float],
               curve: list[tuple[float, float, float]] | None = None, cmin_cov=0.9, dcl=0.02) -> list[dict]:
    """All roots of Cm_total(CL) = Cm_extra(CL) - SM*CL = 0 on the glide line (Level 1).

    curve: precomputed [(CL, cm_extra, coverage)] on cl_grid (SM independent)."""
    if curve is None:
        curve = []
        for cl in cl_grid:
            cm, r = cm_extra_L1(shape, b, table, cl, ws=ws)
            curve.append((cl, cm, r["coverage"]))
    roots = []
    for (c0, m0, v0), (c1, m1, v1) in zip(curve, curve[1:]):
        if v0 <= 0 or v1 <= 0:
            continue
        g0, g1 = m0 - sm * c0, m1 - sm * c1
        if g0 == 0.0 or g0 * g1 < 0:
            lo, hi, glo = c0, c1, g0
            for _ in range(40):                   # bisection on direct evaluations
                mid = 0.5 * (lo + hi)
                gm = cm_extra_L1(shape, b, table, mid, ws=ws)[0] - sm * mid
                if glo * gm <= 0:
                    hi = mid
                else:
                    lo, glo = mid, gm
                if hi - lo < 1e-7:
                    break
            cl = 0.5 * (lo + hi)
            cm, r = cm_extra_L1(shape, b, table, cl, ws=ws)
            # glide-line derivative (central) and fixed-speed derivative
            def tot(c, V=None):
                return cm_extra_L1(shape, b, table, c, ws=ws, V=V)
            (mp, rp), (mm, rm) = tot(cl + dcl), tot(cl - dcl)
            (fp, rfp), (fm, rfm) = tot(cl + dcl, V=r["V"]), tot(cl - dcl, V=r["V"])
            d_glide = (mp - mm) / (2 * dcl) - sm
            d_fixed = (fp - fm) / (2 * dcl) - sm
            nb = min(rp["coverage"], rm["coverage"], rfp["coverage"], rfm["coverage"])
            roots.append(dict(CL=cl, cm_visc=r["cm_visc"], coverage=r["coverage"], dcm_dcl_glide=d_glide,
                              dcm_dcl_fixed_speed=d_fixed, deriv_min_coverage=nb, V=r["V"],
                              re_root=r["re_root"], re_tip=r["re_tip"], re_mac=r["re_mac"], cdp=r["cdp"],
                              dalpha_deg=r["dalpha_deg"], nbranch=r["nbranch"], jagged_fraction=r["jagged_fraction"],
                              bracket_coverage=(v0, v1),
                              stable_glide=d_glide < 0, stable_fixed=d_fixed < 0,
                              valid=r["coverage"] >= cmin_cov and min(v0, v1) >= cmin_cov and nb >= cmin_cov))
    return roots


def avl_only_trim(shape: dict, sm: float) -> float | None:
    """AVL-only: Cm = Cm0 - SM CL.  Trim CL = Cm0/SM (None if SM == 0)."""
    return None if sm == 0 else shape["cm0"] / sm


def cg_from_sm(shape: dict, b: float, sm: float) -> dict:
    mac = shape["mac"] * b
    xcg = shape["xnp"] * b - sm * mac
    return dict(xcg_m=xcg, xcg_over_mac_from_lemac=(xcg - shape["x_lemac"] * b) / mac,
                xcg_mm=1000 * xcg, xnp_mm=1000 * shape["xnp"] * b)


def lateral_at(shape: dict, cl: float) -> tuple[float, float]:
    return shape["clb0"] + shape["clb_slope"] * cl, shape["cnb0"] + shape["cnb_slope"] * cl
