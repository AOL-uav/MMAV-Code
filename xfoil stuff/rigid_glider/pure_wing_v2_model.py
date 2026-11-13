"""v2 wrappers around the (unmodified) pure-wing model.

* SweepSection : OPTIONAL simple-sweep lookup (a bracket / sensitivity, not a new truth).
* ti_fraction_v2 : area fraction of strips whose (cl, Re) cell lies on a polar alpha-interval across which
  xtr_top departs 1.0 (same definition as pure_wing_reanalysis, extended to wrapped tables).
* is_good_v2 : same criteria as pure_wing_study.is_good but with a selectable band and extra evidence modes.
* fin/dihedral AVL deck generation and lateral derivative evaluation (Part 4).
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import math
from pathlib import Path

import pure_wing_model as m
import pure_wing_study as st
import pure_wing_reanalysis as ra
from avl import geometry_text, run_avl
from geometry import Wing

COV_MIN = 0.9
JAG_CLEAN = 0.1


# ------------------------------------------------------------------ simple-sweep lookup
class SweepSection:
    """Wrap a section table with simple-sweep (independence-principle) scaling.

    Sweep line = the QUARTER-CHORD line (the study's sweep definition), Lambda = its angle to the spanwise axis.
    Normal section: freestream normal component V cos(L); normal chord c_n = c cos(L) (streamwise chord c).
      q_n = q cos^2 L ;  lift per unit length along the sweep line gives  cl_n = cl / cos^2 L .
      Re_n = V_n c_n / nu = Re cos^2 L   (re_exp = 2, the self-consistent choice, DEFAULT)
      re_exp = 1 gives Re_n = Re cos L  (the form in the brief, which uses the streamwise chord with the normal velocity;
      kept as the second bracket end).
    Conversions back to the streamwise reference:
      cm_stream = cm_n cos^4 L   (moment ~ q_n c_n^2 = q c^2 cos^4 L; roll component cancels between halves)
      cd_stream = cd_n cos^3 L,  alpha_stream = alpha_n cos L (tan alpha_n = tan alpha / cos L, small angles)
    Uncertainty: infinite-yawed-wing theory; ignores taper (local chord line angle != quarter-chord angle), root/tip
    effects, spanwise boundary-layer flow (which at Re 10-80k with laminar separation is a first-order unknown) and
    the finite-wing induced-flow coupling AVL already includes. A BRACKET only.
    """

    def __init__(self, base, sweep_deg: float, re_exp: int = 2, cm_exp: int = 4):
        self.base, self.sweep, self.re_exp, self.cm_exp = base, sweep_deg, re_exp, cm_exp
        self.c = math.cos(math.radians(sweep_deg))
        self.inv_deg_per_cl = getattr(base, "inv_deg_per_cl", None)

    def transform(self, re: float, cl: float) -> tuple[float, float]:
        return re * self.c ** self.re_exp, cl / self.c ** 2

    def alpha_inv(self, cl: float) -> float:
        return self.base.alpha_inv(cl / self.c ** 2) * self.c

    def lookup(self, re: float, cl: float):
        re_n, cl_n = self.transform(re, cl)
        r = self.base.lookup(re_n, cl_n)
        if r is None:
            return None
        return dict(alpha=r["alpha"] * self.c, cd=r["cd"] * self.c ** 3, cm=r["cm"] * self.c ** self.cm_exp,
                    nbranch=r["nbranch"], steep=r["steep"])


def unwrap(table):
    """-> (base SectionTable, transform(re, cl) -> (re_n, cl_n))"""
    if isinstance(table, SweepSection):
        return table.base, table.transform
    return table, (lambda re, cl: (re, cl))


# ------------------------------------------------------------------ transition-interval fraction
def ti_fraction_v2(shape, b, table, CL, V, nu=m.NU):
    base, tf = unwrap(table)
    wsum = tsum = 0.0
    for s in shape["strips"]:
        c, A = s["c"] * b, s["a"] * b * b
        re_n, cl_n = tf(V * c / nu, CL * s["f"])
        cl_n = abs(cl_n)
        if base.lookup(re_n, cl_n) is None:
            continue
        lo = max(r for r in base.res if r <= re_n)
        hi = min(r for r in base.res if r >= re_n)
        hit = False
        for rs in {lo, hi}:
            seg = ra.segment(base, rs, cl_n)
            if seg is not None and ra.on_transition(*seg):
                hit = True
        wsum += A
        tsum += A * hit
    return tsum / wsum if wsum else None


# ------------------------------------------------------------------ curve, roots, evidence modes
def build_curve_v2(shape, b, table, ws, ti_band=(0.15, 0.6)):
    curve = st.build_curve(shape, b, table, ws)          # unmodified
    for p in curve:
        p["ti"] = None
        if ti_band[0] <= p["cl"] <= ti_band[1] and p["cov"] > 0:
            p["ti"] = ti_fraction_v2(shape, b, table, p["cl"], m.glide_speed(ws, p["cl"]))
    return curve


def roots_from_curve_v2(curve, sm):
    """st.roots_from_curve plus 'ti' (max of the two bracketing grid values; None if unknown)."""
    out = []
    for a, c in zip(curve, curve[1:]):
        if a["cov"] <= 0 or c["cov"] <= 0:
            continue
        g0, g1 = a["cm"] - sm * a["cl"], c["cm"] - sm * c["cl"]
        if g0 * g1 < 0 or g0 == 0:
            t = g0 / (g0 - g1) if g0 != g1 else 0.0
            cl = a["cl"] + t * (c["cl"] - a["cl"])
            if a["dg"] is None or c["dg"] is None or a["df"] is None or c["df"] is None:
                dg = df = None
            else:
                dg = a["dg"] + t * (c["dg"] - a["dg"]) - sm
                df = a["df"] + t * (c["df"] - a["df"]) - sm
            ti = None if a.get("ti") is None or c.get("ti") is None else max(a["ti"], c["ti"])
            out.append(dict(cl=cl, dg=dg, df=df, cov=min(a["cov"], c["cov"]), covd=min(a["covd"], c["covd"]),
                            jag=max(a["jag"], c["jag"]), ti=ti))
    return out


MODES = ("any", "clean", "offti", "clean_offti")


def is_good_v2(r, band=(0.15, 0.6), mode="any") -> bool:
    """Same criteria as pure_wing_study.is_good (stable on BOTH derivatives, coverage>=0.9 incl. derivative cover),
    with a selectable CL band.  mode: any | clean (jag<=0.1) | offti (TI area fraction <0.5) | clean_offti."""
    if r["dg"] is None or not (band[0] <= r["cl"] <= band[1]):
        return False
    if min(r["cov"], r["covd"]) < COV_MIN:
        return False
    if not (r["dg"] < 0 and r["df"] < 0):
        return False
    if mode in ("clean", "clean_offti") and r["jag"] > JAG_CLEAN:
        return False
    if mode in ("offti", "clean_offti"):
        if r.get("ti") is None or r["ti"] >= 0.5:
            return False
    return True


def contiguous_window(points, step=0.005):
    """points: SM values (floats). -> dict(n, runs, widest_n, width (=(n-1)*step), sm_lo, sm_hi).
    A single hit (widest_n == 1) means the true window is between 0 and 2 steps wide."""
    pts = sorted(set(round(p, 4) for p in points))
    if not pts:
        return dict(n=0, runs=0, widest_n=0, width=None, sm_lo=None, sm_hi=None)
    runs, cur = [], [pts[0]]
    for a, b in zip(pts, pts[1:]):
        if round((b - a) / step) == 1:
            cur.append(b)
        else:
            runs.append(cur)
            cur = [b]
    runs.append(cur)
    w = max(runs, key=len)
    return dict(n=len(pts), runs=len(runs), widest_n=len(w), width=(len(w) - 1) * step, sm_lo=w[0], sm_hi=w[-1])


# ------------------------------------------------------------------ fins / dihedral decks
def fin_surface_text(name, x_le, y, z_top, z_bot, chord, mirrored, section="0008", nchord=5, nspan=8):
    s = f"SURFACE\n{name}\n{nchord} 1.0 {nspan} -2.0\n" + ("YDUPLICATE\n0.0\n" if mirrored else "")
    s += f"SECTION\n{x_le:.9f} {y:.9f} {z_top:.9f} {chord:.9f} 0.0\nNACA\n{section}\n"
    s += f"SECTION\n{x_le:.9f} {y:.9f} {z_bot:.9f} {chord:.9f} 0.0\nNACA\n{section}\n"
    return s


def deck_with_fins(w: Wing, name, tip_fin=None, centre_fin=None, nchord=8, nspan=24):
    """tip_fin = dict(area_frac (EACH fin, fraction of wing area), mode 'above'|'sym'): vertical, chord = tip chord,
    at the wing tip.  'sym' splits the height half above / half below the wing plane (same area).
    centre_fin = dict(area_frac, x_qc_over_root_chord): single fin at y=0 above the plane on a (non-modelled, massless)
    boom; chord = 0.5 root chord, quarter chord at x_qc_over_root_chord*root_chord aft of the root LE."""
    txt = geometry_text(w, name, nchord, nspan)
    if tip_fin:
        ct = w.tip_chord
        h = tip_fin["area_frac"] * w.area / ct
        top, bot = (w.tip_z + h, w.tip_z) if tip_fin["mode"] == "above" else (w.tip_z + h / 2, w.tip_z - h / 2)
        txt += fin_surface_text("TipFin", w.tip_x_le, w.span_m / 2, top, bot, ct, True)
    if centre_fin:
        cf = 0.5 * w.root_chord
        h = centre_fin["area_frac"] * w.area / cf
        x_le = centre_fin["x_qc_over_root_chord"] * w.root_chord - 0.25 * cf
        txt += fin_surface_text("CentreFin", x_le, 0.0, h, 0.0, cf, False)
    return txt


def lateral_case(ar, sweep, taper, dih, xcg, work: Path, tag, tip_fin=None, centre_fin=None,
                 alphas=(0.0, 3.0, 6.0, 9.0), cls=(0.2, 0.3, 0.4, 0.5)) -> dict:
    """AVL (b = 1 m): Cl_beta, Cn_beta (stability axes, per rad) evaluated at CL in cls by linear fit; Xnp, CLa."""
    w = m._wing(ar, sweep, taper, dih, xref=xcg)
    deck = deck_with_fins(w, tag, tip_fin, centre_fin)
    pts = run_avl(w, tag, list(alphas), work / tag, m.AVL_EXE, deck_text=deck)
    c = [p.cl for p in pts]
    def fit(v):
        n = len(c); mx, my = sum(c) / n, sum(v) / n
        sl = sum((x - mx) * (y - my) for x, y in zip(c, v)) / sum((x - mx) ** 2 for x in c)
        return my - sl * mx, sl
    a0, a1 = fit([p.clb_per_rad for p in pts])
    n0, n1 = fit([p.cnb_per_rad for p in pts])
    nonlin = max(abs(p.clb_per_rad - (a0 + a1 * p.cl)) for p in pts)
    return dict(tag=tag, xcg=xcg, xnp=sum(p.xnp_m for p in pts) / len(pts), mac=w.mac, cla_per_rad=pts[-1].cla_per_rad,
                cl_beta={str(k): a0 + a1 * k for k in cls}, cn_beta={str(k): n0 + n1 * k for k in cls},
                fit_resid_clb=nonlin, cdi_over_cl2=pts[-1].cdi / pts[-1].cl ** 2)
