"""Balance study from MEASURED flat-plate wing data (Ananda 2012 thesis tables, 10 wings, 40 runs, 3482 rows).

Analysis only.  Reads research/plate_data/ananda2012_flatplate_all_wings.csv (never modified) and, for the clearly separated
"IF IT CARRIES OVER" section only, results/runs/rigid_glider/flat_test_v2/<specimen>/geometry.json (read only).  Writes ONLY to --out
(default results/runs/rigid_glider/plate_balance/).  Nothing is written into the flat-test kit.

Method (all stdlib):
  1. CN = CL cos a + CD sin a ; x_cp/c = X_REF - CM/CN (points with |CN| < CN_MIN skipped); L/D = CL/CD.
  2. Low-angle least-squares fits over FIT_RANGE: lift slope, alpha0, dCM/dCL, aerodynamic centre AC = X_REF - dCM/dCL, CM at CL = 0, residuals.
  3. Trim reading: a balance point x_cg trims where x_cp(alpha) = x_cg, accepted only where x_cp RISES with alpha (statically stable).
  4. Same quantities as an offset from each wing's own fitted AC, in percent of the mean aerodynamic chord.
  5. Trends with aspect ratio, taper ratio, Reynolds number (tables + SVG; no extrapolation).
  6. IF IT CARRIES OVER (reduced to two rows per specimen, whole mm, NOT a CG setting).
The fitted zero-lift moment is NOT zero in these runs, so everything is computed raw AND with it removed (see page section 2/7); the best-L/D balance point
and the bands are NOT resolved by the data and appear only in the "not resolved" table / CSV.

Moment reference: Mc/4, i.e. about the QUARTER CHORD (README; thesis PDF p. 32 "wings were mounted at quarter chord using a wing-tip mounted
sting", Table 3.1 "Mc/4"; PDF p. 49 "Variations in taper ratios ... were done about the quarter chord ... the quarter chord line of all the wings
were fixed").  Reference chord: the mean aerodynamic chord, 3.5 in for every wing (thesis PDF p. 50; Table 3.8 lists c-bar among the reference
quantities).  The explicit data-reduction equation for CM was not located; CM is taken as normalised by that reference chord (README).
All distances x/c are fractions of that chord measured from the leading edge of the mean aerodynamic chord.

Run:  py -B rigid_glider/plate_balance.py [--out DIR]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import json
import math
from pathlib import Path

# =============================================================================================== PARAMETERS
ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "research/plate_data/ananda2012_flatplate_all_wings.csv"
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/plate_balance"
KIT = ROOT / "results/runs/rigid_glider/flat_test_v2"              # read only, for the carry-over section
SPECIMENS = ["F45_span250", "F60_span250", "B45_span400"]
X_REF = 0.25                      # moment reference point, fraction of chord (quarter chord)
CN_MIN = 0.05                     # skip x_cp where |CN| is smaller
FIT_RANGE = (-4.0, 6.0)           # alpha range (deg) of the low-angle fits
GRID_STEP = 0.0025                # x_cg grid step (fraction of chord)
GRID_RANGE = (0.0, 0.80)
MONO_TOL = 0.01                   # a fall in x_cp larger than this (fraction of chord) between neighbouring points is flagged "non-monotonic"
CLMAX_ALPHA_LIMIT = 20.0          # the fallback global maximum of CL is searched up to this alpha (deg)
PEAK_SLOPE_FRAC = 0.25            # FIRST LIFT PEAK: the first angle (after PEAK_SEARCH_START) where the lift slope, over two neighbouring intervals, falls below this fraction of the low-angle slope
PEAK_SEARCH_START = 5.0           # deg
PEAK_FALL = 0.015                 # a later fall of CL by more than this fraction below the first peak (before CLMAX_ALPHA_LIMIT) marks it a true peak (else a flattening)
AR_ONSET_RESOLVED_MIN = 3          # the aft-travel onset angle is called resolved only for aspect ratio >= this (AR 2: offset-dependent, see step2b_resolvability.csv)
STALL_EXTRA_DEG = 5.0             # trims this far beyond the first lift peak are still looked for (reported as stalled)
BAND_FRACS = (0.90, 0.75)         # trimmed L/D >= this fraction of the wing's best
ONSET_BASE_RANGE = (2.0, 6.0)      # alpha range (deg) whose median x_cp is the low-angle baseline for the aft-travel onset
ONSET_RISE = 0.02                 # aft travel 'begins' where x_cp stays this far (fraction of chord) above the baseline up to the first lift peak
NEAR_STALL_FRAC = 0.90            # 'near-stall trim': where lift first reaches this fraction of the first-peak CL
PLATEAU_TOL = 0.01                # x_cp counts as 'at the aerodynamic centre' within this (fraction of chord)
N_NEAREST = 2                     # nearest tested wings used in the carry-over section
D_TAPER_SCALE = 0.5               # distance metric: sqrt(dAR^2 + (d taper / D_TAPER_SCALE)^2): a taper difference of 0.5 counts like one aspect-ratio unit
TESTED_T_OVER_C_PCT = 4.3         # tested plates: thickness / chord (README)
UNTESTED = ["25 deg sweep (tested wings are unswept)", "tip fins", "spar rib (tested plates had none)"]
TAGS = ("MEASURED (Ananda 2012)", "DERIVED from measured", "AVL", "EXTRAPOLATED")
COLORS = ["#7fb4ff", "#6cc08b", "#e89a55", "#c39bff", "#e0b83a", "#ff8a80"]


# =============================================================================================== data
def read_rows(path: Path = DATA):
    rows = []
    with path.open(encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            rows.append(dict(alpha=float(r["alpha_deg"]), CL=float(r["CL"]), CD=float(r["CD"]), CM=float(r["CM_c4"]), Re=float(r["Re"]),
                             AR=int(float(r["AR"])), taper=float(r["taper_ratio"]), run=r["run"], sweep=r["sweep"], seq=int(r["seq"])))
    return rows


def group_runs(rows):
    """-> dict[(AR, taper, run)] = dict(Re, sweeps={'increasing': [...], 'decreasing': [...]}) ; each sweep sorted by alpha."""
    runs = {}
    for r in rows:
        k = (r["AR"], r["taper"], r["run"])
        d = runs.setdefault(k, dict(AR=r["AR"], taper=r["taper"], run=r["run"], Re=r["Re"], sweeps={}))
        d["sweeps"].setdefault(r["sweep"], []).append(r)
    for d in runs.values():
        for s in d["sweeps"]:
            d["sweeps"][s].sort(key=lambda p: p["alpha"])
    return runs


def wing_id(ar, taper):
    return f"AR{ar}_taper{taper:g}"


# =============================================================================================== step 1
def cn_xcp(alpha, cl, cd, cm, x_ref=X_REF, cn_min=CN_MIN):
    """Normal force CN = CL cos(a) + CD sin(a); centre of pressure x_cp/c = x_ref/c - CM/CN (None where |CN| < cn_min)."""
    a = math.radians(alpha)
    cn = cl * math.cos(a) + cd * math.sin(a)
    return cn, (None if abs(cn) < cn_min else x_ref - cm / cn)


def annotate(points):
    out = []
    for p in points:
        cn, xcp = cn_xcp(p["alpha"], p["CL"], p["CD"], p["CM"])
        out.append(dict(p, CN=cn, xcp=xcp, LD=(p["CL"] / p["CD"] if p["CD"] > 0 else None)))
    return out


# =============================================================================================== step 2
def lsq(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    icpt = my - slope * mx
    res = [y - (icpt + slope * x) for x, y in zip(xs, ys)]
    rms = math.sqrt(sum(e * e for e in res) / n)
    return slope, icpt, rms, max(abs(e) for e in res)


def fit_low_angle(points, rng=FIT_RANGE):
    pts = [p for p in points if rng[0] <= p["alpha"] <= rng[1]]
    if len(pts) < 4:
        return None
    s_cl, i_cl, rms_cl, max_cl = lsq([p["alpha"] for p in pts], [p["CL"] for p in pts])
    s_cm, i_cm, rms_cm, max_cm = lsq([p["CL"] for p in pts], [p["CM"] for p in pts])
    return dict(n=len(pts), range=rng, cl_alpha_per_deg=s_cl, alpha0_deg=-i_cl / s_cl, rms_cl=rms_cl, max_cl=max_cl,
                dcm_dcl=s_cm, cm_at_zero_lift=i_cm, rms_cm=rms_cm, max_cm=max_cm, ac=X_REF - s_cm)


# =============================================================================================== step 3
def first_peak(br):
    """FIRST LIFT PEAK of a positive-lift branch (sorted by alpha).  Rule: low-angle slope s0 = least-squares dCL/dalpha over 2 to 6 deg; scan from PEAK_SEARCH_START deg; the first angle at which the slope
    over the next two intervals falls below PEAK_SLOPE_FRAC * s0 ends the rise; the peak is the highest CL up to the point after it.  This finds a true peak (CL then falls) and also the knee where lift
    stops rising and only creeps up afterwards (the plateau of several tested plates).  If the slope never falls that far before CLMAX_ALPHA_LIMIT, the global maximum up to that angle is used.
    -> (index, kind) with kind 'peak', 'flattening' or 'none (global maximum)'."""
    lim = [i for i, p in enumerate(br) if p["alpha"] <= CLMAX_ALPHA_LIMIT]
    if not lim:
        return None, None
    low = [p for p in br if 2.0 <= p["alpha"] <= 6.0]
    s0 = lsq([p["alpha"] for p in low], [p["CL"] for p in low])[0] if len(low) >= 3 else None
    if s0 and s0 > 0:
        for i in lim:
            if br[i]["alpha"] < PEAK_SEARCH_START or i + 2 >= len(br):
                continue
            slope = (br[i + 2]["CL"] - br[i]["CL"]) / (br[i + 2]["alpha"] - br[i]["alpha"])
            if slope < PEAK_SLOPE_FRAC * s0:
                j = max(range(0, i + 2), key=lambda k: br[k]["CL"])
                later = [p["CL"] for p in br[j + 1:] if p["alpha"] <= CLMAX_ALPHA_LIMIT]
                kind = "peak" if later and min(later) < (1 - PEAK_FALL) * br[j]["CL"] else "flattening"
                return j, kind
    return max(lim, key=lambda i: br[i]["CL"]), "none (global maximum)"


def global_max(br):
    lim = [i for i, p in enumerate(br) if p["alpha"] <= CLMAX_ALPHA_LIMIT]
    return max(lim, key=lambda i: br[i]["CL"]) if lim else None


def trim_branch(pts):
    """Positive-lift branch: from the first point with CN >= CN_MIN, contiguous in alpha.  -> (branch, index of the FIRST LIFT PEAK in branch; see first_peak)."""
    br = []
    for p in pts:
        if p["xcp"] is None or p["CN"] < CN_MIN:
            if br:
                break
            continue
        br.append(p)
    if len(br) < 4:
        return br, (None if not br else first_peak(br)[0])
    i_max, _ = first_peak(br)
    return br, i_max


def nonmonotonic(br, i_max):
    """Segments (alpha_lo, alpha_hi, fall) up to the first lift peak where x_cp FALLS with alpha; fall > MONO_TOL is 'major', else 'minor'."""
    out = []
    for i in range(min(i_max, len(br) - 2) + 1):
        a, b = br[i], br[i + 1]
        if b["xcp"] < a["xcp"]:
            out.append(dict(alpha_lo=a["alpha"], alpha_hi=b["alpha"], fall=a["xcp"] - b["xcp"], major=(a["xcp"] - b["xcp"]) > MONO_TOL))
    return out


def trim_at(xg, br, i_max):
    """First stable crossing x_cp(alpha) = xg (x_cp rising with alpha), searched up to the first lift peak + STALL_EXTRA_DEG."""
    a_max = br[i_max]["alpha"]
    for i in range(len(br) - 1):
        a, b = br[i], br[i + 1]
        if a["alpha"] > a_max + STALL_EXTRA_DEG:
            break
        if (a["xcp"] - xg) * (b["xcp"] - xg) <= 0 and b["xcp"] > a["xcp"]:
            t = (xg - a["xcp"]) / (b["xcp"] - a["xcp"])
            al = a["alpha"] + t * (b["alpha"] - a["alpha"])
            cl = a["CL"] + t * (b["CL"] - a["CL"])
            cd = a["CD"] + t * (b["CD"] - a["CD"])
            return dict(alpha=al, CL=cl, CD=cd, LD=cl / cd, stalled=al > a_max + 1e-9)
    return None


def grid():
    n = int(round((GRID_RANGE[1] - GRID_RANGE[0]) / GRID_STEP))
    return [round(GRID_RANGE[0] + i * GRID_STEP, 6) for i in range(n + 1)]


def trim_map(br, i_max):
    return [(x, trim_at(x, br, i_max)) for x in grid()]


def band(tmap, frac, best_idx, best_ld):
    """Contiguous x_cg interval around the best point where a stable PRE-STALL trim exists and trimmed L/D >= frac * best."""
    def ok(i):
        t = tmap[i][1]
        return t is not None and not t["stalled"] and t["LD"] >= frac * best_ld
    lo = hi = best_idx
    while lo - 1 >= 0 and ok(lo - 1):
        lo -= 1
    while hi + 1 < len(tmap) and ok(hi + 1):
        hi += 1
    return tmap[lo][0], tmap[hi][0]


def trim_reading(points):
    """Everything in step 3 for one sweep (points sorted by alpha, annotated).  -> dict or None."""
    br, i_max = trim_branch(points)
    if i_max is None or len(br) < 4:
        return None
    nm = nonmonotonic(br, i_max)
    tmap = trim_map(br, i_max)
    pre = [(i, t) for i, (x, t) in enumerate(tmap) if t is not None and not t["stalled"]]
    if not pre:
        return None
    best_idx, best = max(pre, key=lambda it: it[1]["LD"])
    out = dict(best_ld=best["LD"], best_xcg=tmap[best_idx][0], best_alpha=best["alpha"], best_cl=best["CL"], clmax=br[i_max]["CL"], alpha_clmax=br[i_max]["alpha"],
               xcp_at_clmax=br[i_max]["xcp"], nonmono=nm)
    for f in BAND_FRACS:
        lo, hi = band(tmap, f, best_idx, best["LD"])
        out[f"band{int(round(f * 100))}"] = (lo, hi)
    stalled = [x for x, t in tmap if t is not None and t["stalled"]]
    pre_x = [x for x, t in tmap if t is not None and not t["stalled"]]
    first_stalled = next((x for x, t in tmap if t is not None and t["stalled"] and (not pre_x or x >= min(pre_x))), None)
    out["xcg_stalled_trim"] = first_stalled
    front = min((p for p in br[:i_max + 1]), key=lambda p: p["xcp"])
    out["xcp_front"], out["xcp_front_alpha"] = front["xcp"], front["alpha"]
    out["xcp_front_cl"] = front["CL"]
    out["tmap"] = tmap
    out["branch"] = br
    out["major_nonmono"] = [s for s in nm if s["major"]]
    return out


# =============================================================================================== zero-lift offset, resolved quantities
def cm0_shift(points, cm0):
    """The same points with the fitted zero-lift moment removed (CM - CM0), re-annotated."""
    return annotate([dict(p, CM=p["CM"] - cm0) for p in points])


def cp_shift(cm0, cn):
    """Change of the computed x_cp/c when a zero-lift moment cm0 is removed: x_cp = 0.25 - (CM - cm0)/CN, so the change is +cm0/CN."""
    return cm0 / cn


def interp_cross(br, key_fn, target):
    """First upward crossing of key_fn(point) = target along the branch (linear between neighbouring points) -> (alpha, CL, CD, xcp) or None."""
    for a, b in zip(br, br[1:]):
        ka, kb = key_fn(a), key_fn(b)
        if ka < target <= kb:
            t = (target - ka) / (kb - ka)
            return tuple(a[k] + t * (b[k] - a[k]) for k in ("alpha", "CL", "CD", "xcp"))
    return None


def resolved(points, fit):
    """The quantities that are NOT dominated by the zero-lift moment offset: slope-based aerodynamic centre, centre of pressure at the first lift peak as an
    offset from it, the angle at which the aft travel of x_cp begins (criterion below), L/D near stall."""
    br, i_max = trim_branch(points)
    if i_max is None or fit is None or len(br) < 4:
        return None
    ac = fit["ac"]
    top = br[i_max]
    gi = global_max(br)
    kind = first_peak(br)[1]
    out = dict(ac=ac, bias_vs_quarter=X_REF - ac, clmax=top["CL"], alpha_clmax=top["alpha"], peak_kind=kind, alpha_global_max=br[gi]["alpha"], cl_global_max=br[gi]["CL"], xcp_clmax=top["xcp"], xcp_clmax_offset=100 * (top["xcp"] - ac),
               ld_clmax=top["CL"] / top["CD"])
    # onset of aft travel: baseline = median x_cp over ONSET_BASE_RANGE; onset = first angle after which x_cp stays >= baseline + ONSET_RISE up to the first lift peak
    base = sorted(p["xcp"] for p in br[:i_max + 1] if ONSET_BASE_RANGE[0] <= p["alpha"] <= ONSET_BASE_RANGE[1])
    out.update(onset_alpha=None, onset_cl=None, onset_ld=None, baseline_xcp=None)
    if len(base) >= 2:
        b = base[len(base) // 2] if len(base) % 2 else 0.5 * (base[len(base) // 2 - 1] + base[len(base) // 2])
        out["baseline_xcp"] = b
        thr = b + ONSET_RISE
        j = None
        for k in range(i_max, -1, -1):
            if br[k]["xcp"] < thr:
                j = k
                break
        if j is not None and j < i_max and br[j]["alpha"] >= ONSET_BASE_RANGE[0] - 1e-9:
            a, c = br[j], br[j + 1]
            t = (thr - a["xcp"]) / (c["xcp"] - a["xcp"]) if c["xcp"] != a["xcp"] else 0.0
            al = a["alpha"] + t * (c["alpha"] - a["alpha"])
            cl = a["CL"] + t * (c["CL"] - a["CL"])
            cd = a["CD"] + t * (c["CD"] - a["CD"])
            out.update(onset_alpha=al, onset_cl=cl, onset_ld=cl / cd)
    # near-stall trim: the point where lift first reaches NEAR_STALL_FRAC of the first-peak CL
    x = interp_cross(br[:i_max + 1], lambda p: p["CL"], NEAR_STALL_FRAC * top["CL"])
    out.update(near_alpha=None, near_ld=None, near_xcp_offset=None)
    if x:
        al, cl, cd, xcp = x
        out.update(near_alpha=al, near_ld=cl / cd, near_xcp_offset=100 * (xcp - ac))
    # how far the CP stays within +-PLATEAU_TOL of the aerodynamic centre (alpha extent of the longest contiguous stretch, pre-stall)
    best = cur = None
    runs_ = []
    for p in br[:i_max + 1]:
        if abs(p["xcp"] - ac) <= PLATEAU_TOL:
            cur = [p["alpha"], p["alpha"]] if cur is None else [cur[0], p["alpha"]]
        else:
            if cur:
                runs_.append(cur)
            cur = None
    if cur:
        runs_.append(cur)
    out["plateau_alpha_span"] = max((b - a for a, b in runs_), default=0.0)
    return out


# =============================================================================================== every wing / run / sweep, raw and offset-removed
def analyse(runs):
    results = []
    for (ar, taper, run_), d in sorted(runs.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[1]["Re"])):
        for sweep, pts in sorted(d["sweeps"].items(), key=lambda kv: kv[0] != "increasing"):
            raw = annotate(pts)
            fit = fit_low_angle(raw)
            cm0 = fit["cm_at_zero_lift"] if fit else 0.0
            rem = cm0_shift(pts, cm0)
            fit_rem = fit_low_angle(rem)
            results.append(dict(wing=wing_id(ar, taper), AR=ar, taper=taper, run=run_, Re=d["Re"], sweep=sweep, fit=fit, cm0=cm0,
                                raw=dict(points=raw, trim=trim_reading(raw), res=resolved(raw, fit)),
                                rem=dict(points=rem, trim=trim_reading(rem), res=resolved(rem, fit_rem))))
    return results


def offsets(trim, fit):
    """Offsets from the fitted aerodynamic centre, percent of the mean aerodynamic chord (used ONLY in the 'not resolved' table)."""
    if trim is None or fit is None:
        return None
    ac = fit["ac"]
    o = lambda x: None if x is None else 100.0 * (x - ac)
    return dict(best=o(trim["best_xcg"]), b90_lo=o(trim["band90"][0]), b90_hi=o(trim["band90"][1]), b75_lo=o(trim["band75"][0]), b75_hi=o(trim["band75"][1]),
                front=o(trim["xcp_front"]), best_ld=trim["best_ld"])


def summary_rows(results):
    """The summary rows shown and exported as the study's results: resolved quantities only (no balance-point-for-best-L/D, no bands)."""
    rows = []
    for r in results:
        for version in ("raw", "rem"):
            q = r[version]["res"]
            if not q:
                continue
            rows.append(dict(wing=r["wing"], AR=r["AR"], taper=r["taper"], run=r["run"], Re=r["Re"], sweep=r["sweep"], version="raw" if version == "raw" else "offset removed",
                             ac_pct=100 * q["ac"], bias_pct=100 * q["bias_vs_quarter"], cm0=r["cm0"], clmax=q["clmax"], alpha_clmax=q["alpha_clmax"], peak_kind=q["peak_kind"],
                             alpha_global=q["alpha_global_max"], cl_global=q["cl_global_max"], onset_resolved=r["AR"] >= AR_ONSET_RESOLVED_MIN, xcp_clmax_pct=100 * q["xcp_clmax"],
                             xcp_clmax_offset=q["xcp_clmax_offset"], onset_alpha=q["onset_alpha"], onset_cl=q["onset_cl"], baseline_xcp_pct=None if q["baseline_xcp"] is None else 100 * q["baseline_xcp"],
                             near_alpha=q["near_alpha"], near_ld=q["near_ld"], near_xcp_offset=q["near_xcp_offset"], ld_clmax=q["ld_clmax"], plateau_span=q["plateau_alpha_span"]))
    return rows


def not_resolved_rows(results):
    rows = []
    for r in results:
        for version in ("raw", "rem"):
            o = offsets(r[version]["trim"], r["fit"])
            if o:
                rows.append(dict(wing=r["wing"], run=r["run"], Re=r["Re"], sweep=r["sweep"], version="raw" if version == "raw" else "offset removed", **o))
    return rows


def resolvability(results):
    """Per run (both legs present): each quantity up/down, raw/removed, and the movements.  -> (rows, summary stats per quantity)."""
    by = {}
    for r in results:
        by.setdefault((r["wing"], r["run"]), {})[r["sweep"]] = r
    qs_nr = ["best", "b90_lo", "b90_hi", "b75_lo", "b75_hi", "front"]
    qs_res = ["ac_pct", "xcp_clmax_offset", "onset_alpha", "near_ld"]
    rows = []
    for (wing, run_), d in sorted(by.items()):
        if "increasing" not in d or "decreasing" not in d:
            continue
        get = {}
        for sw, r in d.items():
            for ver in ("raw", "rem"):
                o = offsets(r[ver]["trim"], r["fit"]) or {}
                q = r[ver]["res"] or {}
                vals = dict(o)
                vals.update(ac_pct=None if not q else 100 * q["ac"], xcp_clmax_offset=None if not q else q["xcp_clmax_offset"],
                            onset_alpha=None if not q else q["onset_alpha"], near_ld=None if not q else q["near_ld"])
                get[(sw, ver)] = vals
        for qn in qs_nr + qs_res:
            u_raw, d_raw, u_rem, d_rem = (get[("increasing", "raw")].get(qn), get[("decreasing", "raw")].get(qn), get[("increasing", "rem")].get(qn), get[("decreasing", "rem")].get(qn))
            if None in (u_raw, d_raw, u_rem, d_rem):
                continue
            rows.append(dict(wing=wing, run=run_, Re=d["increasing"]["Re"], quantity=qn, up_raw=u_raw, down_raw=d_raw, up_rem=u_rem, down_rem=d_rem,
                             updown_raw=abs(u_raw - d_raw), updown_rem=abs(u_rem - d_rem), move_up=abs(u_rem - u_raw), move_down=abs(d_rem - d_raw),
                             resolved=qn in qs_res and not (qn == "onset_alpha" and d["increasing"]["AR"] < AR_ONSET_RESOLVED_MIN)))
    stats = {}
    for qn in qs_nr + qs_res + ["onset_alpha_AR2"]:
        if qn == "onset_alpha_AR2":
            rs = [r for r in rows if r["quantity"] == "onset_alpha" and not r["resolved"]]
        elif qn == "onset_alpha":
            rs = [r for r in rows if r["quantity"] == qn and r["resolved"]]
        else:
            rs = [r for r in rows if r["quantity"] == qn]
        if not rs:
            continue
        med = lambda v: sorted(v)[len(v) // 2]
        stats[qn] = dict(n=len(rs), updown_raw_med=med([r["updown_raw"] for r in rs]), updown_raw_max=max(r["updown_raw"] for r in rs),
                         updown_rem_med=med([r["updown_rem"] for r in rs]), updown_rem_max=max(r["updown_rem"] for r in rs),
                         move_med=med([m for r in rs for m in (r["move_up"], r["move_down"])]), move_max=max(m for r in rs for m in (r["move_up"], r["move_down"])),
                         resolved=rs[0]["resolved"])
    # band width of the 90% band for comparison
    widths = []
    for r in results:
        o = offsets(r["raw"]["trim"], r["fit"])
        if o:
            widths.append(o["b90_hi"] - o["b90_lo"])
    stats["_b90_width_med"] = sorted(widths)[len(widths) // 2] if widths else None
    return rows, stats


# =============================================================================================== CSV writers
def fmt(v, n=4):
    return "" if v is None else (f"{v:.{n}f}" if isinstance(v, float) else str(v))


def write_csv(path: Path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def write_all_csvs(out: Path, results, summ, notres, resrows):
    s1 = []
    for r in results:
        for p, pr in zip(r["raw"]["points"], r["rem"]["points"]):
            s1.append([r["wing"], r["AR"], r["taper"], r["run"], fmt(r["Re"], 1), r["sweep"], p["seq"], p["alpha"], p["CL"], p["CD"], p["CM"], fmt(p["LD"], 3), fmt(p["CN"], 4), fmt(p["xcp"], 4), fmt(pr["xcp"], 4)])
    write_csv(out / "step1_cp_table.csv", ["wing", "AR", "taper", "run", "Re", "sweep", "seq", "alpha_deg", "CL", "CD", "CM_c4", "L_over_D", "CN", "x_cp_over_c_raw", "x_cp_over_c_offset_removed"], s1)
    s2 = []
    for r in results:
        f = r["fit"]
        if f:
            s2.append([r["wing"], r["run"], fmt(r["Re"], 1), r["sweep"], f["range"][0], f["range"][1], f["n"], fmt(f["cl_alpha_per_deg"], 5), fmt(f["alpha0_deg"], 3), fmt(f["dcm_dcl"], 5), fmt(f["ac"], 4),
                       fmt(f["cm_at_zero_lift"], 5), fmt(cp_shift(f["cm_at_zero_lift"], 0.2), 4), fmt(cp_shift(f["cm_at_zero_lift"], 0.4), 4), fmt(f["rms_cl"], 5), fmt(f["max_cl"], 5), fmt(f["rms_cm"], 5), fmt(f["max_cm"], 5)])
    write_csv(out / "step2_fits.csv", ["wing", "run", "Re", "sweep", "alpha_min", "alpha_max", "n", "cl_alpha_per_deg", "alpha0_deg", "dCM_dCL", "aero_centre_over_c", "CM_at_zero_lift (should be 0)",
                                       "x_cp_change_if_removed_at_CN0.2", "x_cp_change_if_removed_at_CN0.4", "rms_CL", "max_abs_CL", "rms_CM", "max_abs_CM"], s2)
    write_csv(out / "step3_summary.csv", ["wing", "AR", "taper", "run", "Re", "sweep", "version", "AC_pct_chord", "AC_ahead_of_quarter_chord_pct", "CM_at_zero_lift", "CL_first_peak", "alpha_first_peak_deg", "first_peak_kind",
                                          "xcp_at_first_peak_pct_chord", "xcp_at_first_peak_offset_from_AC_pct_MAC", "baseline_xcp_pct_chord", "aft_travel_onset_alpha", "CL_at_onset", "aft_travel_onset_resolved",
                                          "near_stall_alpha (CL=%.0f%% of first-peak CL)" % (100 * NEAR_STALL_FRAC), "LD_at_near_stall", "xcp_offset_at_near_stall_pct_MAC", "LD_at_first_peak",
                                          "alpha_span_xcp_within_%.0fpct_of_AC" % (100 * PLATEAU_TOL), "alpha_global_max_to_20deg", "CL_global_max_to_20deg"],
              [[s["wing"], s["AR"], s["taper"], s["run"], fmt(s["Re"], 1), s["sweep"], s["version"], fmt(s["ac_pct"], 2), fmt(s["bias_pct"], 2), fmt(s["cm0"], 5), fmt(s["clmax"], 3), fmt(s["alpha_clmax"], 2), s["peak_kind"],
                fmt(s["xcp_clmax_pct"], 2), fmt(s["xcp_clmax_offset"], 2), fmt(s["baseline_xcp_pct"], 2), fmt(s["onset_alpha"], 2), fmt(s["onset_cl"], 3), "yes" if s["onset_resolved"] else "NO (offset-dependent, AR 2)",
                fmt(s["near_alpha"], 2), fmt(s["near_ld"], 2), fmt(s["near_xcp_offset"], 2), fmt(s["ld_clmax"], 2), fmt(s["plateau_span"], 2), fmt(s["alpha_global"], 2), fmt(s["cl_global"], 3)] for s in summ])
    write_csv(out / "step3_NOT_RESOLVED.csv", ["wing", "run", "Re", "sweep", "version", "best_LD", "best_LD_balance_offset_from_AC_pct_MAC", "band90_lo", "band90_hi", "band75_lo", "band75_hi", "most_forward_xcp_offset",
                                               "note"],
              [[n["wing"], n["run"], fmt(n["Re"], 1), n["sweep"], n["version"], fmt(n["best_ld"], 2), fmt(n["best"], 2), fmt(n["b90_lo"], 2), fmt(n["b90_hi"], 2), fmt(n["b75_lo"], 2), fmt(n["b75_hi"], 2), fmt(n["front"], 2),
                "NOT RESOLVED by this data: inside the zero-lift-offset and up/down repeatability error"] for n in notres])
    write_csv(out / "step2b_resolvability.csv", ["wing", "run", "Re", "quantity", "resolved", "up_raw", "down_raw", "up_offset_removed", "down_offset_removed", "abs_up_minus_down_raw", "abs_up_minus_down_removed",
                                                 "abs_move_removing_offset_up", "abs_move_removing_offset_down"],
              [[r["wing"], r["run"], fmt(r["Re"], 1), r["quantity"], "yes" if r["resolved"] else "NO", fmt(r["up_raw"], 3), fmt(r["down_raw"], 3), fmt(r["up_rem"], 3), fmt(r["down_rem"], 3), fmt(r["updown_raw"], 3),
                fmt(r["updown_rem"], 3), fmt(r["move_up"], 3), fmt(r["move_down"], 3)] for r in resrows])
    # step 4: offsets from the aerodynamic centre of the resolved points (first lift peak, near stall), both versions
    s4 = []
    for s in summ:
        s4.append([s["wing"], s["run"], fmt(s["Re"], 1), s["sweep"], s["version"], fmt(s["ac_pct"], 2), fmt(s["xcp_clmax_offset"], 2), fmt(s["near_xcp_offset"], 2), fmt(s["onset_alpha"], 2)])
    write_csv(out / "step4_offsets_from_ac.csv", ["wing", "run", "Re", "sweep", "version", "AC_pct_chord", "xcp_at_first_peak_offset_pct_MAC", "xcp_near_stall_offset_pct_MAC", "aft_travel_onset_alpha"], s4)


# =============================================================================================== step 5 trends (raw, increasing sweep; AC also from the decreasing sweep)
def trend_rows(results):
    by = {}
    for r in results:
        by.setdefault((r["wing"], r["run"]), {})[r["sweep"]] = r
    rows = []
    for (wing, run_), d in sorted(by.items(), key=lambda kv: (kv[1]["increasing"]["AR"], kv[1]["increasing"]["taper"], kv[1]["increasing"]["Re"])):
        up = d["increasing"]
        q, qr = up["raw"]["res"], up["rem"]["res"]
        if not q or not qr:
            continue
        dn = d.get("decreasing")
        qd = dn["raw"]["res"] if dn else None
        rows.append(dict(wing=wing, AR=up["AR"], taper=up["taper"], Re=up["Re"], run=run_, ac_up=100 * q["ac"], ac_down=None if not qd else 100 * qd["ac"], bias=100 * q["bias_vs_quarter"], cm0=up["cm0"],
                         xcp_clmax_offset=q["xcp_clmax_offset"], xcp_clmax_offset_rem=qr["xcp_clmax_offset"], onset_alpha=q["onset_alpha"], onset_alpha_rem=qr["onset_alpha"], clmax=q["clmax"],
                         alpha_clmax=q["alpha_clmax"], near_ld=q["near_ld"], ld_clmax=q["ld_clmax"], plateau_span=q["plateau_alpha_span"]))
    return rows


def write_trends(out: Path, rows):
    write_csv(out / "step5_trends.csv", ["wing", "AR", "taper", "Re", "run", "AC_pct_chord_up", "AC_pct_chord_down", "AC_ahead_of_quarter_chord_pct", "CM_at_zero_lift_up", "xcp_at_first_peak_offset_raw",
                                          "xcp_at_first_peak_offset_removed", "onset_alpha_raw", "onset_alpha_removed", "CL_first_peak", "alpha_first_peak_deg", "LD_near_stall", "LD_at_first_peak", "alpha_span_xcp_within_tol_of_AC"],
              [[r["wing"], r["AR"], r["taper"], fmt(r["Re"], 1), r["run"], fmt(r["ac_up"], 2), fmt(r["ac_down"], 2), fmt(r["bias"], 2), fmt(r["cm0"], 5), fmt(r["xcp_clmax_offset"], 2), fmt(r["xcp_clmax_offset_rem"], 2),
                fmt(r["onset_alpha"], 2), fmt(r["onset_alpha_rem"], 2), fmt(r["clmax"], 3), fmt(r["alpha_clmax"], 2), fmt(r["near_ld"], 2), fmt(r["ld_clmax"], 2), fmt(r["plateau_span"], 2)] for r in rows])


# =============================================================================================== step 6 (reduced): two rows per specimen
def nearest_wings(ar, taper, tested):
    d = sorted(tested, key=lambda w: math.hypot(w[0] - ar, (w[1] - taper) / D_TAPER_SCALE))
    return d[:N_NEAREST]


def kit_numbers(spec, kit=KIT):
    G = json.loads((kit / spec / "geometry.json").read_text())
    v = lambda k: G[k]["value"]
    tt = G["thickness_table"]
    key = "0.6" if "0.6" in tt else list(tt)[0]
    res = [tt[key][k]["value"] for k in tt[key] if (k.startswith("re_tip") or k.startswith("re_mean")) and tt[key][k]["value"]]     # tip and mean chord, CL 0.3 and 0.5
    return dict(spec=spec, np_mm=G["avl_with_tip_fins"]["neutral_point_mm"]["value"], mac_mm=v("mac_mm"), ar=v("aspect_ratio"), taper=v("taper"),
                t_mean_pct=v("thickness_pct_mean_chord"), t_tip_pct=v("thickness_pct_tip_chord"), re_min=min(res), re_max=max(res),
                sweep=v("quarter_chord_sweep_deg"), t_source=G["thickness_pct_mean_chord"]["source"])


def carry_over(results, trends, kit=KIT, specimens=SPECIMENS):
    """Two rows per specimen, whole millimetres, NOT a CG setting.
    Row 1 (low-angle aerodynamic centre): AVL neutral point with fins moved FORWARD by the measured bias (quarter chord minus measured AC, % MAC)
    of the nearest tested wings (all Reynolds numbers, both legs): it ranges from the neutral point (no bias) to neutral point minus the largest bias.
    Row 2 (where near-stall trim might end): row-1 position plus the measured offset of the CP at the first lift peak from the aerodynamic centre
    (raw and offset-removed, both legs, all Re of the nearest wings)."""
    tested = sorted({(r["AR"], r["taper"]) for r in trends})
    re_tested_min = min(r["Re"] for r in trends)
    out = []
    for spec in specimens:
        k = kit_numbers(spec, kit)
        near = nearest_wings(k["ar"], k["taper"], tested)
        sel = [r for r in results if (r["AR"], r["taper"]) in near and r["raw"]["res"] and r["rem"]["res"]]
        bias = [100 * r["raw"]["res"]["bias_vs_quarter"] for r in sel]
        off = [r[v]["res"]["xcp_clmax_offset"] for r in sel for v in ("raw", "rem")]
        b_lo, b_hi = min(bias), max(bias)
        np_, mac = k["np_mm"], k["mac_mm"]
        ac_lo, ac_hi = np_ - b_hi * mac / 100, np_ - b_lo * mac / 100                  # measured AC sits (quarter chord - AC) ahead of the quarter chord, taken as the AVL neutral point
        lo2, hi2 = ac_lo + min(off) * mac / 100, ac_hi + max(off) * mac / 100
        re_low = {}
        for w in near:
            rs = sorted((r for r in trends if (r["AR"], r["taper"]) == w), key=lambda r: r["Re"])
            re_low[w] = (rs[0], rs[-1])
        trend = "; ".join(f"{wing_id(*w)}: {a['bias']:.1f}% MAC ahead at Re {a['Re'] / 1000:.0f}k, {b['bias']:.1f}% at Re {b['Re'] / 1000:.0f}k" for w, (a, b) in re_low.items())
        not_tested = UNTESTED + [f"Re {k['re_min']:,.0f}-{k['re_max']:,.0f} here versus tested minimum {re_tested_min:,.0f} (the bias is larger at lower Re in the data; not extrapolated)",
                                 f"thickness {k['t_mean_pct']:.2f}% of mean chord ({k['t_source']}) versus {TESTED_T_OVER_C_PCT}% tested"]
        base = dict(spec=spec, ar=k["ar"], taper=k["taper"], np_mm=np_, mac_mm=mac, nearest=", ".join(wing_id(*w) for w in near), not_tested="; ".join(not_tested), kit=k, n_runs=len(sel), trend=trend)
        out.append(dict(base, quantity="where the low-angle aerodynamic centre might be", off_lo=-b_hi, off_hi=-b_lo, mm_lo=math.floor(ac_lo), mm_hi=math.ceil(ac_hi)))
        out.append(dict(base, quantity="where near-stall trim (CP at the first lift peak) might end", off_lo=min(off), off_hi=max(off), mm_lo=math.floor(lo2), mm_hi=math.ceil(hi2)))
    return out


def write_carry_over(out: Path, rows):
    write_csv(out / "step6_if_it_carries_over.csv", ["specimen", "AR", "taper", "AVL_neutral_point_with_fins_mm", "MAC_mm", "nearest_tested_wings", "quantity", "mm_aft_of_root_LE_from", "mm_aft_of_root_LE_to",
                                                    "offset_range_pct_MAC", "bias_trend_with_Re (measured)", "n_runs", "NOT_TESTED", "NOTE", "tag"],
              [[r["spec"], r["ar"], r["taper"], fmt(r["np_mm"], 2), fmt(r["mac_mm"], 2), r["nearest"], r["quantity"], r["mm_lo"], r["mm_hi"], f"{r['off_lo']:+.1f} to {r['off_hi']:+.1f}", r["trend"], r["n_runs"], r["not_tested"],
                "NOT a CG setting; does not replace the CG sweep", "EXTRAPOLATED (offsets MEASURED/DERIVED; neutral point AVL)"] for r in rows])


# =============================================================================================== SVG
def svg_plot(title, xl, yl, series, xlim, ylim, hlines=(), w=420, h=300):
    L, R, T, B = 52, 12, 30, 44
    pw, ph = w - L - R, h - T - B
    sx = lambda x: L + (x - xlim[0]) / (xlim[1] - xlim[0]) * pw
    sy = lambda y: T + ph - (y - ylim[0]) / (ylim[1] - ylim[0]) * ph
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" font-family="Segoe UI,Arial" font-size="11" fill="#dde3ea">',
         f'<rect width="100%" height="100%" fill="#1c2229"/>', f'<text x="{w / 2}" y="18" text-anchor="middle" font-size="13" font-weight="bold">{html.escape(title)}</text>']
    for k in range(6):
        gx, gy = xlim[0] + k * (xlim[1] - xlim[0]) / 5, ylim[0] + k * (ylim[1] - ylim[0]) / 5
        o.append(f'<line x1="{sx(gx):.1f}" y1="{T}" x2="{sx(gx):.1f}" y2="{T + ph}" stroke="#333c46"/><line x1="{L}" y1="{sy(gy):.1f}" x2="{L + pw}" y2="{sy(gy):.1f}" stroke="#333c46"/>')
        o.append(f'<text x="{sx(gx):.1f}" y="{T + ph + 14}" text-anchor="middle">{gx:.3g}</text><text x="{L - 6}" y="{sy(gy) + 4:.1f}" text-anchor="end">{gy:.3g}</text>')
    o.append(f'<rect x="{L}" y="{T}" width="{pw}" height="{ph}" fill="none" stroke="#9aa7b4"/>')
    for yv, lab in hlines:
        if ylim[0] <= yv <= ylim[1]:
            o.append(f'<line x1="{L}" y1="{sy(yv):.1f}" x2="{L + pw}" y2="{sy(yv):.1f}" stroke="#e89a55" stroke-dasharray="5 3"/><text x="{L + pw - 3}" y="{sy(yv) - 3:.1f}" text-anchor="end" fill="#e89a55">{html.escape(lab)}</text>')
    for i, (label, pts) in enumerate(series):
        col = COLORS[i % len(COLORS)]
        seg = []
        for p in pts + [None]:
            if p is None or p[1] is None:
                if len(seg) > 1:
                    o.append(f'<polyline points="{" ".join(f"{sx(x):.1f},{sy(min(max(y, ylim[0]), ylim[1])):.1f}" for x, y in seg)}" fill="none" stroke="{col}" stroke-width="1.8"/>')
                seg = []
            else:
                seg.append(p)
        o.append(f'<line x1="{L + 4 + (i % 4) * 96}" y1="{h - 8 - 13 * (i // 4)}" x2="{L + 22 + (i % 4) * 96}" y2="{h - 8 - 13 * (i // 4)}" stroke="{col}" stroke-width="3"/><text x="{L + 26 + (i % 4) * 96}" y="{h - 4 - 13 * (i // 4)}">{html.escape(label)}</text>')
    o.append(f'<text x="{L + pw / 2}" y="{h - 26 - 13 * ((len(series) - 1) // 4)}" text-anchor="middle">{html.escape(xl)}</text>')
    o.append(f'<text x="13" y="{T + ph / 2}" text-anchor="middle" transform="rotate(-90 13 {T + ph / 2})">{html.escape(yl)}</text></svg>')
    return "\n".join(o)


# =============================================================================================== HTML
def tag(s):
    return f' <span class="tag">[{s}]</span>'


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else None


def page(results, summ, notres, resrows, stats, trends, carry, rows_n, n_runs, out: Path):
    import flat_test as ft                                              # palette only (CSS string); importing writes nothing
    h = html.escape
    up_raw = [r for r in results if r["sweep"] == "increasing" and r["raw"]["res"]]
    dn_raw = [r for r in results if r["sweep"] == "decreasing" and r["raw"]["res"]]
    f1 = lambda v, n=1: "-" if v is None else f"{v:.{n}f}"
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Flat-plate balance study</title>"
         f"<style>{ft.CSS}\n.plots{{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px}}.plots svg{{max-width:100%;height:auto}}"
         f".warn{{background:#2c2418;border:2px solid #e89a55;padding:12px 16px}}.nr{{background:#241c1c;border:2px solid #ff8a80;padding:12px 16px}}table.t td,table.t th{{padding:4px 7px;font-size:13.5px}}</style></head><body><main>"]
    P.append("<h1>Flat-plate balance study from measured data</h1>")
    P.append("<p class='note'>Analysis of measured wind-tunnel tables only. Every number is tagged: MEASURED (Ananda 2012) = copied from the thesis tables; DERIVED from measured = our arithmetic on those tables; AVL = from our vortex-lattice outline model; EXTRAPOLATED = measured offsets applied to a wing that was not tested. Nothing here is written into the flat-test kit. This is a static moment balance, not a flight result.</p>")
    # ---- physical reading, numbers from our own tables
    ar3 = next((r for r in up_raw if r["wing"] == "AR3_taper1" and abs(r["Re"] - 60000) < 100), None)
    spans = [r["raw"]["res"]["plateau_alpha_span"] for r in up_raw]
    q3 = ar3["raw"]["res"] if ar3 else None
    res_up = [r for r in up_raw if r["AR"] >= AR_ONSET_RESOLVED_MIN]                                 # wings whose onset angle is resolved
    onsets = [r["raw"]["res"]["onset_alpha"] for r in res_up if r["raw"]["res"]["onset_alpha"] is not None]
    peaks = [r["raw"]["res"]["alpha_clmax"] for r in res_up]
    gaps = [r["raw"]["res"]["alpha_clmax"] - r["raw"]["res"]["onset_alpha"] for r in res_up if r["raw"]["res"]["onset_alpha"] is not None]
    ar2 = [r for r in results if r["AR"] < AR_ONSET_RESOLVED_MIN and r["sweep"] == "increasing" and r["raw"]["res"] and r["rem"]["res"] and r["raw"]["res"]["onset_alpha"] is not None and r["rem"]["res"]["onset_alpha"] is not None]
    ar2_txt = ("For aspect ratio 2 the onset angle is <b>offset-dependent and not resolved</b>: " + "; ".join(f"{r['wing']} Re {r['Re'] / 1000:.0f}k up-leg {r['raw']['res']['onset_alpha']:.1f} deg raw, {r['rem']['res']['onset_alpha']:.1f} deg with the zero-lift moment removed" for r in ar2[:2])
               + " (aspect ratio 3 and up move by about 0.5 deg or less). ") if ar2 else ""
    near_ld_med = med([r["raw"]["res"]["near_ld"] for r in up_raw if r["raw"]["res"]["near_ld"] is not None])
    ldmax_med = med([r["raw"]["res"]["ld_clmax"] for r in up_raw])
    kinds = sorted({r["raw"]["res"]["peak_kind"] for r in up_raw})
    P.append("<section><h2>What the data supports</h2><p>"
             f"Within about 1% of the mean aerodynamic chord (MAC) of the aerodynamic centre, the measured centre of pressure barely moves as the angle changes: on the increasing sweeps it stays within 1% MAC of the aerodynamic centre over a median of {med(spans):.1f} degrees of angle of attack (range {min(spans):.1f} to {max(spans):.1f}). "
             "A balance point there has almost no restoring moment, so the trim angle is nearly indeterminate. "
             f"The centre of pressure starts to move aft clearly (increasing sweeps, aspect ratio 3 and up, {len(onsets)} wings) at a median angle of {med(onsets):.1f} degrees, while the <b>first lift peak</b> (rule in section 4) comes at a median {med(peaks):.1f} degrees (range {min(peaks):.1f} to {max(peaks):.1f}); the aft travel therefore begins a median {med(gaps):.1f} degrees before the first lift peak. "
             + ar2_txt
             + "Aft of the onset a stable trim exists, but only close to stall. "
             + (f"On the AR 3, taper 1, Re 60k plate the aft travel starts at {q3['onset_alpha']:.1f} degrees with L/D {q3['onset_ld']:.1f}; the first lift peak is at {q3['alpha_clmax']:.1f} degrees; at 90% of the first-peak lift L/D is {q3['near_ld']:.1f}; at the first peak it is {q3['ld_clmax']:.1f}. " if q3 and q3["onset_alpha"] is not None else "")
             + f"Across all wings the median L/D at 90% of first-peak lift is {near_ld_med:.1f} and at the first peak {ldmax_med:.1f}. All of this is from unswept, 4.3%-thick plates at Reynolds number 60,000 and above" + tag("DERIVED from measured") + ".</p></section>")
    P.append("<section><h2>1. What this is</h2><p>A flat plate has no airfoil camber, so where its lift acts (the centre of pressure, CP) changes with angle of attack. This page reads the CP out of measured force and moment tables for ten flat-plate wings. It is a <b>static moment balance only</b>: it says where a balance point could sit under the CP at each angle. It says nothing about whether a glide is steady, and it does not show which balance points glide.</p>"
             "<p>Source: G. K. Ananda Krishnan, M.S. thesis, University of Illinois 2012, Appendix B and C tables (aspect ratio 2, 3, 4 at taper ratio 0.5, 0.75, 1 and aspect ratio 5 at taper 1; thickness 4.3% of chord; unswept; several Reynolds numbers, 60,000 and up)" + tag("MEASURED (Ananda 2012)") + f". {n_runs} runs, {rows_n} rows. Each run has an increasing-angle and a decreasing-angle sweep; they are reported separately.</p>"
             "<p><b>Moment reference.</b> Moments are about the <b>quarter chord</b> (thesis PDF p. 32: wings mounted at quarter chord on a wing-tip sting; Table 3.1 &quot;Mc/4&quot;; p. 49: taper was varied about the quarter chord so the quarter-chord line is the same for every wing). The reference chord is the mean aerodynamic chord, 3.5 in for all wings (p. 50). One detail was not found: the thesis equation that turns moment into the coefficient CM. We take it as normalised by that same chord, as the README does. All &quot;% chord&quot; are measured from the leading edge of the mean aerodynamic chord.</p>"
             "<p>Formulas" + tag("DERIVED from measured") + ": normal force CN = CL cos(alpha) + CD sin(alpha); CP position x_cp/c = 0.25 &minus; CM/CN (points with |CN| below 0.05 skipped); L/D = CL/CD. Check: AR 3, taper 1, Re 60k, alpha 4.54: CL 0.277, CD 0.0330, CM 0.0124 gives CN 0.2787, x_cp 0.2055 c, L/D 8.4.</p></section>")
    # ---- zero-lift moment
    cm0s = [r["cm0"] for r in results]
    P.append("<section><h2>2. The zero-lift moment is not zero</h2><p>A symmetric flat plate should give <b>zero moment at zero lift</b>. The fitted zero-lift moment (intercept of CM against CL, alpha " + f"{FIT_RANGE[0]:g} to {FIT_RANGE[1]:g} deg" + ") is not zero in these runs: "
             f"it ranges from {min(cm0s):+.4f} to {max(cm0s):+.4f}" + tag("DERIVED from measured") + ". Whatever the cause (balance offset, sting, tare), the computed CP absorbs it: removing a zero-lift moment CM0 changes the computed x_cp by <b>CM0 / CN</b>. "
             f"Worked sizes: CM0 = &minus;0.008 moves x_cp by &minus;0.040 chord (4% of chord) at CN 0.2 and &minus;0.020 (2%) at CN 0.4; CM0 = +0.002 moves it by +0.010 (1%) at CN 0.2 and +0.005 at CN 0.4. "
             "That is as large as the balance-point differences discussed below, so the study is run twice: <b>raw</b>, and with each sweep's fitted zero-lift moment <b>removed</b>. Both versions are kept in every CSV.</p>")
    P.append("<div class='scroll'><table class='t'><tr><th>wing</th><th>Re</th><th>CM0 up</th><th>CM0 down</th><th>x_cp change if removed, up, CN 0.2</th><th>CN 0.4</th></tr>")
    by = {}
    for r in results:
        by.setdefault((r["wing"], r["run"]), {})[r["sweep"]] = r
    for (wing, run_), d in sorted(by.items(), key=lambda kv: (kv[1]["increasing"]["AR"], kv[1]["increasing"]["taper"], kv[1]["increasing"]["Re"])):
        u, dn = d["increasing"], d.get("decreasing")
        P.append(f"<tr><td>{wing}</td><td class='n'>{u['Re']:,.0f}</td><td class='n'>{u['cm0']:+.4f}</td><td class='n'>{'-' if not dn else format(dn['cm0'], '+.4f')}</td><td class='n'>{cp_shift(u['cm0'], 0.2):+.3f}</td><td class='n'>{cp_shift(u['cm0'], 0.4):+.3f}</td></tr>")
    P.append("</table></div></section>")
    # ---- aerodynamic centre
    P.append("<section><h2>3. The slope-based aerodynamic centre, and how it changes with Reynolds number</h2><p>The aerodynamic centre is where the moment does not change with lift: AC = 0.25 &minus; dCM/dCL from a straight-line fit (alpha " + f"{FIT_RANGE[0]:g} to {FIT_RANGE[1]:g} deg" + "). A zero-lift offset does not change a slope, so this is the best-determined number in the study" + tag("DERIVED from measured") + ".</p>")
    P.append("<div class='scroll'><table class='t'><tr><th>wing</th><th>Re</th><th>AC up, % chord</th><th>AC down</th><th>ahead of quarter chord by, % MAC (up)</th><th>lift slope /deg (up)</th><th>alpha0 deg (up)</th><th>rms CL</th><th>rms CM</th></tr>")
    for (wing, run_), d in sorted(by.items(), key=lambda kv: (kv[1]["increasing"]["AR"], kv[1]["increasing"]["taper"], kv[1]["increasing"]["Re"])):
        u, dn = d["increasing"], d.get("decreasing")
        f = u["fit"]
        P.append(f"<tr><td>{wing}</td><td class='n'>{u['Re']:,.0f}</td><td class='n'>{100 * f['ac']:.1f}</td><td class='n'>{'-' if not dn or not dn['fit'] else format(100 * dn['fit']['ac'], '.1f')}</td><td class='n'>{100 * (X_REF - f['ac']):+.1f}</td><td class='n'>{f['cl_alpha_per_deg']:.4f}</td><td class='n'>{f['alpha0_deg']:.2f}</td><td class='n'>{f['rms_cl']:.4f}</td><td class='n'>{f['rms_cm']:.4f}</td></tr>")
    P.append("</table></div><div class='plots'>")
    ser = []
    for wg in sorted({r["wing"] for r in trends}):
        ser.append((wg, [(r["Re"] / 1000, r["ac_up"]) for r in trends if r["wing"] == wg]))
    for i in range(0, len(ser), 5):
        P.append(svg_plot("Aerodynamic centre vs Reynolds number", "Re, thousands", "AC, % chord", ser[i:i + 5], (50, 170), (15, 30), [(25, "quarter chord")]))
    P.append("</div></section>")
    # ---- CP vs alpha and resolved quantities
    P.append("<section><h2>4. Where the centre of pressure goes</h2><p>Plots: CP position against angle of attack for each wing, one colour per Reynolds number, increasing sweep. Left: raw. Right: fitted zero-lift moment removed. Orange dashed line = the moment reference, 25% chord.</p><div class='plots'>")
    wings = sorted({r["wing"] for r in results})
    for wg in wings:
        rs = [r for r in results if r["wing"] == wg and r["sweep"] == "increasing"]
        for ver, ttl in (("raw", "raw"), ("rem", "zero-lift moment removed")):
            s1 = [(f"Re {r['Re'] / 1000:.0f}k", [(p["alpha"], p["xcp"]) for p in r[ver]["points"] if p["xcp"] is not None and 0 <= p["alpha"] <= 20]) for r in rs]
            P.append(svg_plot(f"{wg}: CP vs alpha ({ttl})", "alpha, deg", "x_cp / c", s1, (0, 20), (0, 0.7), [(0.25, "25% (moment reference)")]))
    P.append("</div>")
    P.append("<p><b>Criteria</b> (parameters at the top of plate_balance.py). <i>Aft travel begins</i>: the baseline is the median x_cp over alpha " + f"{ONSET_BASE_RANGE[0]:g} to {ONSET_BASE_RANGE[1]:g} deg; the onset angle is the first angle after which x_cp stays at least {100 * ONSET_RISE:g}% chord above that baseline up to the first lift peak. "
             f"<i>First lift peak</i> (this page's &quot;the first lift peak&quot; of earlier versions): the lift slope is fitted at low angle (alpha 2 to 6 deg); scanning upward from {PEAK_SEARCH_START:g} deg, the first angle at which the slope over the next two intervals falls below {100 * PEAK_SLOPE_FRAC:g}% of that low-angle slope ends the rise, and the peak is the highest CL up to the point after it. This finds a true peak (lift then falls by more than {100 * PEAK_FALL:g}%: marked &quot;peak&quot; in step3_summary.csv) and also the knee where lift stops rising and only creeps up afterwards (&quot;flattening&quot;, the plateau seen on several plates). If the slope never falls that far before {CLMAX_ALPHA_LIMIT:g} deg the global maximum up to {CLMAX_ALPHA_LIMIT:g} deg is used (&quot;none&quot;). The earlier version of this page used the global maximum up to 20 deg, which often sat on the post-stall plateau; that value is kept in step3_summary.csv as alpha_global_max_to_20deg. "
             f"<i>Near-stall trim</i>: the point where lift first reaches {100 * NEAR_STALL_FRAC:g}% of the first-peak lift. <i>CP at the first lift peak</i>: x_cp at the first lift peak. L/D at these points does not depend on the moment, so it is not affected by the zero-lift offset" + tag("DERIVED from measured") + ".</p>")
    P.append("<p><b>Caution on balance points and bands.</b> Balance points for best L/D and bands of balance points can be derived, but they sit inside the zero-lift-offset error and the up/down repeatability error (section 7). In the same way, a statement such as &quot;no positive-lift trim exists ahead of x&quot; depends on the zero-lift offset: the most forward CP moves by CM0/CN. They are therefore not part of the results here.</p>")
    for sweep, rs in (("increasing", up_raw), ("decreasing", dn_raw)):
        P.append(f"<h3>{sweep} sweep, raw (offset-removed values are in step3_summary.csv)</h3><div class='scroll'><table class='t'><tr><th>wing</th><th>Re</th><th>first-peak CL at alpha</th><th>CP at first peak, % chord</th><th>offset from AC, % MAC</th><th>aft travel begins, deg</th><th>L/D at onset</th><th>L/D at 90% first-peak lift</th><th>L/D at first peak</th><th>kind</th><th>alpha span CP within 1% of AC</th></tr>")
        for r in rs:
            q = r["raw"]["res"]
            P.append(f"<tr><td>{r['wing']}</td><td class='n'>{r['Re']:,.0f}</td><td class='n'>{q['clmax']:.2f} at {q['alpha_clmax']:.1f}</td><td class='n'>{100 * q['xcp_clmax']:.1f}</td><td class='n'>{q['xcp_clmax_offset']:+.1f}</td><td class='n'>{f1(q['onset_alpha'])}{'' if r['AR'] >= AR_ONSET_RESOLVED_MIN else ' *'}</td><td class='n'>{f1(q['onset_ld'])}</td><td class='n'>{f1(q['near_ld'])}</td><td class='n'>{q['ld_clmax']:.1f}</td><td>{q['peak_kind']}</td><td class='n'>{q['plateau_alpha_span']:.1f}</td></tr>")
        P.append("</table></div><p>* aspect ratio 2: the onset angle is offset-dependent (not resolved).</p>")
    P.append("</section>")
    P.append("<section><h2>5. Offsets from each wing's own aerodynamic centre</h2><p>Percent of the mean aerodynamic chord, aft of the wing's fitted AC. <b>The form that could transfer to another wing applies only to the slope-based aerodynamic centre and to the first-lift-peak point</b> (CP at the first lift peak minus AC), because those depend much less on the zero-lift offset" + tag("DERIVED from measured") + ".</p>")
    P.append("<div class='scroll'><table class='t'><tr><th>wing</th><th>Re</th><th>AC % chord</th><th>CP at first peak minus AC (raw)</th><th>(zero-lift moment removed)</th><th>CP at 90% first-peak lift minus AC (raw)</th></tr>")
    for r in up_raw:
        q, qr = r["raw"]["res"], r["rem"]["res"]
        P.append(f"<tr><td>{r['wing']}</td><td class='n'>{r['Re']:,.0f}</td><td class='n'>{100 * q['ac']:.1f}</td><td class='n'>{q['xcp_clmax_offset']:+.1f}</td><td class='n'>{qr['xcp_clmax_offset']:+.1f}</td><td class='n'>{f1(q['near_xcp_offset'])}</td></tr>")
    P.append("</table></div></section>")
    # ---- trends
    P.append("<section><h2>6. Trends across aspect ratio, taper ratio and Reynolds number</h2><p>Plain tables and plots of the resolved quantities" + tag("DERIVED from measured") + ". No curve fits, no extrapolation outside the tested wings.</p>")
    low = {}
    for r in trends:
        k = (r["AR"], r["taper"])
        if k not in low or r["Re"] < low[k]["Re"]:
            low[k] = r
    P.append("<h3>At each wing's lowest tested Reynolds number</h3><div class='scroll'><table class='t'><tr><th>AR</th><th>taper</th><th>Re</th><th>AC % chord</th><th>CP at first peak minus AC</th><th>aft travel begins, deg</th><th>L/D at 90% of first-peak lift</th></tr>")
    for (ar, tp), r in sorted(low.items()):
        P.append(f"<tr><td class='n'>{ar}</td><td class='n'>{tp:g}</td><td class='n'>{r['Re']:,.0f}</td><td class='n'>{r['ac_up']:.1f}</td><td class='n'>{r['xcp_clmax_offset']:+.1f}</td><td class='n'>{f1(r['onset_alpha'])}</td><td class='n'>{f1(r['near_ld'])}</td></tr>")
    P.append("</table></div><h3>Reynolds-number dependence within each wing</h3><div class='scroll'><table class='t'><tr><th>wing</th><th>Re</th><th>AC % chord</th><th>CP at first peak minus AC</th><th>aft travel begins, deg</th><th>L/D at 90% of first-peak lift</th><th>L/D at first peak</th></tr>")
    for r in trends:
        P.append(f"<tr><td>{r['wing']}</td><td class='n'>{r['Re']:,.0f}</td><td class='n'>{r['ac_up']:.1f}</td><td class='n'>{r['xcp_clmax_offset']:+.1f}</td><td class='n'>{f1(r['onset_alpha'])}</td><td class='n'>{f1(r['near_ld'])}</td><td class='n'>{r['ld_clmax']:.1f}</td></tr>")
    P.append("</table></div><div class='plots'>")
    for key, ttl, yl, yl_lim in (("xcp_clmax_offset", "CP at first peak minus AC vs Re", "% MAC", (0, 25)), ("onset_alpha", "Aft travel begins vs Re (AR 2: offset-dependent)", "alpha, deg", (0, 14)), ("near_ld", "L/D at 90% of first-peak lift vs Re", "L/D", (0, 12))):
        ser = [(wg, [(r["Re"] / 1000, r[key]) for r in trends if r["wing"] == wg]) for wg in sorted({r["wing"] for r in trends})]
        for i in range(0, len(ser), 5):
            P.append(svg_plot(ttl, "Re, thousands", yl, ser[i:i + 5], (50, 170), yl_lim))
    for tp in (0.5, 0.75, 1.0):
        pts = [("AC (lowest Re)", [(ar, r["ac_up"]) for (ar, t), r in sorted(low.items()) if t == tp]), ("CP at first peak", [(ar, r["ac_up"] + r["xcp_clmax_offset"]) for (ar, t), r in sorted(low.items()) if t == tp])]
        P.append(svg_plot(f"Taper {tp:g}: AC and CP at first peak vs aspect ratio (lowest Re)", "aspect ratio", "% chord", pts, (1.5, 5.5), (15, 50), [(25, "quarter chord")]))
    for ar in (2, 3, 4):
        pts = [("AC (lowest Re)", [(t, r["ac_up"]) for (a, t), r in sorted(low.items()) if a == ar]), ("CP at first peak", [(t, r["ac_up"] + r["xcp_clmax_offset"]) for (a, t), r in sorted(low.items()) if a == ar])]
        P.append(svg_plot(f"AR {ar}: AC and CP at first peak vs taper ratio (lowest Re)", "taper ratio", "% chord", pts, (0.4, 1.1), (15, 50), [(25, "quarter chord")]))
    P.append("</div></section>")
    # ---- resolvability
    P.append("<section><h2>7. How well resolved is this? Up against down, raw against zero-lift-moment-removed</h2><p>For every run both legs exist. Each quantity is computed four ways (increasing or decreasing sweep; raw or zero-lift moment removed). The table gives the median and largest difference between up and down, and how far the quantity moves when the zero-lift moment is removed. "
             "Units: % MAC for offsets and AC, degrees for the onset angle, L/D for L/D" + tag("DERIVED from measured") + ".</p>")
    names = {"ac_pct": "aerodynamic centre, % chord", "xcp_clmax_offset": "CP at first peak minus AC", "onset_alpha": "aft travel begins, deg", "near_ld": "L/D at 90% of first-peak lift",
             "onset_alpha_AR2": "aft travel begins, deg (aspect ratio 2 only)", "best": "best-L/D balance point offset", "b90_lo": "90% band, forward edge", "b90_hi": "90% band, aft edge", "b75_lo": "75% band, forward edge", "b75_hi": "75% band, aft edge", "front": "most forward CP seen"}
    P.append("<div class='scroll'><table class='t'><tr><th>quantity</th><th>runs</th><th>|up &minus; down|, median (raw)</th><th>largest (raw)</th><th>|up &minus; down|, median (offset removed)</th><th>largest (offset removed)</th><th>moves when offset removed, median</th><th>largest move</th><th>resolved?</th></tr>")
    for qn in ("ac_pct", "xcp_clmax_offset", "onset_alpha", "onset_alpha_AR2", "near_ld", "best", "b90_lo", "b90_hi", "b75_lo", "b75_hi", "front"):
        s = stats.get(qn)
        if s:
            P.append(f"<tr><td>{names[qn]}</td><td class='n'>{s['n']}</td><td class='n'>{s['updown_raw_med']:.2f}</td><td class='n'>{s['updown_raw_max']:.2f}</td><td class='n'>{s['updown_rem_med']:.2f}</td><td class='n'>{s['updown_rem_max']:.2f}</td><td class='n'>{s['move_med']:.2f}</td><td class='n'>{s['move_max']:.2f}</td><td>{'yes' if s['resolved'] else '<b>NO</b>'}</td></tr>")
    nr_keys = ("best", "b90_lo", "b90_hi", "b75_lo", "b75_hi", "front")
    mv = max(stats[k]["move_max"] for k in nr_keys if k in stats)
    ud = max(stats[k]["updown_raw_max"] for k in nr_keys if k in stats)
    P.append(f"</table></div><p>For scale: the median width of the (raw) 90% band is {stats['_b90_width_med']:.1f}% MAC, while the balance-point quantities differ by up to {ud:.1f}% MAC between the up and down legs of the same run and move by up to {mv:.1f}% MAC when the zero-lift moment is removed. "
             "The quantities marked &quot;yes&quot; have small median differences, but single runs can differ more (largest columns), so treat even those with that spread in mind.</p></section>")
    P.append("<section><div class='nr'><h2>NOT RESOLVED BY THIS DATA: best-L/D balance point and balance-point bands</h2><p>Shown only for completeness and so the movement can be seen. <b>Do not use these numbers.</b> They sit inside the zero-lift-offset and up/down repeatability error. Offsets from the aerodynamic centre, % MAC (negative = ahead). Each cell: up raw / down raw / up offset-removed / down offset-removed.</p>"
             "<div class='scroll'><table class='t'><tr><th>wing</th><th>Re</th><th>best-L/D balance point</th><th>90% band, forward edge</th><th>90% band, aft edge</th><th>75% band, forward edge</th><th>75% band, aft edge</th></tr>")
    nrby = {}
    for n in notres:
        nrby.setdefault((n["wing"], n["run"]), {})[(n["sweep"], n["version"])] = n
    runs_sorted = sorted(by.items(), key=lambda kv: (kv[1]["increasing"]["AR"], kv[1]["increasing"]["taper"], kv[1]["increasing"]["Re"]))
    for (wing, run_), d in runs_sorted:
        g = nrby.get((wing, run_), {})
        cells = []
        for key in ("best", "b90_lo", "b90_hi", "b75_lo", "b75_hi"):
            vals = [g.get((sw, ver)) for sw, ver in (("increasing", "raw"), ("decreasing", "raw"), ("increasing", "offset removed"), ("decreasing", "offset removed"))]
            cells.append(" / ".join("-" if v is None or v[key] is None else f"{v[key]:+.1f}" for v in vals))
        P.append(f"<tr><td>{wing}</td><td class='n'>{d['increasing']['Re']:,.0f}</td>" + "".join(f"<td class='n'>{c}</td>" for c in cells) + "</tr>")
    P.append("</table></div></div></section>")
    # ---- carry over (reduced)
    P.append("<section><div class='warn'><h2>8. IF IT CARRIES OVER (a possibility, not a result)</h2>"
             "<p><b>This section is separate on purpose, and it is not a CG setting. It does not replace the CG sweep in the flat-test kit.</b> Two rows per specimen, in whole millimetres aft of the root leading edge, all " + tag("EXTRAPOLATED") + ". "
             "Row 1 takes the AVL neutral point with fins" + tag("AVL") + " and widens it by the measured bias between the aerodynamic centre and the quarter chord (the measured centre sits 0 to 5% MAC ahead of the quarter chord, further forward at lower Reynolds number) of the nearest tested wings at all their Reynolds numbers. "
             "Row 2 adds the measured offset of the CP at the first lift peak from the aerodynamic centre (raw and offset-removed, both legs). Nearest wings are chosen by aspect ratio and taper ratio; none of them was swept, finned or ribbed.</p>")
    for spec in SPECIMENS:
        rs = [c for c in carry if c["spec"] == spec]
        if not rs:
            continue
        k = rs[0]["kit"]
        P.append(f"<h3>{spec}: AR {k['ar']:g}, taper {k['taper']:g}, AVL neutral point with fins {k['np_mm']:.1f} mm{tag('AVL')}, MAC {k['mac_mm']:.1f} mm</h3><p>Nearest tested wings: {rs[0]['nearest']} ({rs[0]['n_runs']} legs).</p>")
        P.append("<div class='scroll'><table class='t'><tr><th>what</th><th>mm aft of root LE</th><th>offset range, % MAC</th></tr>")
        for c in rs:
            P.append(f"<tr><td>{c['quantity']}</td><td class='n'>{c['mm_lo']} to {c['mm_hi']}{tag('EXTRAPOLATED')}</td><td class='n'>{c['off_lo']:+.1f} to {c['off_hi']:+.1f}{tag('DERIVED from measured')}</td></tr>")
        P.append("</table></div>")
        P.append(f"<p>Measured bias trend with Reynolds number: {h(rs[0]['trend'])}.</p>")
        P.append(f"<p><b>Not tested for {spec}:</b> " + "; ".join(h(x) for x in rs[0]["not_tested"].split("; ")) + f". (Thickness: our {k['t_mean_pct']:.2f}% of mean chord, {k['t_tip_pct']:.2f}% of tip chord, against {TESTED_T_OVER_C_PCT}% tested.)</p>")
    P.append("<p>Reading it: the first row says where the low-angle aerodynamic centre of a wing like ours might lie if the tested plates' offset from the neutral point carried over; the second where the aft travel of the CP, and with it near-stall trim, might end. Whether swept, finned, ribbed, much thinner and lower-Reynolds-number wings behave like that is not known.</p></div></section>")
    P.append("<section><h2>9. Method notes and limits</h2><ul>"
             f"<li>Fits: alpha {FIT_RANGE[0]:g} to {FIT_RANGE[1]:g} deg; the positive-lift branch starts at CN 0.05; the first lift peak is found by the slope rule in section 4 (fallback: global maximum up to {CLMAX_ALPHA_LIMIT:g} deg). All parameters are at the top of plate_balance.py.</li>"
             "<li>Angles are wall-corrected and do not land on round numbers; interpolation is linear between neighbouring tabulated points only.</li>"
             "<li>Reference chord for CM is assumed to be the mean aerodynamic chord (section 1).</li>"
             "<li>Moment accuracy: the thesis quotes about 4.6% of reading for the moment (not computed for this plate). <b>The fitted zero-lift moment (section 2) is a separate, larger problem for the centre of pressure at low lift:</b> it is not a percentage of reading, and it moves x_cp by CM0/CN.</li>"
             "<li>The balance-point and band quantities are kept only in the &quot;not resolved&quot; table and step3_NOT_RESOLVED.csv.</li></ul></section>")
    P.append("</main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


# =============================================================================================== main
def run(out: Path = OUT_DEFAULT, data: Path = DATA, kit: Path = KIT):
    out.mkdir(parents=True, exist_ok=True)
    rows = read_rows(data)
    runs = group_runs(rows)
    results = analyse(runs)
    summ = summary_rows(results)
    notres = not_resolved_rows(results)
    resrows, stats = resolvability(results)
    write_all_csvs(out, results, summ, notres, resrows)
    trends = trend_rows(results)
    write_trends(out, trends)
    carry = carry_over(results, trends, kit)
    write_carry_over(out, carry)
    page(results, summ, notres, resrows, stats, trends, carry, len(rows), len(runs), out)
    return dict(rows=rows, runs=runs, results=results, summary=summ, notres=notres, resrows=resrows, stats=stats, trends=trends, carry=carry)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    a = ap.parse_args()
    R = run(a.out)
    print(f"{len(R['runs'])} runs, {len(R['rows'])} rows -> {a.out}")


if __name__ == "__main__":
    main()
