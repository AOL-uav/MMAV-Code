"""Pre-experiment model predictions for larger trailing-edge-up lips.

Writes only under results/runs/rigid_glider/reflex_ext/predict/. Reuses (never
edits) reflex_xfoil, reflex_ext_avl, reflex_geometry. Existing polars are READ.

Run from rigid_glider/ (or repo root):
  py rigid_glider/reflex_ext_predict.py geom repro xfoil tables avl
Stages: geom   geometry validity check (writes geometry_check.*)
        repro  regenerate bent 0008 10%/3 at 80k into predict/xfoil_repro, diff vs stored
        xfoil  new viscous + inviscid polars (skips geometries that fail geom)
        tables prediction / slope / companion-alpha tables from the polars
        avl    inviscid AVL cases, repro row, lip-angle interpolation
NACA 0004 is a thin symmetric PROXY, not a flat plate. All AVL numbers and
inviscid XFOIL numbers are inviscid reference values.
"""

from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import re
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path

import reflex_xfoil as rx
from reflex_geometry import Reflex
import reflex_ext_avl as ea

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/reflex_ext/predict"
XOUT = OUT / "xfoil"
REPRO_OUT = OUT / "xfoil_repro"
AOUT = OUT / "avl"
OLD_ROOTS = [ROOT / "results/runs/rigid_glider/reflex/xfoil",
             ROOT / "results/runs/rigid_glider/reflex_ext/xfoil"]
CONFIG = Path(__file__).with_name("reflex_ext_predict_config.json")
REQUESTED_ALPHAS = [round(-3 + 0.5 * i, 1) for i in range(23)]


def cfg() -> dict:
    return json.loads(CONFIG.read_text())


def write_csv(path: Path, rows: list[dict], keys: list[str] | None = None):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    if keys is None:
        keys = []
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
    with path.open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


# ----------------------------------------------------------------- geometry
def _interp(xs: list[float], ys: list[float], x: float) -> float:
    """Linear interpolation on a strictly increasing xs."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    t = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + t * (ys[hi] - ys[lo])


def geometry_check(thickness: float, chord_fraction: float, angle_deg: float,
                   n_half: int = 160) -> dict:
    """Validity check of reflex_xfoil.coordinates() output (closed-TE bent NACA 00xx)."""
    pts = rx.coordinates(thickness, chord_fraction, angle_deg, n_half)
    up = pts[:n_half + 1]                    # TE -> LE
    lo = [up[-1]] + pts[n_half + 1:]         # LE -> TE
    xh = 1 - chord_fraction
    res = {"thickness": thickness, "chord_fraction": chord_fraction, "angle_deg": angle_deg,
           "n_points": len(pts)}
    dxu = [up[i][0] - up[i + 1][0] for i in range(len(up) - 1)]   # must be > 0
    dxl = [lo[i + 1][0] - lo[i][0] for i in range(len(lo) - 1)]   # must be > 0
    res["upper_x_monotonic"] = all(d > 0 for d in dxu)
    res["lower_x_monotonic"] = all(d > 0 for d in dxl)
    res["min_dx_upper"] = min(dxu)
    res["min_dx_lower"] = min(dxl)
    res["max_x"] = max(p[0] for p in pts)
    # node pair straddling the hinge on the upper surface (original index by cosine spacing)
    straddle = [i for i in range(len(dxu)) if up[i][0] > xh - 0.02 and up[i + 1][0] < xh + 0.02]
    res["min_dx_upper_near_hinge"] = min((dxu[i] for i in straddle), default=None)
    ok_mono = res["upper_x_monotonic"] and res["lower_x_monotonic"]
    if ok_mono:
        ux, uy = [p[0] for p in reversed(up)], [p[1] for p in reversed(up)]  # LE -> TE
        lx, ly = [p[0] for p in lo], [p[1] for p in lo]
        gaps = []
        for x, y in up:
            gaps.append((y - _interp(lx, ly, x), x))
        for x, y in lo:
            gaps.append((_interp(ux, uy, x) - y, x))
        res["min_gap_overall"] = min(g for g, _ in gaps)
        near = [g for g, x in gaps if xh - 0.02 <= x <= xh + 0.03]   # window around the hinge (TE node excluded by window)
        res["min_gap_near_hinge_region"] = min(near) if near else None
        res["surfaces_cross"] = res["min_gap_overall"] < -1e-9
    else:
        res["min_gap_overall"] = None
        res["min_gap_near_hinge_region"] = None
        res["surfaces_cross"] = None
    a = math.radians(angle_deg)
    exp_y = chord_fraction * math.sin(a)
    exp_x = xh + chord_fraction * math.cos(a)
    res["te_y_upper"], res["te_y_lower"] = up[0][1], lo[-1][1]
    res["te_y_expected"] = exp_y
    res["te_y_abs_err"] = max(abs(up[0][1] - exp_y), abs(lo[-1][1] - exp_y))
    res["te_x"], res["te_x_expected"] = up[0][0], exp_x
    res["te_x_abs_err"] = max(abs(up[0][0] - exp_x), abs(lo[-1][0] - exp_x))
    res["te_closed"] = math.hypot(up[0][0] - lo[-1][0], up[0][1] - lo[-1][1]) < 1e-9
    res["valid"] = bool(ok_mono and res["surfaces_cross"] is False and res["te_y_abs_err"] < 1e-9
                        and res["te_x_abs_err"] < 1e-9 and res["te_closed"] and res["max_x"] <= 1 + 1e-12)
    return res


def all_geometries(c: dict) -> list[tuple[str, float, float]]:
    lips = [tuple(c["existing_lip"])] + [tuple(l) for l in c["new_lips"]]
    return [(s, cf, float(a)) for s in c["sections"] for cf, a in lips]


def stage_geom(c: dict) -> dict:
    rows = []
    for sec, cf, ang in all_geometries(c):
        r = geometry_check(int(sec[-2:]) / 100, cf, ang)
        r["section"] = sec
        rows.append(r)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "geometry_check.json").write_text(json.dumps(rows, indent=1))
    write_csv(OUT / "geometry_check.csv", rows)
    bad = [r for r in rows if not r["valid"]]
    print(f"geometry: {len(rows)} checked, {len(bad)} invalid")
    for r in rows:
        print(f"  {r['section']} {r['chord_fraction']:.2f}/{r['angle_deg']:g} valid={r['valid']} "
              f"minDx_up={r['min_dx_upper']:.2e} minDx_up_hinge={r['min_dx_upper_near_hinge']} "
              f"mingap={r['min_gap_overall']} TEy_err={r['te_y_abs_err']:.1e} TEx_err={r['te_x_abs_err']:.1e}")
    return {(r["section"], r["chord_fraction"], r["angle_deg"]): r["valid"] for r in rows}


def valid_map() -> dict:
    p = OUT / "geometry_check.json"
    rows = json.loads(p.read_text())
    return {(r["section"], r["chord_fraction"], r["angle_deg"]): r["valid"] for r in rows}


# -------------------------------------------------------------------- logs
def log_counts(log: str) -> dict:
    return {"viscal_convergence_failures": len(re.findall(r"VISCAL:\s+Convergence failed", log)),
            "sequence_halted": len(re.findall(r"Sequence halted", log, re.I)),
            "mrchue_convergence_failed_msgs": len(re.findall(r"MRCHUE:\s+Convergence failed", log))}


def label_of(section, reynolds, cf, ang, inviscid=False) -> str:
    return (f"{section}_{'inviscid' if inviscid else 'Re' + str(reynolds)}_"
            f"c{int(round(cf * 100)):02d}_a{ang:g}").replace(".", "p")


def find_polar(section, reynolds, cf, ang, inviscid=False):
    """Return (rows, source_dir) searching predict first, then existing results (read only)."""
    lab = label_of(section, reynolds, cf, ang, inviscid)
    for root in [XOUT] + OLD_ROOTS:
        p = root / lab / "polar.dat"
        if p.is_file():
            return rx.read_polar(p), root / lab
    return [], None


# ----------------------------------------------------------------- repro
def stage_repro(c: dict) -> bool:
    rx.OUT = REPRO_OUT
    new = rx.run("NACA_0008", 80000, 0.10, 3.0)
    stored = rx.read_polar(OLD_ROOTS[0] / "NACA_0008_Re80000_c10_a3" / "polar.dat")
    n = {round(r["alpha_deg"], 3): r for r in new["rows"]}
    s = {round(r["alpha_deg"], 3): r for r in stored}
    common = sorted(set(n) & set(s))
    diffs = [abs(n[a][k] - s[a][k]) for a in common for k in ("cl", "cd", "cdp", "cm", "top_xtr", "bot_xtr")]
    res = {"new_count": len(n), "stored_count": len(s), "common_alphas": len(common),
           "only_new": sorted(set(n) - set(s)), "only_stored": sorted(set(s) - set(n)),
           "max_abs_diff": max(diffs) if diffs else None,
           "identical": bool(diffs) and max(diffs) == 0 and set(n) == set(s)}
    (OUT / "xfoil_repro_check.json").write_text(json.dumps(res, indent=1))
    print("XFOIL repro:", res)
    return res["identical"]


# ------------------------------------------------------------------ xfoil
def polar_diag(rows: list[dict], inv_rows: list[dict] | None) -> dict:
    d = {"n": len(rows)}
    if len(rows) < 2:
        return d
    steps = [(b["cl"] - a["cl"], b["cm"] - a["cm"], b["alpha_deg"] - a["alpha_deg"], a, b)
             for a, b in zip(rows, rows[1:]) if b["alpha_deg"] - a["alpha_deg"] <= 0.51]
    d["alpha_min"], d["alpha_max"] = rows[0]["alpha_deg"], rows[-1]["alpha_deg"]
    d["cl_min"], d["cl_max"] = min(r["cl"] for r in rows), max(r["cl"] for r in rows)
    d["cl_nonmonotonic_steps"] = sum(1 for dc, _, da, _, _ in steps if dc < 0)
    d["max_abs_dcl_step"] = max((abs(dc) for dc, *_ in steps), default=None)
    d["max_abs_dcm_step"] = max((abs(dm) for _, dm, *_ in steps), default=None)
    d["max_cd"] = max(r["cd"] for r in rows)
    # erratic-step screen in the working window alpha 0..6: local lift slope outside 0.05..0.16 per deg
    err = [(a["alpha_deg"], dc / da) for dc, _, da, a, b in steps
           if a["alpha_deg"] >= 0 and b["alpha_deg"] <= 6 and not 0.05 <= dc / da <= 0.16]
    d["erratic_steps_alpha0_6"] = len(err)
    d["erratic_steps_alpha0_6_list"] = "; ".join(f"{al:g}:{sl:.3f}" for al, sl in err)
    byA = {round(r["alpha_deg"], 3): r for r in rows}
    if 0.0 in byA and 4.0 in byA:
        d["cl_slope_per_deg_0_4"] = (byA[4.0]["cl"] - byA[0.0]["cl"]) / 4
    if inv_rows:
        bi = {round(r["alpha_deg"], 3): r for r in inv_rows}
        if 0.0 in bi and 4.0 in bi:
            d["inviscid_cl_slope_per_deg_0_4"] = (bi[4.0]["cl"] - bi[0.0]["cl"]) / 4
    return d


def stage_xfoil(c: dict):
    rx.OUT = XOUT
    valid = valid_map()
    lips_new = [tuple(l) for l in c["new_lips"]]
    ex = tuple(c["existing_lip"])
    jobs = []        # (section, reynolds, cf, ang, inviscid)
    skipped = []
    for sec in c["sections"]:
        for cf, a in lips_new:
            ok = valid.get((sec, cf, float(a)), False)
            if not ok:
                skipped.append({"section": sec, "chord_fraction": cf, "angle_deg": a,
                                "reason": "geometry check failed or missing"})
                continue
            jobs.append((sec, 80000, cf, float(a), True))
            for re_ in c["reynolds"]:
                jobs.append((sec, re_, cf, float(a), False))
        # inviscid bent 10/3 and clean needed for the viscous/inviscid ratio, if no stored polar
        for cf, a in ((ex[0], float(ex[1])), (0.10, 0.0)):
            if not find_polar(sec, 80000, cf, a, True)[0] or find_polar(sec, 80000, cf, a, True)[1].parent == XOUT:
                jobs.append((sec, 80000, cf, a, True))
    jobs = list(dict.fromkeys(jobs))

    def one(j):
        return rx.run(j[0], j[1], j[2], j[3], inviscid=j[4])

    with ThreadPoolExecutor(max_workers=c["xfoil_workers"]) as pool:
        results = list(pool.map(one, jobs))
    counts = []
    for r in results:
        log = (Path(r["folder"]) / "xfoil.log").read_text(errors="replace")
        got = {round(x["alpha_deg"], 1) for x in r["rows"]}
        counts.append({"section": r["section"], "inviscid": r["inviscid"], "reynolds": r["reynolds"],
                       "chord_fraction": r["chord_fraction"], "angle_deg": r["angle_deg"],
                       "requested": 23, "returned": r["converged_count"], "exit_code": r["exit_code"],
                       "missing_alphas": " ".join(f"{a:g}" for a in REQUESTED_ALPHAS if a not in got),
                       **log_counts(log), "folder": r["folder"]})
    write_csv(OUT / "xfoil_counts.csv", counts)
    (OUT / "xfoil_skipped_invalid_geometry.json").write_text(json.dumps(skipped, indent=1))
    (OUT / "xfoil_run_summary.json").write_text(json.dumps(
        [{k: v for k, v in r.items() if k != "rows"} for r in results], indent=1))
    print(f"xfoil: {len(results)} runs, skipped {len(skipped)}")
    viscous = [x for x in counts if not x["inviscid"]]
    print(f"  viscous {len(viscous)} polars: returned {sum(x['returned'] for x in viscous)} of "
          f"{sum(x['requested'] for x in viscous)}; viscal fails {sum(x['viscal_convergence_failures'] for x in viscous)}; "
          f"sequence halted {sum(x['sequence_halted'] for x in viscous)}")
    inv = [x for x in counts if x["inviscid"]]
    print(f"  inviscid {len(inv)} polars: returned {sum(x['returned'] for x in inv)} of {sum(x['requested'] for x in inv)}")


# ------------------------------------------------------------- matched Cl
def brackets(rows: list[dict], target: float) -> list[tuple[float, float]]:
    out = {}
    for a, b in zip(rows, rows[1:]):
        if b["alpha_deg"] - a["alpha_deg"] > 0.51:
            continue
        if min(a["cl"], b["cl"]) <= target <= max(a["cl"], b["cl"]) and abs(b["cl"] - a["cl"]) > 1e-6:
            f = (target - a["cl"]) / (b["cl"] - a["cl"])
            al = a["alpha_deg"] + f * (b["alpha_deg"] - a["alpha_deg"])
            out.setdefault(round(al, 5), (al, a["cm"] + f * (b["cm"] - a["cm"])))
    return list(out.values())


def matched(clean: list[dict], bent: list[dict], target: float) -> dict | None:
    sc, sb = rx.at_cl(clean, target), rx.at_cl(bent, target)
    if sc is None or sb is None:
        return None
    bc, bb = brackets(clean, target), brackets(bent, target)
    ds = [b[1] - c_[1] for c_ in bc for b in bb]
    return {"clean_cm": sc["cm"], "bent_cm": sb["cm"], "dcm": sb["cm"] - sc["cm"],
            "clean_cd": sc["cd"], "bent_cd": sb["cd"], "dcd": sb["cd"] - sc["cd"],
            "clean_alpha": sc["alpha_deg"], "bent_alpha": sb["alpha_deg"],
            "n_clean_brk": len(bc), "n_bent_brk": len(bb), "dcm_min": min(ds), "dcm_max": max(ds)}


def lip_key(cf, a) -> str:
    return f"{int(round(cf * 100))}/{a:g}"


def stage_tables(c: dict):
    lips = [tuple(c["existing_lip"])] + [tuple(l) for l in c["new_lips"]]
    long_rows, slope_rows, comp_rows, diag_rows, missing = [], [], [], [], []
    for sec in c["sections"]:
        inv_clean, inv_clean_src = find_polar(sec, 80000, 0.10, 0.0, True)
        for cf, a in lips:
            a = float(a)
            inv_bent, inv_src = find_polar(sec, 80000, cf, a, True)
            inv_a0 = None
            ia_c = {round(r["alpha_deg"], 3): r for r in inv_clean}.get(0.0)
            ia_b = {round(r["alpha_deg"], 3): r for r in inv_bent}.get(0.0)
            if ia_c and ia_b:
                inv_a0 = ia_b["cm"] - ia_c["cm"]
            for re_ in c["reynolds"]:
                clean, cdir = find_polar(sec, re_, 0.10, 0.0)
                bent, bdir = find_polar(sec, re_, cf, a)
                diag_rows.append({"section": sec, "Re": re_, "lip": lip_key(cf, a), "n_clean": len(clean),
                                  "n_bent": len(bent), **{f"bent_{k}": v for k, v in polar_diag(bent, inv_bent).items()}})
                # matched-Cl rows
                for t in c["target_cl"]:
                    row = {"section": sec, "proxy_note": "thin proxy, not flat plate" if sec == "NACA_0004" else "",
                           "lip": lip_key(cf, a), "Re": re_, "Cl": t,
                           "inviscid_dCm_alpha0": inv_a0}
                    m = matched(clean, bent, t)
                    ic, ib = rx.at_cl(inv_clean, t), rx.at_cl(inv_bent, t)
                    inv_m = ib["cm"] - ic["cm"] if (ic and ib) else None
                    row["inviscid_dCm_matched_cl"] = inv_m
                    row["inviscid_clean_cm_matched"] = ic["cm"] if ic else None
                    row["inviscid_bent_cm_matched"] = ib["cm"] if ib else None
                    row["inviscid_alpha_clean"] = ic["alpha_deg"] if ic else None
                    row["inviscid_alpha_bent"] = ib["alpha_deg"] if ib else None
                    if m is None:
                        row["note"] = (f"MISSING: Cl target not bracketed by adjacent returned points "
                                       f"(n_clean={len(clean)}, n_bent={len(bent)}, "
                                       f"clean_brk={len(brackets(clean, t))}, bent_brk={len(brackets(bent, t))})")
                        missing.append(row)
                    else:
                        row.update({"clean_cm": m["clean_cm"], "bent_cm": m["bent_cm"], "viscous_dCm": m["dcm"],
                                    "ratio_to_inviscid_alpha0": m["dcm"] / inv_a0 if inv_a0 else None,
                                    "ratio_to_inviscid_matched_cl": m["dcm"] / inv_m if inv_m else None,
                                    "clean_cd": m["clean_cd"], "bent_cd": m["bent_cd"], "dCd": m["dcd"],
                                    "alpha_clean": m["clean_alpha"], "alpha_bent": m["bent_alpha"],
                                    "n_clean_brk": m["n_clean_brk"], "n_bent_brk": m["n_bent_brk"],
                                    "dCm_min": m["dcm_min"], "dCm_max": m["dcm_max"],
                                    "note": "MULTI-BRACKET (first bracket used)" if m["n_clean_brk"] > 1 or m["n_bent_brk"] > 1 else ""})
                        if inv_a0 is None:
                            row["note"] += "; inviscid alpha0 missing"
                    long_rows.append(row)
                # slopes
                lo_, hi_ = c["slope_cl"]
                srow = {"section": sec, "lip": lip_key(cf, a), "Re": re_, "cl_lo": lo_, "cl_hi": hi_}
                for name, pol in (("clean", clean), ("bent", bent)):
                    p0, p1 = rx.at_cl(pol, lo_), rx.at_cl(pol, hi_)
                    srow[f"{name}_cm_at_lo"] = p0["cm"] if p0 else None
                    srow[f"{name}_cm_at_hi"] = p1["cm"] if p1 else None
                    srow[f"{name}_dCm_dCl"] = (p1["cm"] - p0["cm"]) / (hi_ - lo_) if p0 and p1 else None
                srow["note"] = "" if srow["clean_dCm_dCl"] is not None and srow["bent_dCm_dCl"] is not None \
                    else "MISSING: an endpoint Cl not bracketed; not filled"
                slope_rows.append(srow)
                # companion alpha table (0008 / 0006 only)
                if sec in c["companion_sections"]:
                    cb = {round(r["alpha_deg"], 3): r for r in clean}
                    bb = {round(r["alpha_deg"], 3): r for r in bent}
                    for al in c["companion_alphas_deg"]:
                        x, y = cb.get(float(al)), bb.get(float(al))
                        comp_rows.append({
                            "section": sec, "lip": lip_key(cf, a), "Re": re_, "alpha_deg": al,
                            "clean_CL": x and x["cl"], "clean_Cm": x and x["cm"], "clean_Cd": x and x["cd"],
                            "bent_CL": y and y["cl"], "bent_Cm": y and y["cm"], "bent_Cd": y and y["cd"],
                            "dCL": x and y and y["cl"] - x["cl"], "dCm": x and y and y["cm"] - x["cm"],
                            "note": "" if x and y else "MISSING point; not filled"})
    # clean diagnostics too
    for sec in c["sections"]:
        for re_ in c["reynolds"]:
            clean, _ = find_polar(sec, re_, 0.10, 0.0)
            inv_clean, _ = find_polar(sec, 80000, 0.10, 0.0, True)
            diag_rows.append({"section": sec, "Re": re_, "lip": "clean", "n_clean": len(clean),
                              **{f"bent_{k}": v for k, v in polar_diag(clean, inv_clean).items()}})
    write_csv(OUT / "prediction_table.csv", long_rows)
    write_csv(OUT / "prediction_slopes.csv", slope_rows)
    write_csv(OUT / "companion_alpha_table.csv", comp_rows)
    write_csv(OUT / "xfoil_polar_diagnostics.csv", diag_rows)
    write_csv(OUT / "prediction_missing_cells.csv", missing)
    (OUT / "prediction_table.json").write_text(json.dumps(long_rows, indent=1))
    print(f"tables: {len(long_rows)} prediction rows, {len(missing)} missing, {len(slope_rows)} slope rows, "
          f"{len(comp_rows)} companion rows")
    # compact stdout
    def f(v, n=4):
        return "  --  " if v is None else f"{v:+.{n}f}"
    for sec in ("NACA_0008", "NACA_0006", "NACA_0004"):
        print(f"\n== {sec} matched Cl: clean Cm | bent Cm | viscous dCm | inv dCm a0 | inv dCm @Cl | ratio a0 | dCd | alpha clean/bent ==")
        for r in long_rows:
            if r["section"] != sec:
                continue
            if "viscous_dCm" not in r:
                print(f"{r['lip']:>5} Re{r['Re']//1000:>3}k Cl{r['Cl']:.2f}  MISSING")
                continue
            print(f"{r['lip']:>5} Re{r['Re']//1000:>3}k Cl{r['Cl']:.2f} {f(r['clean_cm'])} {f(r['bent_cm'])} "
                  f"{f(r['viscous_dCm'])} {f(r['inviscid_dCm_alpha0'])} {f(r['inviscid_dCm_matched_cl'])} "
                  f"{f(r['ratio_to_inviscid_alpha0'],2)} {f(r['dCd'],5)} {r['alpha_clean']:+.2f}/{r['alpha_bent']:+.2f}"
                  f"{' ' + r['note'] if r['note'] else ''}")
        for r in slope_rows:
            if r["section"] == sec:
                print(f"slope {r['lip']:>5} Re{r['Re']//1000:>3}k clean {f(r['clean_dCm_dCl'])} bent {f(r['bent_dCm_dCl'])} {r['note']}")


# -------------------------------------------------------------------- AVL
def interp_angle(points: list[tuple[float, float]], target: float) -> dict:
    """points: (angle, trim_cl). Linear interpolation inside the first bracketing adjacent pair."""
    pts = sorted(points)
    for (a0, c0), (a1, c1) in zip(pts, pts[1:]):
        if min(c0, c1) <= target <= max(c0, c1) and c1 != c0:
            ang = a0 + (target - c0) * (a1 - a0) / (c1 - c0)
            n = len(pts)
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            mx, my = sum(xs) / n, sum(ys) / n
            sxx = sum((x - mx) ** 2 for x in xs)
            slope = sum((x - mx) * (y - my) for x, y in pts) / sxx if sxx else 0.0
            icpt = my - slope * mx
            maxres = max(abs(y - (icpt + slope * x)) for x, y in pts)
            return {"angle_deg": ang, "bracket": [a0, a1], "bracket_trim_cl": [c0, c1],
                    "bracketed": True, "linear_fit_max_resid_trim_cl": maxres, "n_points": n}
    return {"angle_deg": None, "bracketed": False, "n_points": len(pts),
            "note": "target trim CL not bracketed by the tested angles; not extrapolated"}


def stage_avl(c: dict):
    base, ext, wing, flight = ea.load_configs()
    a = c["avl"]
    widen = a["widened_probe_alpha_deg"]
    wing = replace(wing, quarter_chord_sweep_deg=a["sweep_deg"])
    xnp = ea.neutral_point(wing, flight, AOUT)
    w = ea.wing_at_margin(wing, xnp, a["margin"])
    tasks = [("rp_full_c10_a3", Reflex(.10, 3, "full", a["split"]), "repro")]
    seen = set()

    def add(pl, cf, ang, kind):
        nm = f"{pl[0]}{int(round(cf * 100))}a{ang:g}"
        if nm in seen:
            return
        seen.add(nm)
        tasks.append((nm, Reflex(cf, float(ang), pl, a["split"]), kind))

    for pl in a["placements"]:
        for cf, ang in a["requested_lips"]:
            add(pl, cf, ang, "requested")
    for cfs, angs in a["interp_extra_angles"].items():
        for ang in angs:
            add("full", float(cfs), ang, "interp_extra")
    with ThreadPoolExecutor(max_workers=a["workers"]) as pool:
        rows = list(pool.map(lambda t: ea.case(t[0], w, t[1], flight, AOUT, widen_probe=widen), tasks))
    for r, t in zip(rows, tasks):
        r["kind"] = t[2]
        r["trim_alpha_gt_8"] = bool(r.get("trim_alpha_deg") is not None and r["trim_alpha_deg"] > 8)
        r["trim_cl_gt_0p5"] = bool(r.get("trim_cl") is not None and r["trim_cl"] > 0.5)
    keys = ["name", "kind", "placement", "chord_fraction", "angle_deg", "trim_cl", "trim_alpha_deg",
            "static_margin_mac", "alpha_range_extended", "cm0", "cl0", "trim_alpha_gt_8", "trim_cl_gt_0p5",
            "speed_m_s", "re_root", "re_tip", "cma_per_rad", "error"]
    write_csv(AOUT.parent / "avl_predict.csv", rows, keys)
    (AOUT.parent / "avl_predict.json").write_text(json.dumps(rows, indent=1))
    # repro
    rp = rows[0]
    exp = a["repro_expected"]
    repro = {"trim_cl": rp.get("trim_cl"), "expected_trim_cl": exp["trim_cl"],
             "static_margin_mac": rp.get("static_margin_mac"), "expected_static_margin_mac": exp["static_margin_mac"]}
    # also exact comparison against stored static.csv row
    stored = ROOT / "results/runs/rigid_glider/reflex/static.csv"
    if stored.is_file():
        for line in csv.DictReader(stored.open()):
            if line["name"] == "s25_full_c10_a3":
                repro["stored_trim_cl"] = float(line["trim_cl"])
                repro["stored_static_margin_mac"] = float(line["static_margin_mac"])
                repro["abs_diff_trim_cl"] = abs(rp["trim_cl"] - float(line["trim_cl"]))
                repro["abs_diff_static_margin"] = abs(rp["static_margin_mac"] - float(line["static_margin_mac"]))
    repro["ok"] = (rp.get("error") is None and abs(rp["trim_cl"] - exp["trim_cl"]) <= 6e-6
                   and abs(rp["static_margin_mac"] - exp["static_margin_mac"]) <= 6e-5)
    (AOUT.parent / "avl_repro.json").write_text(json.dumps(repro, indent=1))
    print("AVL repro:", repro)
    # interpolation
    interp = {}
    for cf in (0.15, 0.20):
        pts = [(r["angle_deg"], r["trim_cl"]) for r in rows[1:]
               if r["placement"] == "full" and abs(r["chord_fraction"] - cf) < 1e-9
               and r.get("trim_cl") is not None]
        interp[f"{int(round(cf * 100))}%"] = {**interp_angle(pts, a["interp_target_trim_cl"]),
                                              "points": sorted(pts),
                                              "label": "INVISCID AVL figure; linear interpolation in lip angle"}
    (AOUT.parent / "avl_lip_angle_for_trim_cl.json").write_text(json.dumps(interp, indent=1))
    print("interpolation:", json.dumps(interp, indent=1))
    print(f"AVL: {len(rows)} cases, errors={sum(bool(r['error']) for r in rows)}")
    for r in rows:
        print(f"  {r['name']:>9} {r['placement']:>5} {r['chord_fraction']:.2f}/{r['angle_deg']:g} "
              f"trimCL={r.get('trim_cl')} alpha={r.get('trim_alpha_deg')} SM={r.get('static_margin_mac')} "
              f"widened={r['alpha_range_extended']} Cm0={r.get('cm0')} a>8={r['trim_alpha_gt_8']} CL>0.5={r['trim_cl_gt_0p5']} err={r['error']}")


STAGES = {"geom": stage_geom, "repro": stage_repro, "xfoil": stage_xfoil,
          "tables": stage_tables, "avl": stage_avl}

if __name__ == "__main__":
    c = cfg()
    for st in sys.argv[1:] or list(STAGES):
        print(f"--- {st}")
        STAGES[st](c)
