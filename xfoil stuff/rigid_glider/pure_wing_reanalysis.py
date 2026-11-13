"""Read-only reanalysis of the pure-wing study outputs (study_roots.csv, study_scans.json, polar_database.csv).

Writes ONLY to results/runs/rigid_glider/pure_wing/reanalysis/.  Model functions are imported and called
read-only (SectionTable lookups, level1) to reconstruct strip cl/Re at each stored root from avl_shapes.json.
Strip cl = CL * f_i, Re_i = V c_i / nu (the model's own definitions).  Transition-interval test uses the exact
(cl, Re) rather than the 0.005-cl node interpolation the model uses internally (small approximation).
py -B rigid_glider/pure_wing_reanalysis.py
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import pure_wing_model as m
from pure_wing_polars import read_database

SRC = m.OUT
OUT = SRC / "reanalysis"
CFG = json.loads((Path(__file__).with_name("pure_wing_config.json")).read_text())
G = CFG["flight"]["g_m_s2"]
BAND = (0.15, 0.6)
SCAN_STEP = 0.005
DL = 0.02


def read_roots():
    out = []
    with (SRC / "study_roots.csv").open(encoding="utf-8") as s:
        for r in csv.DictReader(s):
            for k, v in r.items():
                if k in ("shape", "section"):
                    continue
                try:
                    r[k] = float(v)
                except (TypeError, ValueError):
                    r[k] = None
            out.append(r)
    return out


def good_rows(rows):
    return [r for r in rows if r["stable_glide"] == 1 and r["stable_fixed"] == 1 and r["in_band"] == 1
            and r["coverage"] >= 0.9]


# ---------------------------------------------------------------- transition interval
def segment(table, re_stored, cl):
    """Attached-branch segment (a, b) used at stored Re: same selection rule as SectionTable._branch."""
    hint = table.alpha_inv(cl)
    best = None
    rows = table.rows[re_stored]
    for a, b in zip(rows, rows[1:]):
        if b["alpha"] - a["alpha"] > table.max_gap + 1e-9:
            continue
        lo, hi = min(a["cl"], b["cl"]), max(a["cl"], b["cl"])
        if not lo <= cl <= hi or abs(b["cl"] - a["cl"]) < 1e-9:
            continue
        t = (cl - a["cl"]) / (b["cl"] - a["cl"])
        al = a["alpha"] + t * (b["alpha"] - a["alpha"])
        if best is None or abs(al - hint) < abs(best[0] - hint):
            best = (al, a, b)
    return None if best is None else best[1:]


def on_transition(a, b):
    return (a["xtr_top"] >= 0.999) != (b["xtr_top"] >= 0.999)


def ti_fraction(shape, b, table, CL, V):
    wsum = tsum = 0.0
    for s in shape["strips"]:
        c, A = s["c"] * b, s["a"] * b * b
        re = V * c / m.NU
        cl = abs(CL * s["f"])
        if table.lookup(re, cl) is None:
            continue
        lo = max(r for r in table.res if r <= re)
        hi = min(r for r in table.res if r >= re)
        hit = False
        for rs in {lo, hi}:
            seg = segment(table, rs, cl)
            if seg is not None and on_transition(*seg):
                hit = True
        wsum += A
        tsum += A * hit
    return tsum / wsum if wsum else None


def section_slope_low(shape, b, table, CL, V):
    """Area-weighted section dcm/dcl (fixed Re) with every strip cl scaled to CL (the trim-minus-offset point)."""
    w = d = 0.0
    for s in shape["strips"]:
        c, A = s["c"] * b, s["a"] * b * b
        re = V * c / m.NU
        cl = CL * s["f"]
        rp, rm = table.lookup(re, cl + DL), table.lookup(re, cl - DL)
        if rp is None or rm is None:
            return None
        w += A
        d += A * (rp["cm"] - rm["cm"]) / (2 * DL)
    return d / w


def cm_extra(shape, b, table, CL, ws=None, V=None):
    r = m.level1(shape, b, table, CL, ws=ws, V=V)
    return r["cm_visc"], r["coverage"]


def low_side(shape, b, table, ws, CL0, V0, sm, off=0.05):
    CL = CL0 - off
    if CL < 0.04:
        return None
    out = {}
    pf = cm_extra(shape, b, table, CL + DL, V=V0), cm_extra(shape, b, table, CL - DL, V=V0)
    out["cov_f"] = min(pf[0][1], pf[1][1])
    out["d_fixed"] = (pf[0][0] - pf[1][0]) / (2 * DL)
    pg = cm_extra(shape, b, table, CL + DL, ws=ws), cm_extra(shape, b, table, CL - DL, ws=ws)
    out["cov_g"] = min(pg[0][1], pg[1][1])
    out["d_glide"] = (pg[0][0] - pg[1][0]) / (2 * DL)
    out["margin_fixed"] = sm - out["d_fixed"]
    out["margin_glide"] = sm - out["d_glide"]
    out["sec_slope"] = section_slope_low(shape, b, table, CL, V0)
    return out


def med(x):
    return statistics.median(x) if x else None


def frac(n, d):
    return n / d if d else None


# ---------------------------------------------------------------- main
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = read_roots()
    shapes = json.loads((SRC / "avl_shapes.json").read_text())
    scans = json.loads((SRC / "study_scans.json").read_text())
    tables = m.load_tables()
    R = {}
    txt = []
    def say(s=""):
        txt.append(s)

    G_ = good_rows(rows)
    R["n_root_rows"] = len(rows)
    R["n_stable_inband_cov09_rows"] = len(G_)
    R["note_valid_vs_coverage"] = "Here 'good' = both derivatives <0, in_band, coverage>=0.9 at the root (as requested); the study's 'valid' also required deriv coverage>=0.9."

    # ---- enrich (a)+(b)
    enriched = []
    for r in G_:
        sh = shapes[r["shape"]]
        tb = tables[(r["section"], int(r["ncrit"]))]
        b, ws, V, CL = r["span_m"], r["ws_n_m2"], r["v_m_s"], r["trim_cl"]
        f = ti_fraction(sh, b, tb, CL, V)
        lo = low_side(sh, b, tb, ws, CL, V, r["sm_avl"])
        e = dict(shape=r["shape"], section=r["section"], ncrit=int(r["ncrit"]), span_m=b, ws=ws, sm_avl=r["sm_avl"],
                 trim_cl=CL, re_mac=r["re_mac"], jagged_fraction=r["jagged_fraction"], clean=int(r["clean"]),
                 valid=int(r["valid"]), ti_fraction=f, on_ti=int(f is not None and f >= 0.5),
                 coupled_margin_glide=-r["dcm_dcl_glide"], coupled_margin_fixed=-r["dcm_dcl_fixed"],
                 cm_sec_mean=r["cm_sec_mean"], dcm_dcl_sec=r["dcm_dcl_sec"])
        if lo:
            e.update(low_d_fixed=lo["d_fixed"], low_d_glide=lo["d_glide"], low_margin_fixed=lo["margin_fixed"],
                     low_margin_glide=lo["margin_glide"], low_sec_slope=lo["sec_slope"], low_cov=min(lo["cov_f"], lo["cov_g"]))
        enriched.append(e)
    with (OUT / "roots_enriched.csv").open("w", newline="", encoding="utf-8") as s:
        keys = []
        for e in enriched:
            for k in e:
                if k not in keys:
                    keys.append(k)
        w = csv.DictWriter(s, fieldnames=keys)
        w.writeheader()
        for e in enriched:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in e.items()})

    # ---- (a)
    say("(a) TRANSITION-INTERVAL TALLY (stable, in-band, coverage>=0.9 roots; on-TI = >=50% of covered strip area on an alpha interval where xtr_top departs 1.0 at either bracketing Re)")
    A = {}
    for ncrit in (5, 9):
        for cls, sel in (("SM>=0", lambda e: e["sm_avl"] >= 0), ("SM<0", lambda e: e["sm_avl"] < 0)):
            es = [e for e in enriched if e["ncrit"] == ncrit and sel(e) and e["ti_fraction"] is not None]
            n = len(es)
            on = sum(e["on_ti"] for e in es)
            bins = Counter()
            for e in es:
                x = e["ti_fraction"]
                bins["0" if x == 0 else "(0,0.1)" if x < 0.1 else "[0.1,0.5)" if x < 0.5 else "[0.5,0.9)" if x < 0.9 else "[0.9,1]"] += 1
            xt = Counter((("clean" if e["clean"] else "not_clean"), ("onTI" if e["on_ti"] else "offTI")) for e in es)
            jb = Counter((("jag<=0.1" if e["jagged_fraction"] <= 0.1 else "jag>0.1"), ("onTI" if e["on_ti"] else "offTI")) for e in es)
            A[f"N{ncrit}|{cls}"] = dict(roots=n, on_ti=on, frac_on_ti=frac(on, n), mean_ti_fraction=statistics.mean(e["ti_fraction"] for e in es) if es else None,
                                        ti_distribution=dict(bins), crosstab_clean_vs_onTI=dict((f"{k[0]}|{k[1]}", v) for k, v in xt.items()))
            say(f"  N{ncrit} {cls}: roots {n}, on-TI {on} ({100*(frac(on,n) or 0):.0f}%), dist {dict(bins)}, clean x onTI {dict((f'{k[0]}|{k[1]}', v) for k, v in xt.items())}")
    # does one feature get split by the 0.25/deg cut: of on-TI roots, fraction flagged jagged vs clean
    on_all = [e for e in enriched if e["on_ti"] and e["ti_fraction"] is not None]
    off_all = [e for e in enriched if not e["on_ti"] and e["ti_fraction"] is not None]
    A["all_ncrit_all_sm"] = dict(on_ti=len(on_all), on_ti_and_clean=sum(e["clean"] for e in on_all),
                                 off_ti=len(off_all), off_ti_and_clean=sum(e["clean"] for e in off_all),
                                 on_ti_mean_jagged_fraction=statistics.mean(e["jagged_fraction"] for e in on_all) if on_all else None,
                                 off_ti_mean_jagged_fraction=statistics.mean(e["jagged_fraction"] for e in off_all) if off_all else None)
    say(f"  all: on-TI {len(on_all)} (clean {A['all_ncrit_all_sm']['on_ti_and_clean']}), off-TI {len(off_all)} (clean {A['all_ncrit_all_sm']['off_ti_and_clean']})")
    R["a_transition_interval"] = A

    # ---- (b)
    say("\n(b) SECTION SLOPE BELOW THE HUMP (CL_trim-0.05; fixed Re/V0; margin = SM_AVL - dCm_extra/dCL; <0 = statically unstable at lower CL)")
    B = {}
    for ncrit in (None, 5, 9):
        for cls, sel in (("SM>=0", lambda e: e["sm_avl"] >= 0), ("SM>=0.025", lambda e: e["sm_avl"] >= 0.025)):
            es = [e for e in enriched if (ncrit is None or e["ncrit"] == ncrit) and sel(e) and "low_margin_fixed" in e and e["low_cov"] >= 0.9]
            if not es:
                continue
            for sub, ss in (("all", es), ("clean", [e for e in es if e["clean"]])):
                if not ss:
                    continue
                nf = sum(1 for e in ss if e["low_margin_fixed"] < 0)
                ng = sum(1 for e in ss if e["low_margin_glide"] < 0)
                nsl = [e["low_sec_slope"] for e in ss if e["low_sec_slope"] is not None]
                key = f"N{ncrit or 'both'}|{cls}|{sub}"
                B[key] = dict(roots=len(ss), frac_unstable_below_fixed=nf / len(ss), frac_unstable_below_glide=ng / len(ss),
                              median_section_dcm_dcl_below=med(nsl), median_low_d_fixed=med([e["low_d_fixed"] for e in ss]),
                              median_low_margin_fixed=med([e["low_margin_fixed"] for e in ss]))
                say(f"  {key}: roots {len(ss)}, unstable at lower CL: fixed-speed {100*nf/len(ss):.0f}%, glide {100*ng/len(ss):.0f}%; median section dcm/dcl below {B[key]['median_section_dcm_dcl_below']:.3f}")
    R["b_below_hump"] = B
    n_nolow = sum(1 for e in enriched if "low_margin_fixed" not in e or e["low_cov"] < 0.9)
    R["b_roots_without_valid_low_side_eval"] = n_nolow

    # ---- (c)
    say("\n(c) SM_AVL < 0.025 CLASS (discrete SM = -0.05, -0.025, 0). SCAN FLOOR IS -0.05 => class truncated: stable roots at SM < -0.05 are not examined.")
    neg = [e for e in enriched if e["sm_avl"] < 0.025]
    C = dict(sm_values=sorted(set(e["sm_avl"] for e in neg)), stable_inband_cov09_roots=len(neg))
    C["per_sm"] = {str(sm): len([e for e in neg if e["sm_avl"] == sm]) for sm in C["sm_values"]}
    C["split_clean_jagged"] = dict(Counter("clean" if e["clean"] else "jagged/not_clean" for e in neg))
    C["split_ti"] = dict(Counter("onTI" if e["on_ti"] else "offTI" for e in neg if e["ti_fraction"] is not None))
    C["split_clean_x_ti"] = dict(Counter(f"{'clean' if e['clean'] else 'not_clean'}|{'onTI' if e['on_ti'] else 'offTI'}" for e in neg if e["ti_fraction"] is not None))
    for dim in ("section", "span_m", "ws", "ncrit"):
        C[f"by_{dim}"] = {str(k): v for k, v in sorted(Counter(e[dim] for e in neg).items())}
        C[f"by_{dim}_clean"] = {str(k): v for k, v in sorted(Counter(e[dim] for e in neg if e["clean"]).items())}
    for nm, ss in (("all", neg), ("clean", [e for e in neg if e["clean"]])):
        if ss:
            dd = [e["dcm_dcl_sec"] for e in ss if e["dcm_dcl_sec"] is not None and e["dcm_dcl_sec"] == e["dcm_dcl_sec"]]
            C[f"stats_{nm}"] = dict(n=len(ss), re_mac_min=min(e["re_mac"] for e in ss), re_mac_max=max(e["re_mac"] for e in ss),
                                    trim_cl_min=min(e["trim_cl"] for e in ss), trim_cl_max=max(e["trim_cl"] for e in ss),
                                    frac_cm_sec_positive=sum(1 for e in ss if e["cm_sec_mean"] > 0) / len(ss),
                                    median_cm_sec=med([e["cm_sec_mean"] for e in ss]), median_dcm_dcl_sec=med(dd),
                                    frac_dcm_dcl_sec_negative=sum(1 for x in dd if x < 0) / len(dd) if dd else None,
                                    coupled_margin_glide_median=med([e["coupled_margin_glide"] for e in ss]),
                                    coupled_margin_glide_range=[min(e["coupled_margin_glide"] for e in ss), max(e["coupled_margin_glide"] for e in ss)],
                                    coupled_margin_fixed_median=med([e["coupled_margin_fixed"] for e in ss]),
                                    coupled_margin_fixed_range=[min(e["coupled_margin_fixed"] for e in ss), max(e["coupled_margin_fixed"] for e in ss)])
    # survival under both Ncrit at the same discrete SM, and W/S extent
    for flag in ("any", "clean"):
        sets = defaultdict(lambda: defaultdict(set))
        for e in neg:
            if flag == "clean" and not e["clean"]:
                continue
            sets[(e["shape"], e["section"], e["span_m"], e["ws"], e["sm_avl"])][e["ncrit"]].add(1)
        both = [k for k, v in sets.items() if 5 in v and 9 in v]
        n5 = [k for k, v in sets.items() if 5 in v]
        n9 = [k for k, v in sets.items() if 9 in v]
        ext = defaultdict(set)
        for k in both:
            ext[(k[0], k[1], k[2])].add(k[3])
        C[f"survival_{flag}"] = dict(case_sm_with_N5=len(n5), case_sm_with_N9=len(n9), case_sm_with_both_same_sm=len(both),
                                    ws_extent_histogram_for_both=dict(Counter(len(v) for v in ext.values())),
                                    both_by_section=dict(Counter(k[1] for k in both)))
    R["c_negative_sm_class"] = C
    say(f"  roots {C['stable_inband_cov09_roots']} per SM {C['per_sm']}; clean/jagged {C['split_clean_jagged']}; clean x TI {C['split_clean_x_ti']}")
    for k in ("stats_all", "stats_clean"):
        if k in C:
            say(f"  {k}: " + json.dumps({a: (round(b, 4) if isinstance(b, float) else b) for a, b in C[k].items()}))
    say(f"  by section {C['by_section']} (clean {C['by_section_clean']}); by span {C['by_span_m']}; by W/S {C['by_ws']}; by Ncrit {C['by_ncrit']}")
    say(f"  survival any: {C['survival_any']}")
    say(f"  survival clean: {C['survival_clean']}")

    # ---- (d) CG windows from scans
    say(f"\n(d) CG WINDOWS from study_scans.json. SCAN STEP USED = {SCAN_STEP} in SM (0.5% MAC); width = (n-1)*step of the widest CONTIGUOUS run; a single hit (n=1) => width 0, meaning 'between 0 and 2 steps' (<1% MAC).")
    D = {}
    cgrows = []
    for flag in ("clean", "any"):
        for clsname, sel in (("SM>=0.025", lambda x: x >= 0.025 - 1e-9), ("SM>=0", lambda x: x >= -1e-9), ("SM<0", lambda x: x < -1e-9)):
            idx = defaultdict(dict)
            for s in scans:
                idx[(s["key"], s["section"], s["span"], s["ws"])][s["ncrit"]] = set(round(x, 4) for x in s[flag] if sel(x))
            for tier, fn in (("N5_only_set", lambda v: v.get(5, set())), ("N9_only_set", lambda v: v.get(9, set())),
                             ("both", lambda v: v.get(5, set()) & v.get(9, set()))):
                pct, mm, nonc, single, npts, totalext = [], [], 0, 0, [], []
                for k, v in idx.items():
                    pts = sorted(fn(v))
                    if not pts:
                        continue
                    runs, cur = [], [pts[0]]
                    for a, b2 in zip(pts, pts[1:]):
                        if round((b2 - a) / SCAN_STEP) == 1:
                            cur.append(b2)
                        else:
                            runs.append(cur); cur = [b2]
                    runs.append(cur)
                    nonc += len(runs) > 1
                    widest = max(len(r) for r in runs)
                    single += widest == 1
                    wpct = 100 * (widest - 1) * SCAN_STEP
                    mac_mm = 1000 * shapes[k[0]]["mac"] * k[2]
                    pct.append(wpct)
                    mm.append(mac_mm * (widest - 1) * SCAN_STEP)
                    npts.append(len(pts))
                    cgrows.append(dict(flag=flag, sm_class=clsname, tier=tier, shape=k[0], section=k[1], span=k[2], ws=k[3],
                                       n_points=len(pts), n_runs=len(runs), widest_run_points=widest,
                                       width_pct_mac=wpct, width_mm=mm[-1], sm_min=pts[0], sm_max=pts[-1]))
                D[f"{flag}|{clsname}|{tier}"] = dict(cases=len(pct), noncontiguous_cases=nonc, single_hit_cases=single,
                                                   median_width_pct_mac=med(pct), max_width_pct_mac=max(pct) if pct else None,
                                                   median_width_mm=med(mm), max_width_mm=max(mm) if mm else None,
                                                   median_points=med(npts))
                if flag == "clean":
                    d = D[f"{flag}|{clsname}|{tier}"]
                    say(f"  {flag} {clsname} {tier}: cases {d['cases']}, non-contiguous {d['noncontiguous_cases']}, single-hit {d['single_hit_cases']}, width %MAC med/max {d['median_width_pct_mac']}/{d['max_width_pct_mac']}, mm med/max {d['median_width_mm'] and round(d['median_width_mm'],2)}/{d['max_width_mm'] and round(d['max_width_mm'],2)}")
    R["d_cg_windows"] = D
    with (OUT / "cg_windows.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=list(cgrows[0].keys()))
        w.writeheader()
        w.writerows(cgrows)

    # ---- (e)
    say("\n(e) TRIM-CL RANGE / MAX SECTION CM")
    E = {}
    cleanrows = [r for r in G_ if r["clean"] == 1 and r["valid"] == 1]
    def rng(rs, label):
        if not rs:
            return None
        lo_r, hi_r = min(rs, key=lambda r: r["trim_cl"]), max(rs, key=lambda r: r["trim_cl"])
        f = lambda r: {k: r[k] for k in ("shape", "section", "ncrit", "span_m", "ws_n_m2", "sm_avl", "trim_cl", "re_mac", "jagged_fraction")}
        return dict(n=len(rs), min=lo_r["trim_cl"], max=hi_r["trim_cl"], row_min=f(lo_r), row_max=f(hi_r))
    E["clean_valid_stable_SM>=0.025_all_ncrit"] = rng([r for r in cleanrows if r["sm_avl"] >= 0.025], "")
    grp = defaultdict(lambda: defaultdict(list))
    for r in cleanrows:
        grp[(r["shape"], r["section"], r["span_m"], r["ws_n_m2"], r["sm_avl"])][int(r["ncrit"])].append(r)
    both_rows = [r for k, v in grp.items() if 5 in v and 9 in v for n in (5, 9) for r in v[n]]
    E["clean_both_ncrit_same_discrete_SM"] = rng(both_rows, "")
    E["clean_both_ncrit_same_discrete_SM_SM>=0.025"] = rng([r for r in both_rows if r["sm_avl"] >= 0.025], "")
    E["clean_both_ncrit_same_discrete_SM_SM<0.025"] = rng([r for r in both_rows if r["sm_avl"] < 0.025], "")
    visc, _ = read_database()
    mx = []
    for (sec, nc, re), rs in sorted(visc.items()):
        c = [x for x in rs if x["alpha"] >= 0 and x["cl"] < 0.6]
        if c:
            top = max(c, key=lambda x: x["cm"])
            mx.append(dict(section=sec, ncrit=nc, re=re, max_cm=top["cm"], alpha=top["alpha"], cl=top["cl"], cd=top["cd"], xtr_top=top["xtr_top"], xtr_bot=top["xtr_bot"], pass_=top["pass"]))
    with (OUT / "max_section_cm_per_re.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=list(mx[0].keys()))
        w.writeheader()
        w.writerows(mx)
    E["max_section_cm_overall"] = max(mx, key=lambda x: x["max_cm"])
    E["max_section_cm_by_section_ncrit"] = {f"{s}|N{n}": max((x for x in mx if x["section"] == s and x["ncrit"] == n), key=lambda x: x["max_cm"]) for s in CFG["sections"] for n in (5, 9)}
    R["e_trim_and_cm"] = E
    for k, v in E.items():
        if k.startswith("clean") and v:
            say(f"  {k}: n {v['n']}, trim CL {v['min']:.3f}..{v['max']:.3f}; max row {v['row_max']}")
    say("  max section cm (alpha>=0, cl<0.6) per section/Ncrit (full per-Re table in max_section_cm_per_re.csv):")
    for k, v in E["max_section_cm_by_section_ncrit"].items():
        say(f"    {k}: {v['max_cm']:.4f} at Re {v['re']}, alpha {v['alpha']}, cl {v['cl']}")

    # ---- (f) sweep sensitivity (uses ALL stable in-band cov>=0.9 roots from the CSV)
    say("\n(f) SWEEP SENSITIVITY: same AR/taper/dih/section/span/W/S/Ncrit/SM, sweep 10/20/30/35 (stable, in-band, cov>=0.9 roots)")
    fam = defaultdict(dict)
    allroots = defaultdict(lambda: defaultdict(list))
    for r in rows:
        allroots[(r["ar"], r["taper"], r["dih"], r["section"], r["span_m"], r["ws_n_m2"], int(r["ncrit"]), r["sm_avl"])][r["sweep"]].append(r)
    cl_sp, mg_sp, pres_all, pres_some, total = [], [], 0, 0, 0
    fam_rows = []
    for k, bysw in allroots.items():
        if k[2] != 0:
            continue
        sel = {sw: [x for x in rs if x["stable_glide"] == 1 and x["stable_fixed"] == 1 and x["in_band"] == 1 and x["coverage"] >= 0.9] for sw, rs in bysw.items()}
        have = [sw for sw, rs in sel.items() if rs]
        if not have:
            continue
        total += 1
        if len(have) == 4:
            pres_all += 1
            # one root per sweep needed for a paired comparison; use the lowest-CL root when several
            pick = {sw: min(rs, key=lambda x: x["trim_cl"]) for sw, rs in sel.items()}
            cls_ = [x["trim_cl"] for x in pick.values()]
            mgs = [-x["dcm_dcl_glide"] for x in pick.values()]
            cl_sp.append(max(cls_) - min(cls_))
            mg_sp.append(max(mgs) - min(mgs))
            fam_rows.append(dict(ar=k[0], taper=k[1], section=k[3], span=k[4], ws=k[5], ncrit=k[6], sm=k[7],
                                 cl_spread=cl_sp[-1], margin_glide_spread=mg_sp[-1], cl_mean=statistics.mean(cls_),
                                 **{f"cl_sw{int(sw)}": x["trim_cl"] for sw, x in pick.items()}))
        else:
            pres_some += 1
    with (OUT / "sweep_sensitivity.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=list(fam_rows[0].keys()) if fam_rows else ["none"])
        w.writeheader()
        w.writerows(fam_rows)
    F = dict(families_with_any_root=total, with_root_at_all_4_sweeps=pres_all, root_only_at_some_sweeps=pres_some,
             trim_cl_spread_median=med(cl_sp), trim_cl_spread_p90=sorted(cl_sp)[int(0.9 * len(cl_sp))] if cl_sp else None,
             trim_cl_spread_max=max(cl_sp) if cl_sp else None, margin_spread_median=med(mg_sp),
             margin_spread_p90=sorted(mg_sp)[int(0.9 * len(mg_sp))] if mg_sp else None, margin_spread_max=max(mg_sp) if mg_sp else None)
    # also: relative spread
    rel = [r["cl_spread"] / r["cl_mean"] for r in fam_rows]
    F["trim_cl_rel_spread_median"] = med(rel)
    R["f_sweep"] = F
    say("  " + json.dumps({a: (round(b, 4) if isinstance(b, float) else b) for a, b in F.items()}))

    # ---- (g) mass
    say("\n(g) ALL-UP MASS (g) = W/S * b^2/AR / g")
    ws_list, spans, ars = CFG["study"]["wing_loading_n_m2"], CFG["study"]["span_m"], [3, 4.5, 6]
    with (OUT / "mass_table_g.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.writer(s)
        w.writerow(["span_m", "AR"] + [f"WS{x}" for x in ws_list])
        say("  span  AR | " + " ".join(f"WS{x:>3}" for x in ws_list))
        for b in spans:
            for ar in ars:
                ms = [1000 * x * b * b / ar / G for x in ws_list]
                w.writerow([b, ar] + [f"{v:.2f}" for v in ms])
                say(f"  {b:.2f} {ar:>4} | " + " ".join(f"{v:5.1f}" for v in ms))

    # ---- reproduction checks against summary.json
    summ = json.loads((SRC / "summary.json").read_text())
    chk = {}
    sc = D.get("clean|SM>=0.025|both", {})
    idx = defaultdict(dict)
    for s in scans:
        idx[(s["key"], s["section"], s["span"], s["ws"])][s["ncrit"]] = set(x for x in s["clean"] if x >= 0.025 - 1e-9)
    chk["clean_both_cases_SM>=0.025_reanalysis"] = sum(1 for v in idx.values() if v.get(5, set()) & v.get(9, set()))
    chk["clean_both_cases_SM>=0.025_summary"] = summ["stable_inband_clean_SM>=0.025"]["with_both"]
    chk["clean_N5"] = (sum(1 for v in idx.values() if v.get(5)), summ["stable_inband_clean_SM>=0.025"]["with_N5"])
    chk["clean_N9"] = (sum(1 for v in idx.values() if v.get(9)), summ["stable_inband_clean_SM>=0.025"]["with_N9"])
    chk["summary_both_width_stats"] = summ["clean_stable_width_stats_SM_ge_0p025"]["both"]
    chk["reanalysis_both_width_clean_SM>=0.025"] = D.get("clean|SM>=0.025|both")
    chk["summary_trim_cl_range_claim"] = "0.150..0.376 (clean valid rows SM>=0.025, ALL tiers, not both-only)"
    chk["reanalysis_trim_cl_clean_valid_SM>=0.025_all"] = {k: E["clean_valid_stable_SM>=0.025_all_ncrit"][k] for k in ("n", "min", "max")}
    chk["reanalysis_trim_cl_clean_both_same_SM"] = {k: (E["clean_both_ncrit_same_discrete_SM_SM>=0.025"] or {}).get(k) for k in ("n", "min", "max")}
    R["reproduction_checks"] = chk
    say("\nREPRODUCTION CHECKS vs summary.json")
    for k, v in chk.items():
        say(f"  {k}: {v}")
    (OUT / "reanalysis_summary.json").write_text(json.dumps(R, indent=1, default=str))
    (OUT / "reanalysis_summary.txt").write_text("\n".join(txt), encoding="utf-8")
    print("\n".join(txt))


if __name__ == "__main__":
    main()
