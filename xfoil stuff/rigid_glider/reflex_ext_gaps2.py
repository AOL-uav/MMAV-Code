"""Follow-ups to reflex_ext_gaps.py (writes only under results/runs/rigid_glider/reflex_ext/gaps2/).

item1: saved matched-Cl cell for NACA 0008 20%/10 Re 100k (ITER 400, step 0.5 from -3, PANE 160) vs clean Re 100k.
item2: low-Re clean-section paneling check (PANE 160 vs PPAR 240 only; N never above 240) for the pure-wing
       database: NACA 0006/0008, Re 20k/40k, Ncrit 5/9; alpha schedule of pure_wing_polars.py (0->10, then 0->-2,
       0.5 deg).  Missing points stay missing; nothing is filled.

Run from the repo root:  py -B rigid_glider/reflex_ext_gaps2.py item1 item2
"""

from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import reflex_xfoil as rx
import reflex_ext_predict as rp
import reflex_ext_gaps as g
import pure_wing_polars as pwp

ROOT = g.ROOT
OUT = ROOT / "results/runs/rigid_glider/reflex_ext/gaps2"
GAPS = g.OUT
DB = pwp.DATABASE
SECTIONS = ["NACA_0006", "NACA_0008"]
RES = [20000, 40000]
NCRITS = [5, 9]
PANELS = [160, 240]
ITER_DB = None            # filled from pure_wing_config.json at run time
ITER_NEW = 400
CLS = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40]


# ------------------------------------------------------------------ item 1
def item1(out: Path):
    bent = rx.read_polar(GAPS / "step3/S1b_iter400_step0p5_from_m3/polar.dat")
    clean = rx.read_polar(GAPS / "step1/base160/clean_c10_a0_Re100000/polar.dat")
    rows = []
    for t in (0.20, 0.25, 0.30):
        m = rp.matched(clean, bent, t)
        rows.append(dict(cl=t, lip_dCm=m and m["dcm"], clean_cm=m and m["clean_cm"], bent_cm=m and m["bent_cm"],
                         added_cd=m and m["dcd"], alpha_clean=m and m["clean_alpha"], alpha_bent=m and m["bent_alpha"],
                         n_clean_brackets=len(rp.brackets(clean, t)), n_bent_brackets=len(rp.brackets(bent, t)),
                         note="" if m else "MISSING: not bracketed"))
    sl = {}
    for nm, pol in (("bent", bent), ("clean", clean)):
        p0, p1 = rx.at_cl(pol, 0.15), rx.at_cl(pol, 0.35)
        sl[nm] = (p1["cm"] - p0["cm"]) / 0.20 if p0 and p1 else None
    a25 = rx.at_cl(bent, 0.25)
    for r in rows:
        r.update(points_returned_bent=len(bent), points_requested=23, points_returned_clean=len(clean),
                 bent_dCm_dCl_0p15_0p35=sl["bent"], clean_dCm_dCl_0p15_0p35=sl["clean"],
                 alpha_at_cl0p25_bent=a25 and a25["alpha_deg"],
                 source_bent=str(GAPS / "step3/S1b_iter400_step0p5_from_m3/polar.dat"),
                 source_clean=str(GAPS / "step1/base160/clean_c10_a0_Re100000/polar.dat"))
    g.write_csv(out / "item1_recovered_cell.csv", rows)
    print("item1:", json.dumps(rows, indent=1))


# ------------------------------------------------------------------ item 2
def sweep_script(re: int, ncrit: int, n: int, iters: int) -> str:
    s = "PLOP\nG\n\nLOAD foil.dat\n"
    if n != 160:
        s += f"PPAR\nN {n}\n\n\n"
    s += "PANE\nPSAV paneled.dat\nOPER\n"
    s += f"VISC {re}\nVPAR\nN {ncrit}\n\nITER {iters}\n"
    for k, (a0, a1, da) in enumerate([(0.0, 10.0, 0.5), (0.0, -2.0, -0.5)]):
        s += f"PACC\npolar_p1_{k}.dat\n\nALFA {a0:g}\nINIT\nALFA {a0:g}\nASEQ {a0:g} {a1:g} {da:g}\nPACC\n"
    return s + "\nQUIT\n"


def run_case(root: Path, section: str, re: int, ncrit: int, n: int, iters: int, tag: str) -> dict:
    folder = root / f"{section}_Re{re}_N{ncrit}_pan{n}_{tag}"
    folder.mkdir(parents=True, exist_ok=True)
    if not (folder / "foil.dat").exists():
        pwp.write_foil(folder, section, folder.name)
    r = g.run_xfoil(folder, sweep_script(re, ncrit, n, iters), 300)
    rows = {}
    for k in range(2):
        for x in pwp.parse_polar(folder / f"polar_p1_{k}.dat", -2.0, 10.0):
            rows[round(x["alpha"], 4)] = x
    return dict(section=section, re=re, ncrit=ncrit, n=n, iters=iters, rows=[rows[k] for k in sorted(rows)],
                code=r["code"], folder=str(folder))


def db_rows(section, ncrit, re, pass_=1):
    out = {}
    with DB.open(encoding="utf-8") as s:
        for row in csv.DictReader(s):
            if (row["section"] == section and int(row["ncrit"]) == ncrit and int(float(row["re"])) == re
                    and int(row["pass"]) == pass_):
                out[round(float(row["alpha"]), 4)] = {k: float(row[k]) for k in
                                                       ("alpha", "cl", "cd", "cdp", "cm", "xtr_top", "xtr_bot")}
    return out


def compare_db(rows: list[dict], db: dict) -> dict:
    mine = {round(r["alpha"], 4): r for r in rows}
    common = sorted(set(mine) & set(db))
    per = {k: max((abs(mine[a][k] - db[a][k]) for a in common), default=None)
           for k in ("cl", "cd", "cdp", "cm", "xtr_top", "xtr_bot")}
    return dict(n_mine=len(mine), n_db_pass1=len(db), n_common=len(common),
                only_mine=" ".join(f"{a:g}" for a in sorted(set(mine) - set(db))),
                only_db=" ".join(f"{a:g}" for a in sorted(set(db) - set(mine))),
                **{f"maxdiff_{k}": v for k, v in per.items()})


def as_cl_rows(rows):  # reflex_xfoil style rows for at_cl
    return [dict(alpha_deg=r["alpha"], cl=r["cl"], cd=r["cd"], cm=r["cm"]) for r in rows]


def lsq_slope(pts):
    if len(pts) < 2:
        return None
    n = len(pts)
    mx, my = sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    return sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx if sxx > 0 else None


def transition_onset(rows: list[dict]) -> dict:
    """First alpha (increasing from 0) where xtr_top < 1.0; interval = (previous adjacent point, that point)."""
    up = [r for r in sorted(rows, key=lambda r: r["alpha"]) if r["alpha"] >= -1e-9]
    prev = None
    for r in up:
        if r["xtr_top"] < 0.9999:
            if prev is None or r["alpha"] - prev["alpha"] > 0.51:
                return dict(onset_found=True, interval_ok=False, a_hi=r["alpha"], cl_hi=r["cl"], xtr_hi=r["xtr_top"],
                            note="no adjacent attached (xtr=1) point below onset")
            return dict(onset_found=True, interval_ok=True, a_lo=prev["alpha"], a_hi=r["alpha"], cl_lo=prev["cl"],
                        cl_hi=r["cl"], xtr_hi=r["xtr_top"], note="")
        prev = r
    return dict(onset_found=False, note="xtr_top stays 1.0 over all returned alpha>=0")


def metrics(rows: list[dict]) -> dict:
    pol = as_cl_rows(rows)
    m = {"n_points": len(rows), "alphas": " ".join(f"{r['alpha']:g}" for r in rows)}
    for t in CLS:
        p = rx.at_cl(pol, t)
        m[f"cm_at_cl{t:.2f}"] = p["cm"] if p else None
        m[f"n_brackets_cl{t:.2f}"] = len(rp.brackets(pol, t))
    low = [r for r in rows if r["cl"] < 0.6]
    if low:
        h = max(low, key=lambda r: r["cm"])
        m.update(cm_max_cl_lt_0p6=h["cm"], cm_max_alpha=h["alpha"], cm_max_cl=h["cl"])
        rise = [(r["cl"], r["cm"]) for r in sorted(rows, key=lambda r: r["alpha"]) if 0 <= r["alpha"] <= h["alpha"]]
        fall = [(r["cl"], r["cm"]) for r in sorted(rows, key=lambda r: r["alpha"]) if r["alpha"] >= h["alpha"] and r["cl"] < 0.6]
        m["dcm_dcl_rising_0_to_hump"] = lsq_slope(rise)
        m["dcm_dcl_falling_hump_to_cl0p6"] = lsq_slope(fall)
        m["n_rising_pts"], m["n_falling_pts"] = len(rise), len(fall)
    # zero crossing of cm (+ -> -) on the falling side, after the hump, adjacent points only
    zc = None
    if low:
        srt = [r for r in sorted(rows, key=lambda r: r["alpha"]) if r["alpha"] >= h["alpha"]]
        for a, b in zip(srt, srt[1:]):
            if b["alpha"] - a["alpha"] <= 0.51 and a["cm"] > 0 >= b["cm"]:
                f = a["cm"] / (a["cm"] - b["cm"])
                zc = dict(cl=a["cl"] + f * (b["cl"] - a["cl"]), alpha=a["alpha"] + f * (b["alpha"] - a["alpha"]))
                break
    m["cm_zero_cross_cl"] = zc and zc["cl"]
    m["cm_zero_cross_alpha"] = zc and zc["alpha"]
    on = transition_onset(rows)
    m.update({f"onset_{k}": v for k, v in on.items()})
    return m


def item2(out: Path):
    cfg = pwp.cfg()
    iter_db = cfg["xfoil_iter"]
    cases = [(s, re, nc) for s in SECTIONS for re in RES for nc in NCRITS]
    # gate: 160 panels at the database's own ITER vs database pass-1 rows
    jobs = [(out / "gate", s, re, nc, 160, iter_db, f"iter{iter_db}") for s, re, nc in cases]
    # study: ITER 400, 160 and 240
    jobs += [(out / "item2", s, re, nc, n, ITER_NEW, "iter400") for s, re, nc in cases for n in PANELS]
    with ThreadPoolExecutor(max_workers=4) as pool:
        res = list(pool.map(lambda j: run_case(*j), jobs))
    gate_rows, ok = [], True
    for r in res[:len(cases)]:
        db = db_rows(r["section"], r["ncrit"], r["re"], 1)
        c = compare_db(r["rows"], db)
        gate_rows.append(dict(section=r["section"], re=r["re"], ncrit=r["ncrit"], panels=160, iters=r["iters"], **c))
    # also the ITER-400 160 run vs database
    for r in res[len(cases):]:
        if r["n"] == 160:
            c = compare_db(r["rows"], db_rows(r["section"], r["ncrit"], r["re"], 1))
            gate_rows.append(dict(section=r["section"], re=r["re"], ncrit=r["ncrit"], panels=160, iters=400, **c))
    g.write_csv(out / "item2_db_repro_check.csv", gate_rows)
    study = res[len(cases):]
    by = {(r["section"], r["re"], r["ncrit"], r["n"]): r for r in study}
    cmp_rows, long_rows = [], []
    for s, re, nc in cases:
        m = {n: metrics(by[(s, re, nc, n)]["rows"]) for n in PANELS}
        for n in PANELS:
            long_rows.append(dict(section=s, re=re, ncrit=nc, panels=n, exit=by[(s, re, nc, n)]["code"], **m[n]))
        row = dict(section=s, re=re, ncrit=nc)
        for k in m[160]:
            if k in ("alphas",):
                continue
            a, b = m[160][k], m[240][k]
            row[f"{k}_160"], row[f"{k}_240"] = a, b
            if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
                row[f"{k}_diff"] = b - a
        cmp_rows.append(row)
    g.write_csv(out / "item2_metrics_by_panels.csv", long_rows)
    g.write_csv(out / "item2_comparison_160_vs_240.csv", cmp_rows)
    # text summary
    def f(v, n=4):
        return "  --  " if v is None else f"{v:+.{n}f}"
    L = ["Clean-section paneling check, ITER 400, Ncrit as listed; '--' = missing / not found", ""]
    for r in cmp_rows:
        L.append(f"{r['section']} Re{r['re'] // 1000}k N{r['ncrit']}: points 160={r['n_points_160']} 240={r['n_points_240']}")
        for n in PANELS:
            m = {k[:-4]: v for k, v in r.items() if k.endswith(f"_{n}")}
            on = (f"alpha ({m.get('onset_a_lo')}, {m.get('onset_a_hi')}] cl ({f(m.get('onset_cl_lo'), 3)}, {f(m.get('onset_cl_hi'), 3)})"
                  if m.get("onset_onset_found") and m.get("onset_interval_ok") else
                  f"onset {m.get('onset_note')} a_hi={m.get('onset_a_hi')}")
            L.append(f"  {n}: onset {on}; max cm {f(m.get('cm_max_cl_lt_0p6'))} at a={m.get('cm_max_alpha')} cl={f(m.get('cm_max_cl'), 3)}; "
                     f"cm@0.2 {f(m.get('cm_at_cl0.20'))} cm@0.3 {f(m.get('cm_at_cl0.30'))}; "
                     f"dcm/dcl rise {f(m.get('dcm_dcl_rising_0_to_hump'), 3)} fall {f(m.get('dcm_dcl_falling_hump_to_cl0p6'), 3)}; "
                     f"cm=0 crossing cl {f(m.get('cm_zero_cross_cl'), 3)}")
        L.append("  diffs (240-160): " + ", ".join(f"cm@{t:.2f} {f(r.get(f'cm_at_cl{t:.2f}_diff'))}" for t in CLS))
        L.append("")
    (out / "item2_summary.txt").write_text("\n".join(L) + "\n")
    print("\n".join(L))
    print(json.dumps(gate_rows, indent=0)[:6000])


def main():
    out = OUT
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    st = sys.argv[1:] or ["item1", "item2"]
    if "item1" in st:
        item1(out)
    if "item2" in st:
        item2(out)
    print(f"elapsed {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
