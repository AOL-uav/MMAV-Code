"""Low-Re polar database for symmetric NACA 00xx sections WITH a trailing-edge lip (small-glider trim model).

Writes ONLY under results/runs/rigid_glider/reflex_small/polars/.  Imports (never edits) reflex_xfoil,
reflex_ext_predict, reflex_ext_gaps and pure_wing_polars.  Panel count never exceeds 240.

Run from the repo root:  py -B rigid_glider/reflex_small_polars.py [gate] [run] [tables]
Conventions follow pure_wing/polar_database.csv: columns section,ncrit,re,alpha,cl,cd,cdp,cm,xtr_top,xtr_bot,pass
(+ lip_chord_frac, lip_angle_deg, panels).  Inviscid rows: ncrit=0, re=0.  Clean rows: lip fields 0.
Alpha schedule: 0 -> 10 then 0 -> -4 in 0.5 deg (pure_wing_polars sweep order, downward sweep extended to -4).
Missing / unconverged points stay missing.  Positive Cm nose-up.
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
OUT = ROOT / "results/runs/rigid_glider/reflex_small/polars"
SECTIONS = ["NACA_0006", "NACA_0008"]
LIPS = [(0.10, 3.0), (0.10, 6.0), (0.15, 5.0), (0.15, 8.0), (0.20, 5.0), (0.20, 10.0)]
RES = [15000, 20000, 30000, 40000, 60000, 80000, 100000]
NCRITS = [5, 9]
PANELS = [160, 240]
ITER = 400
TIMEOUT = 300
AMIN, AMAX, DA = -4.0, 10.0, 0.5
GRID = [round(AMIN + i * DA, 4) for i in range(int(round((AMAX - AMIN) / DA)) + 1)]
TARGET_CL = [0.15, 0.20, 0.25, 0.30, 0.40]
FIELDS = pwp.FIELDS + ["lip_chord_frac", "lip_angle_deg", "panels"]
STORED_BENT = ROOT / "results/runs/rigid_glider/reflex_ext/predict/xfoil/NACA_0008_Re80000_c15_a8/polar.dat"


# ------------------------------------------------------------------ pure helpers
def lip_label(cf: float, ang: float) -> str:
    return f"{int(round(cf * 100))}/{ang:g}"


def build_script(reynolds: int | None, ncrit: int, panels: int, iters: int = ITER) -> str:
    """Viscous: two PACC sweeps (0->10, 0->-4) like pwp.script_sweeps; inviscid (reynolds None): one sweep."""
    s = "PLOP\nG\n\nLOAD foil.dat\n"
    if panels != 160:
        s += f"PPAR\nN {panels}\n\n\n"          # blank 1 leaves the change prompt, blank 2 leaves PPAR
    s += "PANE\nPSAV paneled.dat\nOPER\n"
    if reynolds is None:
        return s + (f"PACC\npolar_inv.dat\n\nALFA {AMIN:g}\nASEQ {AMIN:g} {AMAX:g} {DA:g}\nPACC\n\nQUIT\n")
    s += f"VISC {reynolds}\nVPAR\nN {ncrit}\n\nITER {iters}\n"
    for k, (a0, a1, da) in enumerate([(0.0, AMAX, DA), (0.0, AMIN, -DA)]):
        s += f"PACC\npolar_p1_{k}.dat\n\nALFA {a0:g}\nINIT\nALFA {a0:g}\nASEQ {a0:g} {a1:g} {da:g}\nPACC\n"
    return s + "\nQUIT\n"


def merge_sweeps(sweeps: list[list[dict]]) -> list[dict]:
    """Later sweeps override earlier ones at the same alpha (the shared alpha=0 point); sorted by alpha."""
    rows = {}
    for sw in sweeps:
        for r in sw:
            rows[round(r["alpha"], 4)] = r
    return [rows[k] for k in sorted(rows)]


def to_cl_rows(rows: list[dict]) -> list[dict]:
    return [dict(alpha_deg=r["alpha"], cl=r["cl"], cd=r["cd"], cm=r["cm"]) for r in rows]


def lip_moment_cells(clean: list[dict], bent: list[dict], inv_clean: list[dict], inv_bent: list[dict]) -> list[dict]:
    """Matched-Cl lip moment cells (database-style rows in, reflex-style rows made internally)."""
    c, b = to_cl_rows(clean), to_cl_rows(bent)
    ic, ib = to_cl_rows(inv_clean), to_cl_rows(inv_bent)
    out = []
    p0, p1 = rx.at_cl(b, 0.15), rx.at_cl(b, 0.35)
    bslope = (p1["cm"] - p0["cm"]) / 0.20 if p0 and p1 else None
    for t in TARGET_CL:
        m = rp.matched(c, b, t)
        a, bb = rx.at_cl(ic, t), rx.at_cl(ib, t)
        inv = (bb["cm"] - a["cm"]) if a and bb else None
        row = dict(cl=t, bent_dcm_dcl_0p15_0p35=bslope, inviscid_lip_dcm=inv)
        if m:
            row.update(lip_dcm=m["dcm"], added_cd=m["dcd"], clean_cm=m["clean_cm"], bent_cm=m["bent_cm"],
                       alpha_clean=m["clean_alpha"], alpha_bent=m["bent_alpha"],
                       n_clean_brackets=m["n_clean_brk"], n_bent_brackets=m["n_bent_brk"],
                       fraction_of_inviscid=(m["dcm"] / inv) if inv else None, note="")
        else:
            row["note"] = "MISSING: Cl not bracketed by adjacent returned points"
        out.append(row)
    return out


def spread(values: list[float | None]) -> tuple[float, float, int] | None:
    v = [x for x in values if x is not None]
    return (min(v), max(v), len(v)) if v else None


def coverage_row(rows: list[dict]) -> dict:
    got = {round(r["alpha"], 4) for r in rows}
    return dict(requested=len(GRID), returned=len(got & set(GRID)),
                missing_alphas=" ".join(f"{a:g}" for a in GRID if a not in got))


# ------------------------------------------------------------------ running
def write_foil(folder: Path, section: str, cf: float, ang: float, label: str):
    pts = rx.coordinates(int(section[-2:]) / 100, cf, ang)
    (folder / "foil.dat").write_text(label + "\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in pts) + "\n",
                                     encoding="ascii")


def run_case(root: Path, section: str, cf: float, ang: float, re: int | None, ncrit: int, panels: int) -> dict:
    """cf/ang = 0 for clean.  re None = inviscid."""
    tag = "clean" if ang == 0 else f"c{int(round(cf * 100)):02d}_a{ang:g}"
    cond = "inv" if re is None else f"Re{re}_N{ncrit}"
    folder = root / f"{section}_{tag}_{cond}_p{panels}".replace(".", "p")
    folder.mkdir(parents=True, exist_ok=True)
    if not (folder / "foil.dat").exists():
        write_foil(folder, section, cf if ang else 0.10, ang, folder.name)
    r = g.run_xfoil(folder, build_script(re, ncrit, panels), TIMEOUT)
    if re is None:
        rows = pwp.parse_polar(folder / "polar_inv.dat", AMIN, AMAX)
    else:
        rows = merge_sweeps([pwp.parse_polar(folder / f"polar_p1_{k}.dat", AMIN, AMAX) for k in range(2)])
    return dict(section=section, cf=cf if ang else 0.0, ang=ang, re=re, ncrit=ncrit, panels=panels, rows=rows,
                code=r["code"], runaway=r.get("runaway", False), elapsed=r["elapsed"], folder=str(folder),
                iwx=r["log"].count("Array size"), nan=r["log"].count("NaN"))


def read_clean160() -> dict:
    """pure_wing database clean 160-panel viscous polars -> {(section, ncrit, re): rows}; pass 1 preferred."""
    res = {}
    with pwp.DATABASE.open(encoding="utf-8") as s:
        for row in csv.DictReader(s):
            if row["section"] in SECTIONS and int(row["ncrit"]) > 0:
                key = (row["section"], int(row["ncrit"]), int(float(row["re"])))
                a = round(float(row["alpha"]), 4)
                d = res.setdefault(key, {})
                if a not in d or int(row["pass"]) < d[a]["pass"]:
                    d[a] = dict(alpha=float(row["alpha"]), cl=float(row["cl"]), cd=float(row["cd"]),
                                cdp=float(row["cdp"]), cm=float(row["cm"]), xtr_top=float(row["xtr_top"]),
                                xtr_bot=float(row["xtr_bot"]), pass_=int(row["pass"]), **{"pass": int(row["pass"])})
    return {k: [v[a] for a in sorted(v)] for k, v in res.items()}


# ------------------------------------------------------------------ stages
def stage_gate(out: Path) -> dict:
    gate = {}
    r = run_case(out / "gate", "NACA_0008", 0.15, 8.0, 80000, 5, 160)
    stored = rx.read_polar(STORED_BENT)
    cmp_ = g.compare_polars(
        [dict(alpha_deg=x["alpha"], cl=x["cl"], cd=x["cd"], cdp=x["cdp"], cm=x["cm"], top_xtr=x["xtr_top"],
              bot_xtr=x["xtr_bot"]) for x in r["rows"]], stored)
    gate["bent_0008_15_8_Re80k_N5_p160_vs_stored"] = cmp_
    # clean reproduction against the pure_wing database (one case)
    clean_db = read_clean160()
    c = run_case(out / "gate", "NACA_0008", 0.10, 0.0, 40000, 5, 160)
    db = {round(x["alpha"], 4): x for x in clean_db[("NACA_0008", 5, 40000)] if x["pass"] == 1}
    mine = {round(x["alpha"], 4): x for x in c["rows"]}
    common = sorted(set(mine) & set(db))
    gate["clean_0008_Re40k_N5_p160_vs_database_pass1"] = dict(
        n_mine=len(mine), n_db=len(db), n_common=len(common),
        only_mine=sorted(set(mine) - set(db)), only_db=sorted(set(db) - set(mine)),
        **{f"maxdiff_{k}": max((abs(mine[a][k] - db[a][k]) for a in common), default=None)
           for k in ("cl", "cd", "cdp", "cm", "xtr_top", "xtr_bot")})
    (out / "repro_gate.json").write_text(json.dumps(gate, indent=1))
    print("GATE", json.dumps(gate, indent=1))
    return gate


def all_jobs() -> list[tuple]:
    jobs = []
    for s in SECTIONS:
        for p in PANELS:
            jobs.append((s, 0.10, 0.0, None, 0, p))                    # clean inviscid
            for nc in NCRITS:
                for re in RES:
                    jobs.append((s, 0.10, 0.0, re, nc, p))              # clean viscous
            for cf, ang in LIPS:
                jobs.append((s, cf, ang, None, 0, p))                   # bent inviscid
                for nc in NCRITS:
                    for re in RES:
                        jobs.append((s, cf, ang, re, nc, p))
    return jobs


def stage_run(out: Path) -> list[dict]:
    jobs = all_jobs()
    with ThreadPoolExecutor(max_workers=4) as pool:
        res = list(pool.map(lambda j: run_case(out / "raw", *j), jobs))
    return res


def stage_tables(out: Path, res: list[dict]):
    db160 = read_clean160()
    # --- database csv (clean 160 is NOT regenerated or duplicated: it stays in pure_wing/polar_database.csv)
    with (out / "lip_polar_database.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=FIELDS)
        w.writeheader()
        for r in sorted(res, key=lambda r: (r["section"], r["panels"], r["re"] or 0, r["ncrit"], r["cf"], r["ang"])):
            for x in r["rows"]:
                w.writerow(dict(section=r["section"], ncrit=r["ncrit"] if r["re"] else 0, re=r["re"] or 0,
                                alpha=x["alpha"], cl=x["cl"], cd=x["cd"], cdp=x["cdp"], cm=x["cm"],
                                xtr_top=x["xtr_top"], xtr_bot=x["xtr_bot"], **{"pass": 1},
                                lip_chord_frac=r["cf"], lip_angle_deg=r["ang"], panels=r["panels"]))
    by = {(r["section"], r["cf"], r["ang"], r["re"], r["ncrit"] if r["re"] else 0, r["panels"]): r for r in res}
    # --- coverage
    cov = []
    for r in res:
        cov.append(dict(section=r["section"], lip=lip_label(r["cf"], r["ang"]) if r["ang"] else "clean",
                        re=r["re"] or "inviscid", ncrit=r["ncrit"] if r["re"] else "", panels=r["panels"],
                        **coverage_row(r["rows"]), exit=r["code"], runaway_killed=r["runaway"],
                        iwx_warnings=r["iwx"], nan_lines=r["nan"], elapsed_s=round(r["elapsed"], 1), folder=r["folder"]))
    g.write_csv(out / "coverage.csv", cov)

    def clean_rows(s, nc, re, p):
        if p == 160:
            return db160.get((s, nc, re), [])
        return by[(s, 0.0, 0.0, re, nc, p)]["rows"]

    cells = []
    for s in SECTIONS:
        for p in PANELS:
            inv_clean = by[(s, 0.0, 0.0, None, 0, p)]["rows"]
            for cf, ang in LIPS:
                inv_bent = by[(s, cf, ang, None, 0, p)]["rows"]
                for nc in NCRITS:
                    for re in RES:
                        bent = by[(s, cf, ang, re, nc, p)]["rows"]
                        clean = clean_rows(s, nc, re, p)
                        for c in lip_moment_cells(clean, bent, inv_clean, inv_bent):
                            cells.append(dict(section=s, lip=lip_label(cf, ang), lip_chord_frac=cf, lip_angle_deg=ang,
                                              re=re, ncrit=nc, panels=p, points_bent=len(bent), points_clean=len(clean),
                                              **c))
    g.write_csv(out / "lip_moment_table.csv", cells)
    return cells, cov


def fmt(v, n=4):
    return "  --  " if v is None else f"{v:+.{n}f}"


def _num(x):
    return None if x in ("", None) else float(x)


def load_cells(out: Path) -> list[dict]:
    cells = []
    with (out / "lip_moment_table.csv").open(encoding="utf-8") as s:
        for r in csv.DictReader(s):
            for k in ("lip_chord_frac", "lip_angle_deg", "cl", "lip_dcm", "added_cd", "fraction_of_inviscid",
                      "inviscid_lip_dcm", "bent_dcm_dcl_0p15_0p35", "alpha_bent", "alpha_clean"):
                r[k] = _num(r.get(k))
            for k in ("re", "ncrit", "panels", "points_bent", "points_clean"):
                r[k] = int(r[k])
            cells.append(r)
    return cells


def load_cov(out: Path) -> list[dict]:
    with (out / "coverage.csv").open(encoding="utf-8") as s:
        rows = list(csv.DictReader(s))
    for r in rows:
        r["returned"] = int(r["returned"])
    return rows


def range_stats(pairs: list[tuple[float, float]], floor: float = 0.01) -> str:
    """pairs = (reference, other).  Absolute range always; % range only where |reference| >= floor."""
    if not pairs:
        return "no pairs"
    d = [o - r for r, o in pairs]
    pc = [100 * (o - r) / r for r, o in pairs if abs(r) >= floor]
    t = f"abs {min(d):+.4f}..{max(d):+.4f} (median {sorted(d)[len(d) // 2]:+.4f}, n={len(d)})"
    if pc:
        t += f"; % where |ref|>={floor:g}: {min(pc):+.0f}..{max(pc):+.0f} (n={len(pc)})"
    return t


def write_summary(out: Path, cells: list[dict], cov: list[dict]):
    def get(s, lip, re, nc, p, cl=0.25):
        return next((c for c in cells if c["section"] == s and c["lip"] == lip and c["re"] == re and c["ncrit"] == nc
                     and c["panels"] == p and abs(c["cl"] - cl) < 1e-9), None)
    ok = lambda c: c is not None and c.get("lip_dcm") is not None
    lips = [lip_label(cf, ang) for cf, ang in LIPS]
    L = ["Lip polar database summary.  Lip moment = bent cm - clean cm at matched Cl, same Ncrit and paneling; '--' = missing.",
         f"Clean 160-panel reference = pure_wing/polar_database.csv (its own ITER-200 schedule, alpha -2..10 plus quarter-degree fill);"
         f" clean 240 and all bent runs ITER {ITER}, alpha {AMIN:g}..{AMAX:g}.", ""]
    L.append("== Lip moment at Cl 0.25 vs Re (Ncrit 5, 160 panels); (fraction of inviscid lip moment) ==")
    for s in SECTIONS:
        for lip in lips:
            vals = []
            for re in RES:
                c = get(s, lip, re, 5, 160)
                vals.append(f"{re // 1000}k:{fmt(c and c.get('lip_dcm'))}({fmt(c and c.get('fraction_of_inviscid'), 2)})")
            inv = get(s, lip, 100000, 5, 160)
            L.append(f"{s} {lip:>5} inv {fmt(inv and inv.get('inviscid_lip_dcm'))} | " + " ".join(vals))
    L += ["", "== Re trend: mean fraction of inviscid lip moment retained at Cl 0.25 (both sections, Ncrit 5, 160 panels) =="]
    for lip in lips:
        row = []
        for re in RES:
            v = [get(s, lip, re, 5, 160) for s in SECTIONS]
            v = [c["fraction_of_inviscid"] for c in v if ok(c) and c.get("fraction_of_inviscid") is not None]
            row.append(f"{re // 1000}k:{sum(v) / len(v):.2f}" if v else f"{re // 1000}k:--")
        L.append(f"{lip:>5} " + " ".join(row))
    L += ["", "== Ncrit 9 vs Ncrit 5 and 240 vs 160 panels: lip moment at Cl 0.25, per lip (over Re x section) =="]
    for lip in lips:
        pn, pp = [], []
        for s in SECTIONS:
            for re in RES:
                a, b = get(s, lip, re, 5, 160), get(s, lip, re, 9, 160)
                if ok(a) and ok(b):
                    pn.append((a["lip_dcm"], b["lip_dcm"]))
                c = get(s, lip, re, 5, 240)
                if ok(a) and ok(c):
                    pp.append((a["lip_dcm"], c["lip_dcm"]))
        L.append(f"{lip:>5} Ncrit9-Ncrit5: {range_stats(pn)}")
        L.append(f"{'':5} 240-160 (Ncrit5): {range_stats(pp)}")
    L += ["", "== Cells with weak or unusable lip authority / data =="]
    bad = [c for c in cov if c["re"] != "inviscid" and c["lip"] != "clean" and c["returned"] < 20]
    L.append(f"Bent viscous polars returning < 20 of 29 points ({len(bad)}):")
    for c in bad:
        L.append(f"  {c['section']} {c['lip']} Re{c['re']} N{c['ncrit']} p{c['panels']}: {c['returned']}/29")
    cl_bad = [c for c in cov if c["re"] != "inviscid" and c["lip"] == "clean" and c["returned"] < 20]
    L.append(f"Clean 240-panel polars returning < 20 of 29 ({len(cl_bad)}): " + "; ".join(
        f"{c['section']} Re{c['re']} N{c['ncrit']}: {c['returned']}" for c in cl_bad))
    weak = [(s, lip, re) for s in SECTIONS for lip in lips for re in RES
            if ok(get(s, lip, re, 5, 160)) and get(s, lip, re, 5, 160).get("fraction_of_inviscid") is not None
            and get(s, lip, re, 5, 160)["fraction_of_inviscid"] < 0.10]
    L.append(f"Lip moment < 10% of inviscid at Cl 0.25 (Ncrit 5, 160 panels) ({len(weak)} of {len(SECTIONS) * len(lips) * len(RES)} cells): "
             + "; ".join(f"{s[-4:]} {lip} {re // 1000}k" for s, lip, re in weak))
    nm = [c for c in cells if abs(c["cl"] - 0.25) < 1e-9 and c.get("lip_dcm") is None]
    L.append(f"Cells with no Cl 0.25 match (matched-Cl lip moment missing) ({len(nm)} of {len(cells) // len(TARGET_CL)}):")
    from collections import Counter
    cnt = Counter((c["section"][-4:], c["lip"], c["ncrit"], c["panels"]) for c in nm)
    for k, v in sorted(cnt.items()):
        L.append(f"  {k[0]} {k[1]} N{k[2]} p{k[3]}: Re " + ",".join(f"{c['re'] // 1000}k" for c in nm
                 if (c['section'][-4:], c['lip'], c['ncrit'], c['panels']) == k))
    run_bad = [c for c in cov if c["exit"] == "timeout" or c["runaway_killed"] == "True" or int(c["nan_lines"]) > 0]
    L.append(f"XFOIL runs that timed out (300 s), were killed for runaway output, or logged NaN ({len(run_bad)}):")
    for c in run_bad:
        L.append(f"  {c['section']} {c['lip']} Re{c['re']} N{c['ncrit']} p{c['panels']}: exit={c['exit']} runaway={c['runaway_killed']} "
                 f"nan_lines={c['nan_lines']} returned={c['returned']}")
    (out / "summary.txt").write_text("\n".join(L) + "\n")
    print("\n".join(L))


def stage_summary(out: Path):
    write_summary(out, load_cells(out), load_cov(out))


def main():
    out = OUT
    out.mkdir(parents=True, exist_ok=True)
    st = sys.argv[1:] or ["gate", "run", "tables"]
    t0 = time.time()
    if "gate" in st:
        stage_gate(out)
    if "run" in st or "tables" in st:
        res = stage_run(out)
        stage_tables(out, res)
    if "run" in st or "tables" in st or "summary" in st:
        stage_summary(out)
    print(f"elapsed {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
