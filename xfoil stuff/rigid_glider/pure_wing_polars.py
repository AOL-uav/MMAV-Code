"""Part A: clean symmetric-section XFOIL polar database (pure flying wing study).

Run:  py -B rigid_glider/pure_wing_polars.py [repro|inviscid|viscous|tables|all]
Writes only under results/runs/rigid_glider/pure_wing/xfoil*/ and database files.
NACA 0004 is a thin symmetric PROXY, not a flat plate.
Missing / unconverged points stay missing: nothing is filled or extrapolated.
Pass 1 = alpha 0->10 and 0->-2 at 0.5 deg; pass 2 (only where points are
missing) = 0.25 deg steps with more iterations; pass-2 points are ADDED only
where pass 1 has no point at that alpha.
"""

from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import reflex_xfoil as rx   # imported for coordinates() only; nothing is written to its folders

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/pure_wing"
XOUT = OUT / "xfoil"
EXE = ROOT / "tools/xfoil/xfoil.exe"
CONFIG = Path(__file__).with_name("pure_wing_config.json")
DATABASE = OUT / "polar_database.csv"
FIELDS = ["section", "ncrit", "re", "alpha", "cl", "cd", "cdp", "cm", "xtr_top", "xtr_bot", "pass"]


def cfg() -> dict:
    return json.loads(CONFIG.read_text())


def parse_polar(path: Path, amin=-2.0, amax=10.0) -> list[dict]:
    if not path.is_file():
        return []
    rows = {}
    for line in path.read_text(errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 7:
            try:
                a, cl, cd, cdp, cm, t, b = map(float, parts)
            except ValueError:
                continue
            if amin - 1e-9 <= a <= amax + 1e-9 and cd >= 0:
                rows[round(a, 4)] = dict(alpha=a, cl=cl, cd=cd, cdp=cdp, cm=cm, xtr_top=t, xtr_bot=b)
    return [rows[k] for k in sorted(rows)]


def xfoil_exec(folder: Path, script: str, timeout: float, tag: str) -> tuple[str, str]:
    (folder / f"commands_{tag}.in").write_text(script, encoding="ascii")
    try:
        proc = subprocess.run([str(EXE)], input=script, text=True, cwd=folder, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=timeout)
        log, code = proc.stdout, str(proc.returncode)
    except subprocess.TimeoutExpired as err:
        out = err.stdout
        log = out.decode(errors="replace") if isinstance(out, bytes) else (out or "")
        code = "timeout"
    (folder / f"xfoil_{tag}.log").write_text(log, encoding="utf-8", errors="replace")
    return code, log


def write_foil(folder: Path, section: str, label: str) -> None:
    t = int(section[-2:]) / 100
    pts = rx.coordinates(t, 0.10, 0.0)     # chord_fraction/angle 0 => unrotated section
    (folder / "foil.dat").write_text(label + "\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in pts) + "\n",
                                     encoding="ascii")


def script_sweeps(re: int, ncrit: int, sweeps: list[tuple[float, float, float]], iters: int,
                  tag: str) -> str:
    s = f"PLOP\nG\n\nLOAD foil.dat\nPANE\nOPER\nVISC {re}\nVPAR\nN {ncrit}\n\nITER {iters}\n"
    for k, (a0, a1, da) in enumerate(sweeps):
        s += f"PACC\npolar_{tag}{k}.dat\n\nALFA {a0:g}\nINIT\nALFA {a0:g}\nASEQ {a0:g} {a1:g} {da:g}\nPACC\n"
    return s + "\nQUIT\n"


def run_viscous(section: str, re: int, ncrit: int, c: dict) -> dict:
    label = f"{section}_Re{re}_N{ncrit}"
    folder = XOUT / label
    folder.mkdir(parents=True, exist_ok=True)
    for f in folder.glob("polar_*.dat"):
        f.unlink()
    write_foil(folder, section, label)
    amin, amax, da = c["alpha_min"], c["alpha_max"], c["alpha_step"]
    s1 = script_sweeps(re, ncrit, [(0.0, amax, da), (0.0, amin, -da)], c["xfoil_iter"], "p1_")
    code1, _ = xfoil_exec(folder, s1, c["xfoil_timeout_s"], "p1")
    rows = {}
    for k in range(2):
        for r in parse_polar(folder / f"polar_p1_{k}.dat", amin, amax):
            rows[round(r["alpha"], 4)] = {**r, "pass": 1}
    grid = [round(amin + i * da, 4) for i in range(int(round((amax - amin) / da)) + 1)]
    missing1 = [a for a in grid if a not in rows]
    code2 = ""
    if missing1:
        s2 = script_sweeps(re, ncrit, [(0.0, amax, da / 2), (0.0, amin, -da / 2)], c["xfoil_iter_pass2"], "p2_")
        code2, _ = xfoil_exec(folder, s2, c["xfoil_timeout_s"], "p2")
        for k in range(2):
            for r in parse_polar(folder / f"polar_p2_{k}.dat", amin, amax):
                key = round(r["alpha"], 4)
                if key not in rows:
                    rows[key] = {**r, "pass": 2}
    out = [{"section": section, "ncrit": ncrit, "re": re, **rows[k]} for k in sorted(rows)]
    return dict(section=section, re=re, ncrit=ncrit, rows=out, requested=len(grid),
                returned_grid=sum(1 for a in grid if a in rows),
                returned_pass1=sum(1 for a in grid if a in rows and rows[a]["pass"] == 1),
                extra_quarter=sum(1 for k in rows if k not in grid), exit_p1=code1, exit_p2=code2)


def run_inviscid(section: str, c: dict) -> dict:
    label = f"{section}_inviscid"
    folder = XOUT / label
    folder.mkdir(parents=True, exist_ok=True)
    for f in folder.glob("polar_*.dat"):
        f.unlink()
    write_foil(folder, section, label)
    amin, amax, da = c["alpha_min"], c["alpha_max"], c["alpha_step"]
    s = (f"PLOP\nG\n\nLOAD foil.dat\nPANE\nOPER\nPACC\npolar_inv.dat\n\nALFA {amin:g}\n"
         f"ASEQ {amin:g} {amax:g} {da:g}\nPACC\n\nQUIT\n")
    xfoil_exec(folder, s, c["xfoil_timeout_s"], "inv")
    rows = [{"section": section, "ncrit": 0, "re": 0, **r, "pass": 1}
            for r in parse_polar(folder / "polar_inv.dat", amin, amax)]
    return dict(section=section, rows=rows)


def write_database(results: list[dict], inviscid: list[dict]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    with DATABASE.open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=FIELDS)
        w.writeheader()
        for r in inviscid:
            w.writerows(r["rows"])
        for r in sorted(results, key=lambda r: (r["section"], r["ncrit"], r["re"])):
            w.writerows(r["rows"])


def read_database(path: Path = DATABASE) -> tuple[dict, dict]:
    """-> (viscous[(section, ncrit, re)] = rows sorted by alpha, inviscid[section] = rows)."""
    visc, inv = {}, {}
    with path.open(encoding="utf-8") as s:
        for row in csv.DictReader(s):
            rec = {k: (row[k] if k == "section" else float(row[k])) for k in FIELDS}
            if rec["ncrit"] == 0:
                inv.setdefault(rec["section"], []).append(rec)
            else:
                visc.setdefault((rec["section"], int(rec["ncrit"]), int(rec["re"])), []).append(rec)
    for d in (visc, inv):
        for k in d:
            d[k].sort(key=lambda r: r["alpha"])
    return visc, inv


def coverage_table(results: list[dict]) -> None:
    with (OUT / "polar_coverage.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.writer(s)
        w.writerow(["section", "ncrit", "re", "requested", "returned", "returned_pass1_only",
                    "extra_quarter_deg_points", "missing_alphas", "exit_pass1", "exit_pass2"])
        for r in sorted(results, key=lambda r: (r["section"], r["ncrit"], r["re"])):
            have = {round(x["alpha"], 4) for x in r["rows"]}
            c = cfg()
            grid = [round(c["alpha_min"] + i * c["alpha_step"], 4)
                    for i in range(int(round((c["alpha_max"] - c["alpha_min"]) / c["alpha_step"])) + 1)]
            w.writerow([r["section"], r["ncrit"], r["re"], r["requested"], r["returned_grid"],
                        r["returned_pass1"], r["extra_quarter"],
                        " ".join(f"{a:g}" for a in grid if a not in have), r["exit_p1"], r["exit_p2"]])
    # summary by Re
    by = {}
    for r in results:
        k = (r["re"], r["ncrit"])
        a = by.setdefault(k, [0, 0])
        a[0] += r["returned_grid"]
        a[1] += r["requested"]
    (OUT / "polar_coverage_by_re.json").write_text(json.dumps(
        {f"Re{k[0]}_N{k[1]}": {"returned": v[0], "requested": v[1], "fraction": round(v[0] / v[1], 3)}
         for k, v in sorted(by.items())}, indent=1))


# ------------------------------------------------------------------ repro gate
def repro_gate(c: dict) -> dict:
    """Re-run the EXACT reflex_xfoil.run script for NACA_0008 Re80000 a0 into pure_wing/xfoil_repro
    and diff against the stored polar (reflex/xfoil/summary.json)."""
    folder = OUT / "xfoil_repro"
    folder.mkdir(parents=True, exist_ok=True)
    label = "NACA_0008_Re80000_c10_a0"
    write_foil(folder, "NACA_0008", label)
    script = ("PLOP\nG\n\nLOAD foil.dat\nPANE\nOPER\nVISC 80000\nVPAR\nN 5\n\nITER 150\n"
              "PACC\npolar.dat\n\nALFA 0\nINIT\nALFA -3\nASEQ -3 8 0.5\nPACC\n\nQUIT\n")
    xfoil_exec(folder, script, 90, "repro")
    (folder / "polar.dat").exists()
    # the script wrote polar.dat in cwd
    mine = rx.read_polar(folder / "polar.dat")
    stored_all = json.loads((ROOT / "results/runs/rigid_glider/reflex/xfoil/summary.json").read_text())
    stored = [r for r in stored_all if r["section"] == "NACA_0008" and r["reynolds"] == 80000
              and r["angle_deg"] == 0.0][0]["rows"]
    sm = {round(r["alpha_deg"], 4): r for r in stored}
    mm = {round(r["alpha_deg"], 4): r for r in mine}
    diffs = {"cl": 0.0, "cd": 0.0, "cdp": 0.0, "cm": 0.0}
    for a in sorted(set(sm) & set(mm)):
        for k in diffs:
            diffs[k] = max(diffs[k], abs(sm[a][k] - mm[a][k]))
    res = dict(stored_points=len(sm), repro_points=len(mm), common_points=len(set(sm) & set(mm)),
               only_stored=sorted(set(sm) - set(mm)), only_repro=sorted(set(mm) - set(sm)),
               max_abs_diff=diffs, note="stored polar generated by reflex_xfoil.run (chord 0.10, angle 0 = unrotated 0008)")
    # also compare the database pass-1 polar (different sweep order/alpha start) on common alphas
    return res


def compare_database_to_stored(visc: dict) -> dict:
    stored_all = json.loads((ROOT / "results/runs/rigid_glider/reflex/xfoil/summary.json").read_text())
    stored = [r for r in stored_all if r["section"] == "NACA_0008" and r["reynolds"] == 80000
              and r["angle_deg"] == 0.0][0]["rows"]
    sm = {round(r["alpha_deg"], 4): r for r in stored}
    rows = visc.get(("NACA_0008", 5, 80000), [])
    mm = {round(r["alpha"], 4): r for r in rows}
    diffs = {"cl": 0.0, "cd": 0.0, "cdp": 0.0, "cm": 0.0}
    common = sorted(set(sm) & set(mm))
    for a in common:
        for k in diffs:
            diffs[k] = max(diffs[k], abs(sm[a][k if k != "alpha" else "alpha_deg"] - mm[a][k]))
    return dict(common_points=len(common), max_abs_diff=diffs,
                note="database sweep (0->10, 0->-2, INIT at 0) vs stored sweep (-3->8); alpha overlap only")


def write_jaggedness(visc: dict) -> None:
    st = {}
    for (sec, nc, re), rows in visc.items():
        r = [x for x in rows if x["alpha"] >= 0 and abs(x["alpha"] * 2 - round(x["alpha"] * 2)) < 1e-6]
        jump = any(b["cl"] - a["cl"] > 0.1 and b["alpha"] - a["alpha"] <= 0.51 for a, b in zip(r, r[1:]))
        nonm = any(b["cl"] < a["cl"] - 1e-3 and b["alpha"] <= 8 for a, b in zip(r, r[1:]))
        cmmax = max((x["cm"] for x in r if x["cl"] < 0.6), default=None)
        d = st.setdefault(str(re), dict(polars=0, with_cl_jump_gt_0p1_per_half_deg=0, with_nonmonotone_cl_alpha_le_8=0, max_cm_cl_lt_0p6=0.0))
        d["polars"] += 1
        d["with_cl_jump_gt_0p1_per_half_deg"] += int(jump)
        d["with_nonmonotone_cl_alpha_le_8"] += int(nonm)
        d["max_cm_cl_lt_0p6"] = max(d["max_cm_cl_lt_0p6"], cmmax or 0.0)
    (OUT / "polar_jaggedness_by_re.json").write_text(json.dumps(st, indent=1))


def main() -> None:
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    c = cfg()
    OUT.mkdir(parents=True, exist_ok=True)
    if what in ("repro", "all"):
        r = repro_gate(c)
        (OUT / "repro_gate.json").write_text(json.dumps(r, indent=1))
        print("REPRO", json.dumps(r))
    if what in ("inviscid", "all"):
        inv = [run_inviscid(s, c) for s in c["sections"]]
        (OUT / "xfoil_inviscid.json").write_text(json.dumps(inv))
        print("inviscid rows", {r["section"]: len(r["rows"]) for r in inv})
    if what in ("viscous", "all"):
        jobs = [(s, re, n) for s in c["sections"] for n in c["ncrit"] for re in c["reynolds"]]
        with ThreadPoolExecutor(max_workers=c["xfoil_workers"]) as pool:
            results = list(pool.map(lambda j: run_viscous(*j, c), jobs))
        (OUT / "xfoil_viscous_raw.json").write_text(json.dumps(
            [{k: v for k, v in r.items()} for r in results]))
        print("viscous polars", len(results), "points", sum(r["returned_grid"] for r in results),
              "of", sum(r["requested"] for r in results))
    if what in ("tables", "all"):
        results = json.loads((OUT / "xfoil_viscous_raw.json").read_text())
        inv = json.loads((OUT / "xfoil_inviscid.json").read_text())
        write_database(results, inv)
        coverage_table(results)
        visc, _ = read_database()
        g = compare_database_to_stored(visc)
        (OUT / "repro_gate_database_vs_stored.json").write_text(json.dumps(g, indent=1))
        print("database written", DATABASE, g)
        write_jaggedness(visc)


if __name__ == "__main__":
    main()
