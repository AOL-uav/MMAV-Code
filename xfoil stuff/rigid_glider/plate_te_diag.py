"""DIAGNOSIS ONLY: trailing-edge closure / panelling variants of the 4.3 % plate through the existing XFOIL runner.
Writes ONLY under results/runs/rigid_glider/plate_te_diag/.  Not fed to any model; no CG windows; no variant is declared a plate geometry.
Same runner (reflex_small_polars.build_script / merge_sweeps, reflex_ext_gaps.run_xfoil), alpha -4..10 step 0.5, 160 panels, ITER 400.
Cases: asis; trunc (TE ellipse cut off at a finite blunt base 0.2/0.5/1.0 % chord, x rescaled to chord 1); wedge (last 0.5/1.0 % of
chord replaced by a straight sharp wedge to (1,0)); ppar (as-is contour, XFOIL PPAR TE/LE density ratio 0.6 / 1.0 instead of 0.15);
naca0006 control; start2 (as-is, sweeps start at alpha 2 instead of 0).   py -B rigid_glider/plate_te_diag.py
"""
import sys
sys.dont_write_bytecode = True
import csv
import math
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import reflex_small_polars as sp
import reflex_ext_gaps as g
import pure_wing_polars as pwp
import reflex_xfoil as rx
import plate_section as ps

OUT = g.ROOT / "results/runs/rigid_glider/plate_te_diag"
T = 0.043
H = T / 2
RES = [60000, 100000]
NC = [5, 9]
FLOOR = 15
DEFAULT_TE_LE = 0.15


def contour(xs, yf, closed=True):
    up = [(x, yf(x)) for x in reversed(xs)]
    lo = [(x, -yf(x)) for x in xs[1:]]
    return up + lo


def asis():
    return ps.coordinates(T)


def trunc(base):
    """TE ellipse cut where total thickness = base (chord fraction); open blunt base; x rescaled so chord = 1."""
    yb = base / 2
    x1 = 1 - ps.TE_RATIO * H
    xe = x1 + ps.TE_RATIO * H * math.sqrt(1 - (yb / H) ** 2)
    xs = [x for x in ps.upper_x_stations(T) if x < xe] + [xe]
    return [(x / xe, y) for x, y in contour(xs, lambda x: ps.half_thickness_at(x, T))]


def wedge(w):
    """Ellipse up to x = 1-w, then straight line to a sharp TE at (1,0)."""
    xw = 1 - w
    yw = ps.half_thickness_at(xw, T)
    xs = [x for x in ps.upper_x_stations(T) if x < xw] + [xw] + [xw + w * k / 6 for k in range(1, 7)]
    return contour(xs, lambda x: yw * (1 - x) / w if x > xw else ps.half_thickness_at(x, T))


def start_script(re, nc, a0):
    s = f"PLOP\nG\n\nLOAD foil.dat\nPANE\nPSAV paneled.dat\nOPER\nVISC {re}\nVPAR\nN {nc}\n\nITER {sp.ITER}\n"
    for k, (x1, da) in enumerate([(sp.AMAX, sp.DA), (sp.AMIN, -sp.DA)]):
        s += f"PACC\npolar_p1_{k}.dat\n\nALFA {a0:g}\nINIT\nALFA {a0:g}\nASEQ {a0:g} {x1:g} {da:g}\nPACC\n"
    return s + "\nQUIT\n"


def ppar_script(re, nc, ratio):
    s = sp.build_script(re, nc, 160)
    return s.replace("PANE\nPSAV", f"PPAR\nN 160\nT {ratio:g}\n\n\nPANE\nPSAV", 1)


def cases():
    c = [("asis", asis(), "sweep from alpha 0, as-is contour", None)]
    for b in (0.002, 0.005, 0.010):
        c.append((f"trunc{b * 100:g}pct", trunc(b), f"TE ellipse truncated, blunt base {b * 100:g}% chord", None))
    for w in (0.005, 0.010):
        c.append((f"wedge{w * 100:g}pct", wedge(w), f"last {w * 100:g}% chord straight sharp wedge", None))
    for r in (0.6, 1.0):
        c.append((f"ppar_te{r:g}", asis(), f"as-is contour, PPAR TE/LE density ratio {r:g} (default {DEFAULT_TE_LE})", ("ppar", r)))
    c.append(("start2", asis(), "as-is contour, sweeps start at alpha 2", ("start", 2)))
    c.append(("naca0006", rx.coordinates(0.06, 0.10, 0.0), "NACA 0006 control (same runner)", None))
    return c


def run(case, re, nc):
    name, pts, desc, mod = case
    f = OUT / f"{name}_Re{re}_N{nc}"
    f.mkdir(parents=True, exist_ok=True)
    (f / "foil.dat").write_text(f"{name}\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in pts) + "\n", encoding="ascii")
    if mod and mod[0] == "ppar":
        script = ppar_script(re, nc, mod[1])
    elif mod and mod[0] == "start":
        script = start_script(re, nc, mod[1])
    else:
        script = sp.build_script(re, nc, 160)
    r = g.run_xfoil(f, script, sp.TIMEOUT)
    rows = sp.merge_sweeps([pwp.parse_polar(f / f"polar_p1_{k}.dat", sp.AMIN, sp.AMAX) for k in range(2)])
    return dict(case=name, description=desc, re=re, ncrit=nc, rows=rows, exit=r["code"], runaway=r.get("runaway", False),
                nan=r["log"].count("NaN"), first_fail=next((l.strip() for l in r["log"].splitlines() if "Convergence failed" in l), ""))


def cov_row(r):
    rows = r["rows"]
    n = len(rows)
    d = dict(case=r["case"], description=r["description"], re=r["re"], ncrit=r["ncrit"], n_converged=n, n_requested=29,
             alpha_min=min((x["alpha"] for x in rows), default=""), alpha_max=max((x["alpha"] for x in rows), default=""),
             meets_floor_15=int(n >= FLOOR))
    a0 = next((x for x in rows if abs(x["alpha"]) < 1e-9), None)
    d["cl_at_alpha0"] = "" if a0 is None else round(a0["cl"], 4)
    d["max_cl"] = max((x["cl"] for x in rows), default="") if n >= FLOOR else ""
    for t in (0.1, 0.2, 0.3, 0.4):
        p = rx.at_cl(sp.to_cl_rows(rows), t) if n >= FLOOR else None
        d[f"cm_at_cl_{t:g}"] = "" if p is None else round(p["cm"], 4)
    d.update(exit=r["exit"], runaway_killed=r["runaway"], nan_lines=r["nan"], first_failure=r["first_fail"])
    return d


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [(c, re, nc) for c in cases() for re in RES for nc in NC]
    with ThreadPoolExecutor(max_workers=4) as pool:
        res = list(pool.map(lambda j: run(*j), jobs))
    cov = [cov_row(r) for r in res]
    g.write_csv(OUT / "coverage.csv", cov)
    L = ["TE DIAGNOSIS (diagnosis only; nothing here enters the coupled model; no variant is a proposed plate geometry)",
         f"4.3% plate, Re 60k/100k, Ncrit 5/9, 160 panels, alpha -4..10 step 0.5, existing runner.  Floor for quoting cm/cl_max: {FLOOR} of 29 points.", ""]
    for r, c in zip(res, cov):
        rows = r["rows"]
        pre = [x for x in rows if 0 < x["cl"] <= 0.5]
        rng = f"cm(0<cl<=0.5) {min(x['cm'] for x in pre):+.4f}..{max(x['cm'] for x in pre):+.4f}" if (pre and c["n_converged"] >= FLOOR) else ""
        L.append(f"{c['case']:14s} Re{c['re']//1000:>3d}k N{c['ncrit']}: {c['n_converged']:2d}/29 alpha {c['alpha_min']}..{c['alpha_max']} "
                 f"cl(a=0)={c['cl_at_alpha0']} clmax={c['max_cl']} {rng}")
    (OUT / "diag_summary.txt").write_text("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
