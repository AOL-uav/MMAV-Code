"""Record-keeping for plate_te_diag: re-reads the EXISTING polar files (no XFOIL).  Writes plate_te_diag/jumps.csv, keeps the original
coverage.csv as coverage_v1.csv and rewrites coverage.csv with cm_at_cl_* blanked where the bracketing points are >1 alpha step apart
or are the largest-cl-step (jump) pair.   py -B rigid_glider/plate_te_jumps.py"""
import sys
sys.dont_write_bytecode = True
import csv
import shutil

import reflex_small_polars as sp
import reflex_ext_gaps as g
import pure_wing_polars as pwp

D = g.ROOT / "results/runs/rigid_glider/plate_te_diag"
INV = g.ROOT / "results/runs/rigid_glider/plate/polars/raw/PLATE_43_clean_inv_p160/polar_inv.dat"
inv = pwp.parse_polar(INV, sp.AMIN, sp.AMAX)


def inv_cl(a):
    for p, q in zip(inv, inv[1:]):
        if p["alpha"] <= a <= q["alpha"]:
            return p["cl"] + (a - p["alpha"]) / (q["alpha"] - p["alpha"]) * (q["cl"] - p["cl"])


def rows_of(folder):
    return sp.merge_sweeps([pwp.parse_polar(folder / f"polar_p1_{k}.dat", sp.AMIN, sp.AMAX) for k in range(2)])


def adj(rows):
    return [(a, b) for a, b in zip(rows, rows[1:])]


def jump_pair(rows):
    """largest |cl step| between adjacent points, restricted to 0 <= alpha (positive-lift side) and step <= 0.51 deg."""
    c = [(abs(b["cl"] - a["cl"]), a, b) for a, b in adj(rows) if a["alpha"] >= 0 and b["alpha"] - a["alpha"] <= 0.51]
    return max(c, key=lambda t: t[0]) if c else None


def at(rows, a0):
    return next((r for r in rows if abs(r["alpha"] - a0) < 1e-9), None)


def main():
    cov_path = D / "coverage.csv"
    if not (D / "coverage_v1.csv").exists():
        shutil.copy(cov_path, D / "coverage_v1.csv")
    old = list(csv.DictReader((D / "coverage_v1.csv").open(encoding="utf-8")))
    jr, new = [], []
    for c in old:
        f = D / f"{c['case']}_Re{c['re']}_N{c['ncrit']}"
        rows = rows_of(f)
        d = dict(case=c["case"], re=c["re"], ncrit=c["ncrit"], n_converged=len(rows))
        jp = jump_pair(rows) if len(rows) >= 2 else None
        if jp:
            _, a, b = jp
            d.update(alpha_lo=a["alpha"], cl_lo=round(a["cl"], 4), cm_lo=round(a["cm"], 4), xtr_top_lo=a["xtr_top"],
                     alpha_hi=b["alpha"], cl_hi=round(b["cl"], 4), cm_hi=round(b["cm"], 4), xtr_top_hi=b["xtr_top"],
                     cl_step=round(b["cl"] - a["cl"], 4), cm_step=round(b["cm"] - a["cm"], 4))
        for al in (2.0, 3.0):
            r = at(rows, al)
            d[f"visc_over_inv_cl_a{al:g}"] = "" if r is None else round(r["cl"] / inv_cl(al), 3)
        jr.append(d)
        # rewrite coverage cm columns
        flags = []
        for t in (0.1, 0.2, 0.3, 0.4):
            key = f"cm_at_cl_{t:g}"
            if c[key] == "":
                continue
            br = next(((a, b) for a, b in adj(rows) if min(a["cl"], b["cl"]) <= t <= max(a["cl"], b["cl"]) and abs(b["cl"] - a["cl"]) > 1e-6
                       and b["alpha"] - a["alpha"] <= 0.51), None)
            bad = br is None or (jp is not None and br[0] is jp[1] and br[1] is jp[2]) or (br[1]["alpha"] - br[0]["alpha"] > 0.51)
            if bad:
                c[key] = ""
                flags.append(f"{key} blanked: " + ("no bracket" if br is None else "straddles jump"))
        c["cm_flag"] = "; ".join(flags)
        new.append(c)
    g.write_csv(D / "jumps.csv", jr)
    g.write_csv(cov_path, new)
    for d in jr:
        if d["case"] in ("trunc0.5pct", "wedge1pct", "naca0006") and "cl_step" in d:
            print(d["case"], d["re"], d["ncrit"], d["alpha_lo"], d["cl_lo"], d["cm_lo"], d["xtr_top_lo"], "->", d["alpha_hi"], d["cl_hi"], d["cm_hi"], d["xtr_top_hi"],
                  "dcl", d["cl_step"], "dcm", d["cm_step"], "ratio2/3", d["visc_over_inv_cl_a2"], d["visc_over_inv_cl_a3"])


if __name__ == "__main__":
    main()
