#!/usr/bin/env python3
"""AVL 3.52 check of the rectangular AR 3 flat planform (chord 1, span 3, no sweep, no taper, zero thickness, M = 0).
Run: py -B avl_check.py     (stdlib only). Writes avl/*.avl, avl/*.out, avl_results.csv; prints a table.
Two panelings: coarse (8 chordwise x 12 spanwise per half) and fine (24 x 40 per half), cosine spacing both ways.
Moment reference (0.25, 0, 0) as in the CFD/measurement; Sref = 3, Cref = 1, Bref = 3 (full wing).
CM: AVL 'Cmtot' is about Xref and nose-up positive (AVL convention), same sign as the CFD/measurement CM."""
import csv, os, re, subprocess, math
HERE = os.path.dirname(os.path.abspath(__file__))
AVL = os.path.normpath(os.path.join(HERE, "..", "..", "..", "tools", "avl", "avl352.exe"))
OUT = os.path.join(HERE, "avl"); os.makedirs(OUT, exist_ok=True)
ALPHAS = [2, 4, 6, 8]
rows = []
for tag, nc, ns in (("coarse", 8, 12), ("fine", 24, 40)):
    avl = os.path.join(OUT, f"rect_ar3_{tag}.avl")
    open(avl, "w").write(f"""Rectangular AR3 flat planform, {tag} paneling
0.0
0 0 0
3.0  1.0  3.0
0.25 0.0 0.0
0.0
SURFACE
Wing
{nc} 1.0 {ns} 1.0
YDUPLICATE
0.0
SECTION
0.0 0.0 0.0 1.0 0.0
NACA
0000
SECTION
0.0 1.5 0.0 1.0 0.0
NACA
0000
""")
    cmds = ["load " + avl, "oper"]
    for a in ALPHAS:
        cmds += [f"a a {a}", "x"]
    cmds += ["st", "", "", "quit"]
    p = subprocess.run([AVL], input="\n".join(cmds) + "\n", capture_output=True, text=True, cwd=OUT, timeout=300)
    open(os.path.join(OUT, f"{tag}.out"), "w").write(p.stdout)
    blocks = re.split(r"Run case:", p.stdout)[1:]
    for a, blk in zip(ALPHAS, blocks):
        g = lambda key: float(re.search(key + r"\s*=\s*([-0-9.Ee+]+)", blk).group(1))
        assert abs(g("Alpha") - a) < 1e-3, (g("Alpha"), a)
        rows.append([tag, nc, ns, a, g("CLtot"), g("CDind"), g("Cmtot"), g("e")])
    st = p.stdout[p.stdout.rindex("Stability-axis derivatives"):] if "Stability-axis derivatives" in p.stdout else p.stdout
    xnp = float(re.search(r"Xnp\s*=\s*([-0-9.Ee+]+)", st).group(1))
    clal = float(re.search(r"CLa\s*=\s*([-0-9.Ee+]+)", st).group(1))
    rows.append([tag, nc, ns, "slope", clal, "", "", xnp])
with open(os.path.join(HERE, "avl_results.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["paneling", "nchord", "nspan", "alpha_deg_or_slope", "CL_or_CLa_per_rad", "CDind", "Cm_c4", "e_or_Xnp"]); w.writerows(rows)
for r in rows: print(r)
