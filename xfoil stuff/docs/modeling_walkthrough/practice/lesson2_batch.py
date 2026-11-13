"""Lesson 2 and the checks section: writes small variants of the F45 deck into variants/ and runs them with run_case.py.
    py -B lesson2_batch.py            (about a minute; prints the tables quoted in index.html and writes lesson2_results.txt)"""
import re
import sys
import time
from pathlib import Path

import run_case as rc

HERE = Path(__file__).resolve().parent
V = HERE / "variants"
V.mkdir(exist_ok=True)
BASE = (HERE / "f45_bare.avl").read_text()
FINS = (HERE / "f45_fins.avl").read_text()
OUT = []


def say(s=""):
    print(s)
    OUT.append(s)


def deck(name, text):
    (V / f"{name}.avl").write_text(text.replace("f45_bare", name, 1).replace("f45_fins", name, 1), encoding="ascii")
    return V / f"{name}.avl"


def row(label, o, keys=("alpha", "CL", "Xnp", "Cm", "Clb", "Cnb", "e")):
    return f"{label:28s}" + "".join(f"{k}={o[k]:>10.5f}  " if o.get(k) is not None else f"{k}=      n/a  " for k in keys)


# ---- exercise 1: sweep (tip leading-edge x), everything else unchanged.  Header Xref/Cref are NOT recomputed (look at what that does to the margin)
say("EXERCISE 1: change the sweep (tip SECTION Xle), CL = 0.3, header unchanged")
for tipx in (0.020, 0.067547717, 0.110):
    t = BASE.replace("0.067547717 0.125000000 0.0 0.037037037 0.0", f"{tipx:.9f} 0.125000000 0.0 0.037037037 0.0")
    d = deck(f"sweep_x{int(round(tipx * 1000)):03d}", t)
    o = rc.run(d, cl=0.3)
    deg = __import__("math").degrees(__import__("math").atan2(tipx + 0.25 * 0.037037037 - 0.25 * 0.074074074, 0.125))
    say(row(f"tip Xle {tipx:.4f} (qc sweep {deg:5.1f} deg)", o) + f"  CDind={o['CDind']:.5f}")

# ---- exercise 2: tip fins
say("\nEXERCISE 2: tip fins off / on, CL = 0.3")
d0, d1 = deck("bare", BASE), deck("fins", FINS)
for lab, d in (("bare wing", d0), ("with tip fins (TipFin)", d1)):
    say(row(lab, rc.run(d, cl=0.3)))

# ---- exercise 3: a bent tail.  The tail is a second SURFACE behind the wing; "bend" = Ainc of its sections = -bend
tail = """SURFACE
Tail
6 1.0 8 -2.0
YDUPLICATE
0.0
SECTION
0.105000000 0.000000000 0.0 0.030000000 {ainc:.6f}
NACA
0000
SECTION
0.105000000 0.030000000 0.0 0.030000000 {ainc:.6f}
NACA
0000
"""
say("\nEXERCISE 3: tail panel behind the wing, section Ainc = -bend (trailing edge UP), alpha = 0")
for bend in (0, 2, 5, 10):
    d = deck(f"tail_bend{bend:02d}", BASE + tail.format(ainc=-bend))
    o = rc.run(d, alpha=0.0)
    say(row(f"bend {bend:2d} deg", o, ("alpha", "CL", "Cm", "Xnp", "Cma", "CLa")))
say("(CM at zero lift is the 'CM0' of the project; it is zero for a symmetric wing with no bend.)")

# ---- check 1: panel counts on the F45 wing
say("\nCHECK 1: panel counts (chordwise x spanwise per half wing), F45 bare wing, CL = 0.3")
for nc, ns in ((6, 8), (10, 16), (20, 32), (30, 64), (40, 96)):
    t = BASE.replace("20 1.0 32 -2.0", f"{nc} 1.0 {ns} -2.0")
    d = deck(f"panels_{nc}x{ns}", t)
    t0 = time.time()
    o = rc.run(d, cl=0.3)
    if o["alpha"] is None:
        say(f"{nc:3d} x {ns:3d}  vortices={2 * nc * ns:6d}  AVL FAILED; last lines of its screen: " + " | ".join(l.strip() for l in o["_raw"].strip().splitlines()[-4:]))
        continue
    say(f"{nc:3d} x {ns:3d}  vortices={2 * nc * ns:6d}  alpha={o['alpha']:.4f}  Xnp={o['Xnp']:.6f}  CLa={o['CLa']:.4f}  e={o['e']:.4f}  Cnb={o['Cnb']:+.5f}  Clb={o['Clb']:+.5f}  ({time.time() - t0:.1f} s)")

# ---- check 2: known case, rectangular wing AR 6, span 0.25 m
c = 0.25 / 6
rect = f"""rect_AR6
0.0
0 0 0.0
{0.25 * c:.9f} {c:.9f} 0.250000000
{0.25 * c:.9f} 0.0 0.0
SURFACE
Wing
20 1.0 32 -2.0
YDUPLICATE
0.0
SECTION
0.0 0.0 0.0 {c:.9f} 0.0
NACA
0000
SECTION
0.0 0.125 0.0 {c:.9f} 0.0
NACA
0000
"""
say("\nCHECK 2: known case, rectangular wing, span 0.25 m, chord %.6f m (AR 6); the neutral point of a plain wing should sit near the quarter chord" % c)
o = rc.run(deck("rect_AR6", rect), cl=0.3)
say(f"Xnp = {o['Xnp']:.6f} m = {o['Xnp'] / c:.4f} chords from the leading edge  (quarter chord = {0.25 * c:.6f} m);  CLa = {o['CLa']:.4f} per rad")
say("   for comparison, lifting-line theory CLa = 2*pi*AR/(AR+2) = %.4f per rad" % (2 * 3.141592653589793 * 6 / 8))

# ---- check 3: mistakes
say("\nCHECK 3: mistakes and how each shows up (F45 deck, CL = 0.3 requested)")
o = rc.run(d0, cl=0.3)
say(row("correct deck", o, ("CL", "alpha", "Cm", "Xnp", "Clb", "Cnb")))
t = BASE.replace("YDUPLICATE\n0.0\n", "")
o = rc.run(deck("no_ydup", t), cl=0.3)
say(row("YDUPLICATE removed", o, ("CL", "alpha", "Cm", "Xnp", "Clb", "Cnb")) + "  (AVL now models ONE half wing; header areas still full)")
t = BASE.replace("0.013888889 0.057613169 0.250000000", "0.013888889 0.115226338 0.250000000")
o = rc.run(deck("cref_x2", t), cl=0.3)
say(row("Cref doubled in header", o, ("CL", "alpha", "Cm", "Xnp", "Clb", "Cnb")))
t = BASE.replace("0.013888889 0.057613169 0.250000000", "0.006944444 0.057613169 0.250000000")
o = rc.run(deck("sref_half", t), cl=0.3)
say(row("Sref halved in header", o, ("CL", "alpha", "Cm", "Xnp", "Clb", "Cnb")))
t = BASE.replace("0.030021207 0.0 0.0", "0.300212070 0.0 0.0")
o = rc.run(deck("xref_x10", t), cl=0.3)
say(row("Xref 10x too far aft", o, ("CL", "alpha", "Cm", "Xnp", "Clb", "Cnb")))
t = BASE.replace("0.074074074", "74.074074").replace("0.067547717", "67.547717").replace("0.037037037", "37.037037").replace("0.125000000", "125.0")
t = t.replace("0.013888889 0.057613169 0.250000000", "0.013888889 0.057613169 0.250000000")
o = rc.run(deck("mm_sections_m_header", t), cl=0.3)
say(row("sections in mm, header in m", o, ("CL", "alpha", "Cm", "Xnp", "Clb", "Cnb")) + "  (sections 1000x bigger than the header's Sref/Cref/Bref)")
(HERE / "lesson2_results.txt").write_text("\n".join(OUT) + "\n", encoding="utf-8")
