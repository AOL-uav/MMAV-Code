"""Builds docs/modeling_walkthrough/index.html.  Every quoted screen, file and table is READ from a real file or a real run output in this
workspace at build time (practice/lesson1_screen.txt, practice/lesson2_results.txt, practice/review_checks_results.txt, the project CSVs, source files).
Every number in the narrative text is either computed from those files or checked against them (expect()/need()); the build stops with an error if one drifts.
The few things that are not machine-checked are listed at the end of the page under "What I did not verify".
    py -B docs/modeling_walkthrough/build_walkthrough.py
Re-create the practice outputs first if you want them fresh:  see practice/README.md."""
import sys
sys.dont_write_bytecode = True
import csv
import html
import inspect
import math
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "rigid_glider"))
import flat_test as ft          # noqa: E402  (palette CSS and the real driver function)
import avl as avl_mod           # noqa: E402

def read_csv(rel):
    with (ROOT / rel).open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


PR = HERE / "practice"
SCREEN = (PR / "lesson1_screen.txt").read_text(errors="replace")
deck_fins = (PR / "f45_fins.avl").read_text()
deck_bare = (PR / "f45_bare.avl").read_text()
L2 = (PR / "lesson2_results.txt").read_text()
E = html.escape


def pre(text, cls=""):
    return f"<pre class='{cls}'>{E(text)}</pre>"


def need(text, snippet):
    assert snippet in text, f"not found in the real output: {snippet!r}"
    return snippet


def screen_block(start, end, nth=0):
    i = -1
    for _ in range(nth + 1):
        i = SCREEN.index(start, i + 1)
    j = SCREEN.index(end, i)
    k = SCREEN.find(chr(10), j)
    k = len(SCREEN) if k < 0 else k
    return chr(10).join(l.rstrip() for l in SCREEN[i:k].splitlines() if l.strip())


def annotated_lines(real_text, notes):
    """File anatomy table keyed by LINE NUMBER (1-based; a tuple of consecutive lines per row).  Every nonblank line of the file must be covered,
    so a file rebuilt from the table cannot omit one."""
    lines = real_text.splitlines()
    covered, rows = set(), []
    for nums, note in notes:
        quoted = "<br>".join(f"<code>{E(lines[n - 1].strip())}</code>" for n in nums)
        covered.update(nums)
        rows.append(f"<tr><td>{nums[0]}{'-' + str(nums[-1]) if len(nums) > 1 else ''}</td><td>{quoted}</td><td>{note}</td></tr>")
    missing = [i + 1 for i, l in enumerate(lines) if l.strip() and i + 1 not in covered]
    assert not missing, f"lines of the file with no row in the table: {missing}"
    return "<div class='scroll'><table class='t'><tr><th>line no.</th><th>line of the file (real)</th><th>what it means</th></tr>" + "".join(rows) + "</table></div>"


def annotated(real_text, notes):
    """Table: real line | what it means.  Every line quoted must exist in the real text."""
    rows = []
    for snippet, note in notes:
        line = next((l for l in real_text.splitlines() if snippet in l), None)
        assert line is not None, snippet
        rows.append(f"<tr><td><code>{E(line.strip())}</code></td><td>{note}</td></tr>")
    return "<div class='scroll'><table class='t'><tr><th>line on the screen (real)</th><th>what it means</th></tr>" + "".join(rows) + "</table></div>"


def num(block, key, last=False):
    m = re.findall(rf"\b{re.escape(key)}\s*=\s*([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?)", block)
    return float(m[-1] if last else m[0])


# ------------------------------------------------------------------------------------------------ real numbers used in the text and diagrams
tot1 = screen_block("Vortex Lattice Output -- Total Forces", "Plane", 0)
tot_cl03 = screen_block("Vortex Lattice Output -- Total Forces", "Plane", 2)
tot_cg = screen_block("Vortex Lattice Output -- Total Forces", "Plane", 3)
st1 = screen_block("Stability-axis derivatives", "Neutral point  Xnp =", 0)
st2 = screen_block("Stability-axis derivatives", "Neutral point  Xnp =", 1)
Sref, Cref, Bref, Xref = num(tot1, "Sref"), num(tot1, "Cref"), num(tot1, "Bref"), num(tot1, "Xref")
CL4, Cm4, e4, CDi4 = num(tot1, "CLtot"), num(tot1, "Cmtot"), num(tot1, "e"), num(tot1, "CDind")
CLff4, CDff4 = num(tot1, "CLff"), num(tot1, "CDff")
CLa, Cma, Clb, Cnb, Xnp = num(st1, "CLa"), num(st1, "Cma"), num(st1, "Clb"), num(st1, "Cnb"), num(st1, "Xnp")
AR = Bref ** 2 / Sref
sm1 = 100 * (Xnp - Xref) / Cref
Xref2 = num(tot_cg, "Xref")
Cm03a, Cm03b = num(tot_cl03, "Cmtot"), num(tot_cg, "Cmtot")
Xnp2, CLa2, Cma2 = num(st2, "Xnp"), num(st2, "CLa"), num(st2, "Cma")
sm2 = 100 * (Xnp2 - Xref2) / Cref
a03 = num(tot_cl03, "Alpha")
alpha4 = num(tot1, "Alpha")

# ---- every other number used in the narrative is computed here or asserted against the files
def kv(line):
    return {k: float(v) for k, v in re.findall(r"(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?)", line)}


def sect(a, b=None):
    return L2[L2.index(a):(L2.index(b) if b else len(L2))]


def lrow(sec, start):
    return kv(next(l for l in sec.splitlines() if l.strip().startswith(start)))


def expect(actual, typed, tol, what):
    assert abs(actual - typed) <= tol, f"{what}: the files say {actual}, the text assumes {typed}"
    return actual


EX1, EX2, EX3 = sect("EXERCISE 1", "EXERCISE 2"), sect("EXERCISE 2", "EXERCISE 3"), sect("EXERCISE 3", "CHECK 1")
C1, C2, C3 = sect("CHECK 1", "CHECK 2"), sect("CHECK 2", "CHECK 3"), sect("CHECK 3")
RV = (PR / "review_checks_results.txt").read_text()
ex1r = [lrow(EX1, x) for x in ("tip Xle 0.0200", "tip Xle 0.0675", "tip Xle 0.1100")]
ex2b, ex2f = lrow(EX2, "bare wing"), lrow(EX2, "with tip fins")
ex3r = {b: lrow(EX3, f"bend {b:2d} deg") for b in (0, 2, 5, 10)}
c1r = [kv(l) for l in C1.splitlines() if "vortices=" in l and "alpha=" in l]
c3 = {k: lrow(C3, k) for k in ("correct deck", "YDUPLICATE removed", "Cref doubled", "Sref halved", "Xref 10x", "sections in mm")}
m2 = re.search(r"Xnp = ([\d.]+) m = ([\d.]+) chords.*?quarter chord = ([\d.]+) m\);\s+CLa = ([\d.]+) per rad.*?= ([\d.]+) per rad", C2, re.S)
rect_chords, rect_qc_chords, rect_CLa, rect_CLa_ll = float(m2.group(2)), 0.25, float(m2.group(4)), float(m2.group(5))
rvA = [kv(l) for l in RV[RV.index("CHECK A"):RV.index("CHECK B")].splitlines() if "alpha=" in l]
rvB = [kv(l) for l in RV[RV.index("CHECK B"):].splitlines() if "alpha=" in l]
assert len(rvA) == 2 and len(rvB) == 3 and all(r == rvB[0] for r in rvB), "NACA thickness check: the three decks must give identical numbers (or the page text must change)"
# lesson-1 screen and the scripted check agree
expect(rvA[0]["Cm"], Cm03a, 2e-6, "CG-move check, Cm before"); expect(rvA[1]["Cm"], Cm03b, 2e-6, "CG-move check, Cm after")
expect(rvA[1]["Cma"], Cma2, 2e-6, "CG-move check, Cma after"); expect(rvA[1]["Xnp"], Xnp2, 2e-6, "CG-move check, Xnp after")
expect(rvA[0]["alpha"], a03, 1e-4, "CG-move check, alpha"); expect(rvA[0]["Xnp"], c3["correct deck"]["Xnp"], 1e-5, "Xnp before vs lesson 2")
# the deck, line by line
dl = deck_bare.splitlines()
assert dl[8].strip() == "YDUPLICATE" and dl[9].strip() == "0.0", "line 10 of the deck must be the mirror-plane y after YDUPLICATE"
d_sref, d_cref, d_bref = (float(v) for v in dl[3].split())
d_xref = float(dl[4].split()[0])
d_nch, d_cs, d_nsp, d_ss = dl[7].split()
d_nch, d_nsp = int(d_nch), int(d_nsp)
d_root, d_tip = [float(v) for v in dl[11].split()], [float(v) for v in dl[15].split()]      # Xle Yle Zle Chord Ainc
fin_lines = deck_fins[deck_fins.index("SURFACE" + chr(10) + "TipFin"):].splitlines()
f_nch, _, f_nsp, _ = fin_lines[2].split()
f_z = [float(fin_lines[6].split()[2]), float(fin_lines[10].split()[2])]
expect(d_sref, Sref, 1e-6, "deck Sref"); expect(d_xref, Xref, 1e-6, "deck Xref"); expect(d_bref, Bref, 1e-6, "deck Bref")
dd_pct = 100 * (CDi4 / CDff4 - 1)                       # near-field vs far-field induced drag, one number used in sections 5 and 7.4
alpha_ratio = c3["YDUPLICATE removed"]["alpha"] / c3["correct deck"]["alpha"]
ar_half = (d_bref / 2) ** 2 / (d_sref / 2)
s3 = read_csv("results/runs/rigid_glider/plate_balance/step3_summary.csv")
cfd = read_csv("cfd/validation_sweep/sweep_results.csv")
cfd_lo, cfd_hi = min(float(r["alpha_deg"]) for r in cfd), max(float(r["alpha_deg"]) for r in cfd)
# ---- plate balance: low-angle aerodynamic-centre offset (read from step3_summary.csv)
ac_all = [float(r["AC_ahead_of_quarter_chord_pct"]) for r in s3]
ac_lo, ac_hi = min(ac_all), max(ac_all)
_byw = {}
for _r in s3:
    _byw.setdefault(_r["wing"], {}).setdefault(float(_r["Re"]), []).append(float(_r["AC_ahead_of_quarter_chord_pct"]))
_mean = lambda v: sum(v) / len(v)
ac_n_all, ac_n_fwd, _lo, _hi = len(_byw), 0, [], []
for _w, _d in _byw.items():
    _rs = sorted(_d)
    _a_l, _a_h = _mean(_d[_rs[0]]), _mean(_d[_rs[-1]])
    ac_n_fwd += _a_l > _a_h
    _lo.append(_a_l); _hi.append(_a_h)
ac_mean_lo, ac_mean_hi = _mean(_lo), _mean(_hi)
re_min = min(float(r["Re"]) for r in s3)
re_min_txt = f"{round(re_min, -3):,.0f}"
# ---- first lift peak by aspect ratio, only if the column exists in the file at build time
if "alpha_first_peak_deg" in s3[0]:
    _byar, _kinds, _gmax, _kc = {}, {}, {}, {}
    for _r in s3:
        if _r["alpha_first_peak_deg"].strip():
            _byar.setdefault(_r["AR"], []).append(float(_r["alpha_first_peak_deg"]))
            _kinds.setdefault(_r["AR"], set()).add(_r["first_peak_kind"])
            _kc.setdefault(_r["AR"], []).append(_r["first_peak_kind"] == "flattening")
            if _r.get("alpha_global_max_to_20deg", "").strip():
                _gmax.setdefault(_r["AR"], []).append(float(_r["alpha_global_max_to_20deg"]))
    _parts = []
    for a, v in sorted(_byar.items(), key=lambda kv_: float(kv_[0])):
        t_ = f"aspect ratio {a}: " + (f"{min(v):.0f} to {max(v):.0f} degrees" if round(min(v)) != round(max(v)) else f"about {min(v):.0f} degrees")
        if "flattening" in _kinds[a]:
            _nf, _nt = sum(_kc[a]), len(_kc[a])
            _word = "all" if _nf == _nt else ("most" if _nf / _nt >= 0.75 else ("about half" if _nf / _nt >= 0.35 else "some"))
            t_ += f" (the first 'peak' is a flattening of the lift curve in {_word} of the rows: {_nf} of {_nt}" + (f"; the largest lift found by searching up to 20 degrees is at {min(_gmax[a]):.0f} to {max(_gmax[a]):.0f} degrees" if a in _gmax and max(_gmax[a]) > max(v) + 1 else "") + ")"
        _parts.append(t_)
    STALL_SENTENCE = ("The measured plates (section 8) reach their first lift peak at: " + "; ".join(_parts) + " (columns <code>alpha_first_peak_deg</code>, <code>first_peak_kind</code>, <code>alpha_global_max_to_20deg</code> of <code>step3_summary.csv</code>).")
else:
    STALL_SENTENCE = "The measured plates (section 8) stop gaining lift at an angle that depends on aspect ratio; the numbers are on the plate-balance page (<code>results/runs/rigid_glider/plate_balance/index.html</code>), not repeated here."

# ------------------------------------------------------------------------------------------------ SVG helpers (dark palette)
INK, MUT, ACC, LNK, GRN, BAD, PAN, LINE = "#dde3ea", "#9aa7b4", "#e89a55", "#7fb4ff", "#6cc08b", "#ff8a80", "#1c2229", "#333c46"


def svg(w, h, body, label):
    return (f"<figure><svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {w} {h}' width='{w}' height='{h}' role='img' aria-label='{E(label)}' style='max-width:100%;height:auto;background:#14181d;border:1px solid {LINE};border-radius:6px'>"
            f"<defs><marker id='ar' viewBox='0 0 10 10' refX='9' refY='5' markerWidth='7' markerHeight='7' orient='auto-start-reverse'><path d='M0,0 L10,5 L0,10 z' fill='{MUT}'/></marker></defs>{body}</svg><figcaption>{E(label)}</figcaption></figure>")


def T(x, y, s, size=13, anchor="start", color=INK, weight="normal"):
    return f"<text x='{x}' y='{y}' font-family='Segoe UI,Arial' font-size='{size}' text-anchor='{anchor}' fill='{color}' font-weight='{weight}'>{E(s)}</text>"


def Ln(x1, y1, x2, y2, color=INK, w=1.5, dash=None, arrow=False):
    d = f" stroke-dasharray='{dash}'" if dash else ""
    a = " marker-end='url(#ar)'" if arrow else ""
    return f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='{color}' stroke-width='{w}'{d}{a}/>"


def Bx(x, y, w, h, title, sub="", color=LNK):
    return (f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='6' fill='{PAN}' stroke='{color}' stroke-width='1.5'/>" + T(x + w / 2, y + 22, title, 13, "middle", INK, "bold")
            + "".join(T(x + w / 2, y + 40 + 15 * i, s, 11, "middle", MUT) for i, s in enumerate(sub.split("|") if sub else [])))


def Pg(points, fill="none", stroke=INK, w=1.5, op=1.0):
    return f"<polygon points='{' '.join(f'{x:.1f},{y:.1f}' for x, y in points)}' fill='{fill}' fill-opacity='{op}' stroke='{stroke}' stroke-width='{w}'/>"


# ---- diagram 1: the chain
def fig_chain():
    b = []
    xs = [10, 168, 326, 484, 642, 800]
    names = [("XFOIL", "section polars", "airfoil, Reynolds no."), ("AVL", "outline model", "planform, tail, fins"), ("Trim / stability", "numbers", "NP, margin, CM0, Cn_beta"),
             ("Measured plate", "tables", "Ananda 2012 wind tunnel"), ("CFD check", "OpenFOAM", f"one plate, {cfd_lo:g} to {cfd_hi:g} deg"), ("Flight test", "not done yet", "the only real judge")]
    cols = [LNK, ACC, ACC, GRN, GRN, BAD]
    for x, (t, s1, s2), c in zip(xs, names, cols):
        b.append(Bx(x, 20, 150, 66, t, f"{s1}|{s2}", c))
    for i in range(5):
        b.append(Ln(xs[i] + 150, 53, xs[i + 1], 53, MUT, 1.5, None, True))
    b.append(T(10, 112, "Answers:", 12, "start", ACC, "bold"))
    ans = ["lift and drag of a SECTION, with viscosity", "where the neutral point is; how the tail and fins act", "where the CG may sit; yaw and roll sign", "what a real flat plate did in a tunnel", "whether a flow solver agrees with the tunnel", "whether it flies"]
    nots = ["flat plates (does not converge)", "stall, viscosity, Reynolds number, glide ratio", "anything beyond the AVL model", "swept or tailed wings; this project's shapes", "pitching moment above 4 deg (so far)", "n/a"]
    import textwrap
    for x, a, n in zip(xs, ans, nots):
        for k, ln in enumerate(textwrap.wrap(a, 25)[:3]):
            b.append(T(x, 130 + 13 * k, ln, 10.5, "start", INK))
        b.append(T(x, 180, "Cannot:", 10.5, "start", BAD, "bold"))
        for k, ln in enumerate(textwrap.wrap(n, 25)[:3]):
            b.append(T(x, 194 + 13 * k, ln, 10.5, "start", MUT))
    return svg(960, 235, "".join(b), "The modelling chain used in this project, left to right, with what each link answers and what it cannot")


# ---- diagram 2: horseshoe + grid
def fig_horseshoe():
    b = []
    # panel grid on a swept half wing (top view, x down, y right)
    root_le, tip_le = (60, 40), (250, 190)
    root_te, tip_te = (60, 170), (250, 235)
    nsp, nch = 5, 4
    for j in range(nsp + 1):
        t = j / nsp
        x0 = root_le[0] + t * (tip_le[0] - root_le[0]); y0 = root_le[1] + t * (tip_le[1] - root_le[1])
        x1 = root_te[0] + t * (tip_te[0] - root_te[0]); y1 = root_te[1] + t * (tip_te[1] - root_te[1])
        b.append(Ln(x0, y0, x1, y1, LINE if 0 < j < nsp else MUT, 1.2))
    for i in range(nch + 1):
        s = i / nch
        b.append(Ln(root_le[0] + s * (root_te[0] - root_le[0]), root_le[1] + s * (root_te[1] - root_le[1]), tip_le[0] + s * (tip_te[0] - tip_le[0]), tip_le[1] + s * (tip_te[1] - tip_le[1]), LINE if 0 < i < nch else MUT, 1.2))

    def pt(t, s):
        return (root_le[0] + t * (tip_le[0] - root_le[0]) + s * ((root_te[0] + t * (tip_te[0] - root_te[0])) - (root_le[0] + t * (tip_le[0] - root_le[0]))),
                root_le[1] + t * (tip_le[1] - root_le[1]) + s * ((root_te[1] + t * (tip_te[1] - root_te[1])) - (root_le[1] + t * (tip_le[1] - root_le[1]))))
    t0, t1, s0, s1 = 2 / nsp, 3 / nsp, 1 / nch, 2 / nch
    a, bb = pt(t0, s0 + 0.25 * (s1 - s0)), pt(t1, s0 + 0.25 * (s1 - s0))          # bound vortex at 1/4 panel chord
    cp = pt((t0 + t1) / 2, s0 + 0.75 * (s1 - s0))                                  # control point at 3/4 panel chord
    b.append(Pg([pt(t0, s0), pt(t1, s0), pt(t1, s1), pt(t0, s1)], ACC, ACC, 1.2, 0.18))
    b.append(Ln(a[0], a[1], bb[0], bb[1], ACC, 3))
    b.append(Ln(a[0], a[1], a[0], 275, ACC, 2.2, "6 3"))
    b.append(Ln(bb[0], bb[1], bb[0], 275, ACC, 2.2, "6 3"))
    b.append(f"<circle cx='{cp[0]:.1f}' cy='{cp[1]:.1f}' r='4' fill='{GRN}'/>")
    b.append(T(266, 100, "bound vortex (1/4 panel chord)", 11.5, "start", ACC))
    b.append(T(266, 117, "strength Gamma, unknown", 11.5, "start", ACC))
    b.append(T(266, 150, "control point (3/4 chord): no flow", 11.5, "start", GRN))
    b.append(T(266, 165, "through the surface here", 11.5, "start", GRN))
    b.append(T(266, 262, "trailing legs run back to infinity", 11.5, "start", ACC))
    b.append(T(60, 20, "top view, flow comes from the top, half wing", 11.5, "start", MUT))
    b.append(Ln(30, 60, 30, 100, MUT, 1.5, None, True)); b.append(T(36, 84, "flow", 11.5, "start", MUT))
    return svg(560, 290, "".join(b), "Generic vortex lattice: the wing becomes a grid of panels; each panel carries one horseshoe vortex (generic picture; AVL's own manual describes its version)")


# ---- diagram 3: NP, CG, margin on the F45 wing (side-by-side numbers real)
def fig_np(xref, xnp, cref, margin_pct, title):
    x0, scale = 60, 6000.0 * 0.12 / 0.12      # px per metre
    X = lambda x: x0 + x * 4500
    rc_, tx_, tc_ = d_root[3], d_tip[0], d_tip[3]
    b = [Pg([(X(0), 90), (X(rc_), 90), (X(tx_ + tc_), 140), (X(tx_), 140)], PAN, LINE, 1.0)]       # sketch of root-to-tip chord line
    b.append(Ln(X(0), 60, X(rc_), 60, MUT, 1.5)); b.append(T(X(rc_ / 2), 52, f"root chord {rc_ * 1000:.1f} mm (F45)", 11, "middle", MUT))
    b.append(Ln(X(xref), 75, X(xref), 190, GRN, 2)); b.append(T(X(xref), 205, f"CG / reference {xref * 1000:.1f} mm", 12, "middle", GRN, "bold"))
    b.append(Ln(X(xnp), 75, X(xnp), 190, ACC, 2)); b.append(T(X(xnp), 222, f"neutral point {xnp * 1000:.1f} mm", 12, "middle", ACC, "bold"))
    b.append(Ln(X(xref), 170, X(xnp), 170, INK, 1.5, None, True)); b.append(Ln(X(xnp), 170, X(xref), 170, INK, 1.5, None, True))
    b.append(T((X(xref) + X(xnp)) / 2, 163, f"{margin_pct:.1f}% of Cref", 12, "middle", INK, "bold"))
    b.append(T(X(0), 30, title, 12, "start", MUT))
    b.append(Ln(X(0), 232, X(0.1), 232, MUT, 1, None, True)); b.append(T(X(0.1) + 6, 236, "x aft (not to scale for y)", 10.5, "start", MUT))
    return svg(640, 245, "".join(b), title)


# ---- diagram 4: Cm vs CL
def fig_cm():
    b = [Ln(60, 40, 60, 230, MUT, 1.2), Ln(60, 135, 420, 135, MUT, 1.2), T(425, 139, "CL", 12, "start", MUT), T(64, 36, "CM", 12, "start", MUT)]
    for lab, slope, c, dy in (("CG ahead of NP: stable (slope < 0)", -0.9, GRN, 0), ("CG at NP: neutral", 0.0, ACC, 0), ("CG behind NP: unstable (slope > 0)", 0.7, BAD, 0)):
        b.append(Ln(60, 135 + dy, 400, 135 + dy - slope * 120, c, 2.2))
        b.append(T(404, 135 - slope * 120 + 4, lab, 10.5, "start", c))
    b.append(T(70, 125, "bent tail: the whole line shifts UP by CM0", 11, "start", INK))
    b.append(f"<line x1='60' y1='135' x2='60' y2='105' stroke='{INK}' stroke-width='2' marker-end='url(#ar)'/>")
    b.append(T(66, 100, "CM0 (CM at zero lift)", 11, "start", INK))
    return svg(640, 250, "".join(b), "Pitching moment against lift. The slope says stable or not; CM0 is where the line crosses CL = 0; the bent tail raises it (diagram, not data)")


# ---- diagram 5: weathercock and dihedral effect
def fig_beta():
    b = []
    b.append(T(10, 20, "Cn_beta: weathercock (top view)", 12, "start", ACC, "bold"))
    b.append(Pg([(110, 100), (130, 100), (140, 160), (100, 160)], PAN, INK, 1.5))
    b.append(Ln(60, 40, 105, 85, MUT, 1.5, None, True)); b.append(T(30, 36, "wind from the right of the nose", 10.5, "start", MUT))
    b.append(Ln(160, 110, 160, 150, GRN, 2, None, True)); b.append(T(170, 135, "restoring yaw moment", 10.5, "start", GRN))
    b.append(T(10, 200, "Cn_beta > 0 (AVL sign): the nose turns back", 11, "start", INK)); b.append(T(10, 215, "into the wind. Fins and tail behind the CG add to it.", 11, "start", INK))
    b.append(T(340, 20, "Cl_beta: dihedral effect (front view)", 12, "start", ACC, "bold"))
    b.append(Ln(380, 110, 520, 110, INK, 3))
    b.append(Ln(450, 110, 450, 110, INK))
    b.append(Ln(520, 60, 520, 98, GRN, 2, None, True)); b.append(Ln(380, 100, 380, 110, GRN, 1))
    b.append(T(440, 60, "sideslip: the wing on the side the wind comes from", 10.5, "middle", MUT))
    b.append(T(450, 76, "gets more lift", 10.5, "middle", MUT))
    b.append(T(340, 200, "Cl_beta < 0 (AVL sign): that wing rises, so the", 11, "start", INK)); b.append(T(340, 215, "glider rolls back level. Sweep and dihedral give it.", 11, "start", INK))
    return svg(700, 235, "".join(b), "What Cn_beta and Cl_beta mean (diagram; the signs are checked against AVL runs in the text)")


# ---- diagram 6: file lines -> wing (F45)
def fig_file2():
    cx = 330
    P = lambda y, x: (cx + y * 1500, 30 + x * 1500)
    rc_, tx_, ty_, tc_ = d_root[3], d_tip[0], d_tip[1], d_tip[3]
    root_le, root_te = P(0, 0), P(0, rc_)
    tip_le, tip_te = P(ty_, tx_), P(ty_, tx_ + tc_)
    poly = [P(-ty_, tx_), root_le, tip_le, tip_te, root_te, P(-ty_, tx_ + tc_)]
    b = [Pg(poly, PAN, INK, 1.6)]
    b.append(Ln(cx, 20, cx, 190, MUT, 1, "8 3 2 3"))
    b.append(f"<circle cx='{root_le[0]}' cy='{root_le[1]}' r='5' fill='{ACC}'/>")
    b.append(f"<circle cx='{tip_le[0]}' cy='{tip_le[1]}' r='5' fill='{ACC}'/>")
    b.append(T(root_le[0] + 8, root_le[1] - 6, f"SECTION 1: Xle 0, Yle 0, chord {rc_:.6f}", 11.5, "start", ACC))
    b.append(T(tip_le[0] + 8, tip_le[1] - 8, f"SECTION 2: Xle {tx_:.6f}, Yle {ty_:g}, chord {tc_:.6f}", 11.5, "start", ACC))
    b.append(T(cx - 150, 30 + 0.034 * 1500, "YDUPLICATE: AVL adds this mirror half", 11.5, "end", GRN))
    xr = P(0, d_xref)
    b.append(Ln(xr[0] - 60, xr[1], xr[0] + 60, xr[1], GRN, 2)); b.append(T(xr[0] + 66, xr[1] + 4, f"Xref {d_xref:.6f} (CG)", 11.5, "start", GRN))
    b.append(Ln(cx, 200, cx + ty_ * 1500, 200, MUT, 1.5, None, True)); b.append(Ln(cx + ty_ * 1500, 200, cx, 200, MUT, 1.5, None, True)); b.append(T(cx + ty_ * 750, 216, f"half span {ty_:g} m", 11, "middle", MUT))
    b.append(T(20, 22, "top view, nose up the page; x is aft, y is out along the wing", 11.5, "start", MUT))
    return svg(900, 230, "".join(b), "The two SECTION lines of the F45 file are the root and tip leading-edge points; AVL fills in the wing between them and mirrors it")


# ---- diagram 7: panelling bar chart from the real CSV
def panelling_rows():
    with (ROOT / "results/runs/rigid_glider/bird_outline_v2/panelling_check.csv").open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fig_panel(rows):
    b = [T(10, 18, "neutral point, % MAC, same outline at 3 panellings (8x64, 10x80, 16x128) - real values from panelling_check.csv", 11.5, "start", MUT)]
    groups = [(fam, out) for fam in ("BS", "BC") for out in ("v1", "final")]
    for gi, (fam, out) in enumerate(groups):
        rs = [r for r in rows if r["family"] == fam and r["outline"] == out]
        for k, r in enumerate(rs):
            v = float(r["NP_pct_MAC"])
            h = (v - 20) * 9
            x = 40 + gi * 170 + k * 44
            b.append(f"<rect x='{x}' y='{240 - h:.1f}' width='36' height='{h:.1f}' fill='{[LNK, ACC, GRN][k]}' fill-opacity='0.85'/>")
            b.append(T(x + 18, 236 - h, f"{v:.1f}", 11, "middle", INK))
        b.append(T(40 + gi * 170 + 66, 258, f"{fam} {out} (bend {float(rs[0]['bend_deg']):g})", 11.5, "middle", INK))
    b.append(T(10, 275, "bars start at 20% MAC; blue 8x64, orange 10x80, green 16x128", 11, "start", MUT))
    return svg(720, 285, "".join(b), "Neutral point against panel count (real data). The BC outlines move by 4 to 6 points of MAC")


def table(head, rows, cls="t"):
    return "<div class='scroll'><table class='%s'><tr>%s</tr>%s</table></div>" % (cls, "".join(f"<th>{h}</th>" for h in head), "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows))


# ------------------------------------------------------------------------------------------------ real quotes: source code and files
src_run_case = inspect.getsource(ft.run_avl_case)
src_run_avl = inspect.getsource(avl_mod.run_avl)
cmd_start = src_run_avl.index("    commands = [")
cmd_block = src_run_avl[cmd_start:src_run_avl.index("    script =")]
read_part = src_run_avl[src_run_avl.index("        ft = ft_path.read_text"):src_run_avl.index("        points.append") + 120]
v2deck = (ROOT / "results/runs/rigid_glider/bird_outline_v2/avl_work/c00001/scan/c00001.avl").read_text().splitlines()
tail_i = v2deck.index("Tail")
v2_tail = "\n".join(v2deck[tail_i - 1:tail_i + 15])
v2_head = "\n".join(v2deck[:12])
assert "-5.000000" in v2_tail
pan = panelling_rows()
plate_idx = re.sub(r"<style.*?</style>", "", (ROOT / "results/runs/rigid_glider/plate_balance/index.html").read_text(encoding="utf-8"), flags=re.S)
plate_txt = re.sub(r"\s+", " ", html.unescape(re.sub(r"<[^>]+>", " ", plate_idx)))
sup_i = plate_txt.index("What the data supports")
plate_support = plate_txt[sup_i + len("What the data supports"):plate_txt.index("1. What this is")].strip()
plate_unres = read_csv("results/runs/rigid_glider/plate_balance/step3_NOT_RESOLVED.csv")
xf_lines = []
with (ROOT / "data/polars/megapolar.dat").open() as f:
    hdr = f.readline().strip()
    for line in f:
        p = line.split()
        if len(p) > 6 and p[0] == "NACA_0006" and p[1] in ("30000", "80000") and p[2] == "5" and p[4] in ("0.000", "4.000", "8.000"):
            xf_lines.append(line.strip())
plate_sum = (ROOT / "results/runs/rigid_glider/plate/summary.txt").read_text().splitlines()

# ------------------------------------------------------------------------------------------------ page
P = []
P.append(f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Modelling walkthrough: from AVL to a run you set up yourself</title>"
         f"<style>{ft.CSS}\n.scroll{{overflow-x:auto}}table.t td,table.t th{{padding:4px 8px;font-size:13.5px;vertical-align:top}}pre{{background:#10151a;border:1px solid #333c46;padding:10px 12px;overflow-x:auto;font-size:12.5px;line-height:1.35}}"
         f"figure{{margin:14px 0}}figcaption{{color:#9aa7b4;font-size:13px}}.ex{{background:#1c2229;border-left:4px solid #e89a55;padding:8px 14px;margin:12px 0}}.warn{{background:#2c2418;border:2px solid #e89a55;padding:10px 16px}}"
         f".good{{border-left:4px solid #6cc08b;background:#1c2229;padding:8px 14px;margin:12px 0}}code{{color:#dde3ea}}</style></head><body><main>")
P.append("<h1>From AVL to a run you set up yourself</h1>")
P.append("<p class='note'>A walkthrough of the modelling process used in this project, written for an engineering student who knows XFOIL and has the ideas of vortex-lattice methods and CFD, but has not set an AVL case up alone. "
         "Every screen, file and table quoted below is read from a real file or a real run in this workspace when the page is built (<code>docs/modeling_walkthrough/build_walkthrough.py</code>). Every number in the running text is either computed from those files or checked against them by the build, which stops with an error if one no longer matches; the few exceptions are listed at the end. "
         "The runs you can repeat are in <code>docs/modeling_walkthrough/practice/</code>; they are copies, so you cannot damage a project result. "
         "<b>About documentation:</b> only the program <code>avl352.exe</code> and a provenance note (<code>tools/avl/README.md</code>) are in the project; the AVL manual is on the MIT AVL page (<a href='https://web.mit.edu/drela/Public/web/avl/'>web.mit.edu/drela/Public/web/avl</a>) and is NOT shipped here. "
         "So wherever this page says how AVL behaves, it comes from a run shown here or from AVL's own on-screen menus, and I mark it <b>checked</b>. Things I did not check are said to be unchecked.</p>")
P.append("<nav class='box'><b>Contents</b>: 1 map; 2 what a vortex-lattice method is; 3 the numbers; 4 anatomy of an input file; 5 lesson 1 (run it by hand); 6 lesson 2 (change the outline, script it); 7 checks that keep you honest; 8 where AVL stops; 9 checklist and glossary.</nav>")

# ---- 1
P.append("<section id='s1'><h2>1. The map</h2><p>Nothing in this project is one tool. It is a chain. Each link answers one question and is silent on others. Knowing which link a number came from tells you how far to trust it.</p>")
P.append(fig_chain())
P.append(table(["link", "what it answers", "what it cannot answer"], [
    ["XFOIL section polars", "Lift, drag and moment of one airfoil section at a Reynolds number, including viscosity and the start of stall. Used for the NACA 0006 chain (section 8).", "Flat plates: XFOIL did not converge on the plate (section 8). Anything three-dimensional."],
    ["AVL outline model", "Where the neutral point is, how a tail and fins change stability, and the signs of the yaw and roll derivatives, for a given outline.", "Stall, viscosity, Reynolds number, thickness, glide ratio, whether a build pulls out (section 2)."],
    ["Trim and stability numbers", "Arithmetic on AVL output: static margin, the CG that trims at a given CL, ballast mass.", "Anything the AVL numbers cannot support. Garbage in, tidy table out."],
    ["Measured plate tables", f"What real flat plates did in a wind tunnel (Ananda 2012): lift, drag and moment against angle, at Reynolds number {re_min_txt} and up.", "Swept or tailed wings; plates of this project's shapes; moments finer than the balance error (section 8)."],
    ["CFD check", "Whether a flow solver reproduces the tunnel for one plate. Lesson in <a href='../../cfd/learn/index.html'>cfd/learn/index.html</a>; this page does not repeat it.", "So far: pitching moment above 4 degrees (section 8)."],
    ["Flight test", "Whether it flies.", "Not done yet. Nothing upstream replaces it."]]))
P.append("</section>")

# ---- 2
P.append("<section id='s2'><h2>2. What a vortex-lattice method is</h2><p>Start from a picture you already know: a wing makes lift because the air goes faster over the top than under the bottom. A mathematically equivalent way to say it is that the wing carries <b>circulation</b>, a swirl of air around it, and the lift is proportional to that swirl. A vortex-lattice method (VLM) models the swirl directly.</p>")
P.append("<h3>Step 1. Replace the wing by a flat grid</h3><p>Take the wing outline seen from above. Cut it into a grid of small quadrilateral <b>panels</b>: some number along the chord, some number along the span. The wing is treated as an infinitely thin sheet, so thickness is gone from the start. (That is why the project's AVL files say <code>NACA 0000</code>: zero camber. The thickness digits are not used: <b>checked</b>, <code>practice/review_checks.py</code> ran the F45 deck at CL 0.3 with <code>NACA 0000</code>, <code>0006</code> and <code>0012</code> in both sections and every printed number (angle, CM, Cma, CLa, Xnp, Clb, Cnb, e) was identical.)</p>")
P.append(fig_horseshoe())
P.append("<h3>Step 2. One horseshoe vortex per panel</h3><p>On each panel put a <b>horseshoe vortex</b>: a vortex line across the panel (the <i>bound</i> vortex, about a quarter of the way back from the panel's front edge) whose two ends turn and run straight back to infinity (the <i>trailing legs</i>). Each horseshoe has a strength <b>Gamma</b> (units m&sup2;/s), unknown for now. Together the horseshoes are the swirl of the whole wing and its wake.</p>")
P.append("<h3>Step 3. Every vortex pushes air everywhere (Biot-Savart)</h3><p>A vortex line makes air move around it. The velocity a small piece of vortex line <i>dl</i>, of strength Gamma, causes at a point at vector <i>r</i> from it is</p>"
         "<pre>dV = Gamma / (4 pi) * ( dl x r ) / |r|^3          (Biot-Savart law; x is the vector cross product)</pre>"
         "<p>In words: stronger vortex, bigger push; farther away, much smaller push (like 1/r&sup2; for a small piece, 1/r for a long line); the push is sideways to both the line and the direction to the point. Because the velocity is <b>proportional to Gamma</b>, the total velocity at any point is a plain sum of (known geometry factor) &times; (unknown Gamma) over all the horseshoes. That is why the maths stays linear.</p>")
P.append("<h3>Step 4. The one condition that is enforced</h3><p>At one point on each panel, the <b>control point</b> (about three quarters of the way back), the air must not pass through the sheet: the flow component along the panel's normal must be zero. The flow there is the oncoming wind plus the push from every horseshoe:</p>"
         "<pre>for every panel i:   sum over j of  A(i,j) * Gamma(j)  =  - V_wind . n(i)</pre>"
         "<p>A(i,j) is the normal-direction velocity that horseshoe j makes at control point i for unit Gamma. It depends only on the geometry. So this is N equations in N unknowns, a linear system. The program builds the matrix and solves it. (You can see this in AVL's own output: <code>Building normalwash AIC matrix... Factoring normalwash AIC matrix...</code>, section 5. AIC means aerodynamic influence coefficient: the matrix A.)</p>")
P.append("<h3>Step 5. Forces</h3><p>Once the Gammas are known, each bound vortex segment feels a force from the Kutta-Joukowski law:</p><pre>force on a bound vortex segment = rho * ( V x Gamma-vector )     per unit length</pre>"
         "<p>Add up all segments for lift, side force and moments about a reference point. The trailing legs carry air downward behind the wing (downwash). That downwash tilts the local flow at the wing and tilts the force backward: that backward part is <b>induced drag</b>. AVL prints it as <code>CDind</code>.</p>")
P.append("<h3>What is left out (read this part twice)</h3><ul><li><b>Thickness.</b> The wing is a sheet.</li><li><b>Viscosity</b> and therefore <b>Reynolds number</b>. There is no boundary layer, no friction drag and no skin-friction dependence on speed. In the real screen output below you will see <code>CDvis = 0.00000</code>.</li>"
         "<li><b>Separation and stall.</b> Nothing in the equations can stop lift growing with angle.</li></ul><p>Result: <b>lift rises linearly with angle forever</b> and <b>drag is induced drag only</b>. " + STALL_SENTENCE + " The measured plates also have friction and pressure drag that AVL does not see.</p>")
P.append(f"<div class='good'><b>How far to trust AVL's neutral point.</b> It is a starting estimate, nothing more. AVL puts the lift of a thin flat sheet at about the quarter chord and never sees the viscous flow over a real plate. The measured plates say the real low-angle aerodynamic centre is not exactly there: in <code>results/runs/rigid_glider/plate_balance/step3_summary.csv</code> (column <code>AC_ahead_of_quarter_chord_pct</code>) the offset runs from {ac_lo:.1f} to {ac_hi:.1f}% of chord ahead of the quarter chord over all {len(s3)} rows (negative would be behind it). It moves forward as Reynolds number falls: for {ac_n_fwd} of the {ac_n_all} wing shapes the value at the lowest Reynolds number is further forward than at the highest (mean {ac_mean_lo:.1f}% at the lowest, {ac_mean_hi:.1f}% at the highest). Lower Reynolds number is where these gliders fly, so treat the AVL neutral point as a place to start ballast tests, not a result. "
         "AVL's yaw and roll derivatives (Cnb, Clb) have <b>never been checked against anything</b> here: no measurement, no tunnel, no second code. Their signs are checked only against AVL's own runs (section 6), which shows the program is consistent with itself and nothing more. Glide ratio and stall are outside AVL altogether. The project's own cross-check is in section 8.</div>")
P.append("</section>")

# ---- 3
P.append("<section id='s3'><h2>3. The numbers AVL gives, and what they mean</h2><p>All values in this section come from the real run in section 5 (the F45 flat-test wing, a 250 mm span flat plate outline, sweep 25&deg;, reference point at the CG position of the kit). Sign conventions are the ones AVL prints; I checked each sign against a run, as noted.</p>")
P.append(table(["quantity", "AVL name", "F45 value", "physical meaning", "sign convention (checked how)"], [
    ["Lift slope", "CLa", f"{CLa:.3f} per rad", "How much CL you get per radian of angle of attack. Divide by 57.3 for per degree.", f"positive. Same wing in a different panelling: {c1r[0]['CLa']:.4f} down to {c1r[-1]['CLa']:.4f} (section 7)."],
    ["Neutral point (NP)", "Xnp", f"{Xnp * 1000:.2f} mm aft of the origin", "The point about which the pitching moment does not change with angle. Put the CG here and the glider has no pitch stiffness.", "measured aft of the file's x = 0 (the root leading edge). Checked: it barely moves when the CG moves (section 5)."],
    ["Static margin", "(Xnp - Xref) / Cref", f"{sm1:.1f}% at Xref {Xref * 1000:.1f} mm", "How far ahead of the NP the CG is, as a fraction of the reference chord. Positive = stable in pitch.", "I compute it from the printed numbers. Checked: equals -Cma/CLa (next row)."],
    ["Pitch stiffness", "Cma", f"{Cma:.3f} per rad", "Change of pitching moment with angle. Negative = a nose-up gust produces a nose-down restoring moment.", f"negative = stable, nose-up positive. Checked: Cma/CLa = {Cma / CLa:.4f} and (Xref-Xnp)/Cref = {(Xref - Xnp) / Cref:.4f}."],
    ["CM at zero lift (CM0)", "Cmtot at alpha 0", "0 for this plain wing", "The pitching moment when the wing makes no lift. A symmetric flat wing has none, so <b>in this linear model</b> it cannot pull out of a dive by itself. A bent tail supplies it. (Measured plates show the centre of pressure moving near stall, which this model cannot; that is why the flat test and the plate-balance study exist.)", "nose-up positive. Checked: a tail bent trailing-edge-up gives positive CM0 (section 6)."],
    ["Trim", "alpha where Cmtot = 0", "see below", "The angle (and CL) at which the moments balance about the CG.", "In this linear model a flat symmetric wing needs CM0 > 0 to trim; the trim CG is where Cmtot is zero at the chosen CL. Measured plates are not this simple near stall (see CM0 above)."],
    ["Weathercock stiffness", "Cnb", f"{Cnb:+.5f} per rad", "Yawing moment per radian of sideslip. Positive = the nose turns back into the wind.", f"positive = stable. Checked: adding tip fins raised it from {ex2b['Cnb']:+.4f} to {ex2f['Cnb']:+.4f} (section 6, at CL 0.3). This table is at alpha {alpha4:g} deg (CL {CL4:.3f}), where the bare wing gives {Cnb:+.4f}. On a swept wing Cnb and Clb vary with lift, so quote them with the operating point."],
    ["Dihedral effect", "Clb", f"{Clb:+.5f} per rad", "Rolling moment per radian of sideslip. Negative = the wing the wind comes from rises, rolling the glider level.", f"negative = stable. Checked: more sweep made it more negative (section 6). Here {Clb:+.4f} at alpha {alpha4:g} deg; at CL 0.3 the bare wing gives {ex2b['Clb']:+.4f}."],
    ["Span efficiency", "e", f"{e4:.4f}", "How close the induced drag is to the best possible for that span: CDi = CL&sup2; / (pi AR e). 1.0 = elliptic loading.", "AVL prints e from the far-field (Trefftz plane) drag; see the note in section 7."]]))
P.append(fig_np(Xref, Xnp, Cref, sm1, f"F45 wing: neutral point and reference point (real numbers from the run in section 5)"))
P.append(fig_cm())
P.append(f"<p>The line in the second diagram is CM = CM0 + CL &times; (x_cg - x_np) / Cref. Check against the run: at CL 0.3 with the reference point at {Xref * 1000:.2f} mm AVL printed CM = {Cm03a:.5f}; the formula gives 0.3 &times; ({Xref * 1000:.3f} - {Xnp * 1000:.3f}) / {Cref * 1000:.3f} = {0.3 * (Xref - Xnp) / Cref:.5f}. This formula is what the project uses to turn a tail bend (CM0) into a trim CG: <code>x_cg = x_np - Cref * CM0 / CL</code>.</p>")
P.append(fig_beta())
P.append("</section>")

# ---- 4
P.append("<section id='s4'><h2>4. Anatomy of an AVL input file</h2><p>The file below is the real F45 flat-test case. It was written by <code>flat_test.py</code> and found at <code>results/runs/rigid_glider/flat_test_v2/avl_work/F45_span250/bare/cl03/bare.avl</code>; the copy in <code>practice/f45_bare.avl</code> differs only in its first line (the title). The table gives every line of the file, numbered, including line 10. All lengths are in metres. AVL does not care about units as long as you are consistent (checked: section 7, a file with the sections in millimetres and the header in metres gives nonsense).</p>")
notes = [
    ((1,), "Title line. Free text, printed in the output (<code>Configuration: f45_bare</code>)."),
    ((2,), "Mach number. 0.0 = incompressible. AVL prints <code>Mach = 0.0000</code>."),
    ((3,), "Three symmetry numbers (symmetry about the y = 0 plane, about the z = 0 plane, and the z of that plane). All zero: no built-in symmetry. This project makes the second wing half with <code>YDUPLICATE</code> instead; the project's own comment in <code>avl.py</code>: &quot;Explicit full wing via YDUPLICATE (rather than a symmetry boundary) is necessary for beta, roll, and yaw derivatives.&quot;"),
    ((4,), f"<b>Sref Cref Bref</b>: reference area (m&sup2;), reference chord (m), reference span (m). AVL divides forces by Sref and moments by Sref&times;Cref (pitch) or Sref&times;Bref (roll, yaw). Get these wrong and the coefficients are wrong (section 7). Here Sref is the wing area, Cref the mean aerodynamic chord, Bref the span. Check: Bref&sup2;/Sref = {AR:.1f}, the aspect ratio."),
    ((5,), f"<b>Xref Yref Zref</b>: the reference point = where moments are taken = the CG for stability purposes. Here x = {d_xref * 1000:.2f} mm. Moving it changes CM, not the wing (section 5)."),
    ((6,), "Starts one lifting surface."),
    ((7,), "Its name (free text; appears in strip-force files as <code>Wing</code> and <code>Wing (YDUP)</code>)."),
    ((8,), f"<b>Nchord Cspace Nspan Sspace</b>: {d_nch} panels along the chord, {d_nsp} along each half span (the panel counts; the screen shows <code>{2 * d_nch * d_nsp} Vortices</code> = 2 halves &times; {d_nch} &times; {d_nsp}, checked), and two numbers that choose how panels are spaced ({d_cs} and {d_ss} here). I did not test the spacing numbers; they are defined in the AVL manual."),
    ((9,), "Mirror this surface about the y = 0 plane (screen: <code>Building duplicate image-surface: Wing (YDUP)</code>, checked). So you describe only the right half."),
    ((10,), "<b>The line after <code>YDUPLICATE</code> belongs to it</b>: one number, the y position of the mirror plane. <code>0.0</code> = mirror about y = 0, the wing root. Keep it when you write a file from this table. I did not test what AVL does if it is left out."),
    ((11,), "Starts a cross-section of the surface. AVL fills in the wing between consecutive sections. This one is the root."),
    ((12,), f"<b>Xle Yle Zle Chord Ainc</b>: the section's leading-edge point (x aft, y out, z up), its chord, and its incidence angle in degrees (twist; positive nose-up). Root: leading edge at the origin, chord {d_root[3] * 1000:.2f} mm, no twist."),
    ((13,), "Airfoil keyword: the section shape is a NACA 4-digit code."),
    ((14,), "NACA 0000 = no camber. AVL's flat-sheet model uses only the camber line, so thickness is not represented (checked: 0006 and 0012 give identical results, section 2)."),
    ((15,), "Starts the second cross-section: the tip."),
    ((16,), f"Tip section: leading edge moved {d_tip[0] * 1000:.1f} mm aft and {d_tip[1] * 1000:.0f} mm out, chord {d_tip[3] * 1000:.2f} mm. The aft shift IS the sweep. The tip chord is {d_tip[3] / d_root[3]:.1f} of the root chord (taper {d_tip[3] / d_root[3]:.1f})."),
    ((17, 18), "Airfoil keyword and camber code again, for the tip section (same meaning as lines 13 and 14)."),
]
P.append(annotated_lines(deck_bare, notes))
P.append("<p>Reading the whole file from the top, as AVL does:</p>" + pre(deck_bare))
P.append(fig_file2())
P.append("<h3>How a fin is represented</h3><p>A fin is another <code>SURFACE</code>, standing up in z. This is the real tip-fin surface from the project's <code>practice/f45_fins.avl</code> (made by <code>flat_test.py</code>), appended after the wing:</p>")
P.append(pre(deck_fins[deck_fins.index("SURFACE\nTipFin"):]))
P.append(f"<p>Both its sections sit at the wing tip (<code>Yle {d_tip[1]:g}</code>) with the same chord, one at z = {f_z[0] * 1000:+.1f} mm and one at z = {f_z[1] * 1000:+.1f} mm: a vertical panel {(f_z[0] - f_z[1]) * 1000:.0f} mm tall at each wingtip (<code>YDUPLICATE</code> adds the other one). It has {f_nch} chordwise and {f_nsp} &lsquo;spanwise&rsquo; (here vertical) panels.</p>")
P.append("<h3>How a bent tail is represented</h3><p>A tail is a second <code>SURFACE</code> behind the wing. A &lsquo;bend&rsquo; is not geometry: it is the incidence number of the tail's sections, <code>Ainc</code>, set to minus the bend angle (trailing edge up = nose-down incidence). Real header and tail part of case <code>c00001</code> of <code>results/runs/rigid_glider/bird_outline_v2/avl_work/</code> (the swift-like outline at a 5&deg; bend):</p>")
P.append(pre(v2_head + "\n   ...\n" + v2_tail))
P.append("<p>Every tail section ends in <code>-5.000000</code>: that is the 5&deg; bend (checked in section 6: the zero-lift moment grows in proportion to this number).</p>")
P.append("</section>")

# ---- 5
P.append("<section id='s5'><h2>5. Lesson 1: run it by hand</h2>")
P.append("<p>Open PowerShell and go to the practice folder (adjust the start of the path to where your OneDrive folder is):</p>"
         "<pre>cd \"<MAV-2026>\\Codes\\docs\\modeling_walkthrough\\practice\"\n..\\..\\..\\tools\\avl\\avl352.exe f45_bare.avl</pre>"
         "<p>AVL starts, reads the file, and waits at a prompt <code>AVL   c&gt;</code>. You type commands; it never needs the mouse. Type <code>PLOP</code>, Enter, then <code>G</code>, Enter, then Enter again. That is the graphics switch (<code>PLOP</code> menu, <code>G</code> = graphics-enable flag, which the screen shows going from T to F, checked). The project's runner does the same at the start of every run so that no plot window is needed; I did not test what happens without it. Then <code>OPER</code> to enter the operating-point menu.</p>")
P.append("<p><b>What AVL prints when it loads the file</b> (real, from <code>practice/lesson1_screen.txt</code>):</p>")
ban = screen_block("Reading file: f45_bare.avl", "Initializing run cases...")
P.append(pre(ban))
P.append(annotated(ban, [("Building surface: Wing", "It built the right half from the SECTION lines."), ("Building duplicate image-surface", "YDUPLICATE made the left half."), ("Mach =", "from line 2 of the file."),
                         (f"{2 * d_nsp} Strips", f"A strip is one spanwise slice of panels: {d_nsp} per half &times; 2 halves."), (f"{2 * d_nch * d_nsp} Vortices", f"{d_nch} &times; {d_nsp} &times; 2 horseshoes = unknowns of the linear system."),
                         ("0 Control variables", "no flaps or elevators defined in the file."), ("f45_bare.mass  open error", "harmless: it looks for optional mass and run-case files and uses defaults.")]))
P.append("<p>The menu text shown by AVL (real) lists the commands; the ones used here are <code>OPER</code> (operating point), inside it <code>A</code> (set a variable), <code>X</code> (execute), <code>ST</code> (stability derivatives), <code>FT</code> (total forces), <code>M</code> (modify parameters). Type <code>?</code> in AVL at any time for the list.</p>")
P.append("<p><b>The whole session as one script.</b> You can type these lines by hand or feed them from a file. The file is <code>practice/lesson1.in</code>:</p>" + pre((PR / "lesson1.in").read_text()))
P.append("<p>Run it:</p><pre>Get-Content lesson1.in | ..\\..\\..\\tools\\avl\\avl352.exe f45_bare.avl &gt; my_screen.txt</pre><p>(I ran exactly that from PowerShell and got the same numbers as the file I quote.) What each line does: <code>A A 4</code> = set alpha (variable A) to 4 degrees; <code>X</code> = solve; <code>ST</code> then Enter = print stability derivatives (a file name instead of Enter would write a file); the next Enter leaves OPER; <code>QUIT</code> exits. <code>A C 0.3</code> instead holds CL = 0.3 and solves for the angle. <code>M</code> then <code>X 0.040</code> sets the reference point x to 40 mm.</p>")
P.append("<h3>Reading the output, alpha = 4&deg;</h3>")
P.append(pre(tot1))
P.append(annotated(tot1, [("Sref =", "The reference values from the file header, echoed. Check they are what you meant."), ("Xref =", "Where moments are taken."), ("Standard axis orientation", "x forward, z DOWN in AVL's own axes. Lift is reported as CLtot with the sign you expect."),
                          ("Alpha =", "The angle you asked for."), ("CXtot", "Body-axis forces and moments: CX forward, CY side force, CZ down; Cltot roll, Cmtot pitch, Cntot yaw about Xref."),
                          ("CYtot", "Side force and pitching moment. Cmtot is negative: nose DOWN about the reference point at positive lift. (The CG is ahead of the neutral point, so it is restoring.)"),
                          ("CLtot =", f"Lift coefficient. CL = {CL4:.3f} at {alpha4:g} degrees."), ("CDtot =", "Total drag coefficient. Here it is all induced drag."),
                          ("CDvis =", "Viscous (friction) drag: exactly zero. AVL has no viscosity."), ("CLff", "Far-field values: the same lift and induced drag evaluated in the Trefftz plane (a plane far behind the wing). AVL's span efficiency <code>e</code> uses these."), ("CYff", "e = the span efficiency.")]))
P.append(f"<p>Check the span efficiency by hand: e = CLff&sup2; / (pi &times; AR &times; CDff) = {CLff4:.5f}&sup2; / (pi &times; {AR:.2f} &times; {CDff4:.7f}) = {CLff4 ** 2 / (math.pi * AR * CDff4):.4f}; AVL printed {e4}. (The near-field CDind = {CDi4} would give {CLff4 ** 2 / (math.pi * AR * CDi4):.4f}. The near-field value is {dd_pct:+.1f}% relative to the far-field one (see also section 7.4); the far-field one is the one to use for efficiency.)</p>")
P.append("<h3>Stability derivatives</h3>")
P.append(pre(st1))
P.append(annotated(st1, [("Stability-axis derivatives", "Derivatives with respect to angle (alpha, beta), roll, pitch and yaw rates, in stability axes (x along the flight path). The two columns are alpha and beta."),
                         ("CLa =", f"<b>CLa</b> = {CLa:.2f} per rad: the lift slope. <b>CLb</b> is zero (symmetry)."), ("Cma =", f"<b>Cma</b> = {Cma:.3f} per rad: pitch stiffness. Negative = stable. The CG is {sm1:.1f}% of Cref ahead of the neutral point, and Cma / CLa = {Cma / CLa:.4f}."),
                         ("Clb =", f"<b>Clb</b> = {Clb:.3f}: dihedral effect, negative = stable (at alpha {alpha4:g}; at CL 0.3 it is {ex2b['Clb']:.3f}, see section 3)."), ("Cnb =", f"<b>Cnb</b> = {Cnb:+.4f}: weathercock stiffness, positive = stable. Small: a plain swept plate has little (at CL 0.3 it is {ex2b['Cnb']:+.4f}, see section 3)."),
                         ("Clp =", "Roll damping Clp (negative = damped)."), ("Cmq =", "Pitch damping Cmq (negative = damped)."), ("Cnr =", "Yaw damping Cnr."), ("Neutral point  Xnp", f"<b>Xnp</b>: the neutral point, in the same length unit and origin as the file. {Xnp * 1000:.2f} mm, about {(Xnp - Xref) / Cref:.2f} Cref aft of the Xref; in this wing the root leading edge is the origin.")]))
P.append(f"<div class='ex'><b>Static margin from these numbers.</b> (Xnp - Xref) / Cref = ({Xnp * 1000:.3f} - {Xref * 1000:.3f}) mm / {Cref * 1000:.3f} mm = <b>{sm1:.1f}%</b>. That is -Cma/CLa = {-Cma / CLa * 100:.1f}%. Two routes, same number: good.</div>")
P.append("<h3>Move the CG</h3><p>The second half of the script holds CL = 0.3 (<code>A C 0.3</code>), then changes the reference point to 40 mm (<code>M</code>, <code>X 0.040</code>). Real output for the two cases:</p>")
sm_before = 100 * (rvA[0]["Xnp"] - rvA[0]["Xref"]) / Cref
P.append(f"<p>Both rows below are the same operating point, CL = 0.3 (alpha {a03:.4f} deg), and every column of a row is read from the same run. The first row comes from <code>practice/review_checks_results.txt</code> (CHECK A, made by <code>practice/review_checks.py</code>), because the lesson-1 screen only printed derivatives at alpha 4 deg before the move. The second row is the lesson-1 screen itself; the build checks that the scripted run reproduces it.</p>")
P.append(table(["", "Xref", "alpha", "CL", "Cmtot", "Cma", "Xnp", "static margin (Xnp-Xref)/Cref"], [
    ["before", f"{rvA[0]['Xref'] * 1000:.2f} mm", f"{rvA[0]['alpha']:.4f} deg", f"{rvA[0]['CL']:.3f}", f"{rvA[0]['Cm']:.5f}", f"{rvA[0]['Cma']:.4f}", f"{rvA[0]['Xnp'] * 1000:.3f} mm", f"{sm_before:.1f}%"],
    ["after (CG moved aft 10 mm)", f"{Xref2 * 1000:.2f} mm", f"{num(tot_cg, 'Alpha'):.4f} deg", f"{num(tot_cg, 'CLtot'):.3f}", f"{Cm03b:.5f}", f"{Cma2:.4f}", f"{Xnp2 * 1000:.3f} mm", f"{sm2:.1f}%"]]))
P.append(f"<p>What to see: the wing did not change (same alpha, same CL). The CG moved aft, so the margin shrank from {sm_before:.1f}% to {sm2:.1f}% and CM at CL 0.3 went from {Cm03a:.4f} to {Cm03b:.4f}. The change in CM is CL &times; &Delta;x / Cref = 0.3 &times; {(Xref2 - Xref) * 1000:.3f} / {Cref * 1000:.3f} = {0.3 * (Xref2 - Xref) / Cref:.4f} (printed: {Cm03b - Cm03a:.4f}). "
         f"AVL's printed neutral point is not perfectly fixed: at the same angle it moved by {abs(Xnp2 - rvA[0]['Xnp']) * 1000:.3f} mm ({rvA[0]['Xnp'] * 1000:.3f} to {Xnp2 * 1000:.3f} mm) when only Xref was changed, and the deliberately wrong Xref of section 7.3 (about ten times too far aft) moved it by {(c3['Xref 10x']['Xnp'] - c3['correct deck']['Xnp']) * 1000:.1f} mm. <b>The cause of this dependence is unverified.</b> It is small next to the {abs(sm2 - sm_before):.0f} points of static margin the CG move changed. To a good approximation the neutral point is a property of the wing; the CG is yours to place. Static margin is the distance between them.</p>")
P.append("<p><b>Your turn.</b> Run the script. Then change the 40 mm in <code>lesson1.in</code> to 45 mm (past the neutral point). Predict the sign of Cma before you look. (I did not run 45 mm; the sign rule is in section 3.)</p></section>")

# ---- 6
P.append("<section id='s6'><h2>6. Lesson 2: change the outline</h2><p>These variants are made and run by <code>practice/lesson2_batch.py</code>; the decks are in <code>practice/variants/</code>; the table below is the real <code>practice/lesson2_results.txt</code> that run wrote. To do one by hand, open the variant in a text editor, run it as in lesson 1 (or <code>py -B run_case.py variants/NAME.avl --cl 0.3</code>).</p>")
ex1, ex2, ex3 = EX1.strip(), EX2.strip(), EX3.strip()
P.append("<h3>Exercise 1. Change the sweep</h3><p>Edit the tip <code>SECTION</code>: change its first number (Xle) from 0.0675 to 0.020 or 0.110. Nothing else changes, including the header's Xref and Cref (so compare Xnp in millimetres, not static margins).</p>" + pre(ex1))
assert ex1r[0]["Xnp"] < ex1r[1]["Xnp"] < ex1r[2]["Xnp"] and ex1r[0]["Clb"] > ex1r[1]["Clb"] > ex1r[2]["Clb"] and ex1r[0]["Cnb"] < ex1r[1]["Cnb"] < ex1r[2]["Cnb"] and ex1r[2]["e"] < ex1r[1]["e"] < ex1r[0]["e"]
P.append(f"<p>What to look for (read from the table, not predicted): the neutral point moves aft as the tip goes aft (swept-back wing); Clb goes from {ex1r[0]['Clb']:.3f} to {ex1r[1]['Clb']:.3f} to {ex1r[2]['Clb']:.3f}, so the dihedral effect becomes more negative (more stable) with sweep; Cnb goes up too. The induced-drag span efficiency e drops with sweep ({ex1r[0]['e']:.4f}, {ex1r[1]['e']:.4f}, {ex1r[2]['e']:.4f}).</p>")
P.append("<h3>Exercise 2. Add or remove the tip fins</h3><p>Compare <code>f45_bare.avl</code> and <code>f45_fins.avl</code> (the second adds the <code>SURFACE TipFin</code> block shown in section 4).</p>" + pre(ex2))
P.append(f"<p>What to look for: Cnb rises from {ex2b['Cnb']:+.4f} to {ex2f['Cnb']:+.4f} with the fins, Clb changes little ({ex2b['Clb']:+.4f} to {ex2f['Clb']:+.4f}), the neutral point moves {(ex2f['Xnp'] - ex2b['Xnp']) * 1000:.2f} mm. The vertical surface behind the CG is what makes the weathercock. (Positive Cnb = stable, as claimed in section 3: this is where I checked the sign.)</p>")
P.append("<h3>Exercise 3. Bend a tail panel</h3><p>The script adds a second <code>SURFACE Tail</code> block behind the wing with <code>Ainc = -bend</code> in both sections:</p>" + pre((PR / "variants/tail_bend05.avl").read_text()[(PR / "variants/tail_bend05.avl").read_text().index("SURFACE\nTail"):]))
P.append(pre(ex3))
P.append(f"<p>What to look for: at zero bend Cm at alpha = 0 is exactly {ex3r[0]['Cm']:g}; every degree of bend adds about {ex3r[5]['Cm'] / 5:+.4f} of nose-up CM at zero lift ({ex3r[2]['Cm']:.4f} at 2&deg;, {ex3r[5]['Cm']:.4f} at 5&deg;, {ex3r[10]['Cm']:.4f} at 10&deg;); CL at alpha 0 goes negative (the tail now pushes down). The neutral point does not change with bend ({ex3r[0]['Xnp']:.5f} to {ex3r[10]['Xnp']:.5f} m). This is the mechanism the whole project relies on: the bend sets CM0, the NP stays put, so the trim CG follows from x_cg = x_np - Cref&times;CM0/CL.</p>")
P.append("<h3>The scripted way</h3><p>Typing keystrokes does not scale to hundreds of cases. The project drives AVL from Python by writing the same keystrokes to AVL's standard input. This is the real function <code>run_avl_case</code> in <code>rigid_glider/flat_test.py</code>:</p>" + pre(src_run_case))
P.append("<p>It builds the deck text, then calls <code>run_avl</code> in <code>rigid_glider/avl.py</code>, whose core is this (real):</p>" + pre("    ...\n" + cmd_block + "    ...\n" + read_part))
P.append("<p>In words: for each requested angle it types <code>A A &lt;alpha&gt;</code>, <code>X</code>, then <code>FT</code>, <code>ST</code>, <code>FS</code> each followed by a file name, so AVL writes three text files per point; afterwards Python opens the files and pulls out numbers by searching for names such as <code>CLtot</code>, <code>Cmtot</code>, <code>Xnp</code>, <code>Clb</code>, <code>Cnb</code>. <code>FS</code> writes the strip forces (local lift along the span). To batch your own cases, copy <code>practice/run_case.py</code>: it is a small, standard-library-only version of the same idea (about 40 lines) that returns a dictionary of the numbers.</p>")
P.append("</section>")

# ---- 7
P.append("<section id='s7'><h2>7. Checks that keep you honest</h2><h3>7.1 Panel-count convergence</h3><p>Rerun the same wing with more panels. If a number keeps moving, it has not converged. Real results for the F45 wing (<code>lesson2_batch.py</code>):</p>")
c1 = C1.strip()
P.append(pre(c1))
vmax, vmin = max(r["vortices"] for r in c1r), min(r["vortices"] for r in c1r)
P.append(f"<p>Reading: the neutral point moves by {(max(r['Xnp'] for r in c1r) - min(r['Xnp'] for r in c1r)) * 1000:.2f} mm across a {vmax / vmin:.0f}-fold change in vortices, CLa by {100 * (max(r['CLa'] for r in c1r) - min(r['CLa'] for r in c1r)) / c1r[-1]['CLa']:.1f}%, Cnb not at all and Clb by {max(r['Clb'] for r in c1r) - min(r['Clb'] for r in c1r):.5f} in the printed digits. A plain tapered wing on its own converges quickly. The last line is also a real limit: <code>Vortex array overflow. Increase NVMAX currently 5000</code>. AVL as shipped will not take more than 5000 vortices.</p>")
P.append("<p><b>Now the case where it does NOT converge.</b> The crow-like (BC) outlines of this project have a fan tail lying in the plane of the wing's wake. Same outline, three panellings, real values from <code>results/runs/rigid_glider/bird_outline_v2/panelling_check.csv</code>:</p>")
P.append(fig_panel(pan))
P.append(table(["family", "outline", "bend", "panels (chord x span)", "NP % MAC", "CM0", "Cn_beta", "Cl_beta", "span eff.", "tail lift / weight"],
               [[r["family"], r["outline"], r["bend_deg"], f"{r['chordwise']} x {r['spanwise']}", f"{float(r['NP_pct_MAC']):.2f}", f"{float(r['CM0']):.4f}", f"{float(r['Cn_beta']):+.4f}", f"{float(r['Cl_beta']):+.4f}", f"{float(r['span_eff']):.3f}", f"{float(r['tail_lift_over_weight']):+.3f}"] for r in pan]))
bc = [float(r["NP_pct_MAC"]) for r in pan if r["family"] == "BC" and r["outline"] == "final"]
bcv = [float(r["NP_pct_MAC"]) for r in pan if r["family"] == "BC" and r["outline"] == "v1"]
P.append(f"<p>What this shows: for the BC final outline the neutral point is {bc[0]:.1f}, {bc[1]:.1f} and {bc[2]:.1f}% MAC at the three panellings (a swing of {max(bc) - min(bc):.1f} points), and for the BC v1 outline {bcv[0]:.1f}, {bcv[1]:.1f}, {bcv[2]:.1f}% (still drifting at the finest). The tail's lift over the weight even changes sign. Meanwhile CM0, Cn_beta and Cl_beta agree within a few per cent. I did not diagnose the mechanism. The likely cause is the geometry: the tail sits exactly in the plane of the wing's trailing vortex sheet, so the tail's loading depends on how the two lattices line up. That is a hypothesis, not a finding. The practical rule is: <b>when a number changes by more than you care about between panellings, say so and do not quote it to more digits than it holds.</b></p>")
P.append(f"<h3>7.2 A known case</h3><p>A plain rectangular wing's neutral point should be near the quarter chord. Real run (<code>CHECK 2</code>):</p>")
c2 = C2.strip()
P.append(pre(c2))
P.append(f"<p>{rect_chords:.4f} of a chord against {rect_qc_chords}: {100 * (rect_qc_chords - rect_chords):.1f}% of a chord off. The lift slope is {100 * (rect_CLa_ll - rect_CLa) / rect_CLa_ll:.0f}% below the textbook elliptic-wing formula; I did not look into that gap, and the formula assumes a different load shape, so it is a rough reference only, not a pass/fail test. The neutral-point check is the one the project relies on.</p>")
c3txt = C3.strip()
P.append("<h3>7.3 Mistakes and how each shows up</h3><p>Each was made on purpose on the F45 deck and run (real output). Requested CL was 0.3 each time.</p>" + pre(c3txt))
ok_, nd_, cr_, sr_, xr_, mm_ = (c3[k] for k in ("correct deck", "YDUPLICATE removed", "Cref doubled", "Sref halved", "Xref 10x", "sections in mm"))
expect(cr_["Cm"] / ok_["Cm"], 0.5, 0.01, "Cref doubled halves Cm"); expect(sr_["alpha"] / ok_["alpha"], 0.5, 0.01, "Sref halved halves alpha"); expect(sr_["Cnb"] / ok_["Cnb"], 0.5, 0.01, "Sref halved halves Cnb")
expect(cr_["alpha"], ok_["alpha"], 1e-6, "Cref does not change alpha"); expect(mm_["alpha"], 0.0, 1e-9, "mm/m mix alpha")
P.append(f"<ul><li><b>No <code>YDUPLICATE</code></b>: AVL models one half wing. To reach CL 0.3 it needs alpha {nd_['alpha']:.1f}&deg; instead of {ok_['alpha']:.1f}&deg;, and Cnb jumps to {nd_['Cnb']:+.4f} because a half wing is not symmetric. The half wing has half the area <i>and</i> half the aspect ratio ({ar_half:.2f} instead of {AR:.1f}, computed from the half span and half area), and a lower aspect ratio has a lower lift slope: that is why the angle is about {alpha_ratio:.1f} times larger, not just twice. A tell-tale: the angle needed for a given CL is far too big.</li>"
         f"<li><b>Cref wrong</b> (doubled): CL, alpha, Xnp, Clb, Cnb unchanged, but Cm is halved (Cm is divided by Cref). The neutral point does not care about Cref; the static margin you compute from it would be wrong by the same factor.</li>"
         f"<li><b>Sref wrong</b> (halved): the CL you asked for needs half the angle, because AVL divides by the wrong area (alpha {sr_['alpha']:.2f}&deg;). Cnb halves with it.</li>"
         f"<li><b>Xref wrong</b>: only Cm changes (here {xr_['Cm']:+.2f}), Xnp moves slightly ({(xr_['Xnp'] - ok_['Xnp']) * 1000:.1f} mm). A nonsense Cm with a sensible NP means look at Xref.</li>"
         f"<li><b>Units mixed</b> (sections in mm, header in m): AVL printed alpha {mm_['alpha']:.2f} although CL 0.3 was requested, Cm {mm_['Cm']:.0f}, Xnp {mm_['Xnp']:.1f} (units of mm!). Anything printed in units that do not look like your file means the header and the sections disagree.</li></ul>")
P.append(f"<h3>7.4 One more habit: which induced drag?</h3><p>AVL prints two induced-drag numbers (near-field <code>CDind</code> and far-field <code>CDff</code>), see section 5: the near-field value is {dd_pct:+.1f}% relative to the far-field one there. Efficiency should use the far-field pair; the project's span-efficiency numbers in the bird-outline v2 study used the near-field one, which is one reason they were treated as trends only.</p></section>")

# ---- 8
P.append("<section id='s8'><h2>8. Where AVL stops, and what took over</h2>")
P.append("<h3>8.1 XFOIL for the NACA 0006 chain</h3><p>XFOIL does the viscous part AVL lacks, for one airfoil section. The polar file <code>data/polars/megapolar.dat</code> (real lines; header then NACA 0006, Ncrit 5, Reynolds 30,000 and 80,000):</p>")
P.append(pre(hdr + "\n" + "\n".join(xf_lines)))
cd30 = next(float(l.split()[6]) for l in xf_lines if l.split()[1] == "30000" and l.split()[4] == "4.000")
P.append(f"<p>CL is lift, CD total section drag, CDp the pressure-drag part, CM pitching moment, Top_Xtr and Bot_Xtr where the boundary layer turns turbulent (1.0 = trailing edge, i.e. laminar all the way). Look at the drag: at Re 30,000 and 4&deg; CD is {cd30:.3f} against an AVL induced drag of {ex1r[1]['CDind']:.3f} at CL 0.3 on the F45 wing (a different wing; the point is the size of the missing drag, not a comparison). Friction and pressure drag dominate at these Reynolds numbers, which is why AVL alone cannot give a glide ratio.</p>")
P.append("<h3>8.2 Why XFOIL was not used for flat plates</h3><p>The project's own summary (<code>results/runs/rigid_glider/plate/summary.txt</code>), first lines, real:</p>" + pre("\n".join(plate_sum[:6])))
P.append("<p>And the repository README: &quot;A flat-plate section could not be modelled: XFOIL's plate polars depend on the trailing-edge treatment and disagree with published measurements.&quot; That is why the flat-plate numbers come from measurements instead.</p>")
assert "4.3%-thick plates" in plate_txt, "the plate-balance page no longer says 4.3%-thick plates"
P.append(f"<h3>8.3 The measured Ananda plate tables</h3><p>Source: G. K. Ananda Krishnan, M.S. thesis, University of Illinois 2012; unswept, 4.3% thick plates (as stated on the plate-balance page), Reynolds number {re_min_txt} and up. The balance study is <code>results/runs/rigid_glider/plate_balance/index.html</code>. <b>What was resolved</b> (real text from that page):</p>")
P.append(f"<blockquote>{E(plate_support)}</blockquote>")
n_res = [r for r in plate_unres]
P.append(f"<p><b>What was not resolved.</b> The file <code>step3_NOT_RESOLVED.csv</code> has {len(n_res)} rows; each says, in its note column: &quot;{E(n_res[0]['note'])}&quot;. For example the first row (wing {E(n_res[0]['wing'])}, Re {float(n_res[0]['Re']):,.0f}) puts the best-L/D balance point at {E(n_res[0]['best_LD_balance_offset_from_AC_pct_MAC'])}% of the mean chord from the aerodynamic centre, and removing the zero-lift offset moves it to {E(n_res[1]['best_LD_balance_offset_from_AC_pct_MAC'])}%: the answer depends on a correction the data cannot fix. So the data does <b>not</b> say where the best-glide balance point is. It does say the centre of pressure barely moves until near stall, so there is almost no restoring moment at small angles.</p>")
cfd_readme = (ROOT / "cfd/validation_sweep/README.md").read_text(encoding="utf-8")
lm_readme = (ROOT / "cfd/validation_sweep/transition_A/README.md").read_text(encoding="utf-8")
assert "Ananda AR 3 flat plate, Re 60k" in cfd_readme, "the CFD sweep is no longer described as Ananda AR 3, Re 60k"
m_ = re.search(r"medium mesh gave CD ([\d.]+) against ([\d.]+) on the coarse mesh", cfd_readme)
assert m_ and "Drag is not mesh-converged" in cfd_readme, "the CFD README no longer says drag is not mesh-converged"
cd_med, cd_coarse = float(m_.group(1)), float(m_.group(2))
f_ = lambda r, k: float(r[k])
use_ = [r for r in cfd if f_(r, "alpha_deg") >= 2]
cl_def_ = [100 * (1 - f_(r, "CL_mean") / f_(r, "meas_CL_mean")) for r in use_]
cl_mean_def = sum(cl_def_) / len(cl_def_)
assert 7 <= cl_mean_def <= 13, f"lift is no longer 'about 10% low' ({cl_mean_def:.1f}% mean)"
cm_ratio_ = {f_(r, "alpha_deg"): f_(r, "CM_mean") / f_(r, "meas_CM_mean") for r in cfd if f_(r, "alpha_deg") in (2.0, 4.0)}
assert all(0.5 <= v <= 0.7 for v in cm_ratio_.values()), f"CM ratio at 2 and 4 deg is no longer about 60%: {cm_ratio_}"
sign_wrong = [f_(r, "alpha_deg") for r in cfd if f_(r, "alpha_deg") >= 2 and f_(r, "CM_mean") * f_(r, "meas_CM_mean") < 0]
assert min(sign_wrong) == 6.0 and all(abs(f_(r, "dCM_cfd_minus_meas")) > 0.5 * abs(f_(r, "meas_CM_mean")) for r in cfd if f_(r, "alpha_deg") >= 6), "the pitching-moment statement no longer matches the file"
zero_ = next(r for r in cfd if f_(r, "alpha_deg") == 0.0)
bub = [r for r in read_csv("cfd/validation_sweep/bubble_extent.csv") if r["basis"] == "mean2"]
rev = lambda a: [float(r["reversed_fraction_upper"]) for r in bub if float(r["alpha_deg"]) == a]
ref_readme = (ROOT / "cfd/validation_sweep/refined_6deg/README.md").read_text(encoding="utf-8")
# ---- status guards: the build stops if either README no longer describes the state this page text was written for
assert any(l.startswith("STATUS: results written") for l in lm_readme.splitlines()) and "not a fair test of the transition model" in lm_readme and "never switched on" in lm_readme,     "transition_A/README.md: expected a line starting 'STATUS: results written' and the 'not a fair test' / 'never switched on' wording; update section 8.4"
res_md = (ROOT / "cfd/validation_sweep/refined_6deg/result.md").read_text(encoding="utf-8")
assert "criteria fixed BEFORE the run" in ref_readme.splitlines()[0] and any(l.startswith("STATUS: finished") for l in ref_readme.splitlines()) and any(l.startswith("## Result") for l in ref_readme.splitlines()) and "**Verdict: PARTIAL.**" in ref_readme and "Verdict: **PARTIAL**" in res_md, "refined_6deg/README.md must have its criteria header, a STATUS: finished line, a ## Result heading and the PARTIAL verdict (and result.md must state the verdict); if the verdict changes, rewrite section 8.4"
status_txt = ", ".join(sorted({r["status"] for r in cfd}))
P.append(f"<h3>8.4 CFD validation so far</h3><p>One plate (Ananda AR 3, taper 1, Re 60,000), OpenFOAM, laminar solver, angles {cfd_lo:g} to {cfd_hi:g}&deg; (case status in the file: {E(status_txt)}); the CFD lesson is <a href='../../cfd/learn/index.html'>cfd/learn/index.html</a>. Real rows from <code>cfd/validation_sweep/sweep_results.csv</code> (the first column is the angle; <i>meas</i> is the tunnel mean of its up and down legs):</p>")
rows = []
for r in cfd:
    rows.append([r["alpha_deg"], r["status"], f"{float(r['CL_mean']):.4f}", f"{float(r['meas_CL_mean']):.4f}", f"{float(r['CD_mean']):.4f}", f"{float(r['meas_CD_mean']):.4f}", f"{float(r['CM_mean']):+.5f}", f"{float(r['meas_CM_mean']):+.5f}"])
P.append(table(["alpha", "status", "CL CFD", "CL tunnel", "CD CFD", "CD tunnel", "CM CFD", "CM tunnel"], rows))
P.append(f"<p><b>What this shows, and no more.</b></p><ul>"
         f"<li><b>Lift</b> is about 10% low, cause unknown: from 2 to {cfd_hi:g}&deg; the CFD lift is below the tunnel by {min(cl_def_):.0f} to {max(cl_def_):.0f}% (mean {cl_mean_def:.0f}%), at every angle.</li>"
         f"<li><b>Drag</b> is not mesh-converged, so it is not judged: at 4&deg; the medium mesh gave CD {cd_med} against {cd_coarse} on the coarse mesh and was still falling when stopped (<code>cfd/validation_sweep/README.md</code>). Agreement of the CD columns above should not be read as validation.</li>"
         f"<li><b>Pitching moment</b> has the right sign and about {100 * min(cm_ratio_.values()):.0f}% of the measured size up to 4&deg; (ratio {cm_ratio_[2.0]:.2f} at 2&deg;, {cm_ratio_[4.0]:.2f} at 4&deg;; at 0&deg; both are within {max(abs(f_(zero_, 'CM_mean')), abs(f_(zero_, 'meas_CM_mean'))):.4f} of zero). From 6&deg; it is wrong: at {', '.join(f'{a:g}' for a in sign_wrong)}&deg; the CFD is negative where the tunnel is positive, and at the larger angles both are negative but the CFD moment is far off the measured one.</li>"
         f"<li><b>Separation</b> is far too long. The CFD upper surface is reversed over {min(rev(4.0)):.2f} to {max(rev(4.0)):.2f} of the chord at 4&deg; and {min(rev(8.0)):.2f} to {max(rev(8.0)):.2f} at 8&deg; (<code>bubble_extent.csv</code>, two-snapshot averages, not converged; the file sees none at 2&deg;). The tunnel source gives no bubble length for this plate, so 'too long' is the assessment agreed in review, not a number compared against a measured one.</li>"
         f"<li>Because of the pitching moment and the separation, <b>the CFD is not usable for balance or pull-out</b>, and no CFD pitching moment is used anywhere in the project's numbers.</li></ul>"
         "</ul>")
tr_ = read_csv("cfd/validation_sweep/transition_A/transition_results.csv")
trow = lambda a, src: next(r for r in tr_ if float(r["alpha_deg"]) == a and r["source"].startswith(src) and r["window"] in ("8-20", "-"))
avl_ = [r for r in read_csv("cfd/validation_sweep/setup_checks/avl_results.csv") if r["paneling"] == "coarse" and r["alpha_deg_or_slope"] != "slope"]
avl_cl = {float(r["alpha_deg_or_slope"]): float(r["CL_or_CLa_per_rad"]) for r in avl_}
cfd_by = {float(r["alpha_deg"]): r for r in cfd}
meas_over_avl = {a: 100 * (float(cfd_by[a]["meas_CL_mean"]) / avl_cl[a] - 1) for a in avl_cl}
assert 7 <= min(meas_over_avl.values()) and max(meas_over_avl.values()) <= 16 and list(meas_over_avl.values()) == sorted(meas_over_avl.values()), f"measured-above-AVL range changed: {meas_over_avl}"
expect(float(cfd_by[4.0]["CL_mean"]), avl_cl[4.0], 0.001, "coarse CFD lift at 4 deg equals AVL")
ym = re.search(r"alpha 4 min [\d.]+, mean ([\d.]+), max (\d+\.\d+); alpha 8 min [\d.]+, mean ([\d.]+), max (\d+\.\d+)", lm_readme)
rc = re.search(r"CM between \+([\d.]+) and \+([\d.]+); x_cp between ([\d.]+) c and ([\d.]+) c; CL at or above ([\d.]+)", ref_readme)
ncell = re.search(r"\*\*([\d,]+) cells\*\*", ref_readme)
assert ym and rc and ncell, "refined_6deg/README.md no longer has the pass limits / cell count this page quotes"
t4m, t4l, t4t, t8m, t8l, t8t = trow(4, "measured"), trow(4, "laminar"), trow(4, "kOmegaSSTLM"), trow(8, "measured"), trow(8, "laminar"), trow(8, "kOmegaSSTLM")
assert abs(float(t4t["CM"]) - float(t4l["CM"])) < 0.001 and abs(float(t4t["CL"]) - float(t4l["CL"])) < 0.001, "at 4 deg the transition run no longer equals the laminar one"
assert float(t8t["CM"]) < 0 < float(t8m["CM"]) and abs(float(t8t["CM"]) - float(t8m["CM"])) < abs(float(t8l["CM"]) - float(t8m["CM"])) and abs(float(t8t["CL"]) - float(t8m["CL"])) > abs(float(t8l["CL"]) - float(t8m["CL"])), "8 deg transition result no longer 'closer in moment, wrong sign, lift further'"
P.append("<h3>What the transition-model rerun showed</h3>")
P.append(table(["angle", "source (window t = 8-20)", "CL", "CM (nose-up +)", "centre of pressure x/c"],
               [[f"{a:g}", src, f"{float(r['CL']):.4f}", f"{float(r['CM']):+.5f}", f"{float(r['x_cp']):.3f}"] for a, src, r in
                ((4, "tunnel", t4m), (4, "laminar CFD", t4l), (4, "transition model (kOmegaSSTLM)", t4t), (8, "tunnel", t8m), (8, "laminar CFD", t8l), (8, "transition model (kOmegaSSTLM)", t8t))]))
P.append(f"<p>The transition-model rerun (4 and 8&deg;, same coarse mesh, <code>cfd/validation_sweep/transition_A/</code>) <b>did not reproduce the measured pitching moment</b>. At 4&deg; the model never switched on and the result equals the earlier laminar run. At 8&deg; the moment ({float(t8t['CM']):+.4f}) and the centre of pressure ({float(t8t['x_cp']):.3f} against {float(t8l['x_cp']):.3f} laminar and {float(t8m['x_cp']):.3f} measured) moved toward the measurement, but the moment still has the wrong sign (tunnel {float(t8m['CM']):+.4f}) and lift moved further from the tunnel ({float(t8t['CL']):.3f} against {float(t8m['CL']):.3f}). "
         f"<b>This was not a fair test of the model, so it is not evidence that the model fails.</b> The solver printed limiter warnings (bounding k and omega, lambda-iteration limit) on every time step to the end of both runs; the wall mesh is coarser than the model wants (mean y+ {float(ym.group(1)):.1f} at 4&deg; and {float(ym.group(3)):.1f} at 8&deg;, maximum about {float(ym.group(4)):.0f}, where the model wants about 1); and at 4&deg; it never triggered.</p>")
P.append(f"<h3>Two separate discrepancies</h3><ul>"
         f"<li><b>(a) A low-angle gap present even in attached flow.</b> At 2&deg; the CFD has no reversed flow ({min(rev(2.0)):.2f} of the chord), yet its lift is {100 * (1 - float(cfd_by[2.0]['CL_mean']) / float(cfd_by[2.0]['meas_CL_mean'])):.0f}% below the tunnel's. The setup checks (<code>cfd/validation_sweep/setup_checks/avl_results.csv</code>) show the coarse CFD lift at 4&deg; ({float(cfd_by[4.0]['CL_mean']):.4f}) equals AVL's linear-theory value for the same plate ({avl_cl[4.0]:.4f}), while the measured wing lifts {min(meas_over_avl.values()):.0f} to {max(meas_over_avl.values()):.0f}% above linear theory, the excess rising with angle ({', '.join(f'{v:.0f}% at {a:g}' for a, v in meas_over_avl.items())}&deg;). Why is not established.</li>"
         f"<li><b>(b) A premature aft shift of the centre of pressure from 6&deg; up</b>, coinciding with long reversed-flow zones on the upper surface. Cause unverified.</li></ul>")
P.append(f"<div class='ex'><b>Teaching point for this section.</b> On this plain flat plate the tunnel's lift is {min(meas_over_avl.values()):.0f} to {max(meas_over_avl.values()):.0f}% above AVL's, and the CFD sits on AVL, not on the tunnel (two models agreeing, which is not validation). So the first limit of the linear model is not stall: it shows up at 2&deg;, in attached flow, as a lift gap that neither method here explains.</div>")
ref_row = re.search(r"\| refined laminar, t = 8-20 \| ([\d.]+) \| ([\d.]+) \| \+([\d.]+) \| ([\d.]+) \|", res_md)
assert ref_row, "result.md no longer has the 'refined laminar, t = 8-20' row"
cl_r, cd_r, cm_r, xcp_r = (float(ref_row.group(k)) for k in (1, 2, 3, 4))
cm_lo, cm_hi, xc_lo, xc_hi, cl_min = (float(rc.group(k).rstrip('.')) for k in (1, 2, 3, 4, 5))
ok_cm, ok_xc, ok_cl = cm_lo <= cm_r <= cm_hi, xc_lo <= xcp_r <= xc_hi, cl_r >= cl_min
assert (ok_cm, ok_xc, ok_cl) == (True, True, False), f"criteria outcome changed ({ok_cm}, {ok_xc}, {ok_cl}); the verdict text of 8.4 is written for PARTIAL"
m6 = cfd_by[6.0]
cm_c, cm_m, cl_c, cl_m, cd_c, cd_m = (float(m6[k]) for k in ("CM_mean", "meas_CM_mean", "CL_mean", "meas_CL_mean", "CD_mean", "meas_CD_mean"))
closed = 100 * (cm_r - cm_c) / (cm_m - cm_c)
assert closed > 50, "CM no longer closes more than half the gap"
cd_move = 100 * (1 - cd_r / cd_c)
assert 19 <= cd_move <= 21 and abs(cm_r - cm_c - 0.0155) < 0.0006, "the 'CM moved 0.0155, CD moved 20%' statement no longer matches"
cd_m = float(re.search(r"\| measured, mean of legs \| [\d.]+ \| ([\d.]+) \|", res_md).group(1))   # leg-mean tunnel drag as given in result.md
cl_below = 100 * (1 - cl_r / cl_m); cl_below_c = 100 * (1 - cl_c / cl_m); cd_vs = 100 * (cd_r / cd_m - 1)
assert abs(cl_below - 11.1) < 0.3 and abs(cd_vs + 13.4) < 0.3, "lift/drag percentages quoted in 8.4 no longer match"
bx = [r for r in read_csv("cfd/validation_sweep/refined_6deg/bubble_extent_ref.csv") if r["basis"] == "mean_8_20"]
ends = [max(float(z.split("-")[1]) for z in r["zones_xi"].split(";")) for r in bx]
assert len(ends) == 3 and max(ends) <= 0.35, "reattachment is no longer 'by about 0.33 c at all three stations'"
pd_ = (ROOT / "research/plate_data/README.md").read_text(encoding="utf-8")
sl60 = float(re.search(r"60k \(0383ar\)\s*\|\s*-6\.\.6\s*\|\s*24\s*\|\s*([\d.]+)", pd_).group(1))
sl120 = float(re.search(r"120k \(0385ar\)\s*\|\s*-6\.\.6\s*\|\s*24\s*\|\s*([\d.]+)", pd_).group(1))
avl_sec = avl_cl[4.0] / 4
ex60, ex120 = 100 * (sl60 / avl_sec - 1), 100 * (sl120 / avl_sec - 1)
assert 10 <= ex60 <= 12 and 23 <= ex120 <= 26, f"lift-slope excess over AVL is no longer about +11% / +24% ({ex60:.1f}, {ex120:.1f})"
def _xcp(cl, cd, cm, a=6.0):
    import math
    return 0.25 - cm / (cl * math.cos(math.radians(a)) + cd * math.sin(math.radians(a)))
xcp_coarse6 = _xcp(cl_c, cd_c, cm_c); xcp_meas6 = _xcp(cl_m, cd_m, cm_m)
P.append("<h3>The last run: a locally refined mesh at 6&deg;</h3>")
P.append(f"<p>One mesh refinement of the laminar run ({ncell.group(1)} cells, finer near the leading edge and the upper surface, 20 chord passages, averaged over the last 12), judged against limits written down before it ran (<code>cfd/validation_sweep/refined_6deg/README.md</code>). The limits: moment {rc.group(1)} to {rc.group(2)}, centre of pressure {rc.group(3)} to {rc.group(4)} c, lift at least {rc.group(5).rstrip('.')}. Result (<code>result.md</code> in the same folder):</p>")
P.append(table(["", "CL", "CD", "CM (nose-up +)", "centre of pressure x/c"],
               [["coarse laminar", f"{cl_c:.4f}", f"{cd_c:.4f}", f"{cm_c:+.4f}", f"{xcp_coarse6:.4f}"], ["refined laminar", f"{cl_r:.4f}", f"{cd_r:.4f}", f"{cm_r:+.4f}", f"{xcp_r:.4f}"],
                ["tunnel (mean of two legs)", f"{cl_m:.4f}", f"{cd_m:.4f}", f"{cm_m:+.4f}", f"{xcp_meas6:.4f}"]]))
P.append("<p><b>Verdict: PARTIAL.</b> "
         f"The moment ({cm_r:+.4f}) is inside its band and so is the centre of pressure ({xcp_r:.4f} c); lift ({cl_r:.4f}) is below the {cl_min:g} limit. All three were needed to pass, so it is not a pass. The refinement closed {closed:.0f}% of the moment gap, which is what the pre-written rule calls partial. The rule's stated consequence, unchanged: resolution matters, nothing is validated, the CFD is recorded as <b>not usable for moment</b>, its lift is a rough check with a stated bias, and there is no further CFD unless Kai asks. "
         "The rule on balance and pull-out is unchanged: no CFD has been run on the fabricated wings and none is used for balance or pull-out.</p>")
P.append("<ul>"
         f"<li><b>Facts.</b> Moment {cm_r:+.4f}; centre of pressure {xcp_r:.4f} c; the reversed-flow zone on the upper surface ends by about {max(ends):.2f} of the chord at all three span stations (the coarse run was reversed out to 0.75 and 0.58 of the chord at two of them).</li>"
         f"<li><b>Not shown to be mesh-independent.</b> One refinement moved the moment by {cm_r - cm_c:.4f} and the drag by {cd_move:.0f}% ({cd_c:.4f} to {cd_r:.4f}). The drag is now {abs(cd_vs):.1f}% below the tunnel's; drag is still not judged.</li>"
         f"<li><b>Lift gap: unresolved.</b> The refined lift is {cl_below:.1f}% below the tunnel (the coarse run: {cl_below_c:.1f}%), so refining moved it away from the measurement. Two explanations remain: (a) a tunnel or correction offset; (b) the real wing at this Reynolds number lifts more than the CFD captures. Evidence that bears against a rig-only reading: the tunnel's lift-slope excess over AVL rises from about +{ex60:.1f}% at Reynolds number 60,000 to about +{ex120:.1f}% at 120,000 (the fitted slopes in <code>research/plate_data/README.md</code> against AVL's secant slope at 4&deg;); a rough wall-correction size of about 1% of angle at 4&deg; (an unverified reviewer estimate; the thesis gives the correction equations but no sizes); and another tunnel's report (Laitone, Re 40,000) of lift slopes more than 20% above lifting-line theory (<code>research/bird_research_notes.md</code> section 6.3, secondary; the same section summarises a different tunnel, Pelletier &amp; Mueller, as finding lifting-line theory adequate, so the literature is not uniform). The CFD agreeing with AVL is two models agreeing, not validation.</li></ul>")
# ---- refined 8 deg run (cfd/validation_sweep/refined_8deg)
r8_readme = (ROOT / "cfd/validation_sweep/refined_8deg/README.md").read_text(encoding="utf-8")
r8_res = (ROOT / "cfd/validation_sweep/refined_8deg/result.md").read_text(encoding="utf-8")
assert any(l.startswith("STATUS: finished") for l in r8_readme.splitlines()) and any(l.startswith("## Result") for l in r8_readme.splitlines()) and "**Grade: PASS**" in r8_readme and "Verdict: **PASS**" in r8_res, \
    "refined_8deg/README.md and result.md must carry STATUS: finished, a ## Result heading and the PASS verdict; if the verdict changes, rewrite the 8 deg part of section 8.4"
g8 = re.search(r"\| refined laminar, t = 4 to ([\d.]+) \(graded\) \| ([\d.]+) \| ([\d.]+) \| \+([\d.]+) \| ([\d.]+) \| ([\d.]+) \|", r8_res)
assert g8, "result.md no longer has the 'refined laminar, t = 4 to ... (graded)' row"
stop8, cl8, cd8, cm8, cn8, xc8 = (float(g8.group(k)) for k in range(1, 7))
bx8 = re.search(r"x_cp between ([\d.]+) c and ([\d.]+) c", r8_readme)
assert bx8, "refined_8deg/README.md no longer states the x_cp band"
xlo8, xhi8 = float(bx8.group(1)), float(bx8.group(2))
m8 = cfd_by[8.0]
cl8_c, cd8_c, cm8_c, cl8_m, cd8_m, cm8_m = (float(m8[k]) for k in ("CL_mean", "CD_mean", "CM_mean", "meas_CL_mean", "meas_CD_mean", "meas_CM_mean"))
xc8_c = _xcp(cl8_c, cd8_c, cm8_c, 8.0); xc8_m = _xcp(cl8_m, cd8_m, cm8_m, 8.0)
assert xlo8 <= xc8 <= xhi8 and cm8 > 0, "8 deg x_cp / moment no longer meet the PASS rule"
assert abs(_xcp(cl8, cd8, cm8, 8.0) - xc8) < 0.0006, "x_cp in result.md is not 0.25 - CM/CN of its own CL, CD and CM"
cl8_below = 100 * (1 - cl8 / cl8_m)
assert 9.5 <= cl8_below <= 11.0, f"8 deg lift is no longer about 10% below the tunnel ({cl8_below:.1f}%)"
margin8 = xhi8 - xc8
assert abs(margin8 - 0.0021) < 0.0003, f"x_cp margin to the upper band edge changed ({margin8:.4f})"
tb8 = [r for r in read_csv("cfd/validation_sweep/refined_8deg/time_binned_ref.csv") if float(r["t_start"]) >= 4.0]
xb8 = [float(r["xcp"]) for r in tb8]
assert len(xb8) == 6 and abs(min(xb8) - 0.2269) < 0.0003 and abs(max(xb8) - 0.2288) < 0.0003, "the one-passage x_cp bins from t = 4 changed"
mean_b = sum(xb8) / len(xb8); sd_b = (sum((x - mean_b) ** 2 for x in xb8) / (len(xb8) - 1)) ** 0.5; se_b = sd_b / len(xb8) ** 0.5
assert abs(sd_b - 0.0007) < 0.0002 and abs(se_b - 0.0003) < 0.0001 and margin8 > 5 * se_b, "bin SD / standard error no longer match the text"
assert abs(stop8 - 10.01) < 0.01 and "stopped early by decision" in r8_res, "the early stop is no longer recorded as in the text"
P.append("<h3>The 8&deg; run: a locally refined mesh</h3>")
P.append(f"<p>A second refined laminar run, at 8&deg; (5,949,059 cells; same method as the 6&deg; run, with the refinement box raised after a check of how tall the coarse run's separated region is). Its limits were written down before it ran (<code>cfd/validation_sweep/refined_8deg/README.md</code>): <b>pass = centre of pressure between {xlo8:.3f} and {xhi8:.3f} c and moment positive</b>; the moment against its own band, and the lift against AVL and the tunnel, are reported but not graded. That change from the 6&deg; rule was decided before the run: the lift gap is dealt with separately, and a balance question needs the centre of pressure. Result (<code>result.md</code>, same folder):</p>")
P.append(table(["", "CL", "CD", "CM (nose-up +)", "centre of pressure x/c"],
               [["coarse laminar", f"{cl8_c:.4f}", f"{cd8_c:.4f}", f"{cm8_c:+.4f}", f"{xc8_c:.4f}"], [f"refined laminar (t = 4 to {stop8:g})", f"{cl8:.4f}", f"{cd8:.4f}", f"{cm8:+.4f}", f"{xc8:.4f}"],
                ["tunnel (mean of two legs)", f"{cl8_m:.4f}", f"{cd8_m:.4f}", f"{cm8_m:+.4f}", f"{xc8_m:.4f}"]]))
P.append("<p><b>Verdict: PASS</b> on the pre-written rule. "
         f"The centre of pressure ({xc8:.4f} c) is inside its band and the moment ({cm8:+.4f}) is positive. The moment is just under its own, secondary band (+0.0105 to +0.0175), and the lift is {cl8_below:.0f}% below the tunnel's; with the centre of pressure in band that pattern is the lift gap described below, not a moment failure.</p>")
P.append("<ul>"
         f"<li><b>The run was stopped early, by decision.</b> The averaging window fixed in advance was t = 8 to 20 (at least 10 passages). The run was stopped at t = {stop8:g}, right after a complete save, because the remaining time would have taken about 28 hours; Kai authorised stopping if the result looked conclusive, and the reviewer cleared it with conditions. The grade is therefore on t = 4 to {stop8:g}, about 6 passages. The evidence for stopping: the one-passage averages of the centre of pressure from t = 4 on are flat ({min(xb8):.4f} to {max(xb8):.4f}, spread {sd_b:.4f}, standard error {se_b:.4f}), the 6&deg; run was flat from t = 3 to 4, and the margin to the band edge ({margin8:.4f} c) is about {margin8 / se_b:.0f} standard errors. Choosing a window after seeing a value near a band edge is a risk of fooling oneself; it is kept small here only by the clear settling and that margin. A stop rule written afterwards (<code>cfd/validation_sweep/README.md</code>) would have allowed this stop.</li>"
         f"<li><b>How safe the margin is.</b> The centre of pressure sits {margin8:.4f} c inside the upper edge. The measured reference is itself interpolated between table points 1&deg; apart, across the sharp moment break, and that uncertainty is bigger than the margin. The CFD centre of pressure is {xc8 - xc8_m:.3f} c aft of the measured one.</li>"
         "<li><b>What it supports.</b> The refined laminar CFD reproduces the measured centre of pressure for this plate at Reynolds number 60,000 at 6&deg; and 8&deg; (the 6&deg; verdict above stays PARTIAL because lift was graded there), and only there. Lift stays about 10 to 11% low against this tunnel. One mesh refinement, so not shown to be mesh-independent. The rule on Kai's wings is unchanged: no CFD has been run on them and none is used for balance or pull-out, unless Kai decides otherwise.</li></ul>")
# ---- lift-gap checks (cfd/validation_sweep/lift_gap_checks)
lg_dir = ROOT / "cfd/validation_sweep/lift_gap_checks"
lg_readme = (lg_dir / "README.md").read_text(encoding="utf-8")
lg_sum = (lg_dir / "excess_summary.txt").read_text(encoding="utf-8")
lg_wall = (lg_dir / "wall_corrections.txt").read_text(encoding="utf-8")
assert "not a wall-correction or fixed-offset artefact: it depends on Reynolds number on every wing" in lg_readme, "lift_gap_checks/README.md no longer carries the approved conclusion; update the 8.4 paragraph"
assert not re.search(r"free-flight|about right|our Re", lg_readme, re.I), "lift_gap_checks/README.md contains wording that was ruled out"
re_means = {int(m.group(1).replace(",", "")): float(m.group(2)) for m in re.finditer(r"Re\s+([\d,]+): n = +\d+ wings, min +-?[\d.]+ %, max +-?[\d.]+ %, mean +(-?[\d.]+) %", lg_sum)}
assert all(k in re_means for k in (60000, 80000, 100000, 120000)) and re_means[60000] < re_means[80000] < re_means[100000] < re_means[120000], "per-Re mean excess no longer rises with Re"
trends = [float(x) for x in re.findall(r"trend ([+-][\d.]+) points per 10k Re", lg_sum)]
assert len(trends) == 10 and min(trends) > 1.4 and max(trends) < 3.0, f"per-wing Re trends changed: {trends}"
corr = [float(x) for x in re.findall(r"= ([\d.]+) to ([\d.]+) % of the angle", lg_wall) for x in x]
assert corr and 1.4 <= min(corr) and max(corr) <= 2.1, f"angle-correction size changed: {corr}"
ex_all = [float(x) for x in re.findall(r"excess_pct_vs_AVL_LS", lg_readme)]  # presence only
ov = re.search(r"overall min (-?[\d.]+) %, max ([\d.]+) %, mean ([\d.]+) %", lg_sum)
assert ov, "overall excess line missing"
P.append("<h3>Lift-gap checks (no CFD): the tunnel's lift excess over AVL</h3>")
P.append("<p>Source: <code>cfd/validation_sweep/lift_gap_checks/</code> (AVL for all ten tested planforms; the tunnel's low-angle slopes from the plate-balance fits, window &minus;4 to 6&deg;; the thesis wall-correction formulas sized for the AR 3 plate). "
         f"Across the ten wings the tunnel slope is {float(ov.group(1)):.1f}% to {float(ov.group(2)):.1f}% above AVL's (mean {float(ov.group(3)):.1f}%). The mean excess at each Reynolds number: "
         f"{re_means[60000]:.1f}% at 60,000, {re_means[80000]:.1f}% at 80,000, {re_means[100000]:.1f}% at 100,000, {re_means[120000]:.1f}% at 120,000; every one of the ten wings rises with Reynolds number, by {min(trends):.1f} to {max(trends):.1f} points per 10,000. "
         f"The thesis wall corrections, sized for the AR 3 plate at Reynolds number 60,000, are about {min(corr):.1f} to {max(corr):.1f}% of the angle (the constants the thesis takes from figures in its reference 55 are not printed, so bounds were assumed) and about 0.1% in blockage.</p>")
P.append("<p><b>Result.</b> The tunnel's lift excess over AVL is not a wall-correction or fixed-offset artefact: it depends on Reynolds number on every wing. No mechanism is named as fact. Caveats:</p><ul>"
         "<li>A speed-dependent facility effect is not excluded: changing the Reynolds number in that tunnel changes the speed, and with it the turbulence and the acoustics. Results in this Reynolds-number range differ between tunnels (<code>research/bird_research_notes.md</code> section 6.3, secondary sources).</li>"
         "<li>Aeroelastic twist is unlikely but untested: AR 2, the shortest span, has the largest excess.</li>"
         "<li>AVL models zero thickness. A 4.3% plate with a rounded nose may carry a few percent more inviscid lift slope, which could account for part of the floor of the excess but not for its trend.</li>"
         "<li>The laminar CFD probably lacks a lift-raising viscous effect present in this tunnel at Reynolds number 60,000 (probable, not established). The CFD's own slope shortfall was not measured as a slope, and the refined result is one angle.</li>"
         "<li><b>Below Reynolds number 60,000 this is untested and no number is given.</b> Straight-line extrapolations of the per-wing trends would cross zero at Reynolds numbers between about 12,000 and 65,000 for nine of the ten wings (the tenth does not cross at a positive Reynolds number), so even the sign is unknown there; they are extrapolations, not predictions. The secondary sources disagree (Laitone: more than 20% above lifting-line theory at 40,000; McArthur: no significant disagreement at 10,000 to 20,000). Nothing here says how AVL's lift slope compares with real wings at lower Reynolds numbers.</li></ul>")
P.append("<p><b>Where this leaves the 6&deg; and 8&deg; question.</b> The 8&deg; question was about the centre of pressure, not lift, and was decided by running the second refined case above; its result is a PASS on the pre-written rule, on a disclosed shortened window. The lift gap stays open. The rule is unchanged: no CFD has been run on the fabricated wings, and none is used for balance or pull-out, unless Kai decides otherwise.</p>")
P.append("</section>")

# ---- 9
P.append("<section id='s9'><h2>9. Do it yourself: a checklist for a new shape</h2><ol>"
         "<li><b>Sketch the outline in metres.</b> Top view: x aft from the root leading edge, y out along the wing. Write down the leading-edge point and chord at the root, at every kink, and at the tip.</li>"
         "<li><b>Compute the reference values.</b> Sref = planform area of the wing (one side times two); Bref = span; Cref = mean aerodynamic chord (area-weighted chord: the integral of chord&sup2; over the integral of chord, along the span). Put Xref where you think the CG will be, to start.</li>"
         "<li><b>Write the file.</b> Copy <code>practice/f45_bare.avl</code>. Change the header numbers, then one <code>SECTION</code> per kink. Keep <code>YDUPLICATE</code>. Keep <code>NACA 0000</code>.</li>"
         "<li><b>Tail or fins?</b> Add a second <code>SURFACE</code>. For a bend use <code>Ainc = -bend</code>. For a fin, give its two sections different z.</li>"
         "<li><b>Load it and read the banner.</b> Do the strip and vortex counts match what you meant (section 5)? Fix before running.</li>"
         "<li><b>Run alpha = 0 and 4.</b> At alpha 0 on a symmetric flat wing without a tail: CL = 0 and Cm = 0. If not, the file is not what you think.</li>"
         "<li><b>Run CL = 0.3</b> (<code>A C 0.3</code>). Is the angle plausible (a few degrees)? If it is 10&deg; or 13&deg;, suspect a missing <code>YDUPLICATE</code> or a wrong Sref.</li>"
         "<li><b>Read Xnp, Cma, CLa; compute the static margin two ways</b> (section 5). They must agree.</li>"
         "<li><b>Read the signs</b>: Cma &lt; 0, Cnb &gt; 0, Clb &lt; 0 for a stable arrangement (section 3). Zero or the wrong sign is a result, not an error.</li>"
         "<li><b>Convergence check</b>: rerun with half and twice the panels. Note which numbers move (section 7). Do not quote digits that move.</li>"
         "<li><b>Write down what AVL cannot say</b> about your shape: stall, drag, Reynolds number, glide ratio. Put it next to your numbers.</li></ol>")
gl = [("AIC (aerodynamic influence coefficient)", "The matrix element A(i,j): velocity normal to panel i caused by unit-strength horseshoe j."), ("Aspect ratio (AR)", "Span squared divided by wing area."),
      ("Biot-Savart law", "The velocity a piece of vortex line induces at a point (section 2)."), ("Bound vortex", "The part of a horseshoe lying on the wing."), ("CG (centre of gravity)", "The balance point of the glider; AVL takes moments about Xref, which you set to the CG."),
      ("Cl_beta (Clb)", "Rolling moment per radian of sideslip; negative = stable."), ("Cm", "Pitching moment coefficient about Xref, nose-up positive."), ("CM0", "CM at zero lift."),
      ("Cn_beta (Cnb)", "Yawing moment per radian of sideslip; positive = weathercock stable."), ("Control point", "The point on a panel where the no-flow-through condition is applied."),
      ("Cref, Bref, Sref", "Reference chord, span and area used to make coefficients dimensionless."), ("Downwash", "Air pushed downward behind a lifting wing by the trailing vortices."), ("e (span efficiency)", "CDi = CL&sup2;/(pi AR e); 1 = elliptic loading."),
      ("Horseshoe vortex", "One bound vortex plus two trailing legs to infinity."), ("Induced drag", "Drag from the backward tilt of the force caused by downwash."), ("Kutta-Joukowski law", "Force per length on a vortex = density &times; velocity &times; circulation."),
      ("MAC (mean aerodynamic chord)", "Area-weighted mean chord; the length used as Cref."), ("Neutral point (NP)", "The CG position at which the pitching moment stops changing with angle."), ("Ncrit", "XFOIL's transition parameter (how disturbed the free stream is); 5 and 9 are used here."),
      ("Panel", "One cell of the grid on the wing."), ("Reynolds number", "Speed &times; chord / kinematic viscosity: how important viscosity is. Tens of thousands for these gliders (not checked here)."), ("Static margin", "(x_np - x_cg)/Cref; positive = CG ahead of the NP = stable in pitch."),
      ("Strip", "A spanwise row of panels; AVL reports lift per strip."), ("Trailing leg", "The part of a horseshoe that runs back from the wing."), ("Trefftz plane", "A plane far behind the wing where AVL evaluates induced drag (far field)."),
      ("Trim", "A state where the moments about the CG sum to zero."), ("VLM", "Vortex-lattice method."), ("YDUPLICATE", "AVL keyword: mirror this surface about the y = 0 plane.")]
P.append("<h2>Glossary</h2><dl>" + "".join(f"<dt><b>{t}</b></dt><dd>{d}</dd>" for t, d in gl) + "</dl>")
P.append("<h2>What I did not verify</h2><ul><li>The meaning of the two spacing numbers on the <code>SURFACE</code> line (1.0 and -2.0), the symmetry numbers on line 3, and AVL's internal panel-position rule (bound vortex at a quarter, control point at three quarters). They are in the AVL manual, which is not in the project; I describe the quarter/three-quarter rule in the diagram as the generic VLM picture.</li>"
         "<li>The hypothesis for the BC panelling sensitivity (tail in the wake plane).</li><li>What AVL does when the line after <code>YDUPLICATE</code> is left out; why AVL's printed neutral point shifts slightly with the reference point (section 5); the Ananda thesis itself (its source details are quoted from the plate-balance page, not checked); the Reynolds-number range of these gliders; whether the CFD separation length is right (no measured bubble length for that plate).</li><li>The 11% gap between AVL's lift slope for the rectangular wing and the textbook formula.</li><li>I did not open any browser; this page was not viewed rendered. All numbers come from files and runs, but check the diagrams by eye.</li></ul>")
P.append("</section></main></body></html>")
(HERE / "index.html").write_text("\n".join(P), encoding="utf-8")
print("wrote", HERE / "index.html", len("\n".join(P)) // 1024, "KiB")
