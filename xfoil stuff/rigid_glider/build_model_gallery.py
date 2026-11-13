"""One gallery page with a picture for every piece of modelling in this project.   py -B rigid_glider/build_model_gallery.py

Writes ONLY results/runs/model_gallery/index.html.  Reads (never writes, never re-runs) the existing result files.  Stdlib only; runs no AVL, XFOIL or CFD.
Every number in a caption is read from a result file at build time.  Figures that already exist are reused (linked, or for inline SVG copied); where a study
has data but no figure, a new inline SVG is drawn from its CSV.  The refined 6 degree CFD run has finished: its force history is read, read-only,
from the WSL distro with `cat`.
Tags: MODEL (a calculation), MEASURED (literature wind-tunnel data), ASSUMED (dimensions not measured)."""
import sys
sys.dont_write_bytecode = True
import csv
import html
import math
import os
import re
import statistics
import base64
from datetime import datetime
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "results/runs/model_gallery"
RG = ROOT / "results/runs/rigid_glider"
E = html.escape
INK, MUT, ACC, LNK, GRN, BAD, PAN, LINE, GRID = "#dde3ea", "#9aa7b4", "#e89a55", "#7fb4ff", "#6cc08b", "#ff8a80", "#1c2229", "#333c46", "#262e37"
PUR, YEL, CYA = "#c39bff", "#f2d16b", "#5fd0d0"
COLS = [LNK, ACC, GRN, PUR, YEL, CYA, BAD, MUT]
EMBEDDED = set()
SVG_N = [0]
NOTES = []            # disagreements / gaps noticed while building (printed at the end, not fixed)
FIGS = []             # (section id, record)


def rd(rel):
    return list(csv.DictReader((ROOT / rel).open(encoding="utf-8")))


def f(x):
    return float(x)


def rel(path):
    return os.path.relpath(path, OUT).replace("\\", "/")


# ====================================================================================================== tiny SVG chart helpers (dark)
def nice(lo, hi, n=6):
    span = (hi - lo) or 1.0
    raw = span / n
    p = 10 ** math.floor(math.log10(raw))
    step = min((m * p for m in (1, 2, 2.5, 5, 10) if m * p >= raw), default=raw)
    a = math.floor(lo / step) * step
    return [a + i * step for i in range(int(math.ceil((hi - a) / step)) + 1) if a + i * step <= hi + 1e-9 * step], step


def fmt_t(v, step):
    d = max(0, -int(math.floor(math.log10(step))) + (1 if step < 1 else 0)) if step else 2
    return f"{v:.{min(d, 4)}f}"


def T(x, y, s, size=11, anchor="start", color=INK, weight="normal", rot=None):
    r = f" transform='rotate({rot} {x:.1f} {y:.1f})'" if rot is not None else ""
    return f"<text x='{x:.1f}' y='{y:.1f}' font-family='Segoe UI,Arial,sans-serif' font-size='{size}' text-anchor='{anchor}' fill='{color}' font-weight='{weight}'{r}>{E(str(s))}</text>"


def wrap_svg(w, h, body, label):
    return f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {w} {h}' width='{w}' height='{h}' role='img' aria-label='{E(label)}' style='max-width:100%;height:auto;background:#14181d'>{body}</svg>"


def chart(title, xl, yl, series, xlim=None, ylim=None, w=640, h=380, banner=None, hlines=(), bands=(), legend_w=170, xticks=None, note=None):
    """series: dict(name, pts, color, line=True, marker=True, dash=None).  hlines: (y, label, colour).  bands: (y0, y1, label, colour)."""
    ml, mr, mt, mb = 58, legend_w + 14, 38 + (22 if banner else 0), 46
    pw, ph = w - ml - mr, h - mt - mb
    allx = [p[0] for s in series for p in s["pts"]]
    ally = [p[1] for s in series for p in s["pts"]] + [v[0] for v in hlines] + [b[0] for b in bands] + [b[1] for b in bands]
    x0, x1 = xlim or (min(allx), max(allx))
    y0, y1 = ylim or (min(ally), max(ally))
    if x1 == x0:
        x1 = x0 + 1
    if y1 == y0:
        y1 = y0 + 1
    pad = 0 if ylim else 0.06 * (y1 - y0)
    y0, y1 = y0 - pad, y1 + pad
    X = lambda v: ml + (v - x0) / (x1 - x0) * pw
    Y = lambda v: mt + ph - (v - y0) / (y1 - y0) * ph
    b = [f"<rect width='{w}' height='{h}' fill='#14181d'/>", T(ml, 20, title, 13, "start", INK, "bold")]
    if banner:
        b.append(f"<rect x='{ml}' y='26' width='{pw}' height='20' fill='#3a1f1f' stroke='{BAD}'/>" + T(ml + pw / 2, 40, banner, 12, "middle", BAD, "bold"))
    xt, xs = (xticks, 1) if xticks else nice(x0, x1)
    yt, ys = nice(y0, y1)
    for v in xt:
        if x0 - 1e-9 <= v <= x1 + 1e-9:
            b.append(f"<line x1='{X(v):.1f}' y1='{mt}' x2='{X(v):.1f}' y2='{mt + ph}' stroke='{GRID}'/>" + T(X(v), mt + ph + 15, fmt_t(v, xs) if not xticks else f"{v:g}", 10, "middle", MUT))
    for v in yt:
        if y0 - 1e-9 <= v <= y1 + 1e-9:
            b.append(f"<line x1='{ml}' y1='{Y(v):.1f}' x2='{ml + pw}' y2='{Y(v):.1f}' stroke='{GRID}'/>" + T(ml - 5, Y(v) + 3.5, fmt_t(v, ys), 10, "end", MUT))
    b.append(f"<rect x='{ml}' y='{mt}' width='{pw}' height='{ph}' fill='none' stroke='{LINE}'/>")
    for lo, hi, lab, col in bands:
        b.append(f"<rect x='{ml}' y='{Y(hi):.1f}' width='{pw}' height='{max(1, Y(lo) - Y(hi)):.1f}' fill='{col}' fill-opacity='0.16'/>" + T(ml + 4, Y(hi) + 11, lab, 10, "start", col))
    for v, lab, col in hlines:
        b.append(f"<line x1='{ml}' y1='{Y(v):.1f}' x2='{ml + pw}' y2='{Y(v):.1f}' stroke='{col}' stroke-dasharray='5 4'/>" + T(ml + pw - 3, Y(v) - 4, lab, 10, "end", col))
    for i, s in enumerate(series):
        col = s.get("color", COLS[i % len(COLS)])
        pts = sorted(s["pts"]) if s.get("sort", True) else s["pts"]
        if s.get("line", True) and len(pts) > 1:
            d = " ".join(f"{X(p[0]):.1f},{Y(p[1]):.1f}" for p in pts)
            dash = f" stroke-dasharray='{s['dash']}'" if s.get("dash") else ""
            b.append(f"<polyline points='{d}' fill='none' stroke='{col}' stroke-width='{s.get('width', 1.8)}'{dash}/>")
        if s.get("marker", True):
            for p in pts:
                b.append(f"<circle cx='{X(p[0]):.1f}' cy='{Y(p[1]):.1f}' r='{s.get('r', 3.2)}' fill='{col}' fill-opacity='{s.get('op', 1)}'/>")
    b.append(T(ml + pw / 2, h - 8, xl, 11, "middle", MUT) + T(14, mt + ph / 2, yl, 11, "middle", MUT, rot=-90))
    for i, s in enumerate(series):
        col = s.get("color", COLS[i % len(COLS)])
        ly = mt + 6 + 17 * i
        b.append(f"<line x1='{ml + pw + 12}' y1='{ly}' x2='{ml + pw + 30}' y2='{ly}' stroke='{col}' stroke-width='2.4'/>" + T(ml + pw + 35, ly + 4, s["name"], 10.5, "start", INK))
    if note:
        b.append(T(ml, h - 22, note, 10, "start", MUT))
    return wrap_svg(w, h, "".join(b), title)


def bars(title, cats, stacks, ylabel, w=640, h=380, colors=None, legend_w=190, horizontal=False):
    """Vertical stacked bars.  cats: category labels; stacks: list of (name, [values per cat])."""
    ml, mr, mt, mb = 58, legend_w + 14, 38, 74
    pw, ph = w - ml - mr, h - mt - mb
    tot = [sum(s[1][i] for s in stacks) for i in range(len(cats))]
    ymax = (max(tot) or 1) * 1.08
    yt, ys = nice(0, ymax)
    Y = lambda v: mt + ph - v / ymax * ph
    b = [f"<rect width='{w}' height='{h}' fill='#14181d'/>", T(ml, 20, title, 13, "start", INK, "bold")]
    for v in yt:
        b.append(f"<line x1='{ml}' y1='{Y(v):.1f}' x2='{ml + pw}' y2='{Y(v):.1f}' stroke='{GRID}'/>" + T(ml - 5, Y(v) + 3.5, fmt_t(v, ys), 10, "end", MUT))
    bw = pw / len(cats)
    for i, c in enumerate(cats):
        base = 0
        for k, (nm, vals) in enumerate(stacks):
            col = (colors or COLS)[k % len(colors or COLS)]
            v = vals[i]
            if v:
                b.append(f"<rect x='{ml + i * bw + bw * 0.15:.1f}' y='{Y(base + v):.1f}' width='{bw * 0.7:.1f}' height='{Y(base) - Y(base + v):.1f}' fill='{col}' fill-opacity='0.9' stroke='#14181d'/>")
                if Y(base) - Y(base + v) > 12:
                    b.append(T(ml + i * bw + bw / 2, (Y(base) + Y(base + v)) / 2 + 4, f"{v:g}", 10, "middle", "#14181d", "bold"))
            base += v
        b.append(T(ml + i * bw + bw / 2, mt + ph + 14, c, 10, "middle", MUT, rot=None) if len(c) < 9 else T(ml + i * bw + bw / 2 + 4, mt + ph + 12, c, 10, "end", MUT, rot=-30))
    b.append(f"<rect x='{ml}' y='{mt}' width='{pw}' height='{ph}' fill='none' stroke='{LINE}'/>")
    b.append(T(14, mt + ph / 2, ylabel, 11, "middle", MUT, rot=-90))
    for k, (nm, _) in enumerate(stacks):
        col = (colors or COLS)[k % len(colors or COLS)]
        ly = mt + 6 + 17 * k
        b.append(f"<rect x='{ml + pw + 12}' y='{ly - 7}' width='14' height='11' fill='{col}'/>" + T(ml + pw + 32, ly + 3, nm, 10.5, "start", INK))
    return wrap_svg(w, h, "".join(b), title)


def hranges(title, rows, xlabel, w=700, h=None, color=ACC, extra=None):
    """Horizontal range bars: rows = [(label, lo, hi)]"""
    ml, mr, mt, mb = 250, 20, 38, 40
    h = h or mt + mb + 22 * len(rows)
    pw, ph = w - ml - mr, h - mt - mb
    lo = min(0, min(r[1] for r in rows))
    hi = max(r[2] for r in rows) * 1.08
    X = lambda v: ml + (v - lo) / (hi - lo) * pw
    xt, xs = nice(lo, hi)
    b = [f"<rect width='{w}' height='{h}' fill='#14181d'/>", T(10, 20, title, 13, "start", INK, "bold")]
    for v in xt:
        b.append(f"<line x1='{X(v):.1f}' y1='{mt}' x2='{X(v):.1f}' y2='{mt + ph}' stroke='{GRID}'/>" + T(X(v), mt + ph + 14, fmt_t(v, xs), 10, "middle", MUT))
    for i, (lab, a, c) in enumerate(rows):
        y = mt + 11 + 22 * i
        b.append(T(ml - 8, y + 4, lab, 10.5, "end", INK))
        b.append(f"<rect x='{X(a):.1f}' y='{y - 6}' width='{max(3, X(c) - X(a)):.1f}' height='12' rx='2' fill='{color}' fill-opacity='0.85'/>")
    b.append(T(ml + pw / 2, h - 6, xlabel, 11, "middle", MUT))
    return wrap_svg(w, h, "".join(b), title)


# ====================================================================================================== figure registry
TAGS = {"MODEL": LNK, "MEASURED": GRN, "ASSUMED": ACC, "MODEL (CFD)": LNK}
SECTIONS = []


def section(sid, title, intro):
    SECTIONS.append((sid, title, intro))


def add(sid, title, tag, kind, body, caption, source, status, wide=False, small=False, group=""):
    """kind: 'svg' (inline markup), 'img' (path), 'imginv' (path, white plot, CSS invert), 'iframe'."""
    FIGS.append((sid, dict(title=title, tag=tag, kind=kind, body=body, caption=caption, source=source, status=status, wide=wide, small=small, group=group)))


def is_light(path):
    head = Path(path).read_bytes()[:600].decode("utf-8", "ignore").lower()
    return ('fill="white"' in head) or ("fill='white'" in head) or ("#ffffff" in head) or ('fill="#fff"' in head)


XFOIL_NAMES = ("fig6_section_cm_", "fig1_section_cm_vs_cl", "fig3_lip_moment_retained_vs_re")
XFOIL_GROUP = "Earlier XFOIL batch outputs, October 2026 (2D section polars)"


def reuse_img(sid, title, tag, path, caption, wide=False, small=False, force_plain=False, inv=False):
    p = Path(path)
    if not p.exists():
        NOTES.append(f"MISSING FILE (not shown): {p}")
        return
    grp = XFOIL_GROUP if any(p.name.startswith(n) for n in XFOIL_NAMES) else ""
    if inv or (p.suffix == ".svg" and is_light(p) and not force_plain):
        add(sid, title, tag, "imginv", str(p), caption, str(p.relative_to(ROOT)).replace("\\", "/"), "reused", wide, small, grp)
    else:
        add(sid, title, tag, "img", str(p), caption, str(p.relative_to(ROOT)).replace("\\", "/"), "reused", wide, small, grp)
    EMBEDDED.add(p.resolve())


def src(*paths):
    return ", ".join(str(Path(p)).replace("\\", "/") for p in paths)


# ====================================================================================================== 1. flat kit / plate balance
section("s1", "1. Flat kit and the plate-balance study", "Build drawings of the flat-plate wings (F45 and F60 at 250 mm span, B45 at 400 mm) and plots of the measured plate data (Ananda 2012).")
sc = rd("results/runs/rigid_glider/flat_test_v2/scale_table.csv")
kc = lambda row, pre: next(v for k, v in row.items() if k.startswith(pre))
for wing, label in (("F45", "F45 flat wing"), ("F60", "F60 flat wing"), ("B45", "B45 foamboard wing")):
    rows_w = [r for r in sc if r["wing"] == wing]
    row = next((r for r in rows_w if abs(f(kc(r, "thickness_mm")) - 0.6) < 1e-9), rows_w[0])      # the kit's default plate is 3 layers = 0.6 mm
    d = {"F45": "F45_span250", "F60": "F60_span250", "B45": "B45_span400"}[wing]
    reuse_img("s1", f"{label}: the build drawing", "MODEL", RG / "flat_test_v2" / d / "drawing.svg",
              f"Outline, tip-fin and stick layout for the {label}: span {kc(row, 'span_mm')} mm, root chord {kc(row, 'root_chord_mm')} mm, tip chord {kc(row, 'tip_chord_mm')} mm, area {kc(row, 'area_cm2')} cm². "
              f"Estimated wing mass {kc(row, 'wing_mass_g')} g at {kc(row, 'thickness_mm')} mm thickness ({row['material']}).", wide=False)
tp = RG / "flat_test_v2/B45_span400/template_half_wing_letter.svg"
if tp.exists():
    reuse_img("s1", "B45 half-wing print template", "MODEL", tp, "Paper cutting template for the B45 foamboard half wing.", small=True, force_plain=True)

pb = (RG / "plate_balance/index.html").read_text(encoding="utf-8")
pb_svgs = re.findall(r"<svg.*?</svg>", pb, flags=re.S)
s3 = rd("results/runs/rigid_glider/plate_balance/step3_summary.csv")
s5 = rd("results/runs/rigid_glider/plate_balance/step5_trends.csv")
ac = [f(r["AC_ahead_of_quarter_chord_pct"]) for r in s3]
re_lo, re_hi = min(f(r["Re"]) for r in s3), max(f(r["Re"]) for r in s3)
titles = [re.findall(r"<text[^>]*>([^<]*)</text>", s)[:1] for s in pb_svgs]
pick = [(0, "Where the lift acts at small angles, against Reynolds number (first view)", f"Aerodynamic centre of the measured plates (from the pitching-moment slope) against Reynolds number, {re_lo:,.0f} to {re_hi:,.0f}. Across all {len(s3)} study rows the centre lies {min(ac):.1f} to {max(ac):.1f}% of chord ahead of the quarter chord."),
        (1, "Where the lift acts at small angles (second view)", "The same plot for the other wings; the legend names each wing by aspect ratio and taper."),
        (12, "Centre of pressure against angle, AR 3 taper 1 (raw)", "Measured centre of pressure against angle of attack for the AR 3 rectangular plate (raw data)."),
        (13, "Same plate with the zero-lift moment removed", "The same curve after the plate's zero-lift moment is subtracted."),
        (22, "Centre of pressure at first peak minus AC, against Reynolds number", "Distance between the centre of pressure at the first lift peak and the aerodynamic centre, for each wing, against Reynolds number."),
        (26, "Glide ratio at 90% of first-peak lift, against Reynolds number", "Lift-to-drag ratio of the measured plates at 90% of first-peak lift, against Reynolds number."),
        (28, "Aerodynamic centre and CP against aspect ratio, taper 0.5", "Aerodynamic centre and centre of pressure against aspect ratio at the lowest measured Reynolds number, taper 0.5.")]
for i, ttl, cap in pick:
    if i < len(pb_svgs):
        add("s1", ttl, "MEASURED", "svg", pb_svgs[i].replace("<svg ", "<svg style='max-width:100%;height:auto' ", 1), cap, "results/runs/rigid_glider/plate_balance/index.html (inline plot no. %d)" % (i + 1), "reused")
# new: first-peak angle vs AR
seen = {}
for r in s3:
    if r["sweep"] == "increasing" and r["version"] == "raw" and r.get("alpha_first_peak_deg", "").strip():
        seen[(r["wing"], r["Re"])] = (f(r["AR"]), f(r["alpha_first_peak_deg"]), r["first_peak_kind"])
if seen:
    kinds = sorted({v[2] for v in seen.values()})
    ser = [dict(name=f"first {k}", pts=[(v[0], v[1]) for v in seen.values() if v[2] == k], color=COLS[i], line=False, op=0.8) for i, k in enumerate(kinds)]
    byar = {}
    for v in seen.values():
        byar.setdefault(v[0], []).append(v[1])
    cap = "Angle of the first lift peak of each measured plate against aspect ratio. Each dot is one Reynolds number; colour marks a true peak or a flattening of the curve. " + "; ".join(f"AR {a:g}: {min(v):.0f} to {max(v):.0f} deg" for a, v in sorted(byar.items())) + "."
    add("s1", "First lift peak by aspect ratio", "MEASURED", "svg", chart("First lift peak against aspect ratio", "aspect ratio", "angle of first peak, deg", ser, xlim=(1.5, 5.5), ylim=(8, 17), xticks=[2, 3, 4, 5]),
        cap, "results/runs/rigid_glider/plate_balance/step3_summary.csv (alpha_first_peak_deg, first_peak_kind)", "new")
else:
    NOTES.append("GAP: step3_summary.csv has no alpha_first_peak_deg column; first-peak plot not drawn")

# ====================================================================================================== 2. pull-out
section("s2", "2. Pull-out study", "Simulated nose-down drops of the flat-plate wings at different centre-of-gravity positions.")
cases = rd("results/runs/rigid_glider/pullout/cases.csv")
specs = sorted({r["spec"] for r in cases})
colr = {s: COLS[i % len(COLS)] for i, s in enumerate(specs)}


def depth_rows():
    out = []
    for r in cases:
        try:
            d, cl, cg = f(r["swoop_depth_m"]), f(r["cl_final"]), f(r["cg_mm"])
        except ValueError:
            continue
        if 0.01 < cl < 2:
            out.append((r["spec"], d, cl, cg))
    return out


dr_ = depth_rows()
ser = [dict(name=s, pts=[(c, d) for sp, d, c, g in dr_ if sp == s], color=colr[s], line=False, op=0.55, r=2.6) for s in specs]
ser = [s for s in ser if s["pts"]]
alld = [d for _, d, _, _ in dr_]
add("s2", "Swoop depth against the lift coefficient it settles at", "MODEL", "svg", chart("Pull-out depth vs final CL", "CL the glide settles at", "swoop depth, m", ser, ylim=(0, 250)),
    f"Swoop depth (m) against the CL at which the glide settles. Each dot is one simulated drop ({len(dr_)} of {len(cases)} cases have a depth); colour is the specimen. Depths run {min(alld):.0f} to {max(alld):.0f} m; the axis stops at 250 m and {sum(1 for d in alld if d > 250)} deeper cases are off the plot.",
    "results/runs/rigid_glider/pullout/cases.csv (swoop_depth_m, cl_final)", "new")
ser = [dict(name=s, pts=[(g, d) for sp, d, c, g in dr_ if sp == s], color=colr[s], line=False, op=0.55, r=2.6) for s in specs]
ser = [s for s in ser if s["pts"]]
add("s2", "Swoop depth against CG position", "MODEL", "svg", chart("Pull-out depth vs CG", "CG, mm aft of the root leading edge", "swoop depth, m", ser, ylim=(0, 250)),
    f"Swoop depth (m) against CG position (mm aft of the root leading edge) for the same {len(dr_)} cases; colour is the specimen. The axis stops at 250 m.",
    "results/runs/rigid_glider/pullout/cases.csv (swoop_depth_m, cg_mm)", "new")
verd = sorted({r["verdict"] for r in cases})
short = lambda v: v.split("(")[0].strip().title()
stacks = [(short(v), [sum(1 for r in cases if r["spec"] == s and r["verdict"] == v) for s in specs]) for v in verd]
tot = {v: sum(1 for r in cases if r["verdict"] == v) for v in verd}
add("s2", "How the simulated cases split across outcomes", "MODEL", "svg", bars("Outcome of each simulated case, by specimen", specs, stacks, "number of cases", w=760),
    "Stacked bars: number of simulated cases per specimen, split by verdict. Totals by verdict: " + "; ".join(f"{short(v)} {tot[v]}" for v in verd) + f" ({len(cases)} cases).",
    "results/runs/rigid_glider/pullout/cases.csv (verdict)", "new", wide=True)
cs = [(s, sum(1 for r in cases if r["spec"] == s and r["cannot_say"] == "True"), sum(1 for r in cases if r["spec"] == s and r["cannot_say"] != "True")) for s in specs]
why = {}
for r in cases:
    if r["cannot_say"] == "True":
        for part in r["cannot_say_why"].split(";"):
            why[part.strip()] = why.get(part.strip(), 0) + 1
ncs = sum(c[1] for c in cs)
add("s2", "Cases flagged cannot_say, by specimen", "MODEL", "svg", bars("Cases the model can and cannot call", specs, [("cannot say", [c[1] for c in cs]), ("can say", [c[2] for c in cs])], "number of cases", w=760, colors=[BAD, GRN]),
    f"Number of cases per specimen with and without the cannot_say flag; {ncs} of {len(cases)} cases carry it. Recorded reasons (a case can have two): " + "; ".join(f"{k} {v}" for k, v in sorted(why.items(), key=lambda kv: -kv[1])[:5]) + ".",
    "results/runs/rigid_glider/pullout/cases.csv (cannot_say, cannot_say_why)", "new", wide=True)
kb = rd("results/runs/rigid_glider/pullout/kit_band.csv")
c_lo = [k for k in kb[0] if "min %" in k][0]
c_hi = [k for k in kb[0] if "max %" in k][0]
rowsk = [(f"{r['specimen']} / {r['variant']}", f(r[c_lo]), f(r[c_hi])) for r in kb if r[c_lo].strip()]
nobands = [f"{r['specimen']} / {r['variant']}" for r in kb if not r[c_lo].strip()]
add("s2", "Band of CG that settles into a glide, kit specimens", "MODEL", "svg", hranges("Settling CG band: offset aft of the neutral point, % of MAC", rowsk, "offset aft of AVL neutral point, % MAC"),
    f"For each kit specimen and plate-offset variant, the range of CG positions (% of MAC aft of the AVL neutral point) that settled after both disturbed starts; bars span {min(r[1] for r in rowsk):.1f} to {max(r[2] for r in rowsk):.1f}%. No bar for {len(nobands)} rows ({', '.join(nobands)}).",
    "results/runs/rigid_glider/pullout/kit_band.csv", "new", wide=True)
for sp in ("F45_span250", "F60_span250", "B45_span400", "BS_printed_250"):
    p = RG / f"pullout/trajectories_{sp}.svg"
    n_sp = sum(1 for r in cases if r["spec"] == sp)
    reuse_img("s2", f"Simulated flight paths, {sp}", "MODEL", p, f"Simulated flight paths of specimen {sp} ({n_sp} cases), one per CG position, each released at the origin.")

# ====================================================================================================== 3. bird outlines
section("s3", "3. Bird outlines: the swift", "Designed swift-like wing outlines and the measured chord of real swift wings (Lentink and de Kat 2014).")
reuse_img("s3", "Swift v4 outlines A, B and C side by side", "MODEL", RG / "bird_outline_v4_swift/outlines.svg", "Top view of the three v4 swift outlines: A follows the measured bird, B is a hybrid, C is the extended-wing variant.", wide=True)
reuse_img("s3", "Chord along the span: bird against the designed outlines", "MEASURED", RG / "bird_outline_v4_swift/chord_overlay.svg", "Measured swift chord distribution overlaid on the designed outlines.", wide=True)
cc = rd("results/runs/rigid_glider/bird_outline_v4_swift/chord_comparison.csv")
md = (ROOT / "research/birds/planform_and_gliding.md").read_text(encoding="utf-8")
tbl = re.findall(r"^\| (0\.\d+)[^|]*\| ([\d.]+) \| ([\d.]+) \|$", md[md.index("### 1.2 Chord along the span"):], flags=re.M)
ckeys = list(cc[0].keys())
xk = ckeys[0]
bad_md = []
for pos, meas, ell in tbl:
    for r in cc:
        if abs(f(r[xk]) - f(pos)) < 1e-9:
            if abs(f(r[ckeys[1]]) - f(meas)) > 0.011 or abs(f(r[ckeys[2]]) - f(ell)) > 0.011:
                bad_md.append(pos)
if bad_md:
    NOTES.append(f"DISAGREEMENT: research note chord table and chord_comparison.csv differ at positions {bad_md}")
ser = [dict(name="swift, fully extended (note)", pts=[(f(p), f(m)) for p, m, e in tbl], color=GRN, width=2.6),
       dict(name="exact ellipse (note)", pts=[(f(p), f(e)) for p, m, e in tbl], color=MUT, dash="5 4", marker=False)]
for i, k in enumerate(ckeys[4:]):
    ser.append(dict(name=k, pts=[(f(r[xk]), f(r[k])) for r in cc if r[k].strip()], color=COLS[(i + 1) % len(COLS)], marker=False))
add("s3", "Bird chord data against the designed outlines", "MEASURED", "svg", chart("Chord / mean chord along the wing", "position along the wing (0 body edge, 1 tip)", "chord / mean chord", ser, w=760, legend_w=210),
    f"Chord divided by mean chord along the wing. Green: measured swift wing ({len(tbl)} digitised points); dashed: fitted ellipse; other lines: the designed outlines. At 40% of the span the measured value is {next(f(m) for p, m, e in tbl if p == '0.40')}. At 20% of the span the v2 outline gives {f(next(r[ckeys[4]] for r in cc if abs(f(r[xk]) - 0.2) < 1e-9)):.2f} and the bird {f(next(r[ckeys[1]] for r in cc if abs(f(r[xk]) - 0.2) < 1e-9)):.2f}.",
    "research/birds/planform_and_gliding.md (table 1.2) and results/runs/rigid_glider/bird_outline_v4_swift/chord_comparison.csv", "new", wide=True)
for nme, d in (("A", "A_printed_span250"), ("B", "B_printed_span250")):
    reuse_img("s3", f"Swift v4 {nme}: build drawing (250 mm printed)", "MODEL", RG / "bird_outline_v4_swift" / d / "drawing.svg", f"Dimensioned drawing of outline {nme} at 250 mm span, with tail, bend and ballast layout.")
vr = rd("results/runs/rigid_glider/bird_outline_v4_swift/diagnosis/variants.csv")
np_k = [k for k in vr[0] if k.startswith("NP %")][0]
bd_k = [k for k in vr[0] if "bend for SM 10" in k][0]
add("s3", "Neutral point of the v2 outline and its variants", "MODEL", "svg",
    bars("Neutral point by outline variant", [r["variant"] for r in vr], [("NP, % MAC", [f(r[np_k]) for r in vr])], "neutral point, % of MAC", w=760, colors=[LNK], legend_w=120),
    "AVL neutral point (% MAC) of the v2 final outline and of variants with one feature switched off at a time: " + "; ".join(f"{r['variant']} {f(r[np_k]):.1f}%" for r in vr) + f". The tail bend for a 10% static margin ranges {min(f(r[bd_k]) for r in vr):.1f} to {max(f(r[bd_k]) for r in vr):.1f} degrees across the variants.",
    "results/runs/rigid_glider/bird_outline_v4_swift/diagnosis/variants.csv", "new", wide=True)
pc = rd("results/runs/rigid_glider/bird_outline_v2/panelling_check.csv")
groups = sorted({(r["family"], r["outline"]) for r in pc})
ser = [dict(name=f"{fam} {o}", pts=[(f(r["spanwise"]), f(r["NP_pct_MAC"])) for r in pc if r["family"] == fam and r["outline"] == o], color=COLS[i % len(COLS)]) for i, (fam, o) in enumerate(groups)]
swing = {f"{fam} {o}": max(f(r["NP_pct_MAC"]) for r in pc if (r["family"], r["outline"]) == (fam, o)) - min(f(r["NP_pct_MAC"]) for r in pc if (r["family"], r["outline"]) == (fam, o)) for fam, o in groups}
add("s3", "Neutral point against panel count, v2 outlines", "MODEL", "svg", chart("Neutral point against panel count, v2 outlines", "spanwise panels per half wing", "neutral point, % MAC", ser, w=700),
    "AVL neutral point of each v2 outline at three spanwise panel counts. Spread between the finest and coarsest panelling (points of MAC): " + "; ".join(f"{k} {v:.1f}" for k, v in swing.items()) + ".",
    "results/runs/rigid_glider/bird_outline_v2/panelling_check.csv", "new")

# ====================================================================================================== 5. CFD
section("s5", "4. CFD of the Ananda AR 3 plate (Re 60,000)", "OpenFOAM results for the Ananda AR 3 rectangular plate, compared with the wind-tunnel data and with AVL.")
sw = rd("cfd/validation_sweep/sweep_results.csv")
av = [r for r in rd("cfd/validation_sweep/setup_checks/avl_results.csv") if r["paneling"] == "coarse" and r["alpha_deg_or_slope"] != "slope"]
tr = [r for r in rd("cfd/validation_sweep/transition_A/transition_results.csv") if r["window"] in ("8-20", "-")]
ser_cl = [dict(name="tunnel (measured)", pts=[(f(r["alpha_deg"]), f(r["meas_CL_mean"])) for r in sw], color=GRN, width=2.6),
          dict(name="laminar CFD, coarse mesh", pts=[(f(r["alpha_deg"]), f(r["CL_mean"])) for r in sw], color=LNK),
          dict(name="AVL linear theory", pts=[(f(r["alpha_deg_or_slope"]), f(r["CL_or_CLa_per_rad"])) for r in av], color=ACC, dash="5 4"),
          dict(name="transition-model CFD", pts=[(f(r["alpha_deg"]), f(r["CL"])) for r in tr if r["source"].startswith("kOmega")], color=PUR, line=False, r=5)]
m_av = {f(r["alpha_deg"]): f(r["meas_CL_mean"]) / next(f(a["CL_or_CLa_per_rad"]) for a in av if f(a["alpha_deg_or_slope"]) == f(r["alpha_deg"])) - 1 for r in sw if any(f(a["alpha_deg_or_slope"]) == f(r["alpha_deg"]) for a in av)}
add("s5", "Lift against angle: tunnel, CFD, AVL", "MODEL", "svg", chart("Lift coefficient vs angle", "angle of attack, deg", "CL", ser_cl, w=700),
    f"CL against angle of attack. Green: measured; blue: coarse-mesh laminar CFD; orange dashed: AVL; purple dots: the two transition-model runs. At 4 deg CFD gives {f(next(r['CL_mean'] for r in sw if f(r['alpha_deg']) == 4)):.4f} and AVL {f(next(r['CL_or_CLa_per_rad'] for r in av if f(r['alpha_deg_or_slope']) == 4)):.4f}; the measured value is {100 * min(m_av.values()):.0f} to {100 * max(m_av.values()):.0f}% above AVL across the angles compared.",
    "cfd/validation_sweep/sweep_results.csv; setup_checks/avl_results.csv; transition_A/transition_results.csv", "new")
ser_cm = [dict(name="tunnel (measured)", pts=[(f(r["alpha_deg"]), f(r["meas_CM_mean"])) for r in sw], color=GRN, width=2.6),
          dict(name="laminar CFD, coarse mesh", pts=[(f(r["alpha_deg"]), f(r["CM_mean"])) for r in sw], color=LNK),
          dict(name="AVL linear theory", pts=[(f(r["alpha_deg_or_slope"]), f(r["Cm_c4"])) for r in av], color=ACC, dash="5 4"),
          dict(name="transition-model CFD", pts=[(f(r["alpha_deg"]), f(r["CM"])) for r in tr if r["source"].startswith("kOmega")], color=PUR, line=False, r=5)]
signs = [f(r["alpha_deg"]) for r in sw if f(r["alpha_deg"]) >= 2 and f(r["CM_mean"]) * f(r["meas_CM_mean"]) < 0]
add("s5", "Pitching moment against angle: tunnel, CFD, AVL", "MODEL", "svg", chart("Pitching moment about the quarter chord (nose-up +)", "angle of attack, deg", "CM", ser_cm, w=700),
    f"CM about the quarter chord (nose-up positive) against angle of attack, same four sources. At 4 deg the CFD value is {100 * f(next(r['CM_mean'] for r in sw if f(r['alpha_deg']) == 4)) / f(next(r['meas_CM_mean'] for r in sw if f(r['alpha_deg']) == 4)):.0f}% of the measured value. CFD and measured CM have opposite signs at {', '.join(f'{a:g}' for a in signs)} deg.",
    "cfd/validation_sweep/sweep_results.csv; setup_checks/avl_results.csv; transition_A/transition_results.csv", "new")
ser_cd = [dict(name="tunnel (measured)", pts=[(f(r["alpha_deg"]), f(r["meas_CD_mean"])) for r in sw], color=GRN, width=2.6),
          dict(name="laminar CFD*", pts=[(f(r["alpha_deg"]), f(r["CD_mean"])) for r in sw], color=LNK),
          dict(name="AVL induced drag only", pts=[(f(r["alpha_deg_or_slope"]), f(r["CDind"])) for r in av], color=ACC, dash="5 4")]
add("s5", "Drag against angle", "MODEL", "svg", chart("Drag coefficient vs angle", "angle of attack, deg", "CD", ser_cd, w=700, note="* CFD drag is not mesh-converged"),
    f"CD against angle of attack: measured, laminar CFD (not mesh-converged) and AVL induced drag. At 4 deg AVL gives {f(next(r['CDind'] for r in av if f(r['alpha_deg_or_slope']) == 4)):.4f} and the measured value is {f(next(r['meas_CD_mean'] for r in sw if f(r['alpha_deg']) == 4)):.4f}.",
    "cfd/validation_sweep/sweep_results.csv; setup_checks/avl_results.csv", "new")
bu = [r for r in rd("cfd/validation_sweep/bubble_extent.csv") if r["basis"] == "mean2"]
spans = sorted({r["span_frac"] for r in bu})
ser = [dict(name=f"{f(s) * 100:.0f}% of the half span", pts=[(f(r["alpha_deg"]), f(r["reversed_fraction_upper"])) for r in bu if r["span_frac"] == s], color=COLS[i]) for i, s in enumerate(spans)]
add("s5", "Reversed-flow fraction of the upper surface (CFD)", "MODEL", "svg", chart("Reversed-flow fraction of the upper chord (CFD)", "angle of attack, deg", "fraction of chord with reversed flow", ser, ylim=(0, 1), w=660),
    f"Fraction of the upper chord with reversed flow in the CFD, at three span stations, from saved snapshots (two-snapshot averages). Zero at 2 deg; {min(f(r['reversed_fraction_upper']) for r in bu if f(r['alpha_deg']) == 4):.2f} to {max(f(r['reversed_fraction_upper']) for r in bu if f(r['alpha_deg']) == 4):.2f} at 4 deg; up to {max(f(r['reversed_fraction_upper']) for r in bu if f(r['alpha_deg']) == 8):.2f} at 8 deg.",
    "cfd/validation_sweep/bubble_extent.csv", "new")
for fn, ttl, cap in (("Cp_alpha8_lam_vs_LM.svg", "Surface pressure at 8 deg: laminar against transition model", "Mid-span pressure coefficient on the upper and lower surface at 8 deg, laminar and transition-model CFD."),
                     ("shear_alpha8_lam_vs_LM.svg", "Wall shear at 8 deg: laminar against transition model", "Upper-surface wall shear at 8 deg, laminar and transition-model CFD; negative values are reversed flow."),
                     ("CM_history_alpha8.svg", "Pitching-moment history at 8 deg, transition run", "Pitching moment against time in the 8 deg transition-model run."),
                     ("transition_alpha8.svg", "Transition-model quantities along the chord (8 deg)", "Eddy-viscosity ratio and intermittency along the chord at 8 deg.")):
    reuse_img("s5", ttl, "MODEL", ROOT / "cfd/validation_sweep/transition_A/plots" / fn, cap)
for fn, ttl in (("Cp_span50.svg", "Mid-span surface pressure, laminar sweep"), ("shear_upper_span50.svg", "Mid-span upper-surface wall shear, laminar sweep")):
    reuse_img("s5", ttl, "MODEL", ROOT / "cfd/validation_sweep/plots" / fn, "Mid-span laminar CFD results at several angles of attack, from saved snapshots.")
for fn, ttl, cap in (("a08_slice_speed.png", "Flow speed around the plate at 8 deg (CFD snapshot)", "Flow speed on a slice through the plate at 8 deg; single CFD snapshot. Blue is low speed."),
                     ("a08_plate_pressure_top.png", "Pressure on the upper surface at 8 deg (CFD snapshot)", "Pressure on the upper face of the plate at 8 deg; single CFD snapshot.")):
    reuse_img("s5", ttl, "MODEL", ROOT / "cfd/learn/img" / fn, cap)

# ---- more CFD figures: existing files, plus force histories of the finished coarse sweep cases (read-only cat)
CFDT = "MODEL (CFD)"
hist = {}
for a_ in (0, 2, 6, 8, 10, 12):
    try:
        res_ = subprocess.run(["wsl", "-d", "MMAV-CFD", "-u", "cfd", "--", "cat", f"/home/cfd/run/vp_sweep_a{a_:02d}/postProcessing/forceCoeffs1/0/coefficient.dat"], capture_output=True, text=True, timeout=120, env=dict(os.environ, MSYS_NO_PATHCONV="1"))
        pts_ = []
        for ln in res_.stdout.splitlines():
            q = ln.split()
            if len(q) >= 8 and not ln.startswith("#") and f(q[0]) > 1.0:
                pts_.append((f(q[0]), f(q[4]), f(q[7])))
        if pts_:
            st_ = max(1, len(pts_) // 500)
            hist[a_] = pts_[::st_]
    except Exception as ex:                                              # noqa: BLE001
        NOTES.append(f"GAP: no force history for sweep alpha {a_} ({ex})")
if hist:
    tmax_ = {a_: h_[-1][0] for a_, h_ in hist.items()}
    for idx_, nm_, ttl_, lab_ in ((1, "CL", "Lift history of the coarse sweep cases", "CL"), (2, "CM", "Pitching-moment history of the coarse sweep cases", "CM (nose-up +)")):
        sr_ = [dict(name=f"alpha {a_} deg", pts=[(q[0], q[idx_]) for q in h_], color=COLS[i % len(COLS)], marker=False, sort=False, width=1.4) for i, (a_, h_) in enumerate(sorted(hist.items()))]
        add("s5", ttl_, CFDT, "svg", chart(f"{nm_} against time, laminar coarse sweep", "time, chord passages", lab_, sr_, w=700),
            f"{nm_} against time from the force-coefficient files of the finished laminar coarse-mesh cases at alpha {', '.join(str(a_) for a_ in sorted(hist))} deg; the cases end at t = {min(tmax_.values()):.0f} to {max(tmax_.values()):.0f} chord passages. Samples before t = 1 are omitted.",
            "WSL case folders vp_sweep_a00 to vp_sweep_a12, postProcessing/forceCoeffs1 (read-only copy of the force files)", "new")
for fn_, ttl_, cap_ in (("CL_history_alpha4.svg", "Lift history at 4 deg: laminar and transition model", "CL against time at 4 deg, laminar and transition-model runs."),
                        ("CL_history_alpha8.svg", "Lift history at 8 deg: laminar and transition model", "CL against time at 8 deg, laminar and transition-model runs."),
                        ("CM_history_alpha4.svg", "Pitching-moment history at 4 deg: laminar and transition model", "CM about the quarter chord against time at 4 deg, laminar and transition-model runs."),
                        ("Cp_alpha4_lam_vs_LM.svg", "Surface pressure at 4 deg: laminar against transition model", "Mid-span pressure coefficient on the upper and lower surface at 4 deg, laminar and transition-model CFD."),
                        ("shear_alpha4_lam_vs_LM.svg", "Wall shear at 4 deg: laminar against transition model", "Upper-surface wall shear at 4 deg, laminar and transition-model CFD."),
                        ("transition_alpha4.svg", "Transition-model quantities along the chord (4 deg)", "Eddy-viscosity ratio and intermittency along the chord at 4 deg.")):
    reuse_img("s5", ttl_, CFDT, ROOT / "cfd/validation_sweep/transition_A/plots" / fn_, cap_)
for fn_, ttl_ in (("Cp_span25.svg", "Surface pressure at 25% of the half span, laminar sweep"), ("Cp_span75.svg", "Surface pressure at 75% of the half span, laminar sweep"),
                  ("shear_upper_span25.svg", "Upper-surface wall shear at 25% of the half span, laminar sweep"), ("shear_upper_span75.svg", "Upper-surface wall shear at 75% of the half span, laminar sweep")):
    reuse_img("s5", ttl_, CFDT, ROOT / "cfd/validation_sweep/plots" / fn_, "Laminar CFD results at several angles of attack, from saved snapshots.")
reuse_img("s5", "Coarse mesh, 4 deg: force history", CFDT, ROOT / "cfd/validation_scoping/coarse_forces_t8-20.png", "CL, CD and CM about the quarter chord against time for the coarse-mesh 4 deg scoping run; the shaded interval (t = 8 to 20) is the averaging window.", inv=True)
reuse_img("s5", "Medium mesh, 4 deg: force history", CFDT, ROOT / "cfd/validation_scoping/medium_forces_t2-4.png", "CL, CD and CM against time for the medium-mesh 4 deg scoping run, t = 2 to 4 chord passages.", inv=True)
for fn_, ttl_, cap_ in (("plate_pressure.png", "2D teaching plate: pressure", "Pressure divided by density (colour scale -0.4 to 0.4) around a 2D flat plate; teaching case, not the AR 3 plate."),
                        ("plate_speed.png", "2D teaching plate: speed", "Flow speed around the same 2D teaching plate."),
                        ("plate_streamlines.png", "2D teaching plate: streamlines", "Streamlines over speed colouring for the same 2D teaching plate."),
                        ("cavity_speed.png", "Lid-driven cavity: speed", "Speed in the stock lid-driven cavity tutorial (colour scale 1.8e-4 to 0.85 m/s).")):
    reuse_img("s5", ttl_, CFDT, ROOT / "cfd/learn/img" / fn_, cap_)

# ---- refined 6 degree run, finished: read-only cat through WSL
ref_readme = (ROOT / "cfd/validation_sweep/refined_6deg/README.md").read_text(encoding="utf-8")
rc = re.search(r"CM between \+([\d.]+) and \+([\d.]+); x_cp between ([\d.]+) c and ([\d.]+) c; CL at or above ([\d.]+)", ref_readme)
meas6 = next(r for r in sw if f(r["alpha_deg"]) == 6.0)
av6 = next(f(r["CL_or_CLa_per_rad"]) for r in av if f(r["alpha_deg_or_slope"]) == 6)
rows_ref = []
try:
    cmd = ["wsl", "-d", "MMAV-CFD", "-u", "cfd", "--", "cat", "/home/cfd/run/vp_ref_a06/postProcessing/forceCoeffs1/0/coefficient.dat"]
    env = dict(os.environ, MSYS_NO_PATHCONV="1")
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=120, env=env)
    for ln in res.stdout.splitlines():
        p = ln.split()
        if len(p) >= 8 and not ln.startswith("#") and f(p[0]) > 1.0:
            rows_ref.append((f(p[0]), f(p[1]), f(p[4]), f(p[7])))
except Exception as ex:                                              # noqa: BLE001
    NOTES.append(f"GAP: could not read the refined-run force history ({ex})")
if len(rows_ref) > 50:
    t_end = rows_ref[-1][0]
    step = max(1, len(rows_ref) // 700)
    thin = rows_ref[::step]
    tcut = t_end - 0.0
    sel_ = [r for r in rows_ref if 8.0 <= r[0] <= 20.0]
    wts_ = [(sel_[min(i + 1, len(sel_) - 1)][0] - sel_[max(i - 1, 0)][0]) for i in range(len(sel_))]
    mean_cl = sum(w_ * r[2] for w_, r in zip(wts_, sel_)) / sum(wts_)
    mean_cm = sum(w_ * r[3] for w_, r in zip(wts_, sel_)) / sum(wts_)
    cl_ser = [dict(name="refined run, CL(t)", pts=[(r[0], r[2]) for r in thin], color=LNK, marker=False, sort=False)]
    cm_ser = [dict(name="refined run, CM(t)", pts=[(r[0], r[3]) for r in thin], color=LNK, marker=False, sort=False)]
    ref_cap = (f"CL against time for the refined laminar 6 deg run (4,421,076 cells), finished at t = {t_end:.0f} chord passages ({len(rows_ref)} samples, thinned for plotting). Mean over t = 8 to 20: CL {mean_cl:.4f}. "
               f"Horizontal lines: tunnel, coarse CFD and AVL at 6 deg. Outcome recorded in the run README (cfd/validation_sweep/refined_6deg): PARTIAL; lift below its limit of {rc.group(5).rstrip('.')}.")
    add("s5", "Refined 6 deg run: lift history", "MODEL (CFD)", "svg",
        chart("Refined laminar 6 deg, CL history", "time, chord passages", "CL", cl_ser, hlines=[(f(meas6["meas_CL_mean"]), "tunnel", GRN), (f(meas6["CL_mean"]), "coarse CFD", MUT), (av6, "AVL", ACC)], w=700),
        ref_cap, "cfd/validation_sweep/refined_6deg (force-coefficient file); limits from README.md", "new")
    add("s5", "Refined 6 deg run: pitching-moment history", "MODEL (CFD)", "svg",
        chart("Refined laminar 6 deg, CM history", "time, chord passages", "CM (nose-up +)", cm_ser, hlines=[(f(meas6["meas_CM_mean"]), "tunnel", GRN), (f(meas6["CM_mean"]), "coarse CFD", MUT)], bands=[(f(rc.group(1)), f(rc.group(2)), "limit band", GRN)], w=700),
        f"CM about the quarter chord against time for the same run; mean over t = 8 to 20: CM {mean_cm:+.4f}. Horizontal lines: tunnel and coarse CFD at 6 deg; shaded band: CM +{rc.group(1)} to +{rc.group(2)}, the limit written in the run README before the run. Outcome recorded there: PARTIAL.", "same file as the lift history", "new")
else:
    NOTES.append("GAP: refined 6 deg force history not readable; figure omitted")

# ---- refined 8 degree run (stopped early by decision): read-only cat through WSL
r8_readme = (ROOT / "cfd/validation_sweep/refined_8deg/README.md").read_text(encoding="utf-8")
bx8 = re.search(r"x_cp between ([\d.]+) c and ([\d.]+) c", r8_readme)
cmb8 = re.search(r"CM between \+([\d.]+) and \+([\d.]+)", r8_readme)
grade8 = re.search(r"\*\*Grade: (PASS|PARTIAL|FAIL)\*\*", r8_readme)
meas8 = next(r for r in sw if f(r["alpha_deg"]) == 8.0)
av8 = next(f(r["CL_or_CLa_per_rad"]) for r in av if f(r["alpha_deg_or_slope"]) == 8)
rows8 = []
try:
    cmd = ["wsl", "-d", "MMAV-CFD", "-u", "cfd", "--", "cat", "/home/cfd/run/vp_ref_a08/postProcessing/forceCoeffs1/0/coefficient.dat"]
    res8 = subprocess.run(cmd, capture_output=True, text=True, timeout=120, env=dict(os.environ, MSYS_NO_PATHCONV="1"))
    for ln in res8.stdout.splitlines():
        p_ = ln.split()
        if len(p_) >= 8 and not ln.startswith("#") and f(p_[0]) > 1.0:
            rows8.append((f(p_[0]), f(p_[1]), f(p_[4]), f(p_[7])))
except Exception as ex:                                              # noqa: BLE001
    NOTES.append(f"GAP: could not read the refined 8 deg force history ({ex})")
if len(rows8) > 50 and bx8 and cmb8 and grade8:
    t8 = rows8[-1][0]
    thin8 = rows8[::max(1, len(rows8) // 700)]
    sel8 = [r for r in rows8 if 4.0 <= r[0] <= t8]
    w8 = [(sel8[min(i + 1, len(sel8) - 1)][0] - sel8[max(i - 1, 0)][0]) for i in range(len(sel8))]
    m_cl8 = sum(w_ * r[2] for w_, r in zip(w8, sel8)) / sum(w8)
    m_cd8 = sum(w_ * r[1] for w_, r in zip(w8, sel8)) / sum(w8)
    m_cm8 = sum(w_ * r[3] for w_, r in zip(w8, sel8)) / sum(w8)
    cn8 = m_cl8 * math.cos(math.radians(8)) + m_cd8 * math.sin(math.radians(8))
    xcp8 = 0.25 - m_cm8 / cn8
    cl8_ser = [dict(name="refined run, CL(t)", pts=[(r[0], r[2]) for r in thin8], color=LNK, marker=False, sort=False)]
    cm8_ser = [dict(name="refined run, CM(t)", pts=[(r[0], r[3]) for r in thin8], color=LNK, marker=False, sort=False)]
    add("s5", "Refined 8 deg run: lift history", "MODEL (CFD)", "svg",
        chart("Refined laminar 8 deg, CL history", "time, chord passages", "CL", cl8_ser, hlines=[(f(meas8["meas_CL_mean"]), "tunnel", GRN), (f(meas8["CL_mean"]), "coarse CFD", MUT), (av8, "AVL", ACC)], w=700),
        f"CL against time for the refined laminar 8 deg run (5,949,059 cells), stopped early by decision at t = {t8:.2f} chord passages ({len(rows8)} samples, thinned for plotting). Mean over t = 4 to {t8:.2f}: CL {m_cl8:.4f}. Horizontal lines: tunnel, coarse CFD and AVL at 8 deg. "
        f"Outcome recorded in the run README (cfd/validation_sweep/refined_8deg): {grade8.group(1)}, graded on the centre of pressure and the sign of the moment; lift is reported, not graded.",
        "cfd/validation_sweep/refined_8deg (force-coefficient file); limits from README.md", "new")
    add("s5", "Refined 8 deg run: pitching-moment history", "MODEL (CFD)", "svg",
        chart("Refined laminar 8 deg, CM history", "time, chord passages", "CM (nose-up +)", cm8_ser, hlines=[(f(meas8["meas_CM_mean"]), "tunnel", GRN), (f(meas8["CM_mean"]), "coarse CFD", MUT)], bands=[(f(cmb8.group(1)), f(cmb8.group(2).rstrip(".")), "secondary band", GRN)], w=700),
        f"CM about the quarter chord against time for the same run; mean over t = 4 to {t8:.2f}: CM {m_cm8:+.4f}, centre of pressure {xcp8:.4f} c (graded band {bx8.group(1)} to {bx8.group(2)} c). Horizontal lines: tunnel and coarse CFD at 8 deg; shaded band: CM +{cmb8.group(1)} to +{cmb8.group(2).rstrip('.')}, the secondary band in the run README (reported, not graded).",
        "same file as the lift history", "new")
else:
    NOTES.append("GAP: refined 8 deg force history or README criteria not readable; figures omitted")

# ====================================================================================================== 6. everything else
section("s6", "5. Other figures", "Architecture comparisons, AVL baseline curves, earlier XFOIL batch outputs and earlier project figures.")
for fn, ttl, cap in (("fig1_uncensored_window_by_architecture.svg", "CG window by architecture (pure wing, lip, tail)", "Width of the CG range with a stable trim, for each architecture and wing loading."),
                     ("fig2_window_centre_extent_P07.svg", "Where each CG window sits on the CG rail", "Position of each architecture's CG range along the CG rail for the AR 4.5 candidate."),
                     ("fig3_mass_ballast_budget.svg", "Mass and ballast budget", "Mass and ballast budget per architecture.")):
    reuse_img("s6", ttl, "MODEL", RG / "arch_compare_v2" / fn, cap)
for fn, ttl, cap in (("fig1_section_cm_vs_cl.svg", "Section pitching moment against lift (XFOIL, plate section)", "XFOIL section pitching moment against lift for the thin sections tried."),
                     ("fig2_cg_window_by_architecture.svg", "CG window by architecture, plate section", "CG range for the plate section, by architecture.")):
    reuse_img("s6", ttl, "MODEL", RG / "plate" / fn, cap)
for fn, ttl in (("baseline_cl.svg", "Baseline wing: lift against angle (AVL)"), ("baseline_cm.svg", "Baseline wing: pitching moment against angle (AVL)")):
    reuse_img("s6", ttl, "MODEL", RG / fn, "AVL curve for the original baseline wing.")
fh = (RG / "figures.html").read_text(encoding="utf-8")
h2 = ""
for m in re.finditer(r"<h2>(.*?)</h2>|<figure><img src=\"([^\"]+)\"[^>]*><figcaption>(.*?)</figcaption></figure>", fh, flags=re.S):
    if m.group(1):
        h2 = html.unescape(re.sub(r"<[^>]+>", "", m.group(1))).strip()
        continue
    fp = (RG / m.group(2)).resolve()
    if fp in EMBEDDED or not fp.exists():
        continue
    cap_ = html.unescape(re.sub(r"<[^>]+>", "", m.group(3))).strip()
    reuse_img("s6", cap_, "MODEL", fp, "Earlier project figure.")
LINKS = [("Plate-balance study", RG / "plate_balance/index.html"), ("Flat-wing test package", RG / "flat_test_v2/index.html"), ("Pull-out study", RG / "pullout/index.html"),
         ("Swift v4 outline study", RG / "bird_outline_v4_swift/index.html"), ("Swift v4 diagnosis of v2", RG / "bird_outline_v4_swift/diagnosis/index.html"), ("Bird outline v2 study", RG / "bird_outline_v2/index.html"),
         ("Findings", RG / "findings.html"), ("Process", RG / "process.html"),
         ("Modelling walkthrough (teaching page)", ROOT / "docs/modeling_walkthrough/index.html"), ("CFD lesson", ROOT / "cfd/learn/index.html")]

# ====================================================================================================== page
CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37;--bad:#ff8a80;--good:#6cc08b}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 "Segoe UI",system-ui,sans-serif}
main{max-width:1180px;margin:0 auto;padding:28px 20px 64px;display:flex;flex-direction:column;gap:28px}
h1{font-size:28px;margin:0}h2{font-size:22px;margin:0;border-bottom:1px solid var(--line);padding-bottom:6px}p{margin:0;max-width:80ch}a{color:var(--link)}code{font-size:13.5px}
nav{display:flex;flex-wrap:wrap;gap:8px 18px;background:var(--panel);border:1px solid var(--line);padding:10px 14px;position:sticky;top:0;z-index:5}
section{display:flex;flex-direction:column;gap:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(520px,1fr));gap:18px;align-items:start}
figure{margin:0;background:var(--panel);border:1px solid var(--line);padding:10px;display:flex;flex-direction:column;gap:8px}
figure.wide{grid-column:1/-1}figure.small{max-width:300px}
figure img,figure svg{display:block;max-width:100%;height:auto;margin:0 auto}
.fit{max-height:620px;width:auto}
figure .inv{background:#fff;filter:invert(1) hue-rotate(180deg)}
figure .plain{background:#fff}
figcaption{font-size:14px;color:var(--muted)}figcaption b.t{color:var(--ink);font-size:15px}
.pill{display:inline-block;font-size:11px;font-weight:700;border:1px solid;border-radius:10px;padding:0 8px;margin-right:6px;letter-spacing:.03em}
.src{font-size:12px;color:var(--muted);word-break:break-all}.box{background:var(--panel);border:2px solid var(--accent);padding:10px 16px}
ul{margin:0;padding-left:22px}
h3.grp{grid-column:1/-1;font-size:17px;margin:14px 0 0;font-weight:600;color:var(--muted);border-bottom:1px solid var(--line)}
"""


def pill(tag):
    c = TAGS[tag]
    return f"<span class='pill' style='color:{c};border-color:{c}'>{tag}</span>"


def embed(path, kind, small, alt):
    """Return an <img> data URI (PNG) or inline <svg> (ids prefixed so several SVGs cannot clash)."""
    p = Path(path)
    if p.suffix.lower() == ".png":
        b64 = base64.b64encode(p.read_bytes()).decode("ascii")
        return f"<img class='{'fit inv' if kind == 'imginv' else 'fit'}' alt='{E(alt)}' src='data:image/png;base64,{b64}'{' width=280' if small else ''}>"
    t = p.read_text(encoding="utf-8")
    t = re.sub(r"<\?xml.*?\?>", "", t, flags=re.S)
    t = re.sub(r"<!DOCTYPE.*?>", "", t, flags=re.S)
    SVG_N[0] += 1
    pre = f"g{SVG_N[0]}-"
    for i in set(re.findall(r"\sid=[\"']([^\"']+)[\"']", t)):
        t = re.sub(r"(\sid=[\"'])" + re.escape(i) + r"([\"'])", lambda mm: mm.group(1) + pre + i + mm.group(2), t)
        t = t.replace(f"url(#{i})", f"url(#{pre}{i})")
        t = re.sub(r"(href=[\"'])#" + re.escape(i) + r"([\"'])", lambda mm: mm.group(1) + "#" + pre + i + mm.group(2), t)
    m = re.search(r"<svg[\s>][^>]*>", t)
    tag = m.group(0)
    cls = "fit inv" if kind == "imginv" else ("fit plain" if small and re.search(r"template|field_pack", str(p)) else "fit")
    new_tag = re.sub(r"\s(class|width|height)=[\"'][^\"']*[\"']", "", tag) if small else re.sub(r"\s(class)=[\"'][^\"']*[\"']", "", tag)
    extra = f" class='{cls}' role='img' aria-label='{E(alt)}'" + (" style='width:280px;height:auto'" if small else "")
    if "style=" in new_tag:
        new_tag = new_tag.replace("style=\"", "style=\"max-width:100%;", 1).replace("style='", "style='max-width:100%;", 1)
        new_tag = new_tag.replace("<svg", "<svg" + extra.replace(" style='width:280px;height:auto'", ""), 1)
    else:
        new_tag = new_tag.replace("<svg", "<svg" + extra + ("" if small else " style='max-width:100%'"), 1)
    return t[:m.start()] + new_tag + t[m.end():]


def grid_html(sid):
    figs = [r for s_, r in FIGS if s_ == sid]
    out, last = [], ""
    for r in sorted(figs, key=lambda r: r["group"] != ""):
        if r["group"] != last:
            out.append(f"<h3 class='grp'>{E(r['group'])}</h3>")
            last = r["group"]
        out.append(fig_html(r))
    return "".join(out)


def fig_html(r):
    k = r["kind"]
    if k == "svg":
        body = r["body"]
    else:
        body = embed(r["body"], k, r["small"], r["title"])
    cap = r["caption"] if r["caption"].startswith("<") else E(r["caption"]).replace("&#x27;", "'")
    return (f"<figure class='{'wide' if r['wide'] else ''}{' small' if r['small'] else ''}'>{body}<figcaption><b class='t'>{pill(r['tag'])}{E(r['title'])}</b><br>{cap}"
            f"<div class='src'>{E(r['source'])}</div></figcaption></figure>")


SCRUB = [
    (r"Kai(?:&#x27;|\u2019|')s wings", "the fabricated wings"),
    (r"Kai(?:&#x27;|\u2019|')s decisions", "design decisions"),
    (r"is Kai(?:&#x27;|\u2019|')s decision", "is a design decision"),
    (r"is Kai(?:&#x27;|\u2019|')s(?=</text>)", "is a"),
    (r"What you see", "What is seen"),
    (r"from our computer model", "from the computer model"),
    (r"not our wings", "not the fabricated wings"),
    (r"on your print", "on the print"),
    (r";\s*see the angle gauge on the template notes page", ""),
]


def scrub(page):
    parts = re.split(r"(data:image/png;base64,[A-Za-z0-9+/=]+)", page)
    for i in range(0, len(parts), 2):
        for a, b in SCRUB:
            parts[i] = re.sub(a, b, parts[i])
    out = "".join(parts)
    rest = re.sub(r"data:image/png;base64,[A-Za-z0-9+/=]+", "", out)
    for pat in (r"Kai", r"\byou\b", r"\byour\b", r"\buser\b", r"reviewer", r"session", r"agent", r"\bour\b", r"\bwe\b", r"\bus\b(?! Letter)", r"let's"):
        for m in re.finditer(pat, rest, flags=re.I):
            NOTES.append(f"PERSONAL-NOTE HIT /{pat}/: ..." + rest[max(0, m.start() - 40):m.end() + 40].replace("\n", " "))
    return out


def build():
    nfig = len(FIGS)
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Model gallery</title><style>{CSS}</style></head><body><main>"]
    P.append("<header><h1>MMAV glider modelling figures</h1><p>Figures from the MMAV glider modelling studies, grouped by study. No flight results yet.</p>"
             f"<p class='legend'>{pill('MODEL')} calculation &nbsp; {pill('MEASURED')} wind-tunnel literature data</p></header>")
    P.append("<nav>" + "".join(f"<a href='#{sid}'>{E(t)}</a>" for sid, t, _ in SECTIONS) + "</nav>")
    for sid, title, intro in SECTIONS:
        P.append(f"<section id='{sid}'><h2>{E(title)}</h2><p>{E(intro)}</p><div class='grid'>" + grid_html(sid) + "</div></section>")
    P.append("<section id='links'><h2>Related pages</h2><p>Project folder paths.</p><ul>" + "".join(f"<li>{E(t)}: <span class='src'>{E(str(p.relative_to(ROOT)).replace(chr(92), '/'))}</span></li>" for t, p in LINKS if p.exists()) + "</ul></section>")
    P.append("</main></body></html>")
    return "\n".join(P)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    page = scrub(build())
    (OUT / "index.html").write_text(page, encoding="utf-8")
    print(f"wrote {OUT / 'index.html'}  {len(page) // 1024} KiB, {len(FIGS)} figures")
    for sid, t, _ in SECTIONS:
        figs = [r for s, r in FIGS if s == sid]
        print(f"  {t}: {len(figs)} figures ({sum(1 for r in figs if r['status'] == 'new')} new)")
    for n in NOTES:
        print("NOTE:", n)


if __name__ == "__main__":
    main()
