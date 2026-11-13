#!/usr/bin/env python3
"""Separation / reattachment extent and Cp plots from surface_profiles.csv (stdlib only).
Run: python bubble_analysis.py   (from scripts/ or anywhere)  -> ../bubble_extent.csv, ../plots/*.svg, prints a table.

Data: only TWO saved snapshots per case exist (t = 15 and t = 20 chord passages; no time-averaged fields). The
'mean2' basis averages those two snapshots; 't15' and 't20' use each alone.

Shear sign: OpenFOAM's wallShearStress comes out NEGATIVE in the flow direction on the clearly attached lower
surface (alpha 2-8), i.e. it is the stress on the fluid. So s = -tau_c > 0 means forward (attached) flow along the
surface and s < 0 means reversed flow.
Detection (upper surface only): s is smoothed with a moving average of +-WIN chord; the leading-edge nose region
(xi < XMIN) is ignored; a reversed ZONE is a stretch with smoothed s < 0 at least HOLD chord long. SEPARATION = start of the first zone,
REATTACHMENT = its end; a zone still open at the trailing edge = 'no reattachment before TE'.
Also reported: reversed_fraction = fraction of the upper chord (XMIN..1) where smoothed s < 0.
"""
import csv, os, collections

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "surface_profiles.csv")
OUT = os.path.join(HERE, "..", "bubble_extent.csv")
PLOTS = os.path.join(HERE, "..", "plots")
os.makedirs(PLOTS, exist_ok=True)
WIN, XMIN, HOLD = 0.03, 0.03, 0.05

D = collections.defaultdict(list)
for r in csv.DictReader(open(SRC)):
    D[(int(r["alpha_deg"]), float(r["time"]), float(r["span_frac"]), r["surface"])].append(
        (float(r["xi"]), float(r["Cp"]), -float(r["tau_c"])))
for k in D:
    D[k].sort()


def smooth(xs, ys, w):
    out = []
    for i, x in enumerate(xs):
        v = [ys[j] for j in range(len(xs)) if abs(xs[j] - x) <= w]
        out.append(sum(v) / len(v))
    return out


def profile(alpha, span, surf, basis):
    """returns xi list, Cp list, s list for the chosen basis"""
    if basis == "mean2":
        a = D[(alpha, 15.0, span, surf)]
        b = D[(alpha, 20.0, span, surf)]
        n = min(len(a), len(b))
        return ([a[i][0] for i in range(n)], [(a[i][1] + b[i][1]) / 2 for i in range(n)], [(a[i][2] + b[i][2]) / 2 for i in range(n)])
    a = D[(alpha, float(basis[1:]), span, surf)]
    return [p[0] for p in a], [p[1] for p in a], [p[2] for p in a]


def analyse(xs, s):
    """reversed segments of the smoothed upper-surface shear (s < 0) that are at least HOLD chords long"""
    ss = smooth(xs, s, WIN)
    idx = [i for i, x in enumerate(xs) if x >= XMIN]
    segs, start = [], None
    for i in idx:
        if ss[i] < 0 and start is None:
            start = xs[i]
        if ss[i] >= 0 and start is not None:
            if xs[i] - start >= HOLD:
                segs.append((start, xs[i]))
            start = None
    if start is not None and xs[idx[-1]] - start >= HOLD:
        segs.append((start, None))          # still reversed at the end of the plate
    frac = sum(((e if e is not None else 1.0) - b) for b, e in segs) / (1.0 - XMIN)
    return segs, frac


rows = []
for alpha in (2, 4, 6, 8):
    for span in (0.25, 0.5, 0.75):
        for basis in ("mean2", "t15", "t20"):
            xs, cp, sh = profile(alpha, span, "U", basis)
            segs, frac = analyse(xs, sh)
            if not segs:
                sep = reat = ""; note = "no sustained reversed flow (attached)"
            else:
                sep = round(segs[0][0], 3)
                reat = "" if segs[0][1] is None else round(segs[0][1], 3)
                note = ("first reversed zone: separation -> reattachment" if reat != "" else "separated, no reattachment before TE")
                if len(segs) > 1:
                    note += "; %d reversed zones in all" % len(segs)
            allsegs = ";".join("%.2f-%s" % (b, "TE" if e is None else "%.2f" % e) for b, e in segs)
            rows.append([alpha, span, basis, sep, reat, round(frac, 3), len(segs), allsegs, note])
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["alpha_deg", "span_frac", "basis", "separation_xi", "reattachment_xi", "reversed_fraction_upper", "n_reversed_zones", "all_reversed_zones_xi", "note"])
    w.writerows(rows)
print("wrote", os.path.abspath(OUT))
print("alpha span | mean2: sep reat revfrac nzones zones | t15 zones | t20 zones")
by = collections.defaultdict(dict)
for r in rows:
    by[(r[0], r[1])][r[2]] = r
fmt = lambda v: "-" if v == "" else f"{v:.2f}"
for (a, sp), d in by.items():
    m, t1, t2 = d["mean2"], d["t15"], d["t20"]
    print(f"{a:>3} {sp:.2f} | {fmt(m[3])} {fmt(m[4])} {m[5]:.2f} n={m[6]} [{m[7]}] | [{t1[7]}] | [{t2[7]}]")

# ---------------- dark SVG plots
BG, PANEL, INK, MUT, LINE = "#14181d", "#1c2229", "#dde3ea", "#9aa7b4", "#333c46"
COL = {2: "#7fb4ff", 4: "#6cc08b", 6: "#e89a55", 8: "#ff8a80"}


def svg_plot(fname, title, series, ylab, yinv, ylim, zero=True):
    W, H, L, R, T, B = 760, 440, 62, 20, 40, 48
    px = lambda x: L + x * (W - L - R)
    py = lambda y: (T + (y - ylim[0]) / (ylim[1] - ylim[0]) * (H - T - B)) if yinv else (H - B - (y - ylim[0]) / (ylim[1] - ylim[0]) * (H - T - B))
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="Segoe UI,sans-serif" font-size="12">',
         f'<rect width="{W}" height="{H}" fill="{BG}"/><rect x="{L}" y="{T}" width="{W-L-R}" height="{H-T-B}" fill="{PANEL}" stroke="{LINE}"/>',
         f'<text x="{L}" y="22" fill="{INK}" font-size="14">{title}</text>']
    for k in range(0, 11, 2):
        x = px(k / 10)
        o.append(f'<line x1="{x}" y1="{T}" x2="{x}" y2="{H-B}" stroke="{LINE}" stroke-width="0.5"/><text x="{x}" y="{H-B+16}" fill="{MUT}" text-anchor="middle">{k/10:.1f}</text>')
    nt = 6
    for k in range(nt + 1):
        v = ylim[0] + (ylim[1] - ylim[0]) * k / nt
        y = py(v)
        o.append(f'<line x1="{L}" y1="{y}" x2="{W-R}" y2="{y}" stroke="{LINE}" stroke-width="0.5"/><text x="{L-6}" y="{y+4}" fill="{MUT}" text-anchor="end">{v:.3g}</text>')
    if zero and ylim[0] < 0 < ylim[1]:
        o.append(f'<line x1="{L}" y1="{py(0)}" x2="{W-R}" y2="{py(0)}" stroke="{MUT}" stroke-width="1"/>')
    o.append(f'<text x="{(L+W-R)/2}" y="{H-8}" fill="{INK}" text-anchor="middle">chord fraction x/c (from leading edge)</text>')
    o.append(f'<text x="14" y="{(T+H-B)/2}" fill="{INK}" text-anchor="middle" transform="rotate(-90 14 {(T+H-B)/2})">{ylab}</text>')
    ly = T + 14
    for (lab, xs, ys, col, dash) in series:
        pts = " ".join(f"{px(x):.1f},{py(max(ylim[0], min(ylim[1], y))):.1f}" for x, y in zip(xs, ys))
        o.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="1.6"{" stroke-dasharray=%s" % chr(34)+dash+chr(34) if dash else ""}/>')
        o.append(f'<line x1="{W-R-150}" y1="{ly-4}" x2="{W-R-125}" y2="{ly-4}" stroke="{col}" stroke-width="2"{" stroke-dasharray=%s" % chr(34)+dash+chr(34) if dash else ""}/><text x="{W-R-120}" y="{ly}" fill="{INK}">{lab}</text>')
        ly += 16
    o.append("</svg>")
    open(os.path.join(PLOTS, fname), "w", encoding="utf-8").write("\n".join(o))


for span in (0.25, 0.5, 0.75):
    ser = []
    for a in (2, 4, 6, 8):
        xu, cu, _ = profile(a, span, "U", "mean2")
        xl, cl, _ = profile(a, span, "L", "mean2")
        ser.append((f"alpha {a} upper", xu, cu, COL[a], None))
        ser.append((f"alpha {a} lower", xl, cl, COL[a], "5 3"))
    svg_plot(f"Cp_span{int(span*100)}.svg", f"Surface pressure coefficient Cp, span {int(span*100)} % of semi-span (mean of the t=15 and t=20 snapshots)",
             ser, "Cp  (axis inverted: suction up)", True, (-1.6, 1.0))
    ser = []
    for a in (2, 4, 6, 8):
        xu, _, su = profile(a, span, "U", "mean2")
        ser.append((f"alpha {a}", xu, smooth(xu, su, WIN), COL[a], None))
    svg_plot(f"shear_upper_span{int(span*100)}.svg", f"Upper-surface streamwise wall shear s (positive = forward flow), span {int(span*100)} %, 2-snapshot mean",
             ser, "s = -wallShearStress . e_chord", False, (-0.004, 0.012))
print("plots in", os.path.abspath(PLOTS))
