#!/usr/bin/env python3
"""Analysis of the kOmegaSSTLM transition test (alpha 4 and 8). stdlib only.
Reads: force histories from the distro (UNC path on Windows), ../surface_profiles_lm.csv, ../near_surface_lm.csv,
       ../../surface_profiles.csv (laminar surface data), ../../bubble_extent.csv, ../../sweep_results.csv.
Writes: ../transition_results.csv, ../bubble_extent_lm.csv, ../transition_profile.csv, ../plots/*.svg; prints tables."""
import csv, math, os, sys, collections
HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "..")
SW = os.path.join(HERE, "..", "..")
sys.path.insert(0, SW)
from collect_sweep import read_coeffs, default_runs
RUNS = default_runs()
PLOTS = os.path.join(A, "plots"); os.makedirs(PLOTS, exist_ok=True)


def wmean(rows, t0, t1, k):
    sel = [r for r in rows if t0 <= r[0] <= t1]
    ts = [r[0] for r in sel]
    w = []
    for i, t in enumerate(ts):
        lo = ts[i - 1] if i > 0 else t; hi = ts[i + 1] if i < len(ts) - 1 else t
        w.append((hi - lo) / (2.0 if 0 < i < len(ts) - 1 else 1.0))
    m = sum(x * r[k] for x, r in zip(w, sel)) / sum(w)
    sd = (sum(x * (r[k] - m) ** 2 for x, r in zip(w, sel)) / sum(w)) ** 0.5
    return m, sd


def slope10(rows, t0, t1, k):
    sel = [(r[0], r[k]) for r in rows if t0 <= r[0] <= t1]
    n = len(sel); mt = sum(t for t, _ in sel) / n; my = sum(y for _, y in sel) / n
    return 10 * sum((t - mt) * (y - my) for t, y in sel) / sum((t - mt) ** 2 for t, _ in sel)


# ------------------------------------------------------------ 1. force windows
lam = {4: os.path.join(RUNS, "vp_a4_coarse"), 8: os.path.join(RUNS, "vp_sweep_a08")}
lm = {4: os.path.join(RUNS, "vp_lm_a04"), 8: os.path.join(RUNS, "vp_lm_a08")}
hist = {("lm", a): read_coeffs(p) for a, p in lm.items()}
hist.update({("lam", a): read_coeffs(p) for a, p in lam.items()})
sweep = {float(r["alpha_deg"]): r for r in csv.DictReader(open(os.path.join(SW, "sweep_results.csv")))}
COEF = (("CL", 2), ("CD", 1), ("CM", 3))
out = []
hdr = ["alpha_deg", "source", "window", "CL", "CL_sd", "CD", "CD_sd", "CM", "CM_sd", "CN", "x_cp"]
tab = {}


def addrow(alpha, src, win, vals, sds):
    ar = math.radians(alpha)
    cn = vals[0] * math.cos(ar) + vals[1] * math.sin(ar)
    xcp = 0.25 - vals[2] / cn
    row = [alpha, src, win] + [x for pair in zip(vals, sds) for x in pair] + [cn, xcp]
    out.append([round(v, 5) if isinstance(v, float) else v for v in row])
    tab[(alpha, src, win)] = row


for a in (4, 8):
    s = sweep[float(a)]
    addrow(a, "measured (Ananda, interpolated)", "-", [float(s["meas_CL_mean"]), float(s["meas_CD_mean"]), float(s["meas_CM_mean"])], ["", "", ""])
    for src in ("lam", "lm"):
        h = hist[(src, a)]
        for win, (t0, t1) in (("8-20", (8, 20)), ("10-20", (10, 20)), ("20-30", (20, 30)), ("10-30", (10, 30))):
            if src == "lam" and t1 > 20: continue
            vals, sds = zip(*[wmean(h, t0, t1, k) for _, k in COEF])
            addrow(a, "laminar CFD" if src == "lam" else "kOmegaSSTLM CFD", win, list(vals), list(sds))
with open(os.path.join(A, "transition_results.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(hdr); w.writerows(out)

print("== force coefficients (time-weighted means, sd of fluctuation)")
for r in out:
    print("%d  %-32s %-6s CL %.4f CD %.4f CM %+.4f | xcp %.3f" % (r[0], r[1], r[2], r[3], r[5], r[7], r[10]))
print("== drift (LM): 10-20 vs 20-30 halves and slope per 10 chords over 10-30")
for a in (4, 8):
    h = hist[("lm", a)]
    for nm, k in COEF:
        m1, _ = wmean(h, 10, 20, k); m2, _ = wmean(h, 20, 30, k)
        print("alpha %d %s: half1 %.4f half2 %.4f diff %+.4f (%+.1f%%) slope/10 %.4f" % (a, nm, m1, m2, m2 - m1, 100 * (m2 - m1) / ((m1 + m2) / 2), slope10(h, 10, 30, k)))

# ------------------------------------------------------------ 2. reversed-flow extent (same method as bubble_extent.csv)
WIN, XMIN, HOLD = 0.03, 0.03, 0.05
D = collections.defaultdict(list)
for r in csv.DictReader(open(os.path.join(A, "surface_profiles_lm.csv"))):
    D[(int(r["alpha_deg"]), float(r["time"]), float(r["span_frac"]), r["surface"])].append((float(r["xi"]), float(r["Cp"]), -float(r["tau_c"])))
for k in list(D): D[k].sort()
DL = collections.defaultdict(list)
for r in csv.DictReader(open(os.path.join(SW, "surface_profiles.csv"))):
    DL[(int(r["alpha_deg"]), float(r["time"]), float(r["span_frac"]), r["surface"])].append((float(r["xi"]), float(r["Cp"]), -float(r["tau_c"])))
for k in list(DL): DL[k].sort()


def smooth(xs, ys, w):
    return [sum(ys[j] for j in range(len(xs)) if abs(xs[j] - x) <= w) / sum(1 for j in range(len(xs)) if abs(xs[j] - x) <= w) for x in xs]


def prof(src, alpha, span, surf, basis, times):
    S = D if src == "lm" else DL
    if basis == "mean2":
        a = S[(alpha, times[0], span, surf)]; b = S[(alpha, times[1], span, surf)]
        n = min(len(a), len(b))
        return [a[i][0] for i in range(n)], [(a[i][1] + b[i][1]) / 2 for i in range(n)], [(a[i][2] + b[i][2]) / 2 for i in range(n)]
    a = S[(alpha, float(basis[1:]), span, surf)]
    return [p[0] for p in a], [p[1] for p in a], [p[2] for p in a]


def analyse(xs, s):
    ss = smooth(xs, s, WIN)
    idx = [i for i, x in enumerate(xs) if x >= XMIN]
    segs, start = [], None
    for i in idx:
        if ss[i] < 0 and start is None: start = xs[i]
        if ss[i] >= 0 and start is not None:
            if xs[i] - start >= HOLD: segs.append((start, xs[i]))
            start = None
    if start is not None and xs[idx[-1]] - start >= HOLD: segs.append((start, None))
    frac = sum(((e if e is not None else 1.0) - b) for b, e in segs) / (1.0 - XMIN)
    return segs, frac


brow = []
print("== reversed-flow zones, upper surface (x/c)  [basis mean2 = mean of the two saved snapshots]")
for src, times, bases in (("lm", (25.0, 30.0), ("mean2", "t25", "t30")), ("lam", (15.0, 20.0), ("mean2", "t15", "t20"))):
    for alpha in (4, 8):
        for span in (0.25, 0.5, 0.75):
            for basis in bases:
                xs, cp, sh = prof(src, alpha, span, "U", basis, times)
                segs, frac = analyse(xs, sh)
                z = ";".join("%.2f-%s" % (b, "TE" if e is None else "%.2f" % e) for b, e in segs)
                brow.append([alpha, span, "kOmegaSSTLM" if src == "lm" else "laminar", basis, round(frac, 3), len(segs), z,
                             "no reattachment before TE" if segs and segs[-1][1] is None else ("attached" if not segs else "reattaches (last zone ends before TE)")])
                if basis == "mean2":
                    print("alpha %d span %.2f %-12s frac %.2f zones [%s]" % (alpha, span, "LM" if src == "lm" else "laminar", frac, z))
with open(os.path.join(A, "bubble_extent_lm.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["alpha_deg", "span_frac", "model", "basis", "reversed_fraction_upper", "n_zones", "zones_xi", "note"]); w.writerows(brow)

# ------------------------------------------------------------ 3. transition indicators (mid-span, band above upper surface)
NS = collections.defaultdict(list)
for r in csv.DictReader(open(os.path.join(A, "near_surface_lm.csv"))):
    NS[(int(r["alpha_deg"]), float(r["time"]))].append((float(r["xi"]), float(r["h"]), float(r["gammaInt"]), float(r["nut_over_nu"]), float(r["k"])))
prow = []
print("== near-surface (h < 0.03 c above upper surface) at mid-span, mean of t = 25 and 30: chord bin: max nut/nu, min gammaInt, mean gammaInt")
for a in (4, 8):
    print("alpha", a)
    for b0 in [i * 0.1 for i in range(10)]:
        vals = []
        for t in (25.0, 30.0):
            sel = [p for p in NS[(a, t)] if b0 <= p[0] < b0 + 0.1 and p[1] < 0.03]
            if sel:
                vals.append((max(p[3] for p in sel), min(p[2] for p in sel), sum(p[2] for p in sel) / len(sel), max(p[4] for p in sel)))
        if vals:
            mv = [sum(v[i] for v in vals) / len(vals) for i in range(4)]
            prow.append([a, round(b0, 2), round(mv[0], 3), round(mv[1], 4), round(mv[2], 4), "%.3g" % mv[3]])
            print("  x/c %.1f-%.1f  max nut/nu %7.2f   min gammaInt %.3f   mean gammaInt %.3f   max k %.3g" % (b0, b0 + 0.1, mv[0], mv[1], mv[2], mv[3]))
with open(os.path.join(A, "transition_profile.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["alpha_deg", "x_over_c_bin_start", "max_nut_over_nu", "min_gammaInt", "mean_gammaInt", "max_k"]); w.writerows(prow)

# ------------------------------------------------------------ 4. dark SVG plots
BG, PANEL, INK, MUT, LINE = "#14181d", "#1c2229", "#dde3ea", "#9aa7b4", "#333c46"


def svg_plot(fname, title, series, xlab, ylab, xlim, ylim, yinv=False):
    W, H, L, R, T, B = 760, 440, 62, 20, 40, 48
    px = lambda x: L + (x - xlim[0]) / (xlim[1] - xlim[0]) * (W - L - R)
    py = lambda y: (T + (y - ylim[0]) / (ylim[1] - ylim[0]) * (H - T - B)) if yinv else (H - B - (y - ylim[0]) / (ylim[1] - ylim[0]) * (H - T - B))
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="Segoe UI,sans-serif" font-size="12">',
         f'<rect width="{W}" height="{H}" fill="{BG}"/><rect x="{L}" y="{T}" width="{W-L-R}" height="{H-T-B}" fill="{PANEL}" stroke="{LINE}"/>',
         f'<text x="{L}" y="22" fill="{INK}" font-size="14">{title}</text>']
    for k in range(6):
        v = xlim[0] + (xlim[1] - xlim[0]) * k / 5
        o.append(f'<line x1="{px(v)}" y1="{T}" x2="{px(v)}" y2="{H-B}" stroke="{LINE}" stroke-width="0.5"/><text x="{px(v)}" y="{H-B+16}" fill="{MUT}" text-anchor="middle">{v:.3g}</text>')
    for k in range(7):
        v = ylim[0] + (ylim[1] - ylim[0]) * k / 6
        o.append(f'<line x1="{L}" y1="{py(v)}" x2="{W-R}" y2="{py(v)}" stroke="{LINE}" stroke-width="0.5"/><text x="{L-6}" y="{py(v)+4}" fill="{MUT}" text-anchor="end">{v:.3g}</text>')
    if ylim[0] < 0 < ylim[1]:
        o.append(f'<line x1="{L}" y1="{py(0)}" x2="{W-R}" y2="{py(0)}" stroke="{MUT}"/>')
    o.append(f'<text x="{(L+W-R)/2}" y="{H-8}" fill="{INK}" text-anchor="middle">{xlab}</text>')
    o.append(f'<text x="14" y="{(T+H-B)/2}" fill="{INK}" text-anchor="middle" transform="rotate(-90 14 {(T+H-B)/2})">{ylab}</text>')
    ly = T + 14
    for (lab, xs, ys, col, dash) in series:
        pts = " ".join(f"{px(x):.1f},{py(max(ylim[0], min(ylim[1], y))):.1f}" for x, y in zip(xs, ys))
        da = f' stroke-dasharray="{dash}"' if dash else ""
        o.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="1.5"{da}/>')
        o.append(f'<line x1="{W-R-190}" y1="{ly-4}" x2="{W-R-165}" y2="{ly-4}" stroke="{col}" stroke-width="2"{da}/><text x="{W-R-160}" y="{ly}" fill="{INK}">{lab}</text>')
        ly += 16
    o.append("</svg>")
    open(os.path.join(PLOTS, fname), "w", encoding="utf-8").write("\n".join(o))


CL_, CM_ = "#e89a55", "#7fb4ff"
for a in (4, 8):
    ser = []
    for src, times, col in (("lam", (15.0, 20.0), "#9aa7b4"), ("lm", (25.0, 30.0), "#e89a55")):
        for surf, dash in (("U", None), ("L", "5 3")):
            xs, cp, _ = prof(src, a, 0.5, surf, "mean2", times)
            ser.append(("%s %s" % ("laminar" if src == "lam" else "kOmegaSSTLM", "upper" if surf == "U" else "lower"), xs, cp, col, dash))
    svg_plot(f"Cp_alpha{a}_lam_vs_LM.svg", f"Cp, alpha {a}, mid-span (mean of two snapshots): laminar vs kOmegaSSTLM", ser, "x/c", "Cp (suction up)", (0, 1), (-2.0, 1.0), True)
    ser = []
    for src, times, col in (("lam", (15.0, 20.0), "#9aa7b4"), ("lm", (25.0, 30.0), "#e89a55")):
        xs, _, sh = prof(src, a, 0.5, "U", "mean2", times)
        ser.append(("laminar" if src == "lam" else "kOmegaSSTLM", xs, smooth(xs, sh, WIN), col, None))
    svg_plot(f"shear_alpha{a}_lam_vs_LM.svg", f"Upper-surface streamwise shear s (>0 forward), alpha {a}, mid-span", ser, "x/c", "s = -wallShearStress . e_chord", (0, 1), (-0.004, 0.012))
    for nm, k, yl, lab in (("CL", 2, (0, 0.7), "CL"), ("CM", 3, (-0.07, 0.05), "CM (c/4, nose-up +)")):
        ser = []
        for src, col in (("lam", "#9aa7b4"), ("lm", "#e89a55")):
            h = [r for r in hist[(src, a)] if r[0] >= 1.0]
            step = max(1, len(h) // 700)
            ser.append(("laminar" if src == "lam" else "kOmegaSSTLM", [r[0] for r in h[::step]], [r[k] for r in h[::step]], col, None))
        svg_plot(f"{nm}_history_alpha{a}.svg", f"{lab} history, alpha {a}", ser, "t (chord passages)", lab, (0, 30), yl)
for a in (4, 8):
    ser = []
    for label, idx, col in (("max nut/nu (log10)", 2, "#e89a55"), ("min gammaInt x 3", 3, "#7fb4ff")):
        xs = [r[1] + 0.05 for r in prow if r[0] == a]
        ys = [(math.log10(max(r[2], 1e-3)) if idx == 2 else 3 * r[3]) for r in prow if r[0] == a]
        ser.append((label, xs, ys, col, None))
    svg_plot(f"transition_alpha{a}.svg", f"Near-surface (h below 0.03 c) turbulent viscosity and gammaInt, alpha {a}, mid-span", ser, "x/c (0.1 bins)", "see legend", (0, 1), (-3, 3))
print("plots:", sorted(os.listdir(PLOTS)))
