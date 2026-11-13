#!/usr/bin/env python3
"""Zero-cost lift-gap checks (no CFD): AVL for all ten Ananda flat-plate planforms, tunnel-slope excess table, wall-correction size.
Run:  py -B lift_gap_checks.py        (stdlib only; runs AVL 3.52 serially, about a minute)
Reads : research/plate_data/ananda2012_flatplate_all_wings.csv (not used for slopes, only to confirm the wing list),
        results/runs/rigid_glider/plate_balance/step2_fits.csv  (tunnel low-angle fits, alpha -4..6 deg; read only),
        research/plate_data/ananda2012_flatplate_AR3_taper1.csv (corrected CL and CD of the AR 3 plate for Task C).
Writes: avl_planforms.csv, panelling_check.csv, excess_table.csv, excess_summary.txt, excess_vs_Re.svg, wall_corrections.txt, avl/*  (all here).

GEOMETRY (assumptions stated, thesis PDF pp. 33-35 = printed 33-35 text on PDF p. 50-51, Fig. 3.14 spans):
  * every wing has mean aerodynamic chord 3.5 in (PDF p. 50, "3.5 in was chosen for all wings tested");
  * spans (in), Fig. 3.14 (PDF p. 51): taper 1: AR2 7, AR3 10.5, AR4 14, AR5 17.5; taper 0.75: AR2 6.95, AR3 10.43, AR4 13.91;
    taper 0.5: AR2 6.75, AR3 10.13, AR4 13.5;
  * straight-tapered trapezoids, taper = tip chord / root chord, QUARTER-CHORD LINE STRAIGHT AND UNSWEPT (PDF p. 49: "variations in taper
    ratios ... were done about the quarter chord"), so Xle(y) = 0.25 (c_root - c(y));
  * root chord from MAC = (2/3) c_r (1 + l + l^2)/(1 + l); AR = b^2/S is then checked against the nominal AR (must agree within 1 %);
  * flat, zero thickness, zero camber, no twist (NACA 0000 section), Mach 0, AVL panelling 8 chordwise x 12 spanwise per half wing
    (the 'coarse' density of ../setup_checks, which agreed with the fine one to 0.00002 in CL).
"""
import csv, math, os, re, subprocess, collections, sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODES = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
AVL = os.path.join(CODES, "tools", "avl", "avl352.exe")
OUTAVL = os.path.join(HERE, "avl"); os.makedirs(OUTAVL, exist_ok=True)
MAC = 3.5
SPANS = {(2, 1.0): 7.0, (3, 1.0): 10.5, (4, 1.0): 14.0, (5, 1.0): 17.5, (2, 0.75): 6.95, (3, 0.75): 10.43, (4, 0.75): 13.91,
         (2, 0.5): 6.75, (3, 0.5): 10.13, (4, 0.5): 13.5}
ALPHAS = [0, 2, 4, 6]            # run in AVL; a symmetric zero-camber plate has CL(-a) = -CL(a), used for the -4 and -2 deg points


def planform(ar, lam):
    b = SPANS[(ar, lam)]
    cr = MAC / ((2 / 3) * (1 + lam + lam * lam) / (1 + lam)); ct = lam * cr
    S = b * (cr + ct) / 2
    return dict(AR=ar, taper=lam, b=b, cr=cr, ct=ct, S=S, AR_check=b * b / S)


def run_avl(tag, pf, nc, ns):
    f = os.path.join(OUTAVL, f"{tag}.avl")
    cr, ct, b = pf["cr"], pf["ct"], pf["b"]
    open(f, "w").write(f"""Flat plate AR{pf['AR']} taper {pf['taper']} ({nc}x{ns})
0.0
0 0 0
{pf['S']:.5f} {MAC} {b:.4f}
{0.25 * cr:.5f} 0.0 0.0
0.0
SURFACE
Wing
{nc} 1.0 {ns} 1.0
YDUPLICATE
0.0
SECTION
0.0 0.0 0.0 {cr:.5f} 0.0
NACA
0000
SECTION
{0.25 * (cr - ct):.5f} {b / 2:.4f} 0.0 {ct:.5f} 0.0
NACA
0000
""")
    # relative file name below: AVL truncates long path names
    cmds = ["load " + os.path.basename(f), "oper"] + sum([[f"a a {a}", "x"] for a in ALPHAS], []) + ["a a 0", "x", "st", "", "", "quit"]
    p = subprocess.run([AVL], input="\n".join(cmds) + "\n", capture_output=True, text=True, cwd=OUTAVL, timeout=300)
    open(os.path.join(OUTAVL, tag + ".out"), "w").write(p.stdout)
    blocks = re.split(r"Run case:", p.stdout)[1:]
    cl = {}
    for a, blk in zip(ALPHAS, blocks):
        assert abs(float(re.search(r"Alpha\s*=\s*([-0-9.Ee+]+)", blk).group(1)) - a) < 1e-3
        cl[a] = float(re.search(r"CLtot\s*=\s*([-0-9.Ee+]+)", blk).group(1))
    st = p.stdout[p.stdout.rindex("Stability-axis derivatives"):]
    cla_rad = float(re.search(r"CLa\s*=\s*([-0-9.Ee+]+)", st).group(1))
    # least-squares line through the same alpha set as the tunnel fit window (-4..6 deg)
    cl[-2], cl[-4] = -cl[2], -cl[4]
    xs = [-4, -2, 0, 2, 4, 6]; ys = [cl[a] for a in xs]; mx = sum(xs) / len(xs); my = sum(ys) / len(ys)
    ls = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    return dict(tangent0=cla_rad * math.pi / 180, secant4=cl[4] / 4.0, ls_fit=ls, CL=cl)


# ------------------------------------------------------------------------------ Task A
rows_A = []
avl = {}
for (ar, lam) in sorted(SPANS):
    pf = planform(ar, lam)
    assert abs(pf["AR_check"] - ar) / ar < 0.01, (ar, lam, pf["AR_check"])
    res = run_avl(f"AR{ar}_taper{lam:g}_8x12", pf, 8, 12)
    avl[(ar, lam)] = res
    rows_A.append([ar, lam, pf["b"], round(pf["cr"], 4), round(pf["ct"], 4), round(pf["S"], 3), round(pf["AR_check"], 3),
                   round(res["tangent0"], 5), round(res["secant4"], 5), round(res["ls_fit"], 5)])
with open(os.path.join(HERE, "avl_planforms.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["AR_nominal", "taper", "span_in", "root_chord_in", "tip_chord_in", "area_in2", "AR_from_b2_over_S",
                "AVL_CLa_tangent_at_0_per_deg", "AVL_CL4_over_4_secant_per_deg", "AVL_LS_fit_-4_to_6_per_deg"])
    w.writerows(rows_A)

# panelling sensitivity on AR 3 taper 1
pan = []
pf3 = planform(3, 1.0)
for nc, ns in ((6, 8), (8, 12), (12, 20), (24, 40)):
    r = run_avl(f"AR3_taper1_{nc}x{ns}", pf3, nc, ns)
    pan.append([nc, ns, round(r["tangent0"], 5), round(r["secant4"], 5), round(r["ls_fit"], 5)])
base = pan[1][4]
with open(os.path.join(HERE, "panelling_check.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["nchord", "nspan_per_half", "tangent0_per_deg", "secant4_per_deg", "LS_fit_per_deg", "LS_fit_vs_8x12_pct"])
    for r in pan: w.writerow(r + [round(100 * (r[4] / base - 1), 3)])
pan_spread = max(abs(100 * (r[4] / base - 1)) for r in pan)

# ------------------------------------------------------------------------------ Task B
fits = collections.defaultdict(list)
for r in csv.DictReader(open(os.path.join(CODES, "results", "runs", "rigid_glider", "plate_balance", "step2_fits.csv"))):
    fits[(r["wing"], r["run"], float(r["Re"]))].append(r)
tab = []
for (wing, run, Re), legs in sorted(fits.items(), key=lambda kv: (kv[0][0], kv[0][2])):
    m = re.match(r"AR(\d+)_taper([\d.]+)", wing); ar, lam = int(m.group(1)), float(m.group(2))
    sl = [float(l["cl_alpha_per_deg"]) for l in legs]
    t = sum(sl) / len(sl)
    leg_diff = max(sl) - min(sl)
    # rough standard error of one fit slope: rms_CL / sqrt(Sxx), Sxx ~ n * (10 deg)^2 / 12 for the -4..6 window
    se = sum(float(l["rms_CL"]) / math.sqrt(int(l["n"]) * 100 / 12) for l in legs) / len(legs) / math.sqrt(len(legs))
    a = avl[(ar, lam)]
    for kind in ("ls_fit", "secant4", "tangent0"):
        pass
    tab.append(dict(wing=wing, run=run, AR=ar, taper=lam, Re=Re, Re_nom=int(round(Re / 10000.0)) * 10000, tunnel=t, leg_diff=leg_diff, se=se,
                    avl_ls=a["ls_fit"], avl_sec=a["secant4"], avl_tan=a["tangent0"],
                    ex_ls=100 * (t / a["ls_fit"] - 1), ex_sec=100 * (t / a["secant4"] - 1), ex_tan=100 * (t / a["tangent0"] - 1),
                    ex_err=100 * math.sqrt(se * se + (leg_diff / 2) ** 2) / a["ls_fit"]))
with open(os.path.join(HERE, "excess_table.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["wing", "AR", "taper", "run", "Re", "tunnel_slope_per_deg(-4..6 fit, mean of up/down legs)", "up_down_leg_difference", "approx_slope_s.e.",
                "AVL_slope_LS_-4..6", "excess_pct_vs_AVL_LS", "approx_uncertainty_pct", "AVL_secant_CL4/4", "excess_pct_vs_secant", "AVL_tangent_0", "excess_pct_vs_tangent"])
    for r in tab:
        w.writerow([r["wing"], r["AR"], r["taper"], r["run"], round(r["Re"]), round(r["tunnel"], 5), round(r["leg_diff"], 5), round(r["se"], 5),
                    round(r["avl_ls"], 5), round(r["ex_ls"], 2), round(r["ex_err"], 2), round(r["avl_sec"], 5), round(r["ex_sec"], 2), round(r["avl_tan"], 5), round(r["ex_tan"], 2)])

out = []
P = out.append
exs = [r["ex_ls"] for r in tab]
P(f"AVL panelling check, AR 3 taper 1 (LS slope over -4..6 deg, 8x12 per half = reference): " + "; ".join(f"{r[0]}x{r[1]}: {r[4]:.5f} ({100 * (r[4] / base - 1):+.2f} %)" for r in pan) + f". Largest difference {pan_spread:.2f} %.")
P("")
P(f"{len(tab)} runs, {len(set(r['wing'] for r in tab))} wings. Excess of the tunnel low-angle slope (fit -4..6 deg, mean of up and down legs) over AVL's slope fitted over the same -4..6 deg (primary comparison):")
P(f"  overall min {min(exs):.1f} %, max {max(exs):.1f} %, mean {sum(exs) / len(exs):.1f} %. Against AVL's secant slope CL(4)/4: min {min(r['ex_sec'] for r in tab):.1f} %, max {max(r['ex_sec'] for r in tab):.1f} %; against the tangent slope at 0 deg: min {min(r['ex_tan'] for r in tab):.1f} %, max {max(r['ex_tan'] for r in tab):.1f} %.")
P("  per nominal Reynolds number (spread across wings at the same Re):")
for re_ in sorted(set(r["Re_nom"] for r in tab)):
    v = [r["ex_ls"] for r in tab if r["Re_nom"] == re_]
    mean = sum(v) / len(v); sd = (sum((x - mean) ** 2 for x in v) / len(v)) ** 0.5
    P(f"    Re {re_:>7,}: n = {len(v):2d} wings, min {min(v):5.1f} %, max {max(v):5.1f} %, mean {mean:5.1f} %, sd {sd:4.1f} points")
P("  per wing (excess against Reynolds number): range over the Re tested and the least-squares trend per 10,000 of Re:")
for wing in sorted(set(r["wing"] for r in tab)):
    rs = sorted([r for r in tab if r["wing"] == wing], key=lambda r: r["Re"])
    xs = [r["Re"] / 1e4 for r in rs]; ys = [r["ex_ls"] for r in rs]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    tr = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    P(f"    {wing:<14} Re {rs[0]['Re'] / 1e3:.0f}k-{rs[-1]['Re'] / 1e3:.0f}k ({len(rs)} runs): {ys[0]:5.1f} % -> {ys[-1]:5.1f} %  (trend {tr:+.2f} points per 10k Re)")
P("  mean up/down-leg slope difference " + f"{sum(r['leg_diff'] for r in tab) / len(tab):.4f} per deg ({100 * sum(r['leg_diff'] / r['avl_ls'] for r in tab) / len(tab):.1f} % of AVL's slope); mean approximate fit s.e. {100 * sum(r['se'] / r['avl_ls'] for r in tab) / len(tab):.1f} % of AVL's slope.")
open(os.path.join(HERE, "excess_summary.txt"), "w").write("\n".join(out) + "\n")
print("\n".join(out))

# SVG: excess against Re, one line per wing
W, H, L, R, T, B = 820, 480, 64, 190, 36, 52
cols = {"AR2": "#ff8a80", "AR3": "#7fb4ff", "AR4": "#6cc08b", "AR5": "#e89a55"}
dash = {1.0: "", 0.75: "6 3", 0.5: "2 3"}
ys_all = [r["ex_ls"] for r in tab]; ylo, yhi = math.floor(min(ys_all) / 5) * 5 - 5, math.ceil(max(ys_all) / 5) * 5 + 5
px = lambda re_: L + (re_ - 50000) / (170000 - 50000) * (W - L - R)
py = lambda y: T + (yhi - y) / (yhi - ylo) * (H - T - B)
s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="Segoe UI,sans-serif" font-size="12"><rect width="{W}" height="{H}" fill="#ffffff"/>',
     f'<text x="{L}" y="22" font-size="14" fill="#14181d">Tunnel low-angle lift slope above AVL (percent), by Reynolds number, all ten flat plates</text>']
for y in range(int(ylo), int(yhi) + 1, 5):
    s.append(f'<line x1="{L}" y1="{py(y)}" x2="{W - R}" y2="{py(y)}" stroke="#d8dde2"/><text x="{L - 6}" y="{py(y) + 4}" text-anchor="end" fill="#333">{y}</text>')
for re_ in range(60000, 170000, 20000):
    s.append(f'<line x1="{px(re_)}" y1="{T}" x2="{px(re_)}" y2="{H - B}" stroke="#d8dde2"/><text x="{px(re_)}" y="{H - B + 16}" text-anchor="middle" fill="#333">{re_ // 1000}k</text>')
s.append(f'<text x="{(L + W - R) / 2}" y="{H - 10}" text-anchor="middle" fill="#14181d">Reynolds number</text><text x="16" y="{(T + H - B) / 2}" transform="rotate(-90 16 {(T + H - B) / 2})" text-anchor="middle" fill="#14181d">excess over AVL slope, %</text>')
ly = T + 6
for wing in sorted(set(r["wing"] for r in tab)):
    rs = sorted([r for r in tab if r["wing"] == wing], key=lambda r: r["Re"])
    m = re.match(r"AR(\d+)_taper([\d.]+)", wing); c = cols["AR" + m.group(1)]; d = dash[float(m.group(2))]
    pts = " ".join(f"{px(r['Re']):.1f},{py(r['ex_ls']):.1f}" for r in rs)
    da = f' stroke-dasharray="{d}"' if d else ""
    s.append(f'<polyline points="{pts}" fill="none" stroke="{c}" stroke-width="2"{da}/>' + "".join(f'<circle cx="{px(r["Re"]):.1f}" cy="{py(r["ex_ls"]):.1f}" r="3" fill="{c}"/>' for r in rs))
    s.append(f'<line x1="{W - R + 14}" y1="{ly}" x2="{W - R + 44}" y2="{ly}" stroke="{c}" stroke-width="2"{da}/><text x="{W - R + 50}" y="{ly + 4}" fill="#14181d">AR {m.group(1)}, taper {m.group(2)}</text>')
    ly += 18
s.append("</svg>")
open(os.path.join(HERE, "excess_vs_Re.svg"), "w", encoding="utf-8").write("\n".join(s))
print("wrote excess_vs_Re.svg")

# ------------------------------------------------------------------------------ Task C: sizes of the thesis wall corrections, AR 3 taper 1, Re 60k
# Inputs from the thesis: test section 2.8 x 4.0 ft (README, PDF p. 32/54: "rectangular test section measuring 2.8 x 4.0 ft"); wing span vertical (PDF p. 32);
# MSF = b/H, maximum 0.52 (PDF p. 50) => H = 2.8 ft = 33.6 in is the dimension along the span; 4.0 ft = 48 in is the dimension along the lift direction.
# Eq. 3.20-3.28 are quoted in the folder README. K1, tau1, delta, tau2 are 'determined from figures in Ref. 55' and are NOT given in the thesis text.
# Here: delta is computed from the image-vortex system of a centred horseshoe vortex (span pi*b/4) in a closed rectangular tunnel (my own calculation,
# stated assumption: wing centred in the section); K1*tau1 and tau2 are bounded, not looked up.
def delta_images(b, Bspan, Wlift, nmax=60):
    s_ = math.pi * b / 4.0
    def upwash_unit(zeta):          # boundary-induced velocity per unit circulation along the lift direction (positive = increases alpha)
        v = 0.0
        for m in range(-nmax, nmax + 1):
            for n in range(-nmax, nmax + 1):
                if m == 0 and n == 0: continue
                yi, zi, sg = m * Wlift, n * Bspan, (-1) ** abs(m)
                for zl, gam in ((zi + s_ / 2, +1.0), (zi - s_ / 2, -1.0)):   # two trailing legs of the image horseshoe
                    d2 = yi * yi + (zeta - zl) ** 2
                    v += -sg * gam * (zeta - zl) / (4 * math.pi * d2)         # v_y of a semi-infinite leg (strength sg*gam, line along +x), see README
        return -v   # upwash = opposite to the wing's own downwash (+y)
    zs = [(-s_ / 2 + s_ * (i + 0.5) / 40) for i in range(40)]
    avg = sum(upwash_unit(z) for z in zs) / len(zs)
    return upwash_unit(0.0), avg, s_

tun_B, tun_W = 33.6, 48.0
b3 = SPANS[(3, 1.0)]; S3 = pf3["S"]; C = tun_B * tun_W
u0, uavg, s3 = delta_images(b3, tun_B, tun_W)
res = []
Q = res.append
Q("Wall-correction sizes for the AR 3 taper 1 plate (thesis PDF pp. 57-59 = printed 41-43, Eq. 3.20-3.28), computed here")
Q(f"Test section {tun_B} in (span direction, H) x {tun_W} in (lift direction) = {C:.0f} in^2; model span {b3} in, area {S3:.2f} in^2, S/C = {S3 / C:.5f}; MSF = b/H = {b3 / tun_B:.3f}.")
rows_t = [r for r in csv.DictReader(open(os.path.join(CODES, "research", "plate_data", "ananda2012_flatplate_AR3_taper1.csv"))) if r["run"] == "0383ar"]
def interp(alpha, key, leg):
    L_ = sorted([(float(r["alpha_deg"]), float(r[key])) for r in rows_t if r["sweep"] == leg])
    for (a0, v0), (a1, v1) in zip(L_, L_[1:]):
        if a0 <= alpha <= a1: return v0 + (alpha - a0) / (a1 - a0) * (v1 - v0)
CD0 = min(float(r["CD"]) for r in rows_t if abs(float(r["alpha_deg"])) < 3)
pts_ = [(float(r["CL"]) ** 2, float(r["CD"])) for r in rows_t if abs(float(r["CL"])) < 0.5 and abs(float(r["alpha_deg"])) < 8]
mx = sum(p[0] for p in pts_) / len(pts_); my = sum(p[1] for p in pts_) / len(pts_)
k_ind = sum((x - mx) * (y - my) for x, y in pts_) / sum((x - mx) ** 2 for x, _ in pts_)
Q(f"From the table (run 0383ar): minimum CD {CD0:.4f}; CD against CL^2 slope {k_ind:.3f} (fit over |CL| < 0.5, |alpha| < 8 deg).")
for alpha in (4.0, 6.0):
    cl = sum(interp(alpha, "CL", leg) for leg in ("increasing", "decreasing")) / 2
    cd = sum(interp(alpha, "CD", leg) for leg in ("increasing", "decreasing")) / 2
    cmv = sum(interp(alpha, "CM_c4", leg) for leg in ("increasing", "decreasing")) / 2
    slope = 0.0607            # per deg, README fit -6..6
    # blockage
    sb_lo, sb_hi = 0.5 * 4.7 / C ** 1.5, 1.2 * 4.7 / C ** 1.5       # K1*tau1 in 0.5..1.2 (assumed bounds), wing volume ~ 0.9*S*0.15 in = 4.7 in^3
    cdi = k_ind * cl * cl
    wb_a = S3 / (4 * C) * CD0 + 5 * S3 / (4 * C) * (cd - cdi - CD0 * 0)            # reading 1: CDi = k*CL^2 (induced), CD0 = minimum drag
    wb_a = S3 / (4 * C) * CD0 + 5 * S3 / (4 * C) * max(0.0, cd - cdi)
    wb_b = S3 / (4 * C) * CD0 + 5 * S3 / (4 * C) * max(0.0, cd - (cdi - CD0) - CD0)   # reading 2: Eq. 3.22 as printed (CDi = k CL^2 - CD0)
    eT_lo, eT_hi = sb_lo + min(wb_a, wb_b), sb_hi + max(wb_a, wb_b)
    # streamline curvature
    for lab, u in (("centre of the span", u0), ("span average", uavg)):
        d_alpha_rad = (S3 * cl / (2 * s3)) * u            # delta_alpha = upwash / U, Gamma/U = S CL / (2 s)
        d_alpha = math.degrees(d_alpha_rad)
        delta = d_alpha_rad * C / (S3 * cl)
        lo, hi = d_alpha, d_alpha * 1.3                   # tau2 between 0 and 0.3 (assumed bound) multiplies by (1 + tau2)
        dcm_lo, dcm_hi = 0.125 * 0.0 * (d_alpha / cl) * slope, 0.125 * 0.3 * (d_alpha / cl) * slope
        Q(f"alpha_u about {alpha:g} deg (tunnel CL {cl:.4f}, CD {cd:.4f}, CM {cmv:+.4f}): [{lab}] boundary factor delta = {delta:.3f}; angle correction {lo:.3f} to {hi:.3f} deg = {100 * lo / alpha:.2f} to {100 * hi / alpha:.2f} % of the angle (tau2 0..0.3 assumed)")
    Q(f"   blockage: solid eps_sb {sb_lo:.1e} to {sb_hi:.1e} (K1 tau1 assumed 0.5..1.2), wake eps_wb {min(wb_a, wb_b):.1e} to {max(wb_a, wb_b):.1e}; eps_T {eT_lo:.1e} to {eT_hi:.1e}; q_c/q_u - 1 = {100 * ((1 + eT_lo) ** 2 - 1):.3f} to {100 * ((1 + eT_hi) ** 2 - 1):.3f} %; CL and CM change by the inverse of that factor ({100 * (1 / (1 + eT_hi) ** 2 - 1):.3f} to {100 * (1 / (1 + eT_lo) ** 2 - 1):.3f} %)")
    d_a = math.degrees((S3 * cl / (2 * s3)) * uavg)
    Q(f"   effect on the lift slope if the plotted angle is shifted by delta_alpha: slope reduced by about {100 * d_a / alpha:.2f} to {100 * 1.3 * d_a / alpha:.2f} % (CL itself unchanged)")
    Q(f"   moment term 0.125 tau2 delta_SC (dCL/dalpha) with tau2 0..0.3: 0 to {0.125 * 0.3 * (d_a / cl) * slope:.5f} (tunnel CM at this angle {cmv:+.4f})")
open(os.path.join(HERE, "wall_corrections.txt"), "w").write("\n".join(res) + "\n")
print("\n".join(res))
