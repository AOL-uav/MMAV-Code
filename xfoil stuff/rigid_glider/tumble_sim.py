"""Tumble / flight-mode simulation: the Li, Goodwill, Wang & Ristroph 2022 quasi-steady 2-D falling-plate model, implemented as published.

Source: Li, Goodwill, Wang, Ristroph, "Centre of mass location, flight modes, stability and dynamic modelling of gliders", J. Fluid Mech. 937, A6 (2022),
doi 10.1017/jfm.2022.89, sections 4 and 5.  Equation numbers in comments below are THEIR equation numbers.  Tags: CITED (from the paper), CALCULATED, ASSUMED, JUDGEMENT.

Nothing here modifies or imports the pull-out simulation.  Standard library only.
    py -B rigid_glider/tumble_sim.py stepA      validation against the paper-plane case (writes results/runs/tumble_sim/stepA_*)
    py -B rigid_glider/tumble_sim.py stepB      glider application (only meaningful after Step A passes; see research/tumble_model/README.md)
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import multiprocessing as mp
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "results/runs/tumble_sim"

# ------------------------------------------------------------------------------------------------ constants (CITED, Li et al. 2022 section 5)
CL1, CL2 = 5.2, 0.95                 # CITED: C_L^1, C_L^2 (below eq 5.2)
ALPHA0, DELTA = math.radians(14.0), math.radians(6.0)    # CITED: stall angle alpha_0 = 14 deg, smoothness delta = 6 deg (below eq 5.2)
CD0, CD1, CDPI2 = 0.1, 5.0, 1.9      # CITED: C_D^0, C_D^1, C_D^{pi/2} (below eq 5.4)
CCP0, CCP1, CCP2 = 0.3, 3.5, 0.2     # CITED: C_CP^0, C_CP^1, C_CP^2 (below eq 5.6)
CR = 1.1                             # CITED: rotational lift coefficient C_R (section 5, after eq 4.11)
HALF_PI = 0.5 * math.pi


# ------------------------------------------------------------------------------------------------ aerodynamic coefficients, eqs 5.1-5.7
def _f(a):
    """Eq 5.2: selection function f(alpha) = (1 - tanh((alpha - alpha0)/delta))/2, a in [0, pi/2]."""
    return 0.5 * (1.0 - math.tanh((a - ALPHA0) / DELTA))


def _cl_t(a):
    """Eq 5.1: tilde C_L on [0, pi/2]."""
    f = _f(a)
    return f * CL1 * math.sin(a) + (1.0 - f) * CL2 * math.sin(2.0 * a)


def _cd_t(a):
    """Eq 5.4: tilde C_D on [0, pi/2]."""
    f = _f(a)
    s2 = math.sin(a) ** 2
    return f * (CD0 + CD1 * s2) + (1.0 - f) * CDPI2 * s2


def _cp_t(a):
    """Eq 5.6: tilde l_CP / l on [0, pi/2]."""
    f = _f(a)
    return f * (CCP0 - CCP1 * a * a) + (1.0 - f) * CCP2 * (1.0 - a / HALF_PI)


def coeffs(alpha):
    """alpha in [-pi, pi] -> (C_L, C_D, l_CP/l) by the symmetry extensions 5.3, 5.5, 5.7 (exactly as printed)."""
    a = abs(alpha)
    if alpha >= 0.0:
        if alpha <= HALF_PI:                                   # [0, pi/2]
            return _cl_t(alpha), _cd_t(alpha), _cp_t(alpha)
        b = math.pi - alpha                                    # [pi/2, pi]
        return -_cl_t(b), _cd_t(b), -_cp_t(b)
    if alpha >= -HALF_PI:                                      # [-pi/2, 0]
        return -_cl_t(a), _cd_t(a), _cp_t(a)
    b = math.pi - a                                            # [-pi, -pi/2]
    return _cl_t(b), _cd_t(b), -_cp_t(b)


# ------------------------------------------------------------------------------------------------ the plate (all per unit span)
class Plate:
    """2-D plate per unit span.  l chord, m mass, I moment of inertia about the CoM, lcm CoM offset from the CoV (mid-chord), positive toward +x' (leading edge).
    rho fluid density, g gravity, vol_per_span = V_3D / s (cross-section area), used for m' and tau_B (section 4, text after eq 4.9; section 6.1)."""

    def __init__(self, l, m, I, lcm, rho, g, vol_per_span):
        self.l, self.m, self.I, self.lcm, self.rho, self.g = l, m, I, lcm, rho, g
        self.m22 = math.pi * rho * l * l / 4.0                                    # CITED: m22 = pi rho l^2 / 4 (text after 4.9); m11 = 0
        a = 2.0 * lcm / l
        self.Ia = math.pi * rho * l ** 4 * (1.0 + 8.0 * a * a) / 128.0           # CITED: I_a about the CoM (text after 4.9)
        self.mp = m - rho * vol_per_span                                          # CITED: m' = m - rho_f V_3D / s (section 6.1)
        self.tauB_k = -rho * g * vol_per_span * lcm                               # CITED: tau_B = -rho_f g (V_3D/s) l_CM cos(theta) (section 6.1)
        self.a2 = a
        # eq 4.15 geometric bracket: plus sign if 2 l_CM / l <= 1, minus sign otherwise
        sgn = 1.0 if a <= 1.0 else -1.0
        self.tauR_k = -(1.0 / 128.0) * rho * l ** 4 * CDPI2 * ((a + 1.0) ** 4 + sgn * (a - 1.0) ** 4)

    def deriv(self, s):
        """State s = (x, y, theta, vx', vy', omega) -> time derivative, eqs 4.4-4.9."""
        x, y, th, vx, vy, w = s
        rho, l, lcm = self.rho, self.l, self.lcm
        vcv = vy - w * lcm                                       # CITED: CoV velocity y' component v_y' - omega l_CM (section 4)
        U = math.sqrt(vx * vx + vcv * vcv)
        alpha = math.atan2(vcv, vx)                              # CITED: tan(alpha) = v_y^CV / v_x^CV, alpha in [-pi, pi]
        cl, cd, cpl = coeffs(alpha)
        lcp = cpl * l
        # eq 4.12 lift (translational 4.10 + rotational 4.11) and eq 4.13 drag
        kL = 0.5 * rho * l * (cl * U - l * CR * w)
        Lx, Ly = kL * vcv, -kL * vx
        kD = -0.5 * rho * l * cd * U
        Dx, Dy = kD * vx, kD * vcv
        # eq 4.14 translational torque about the CoM
        tauT = -0.5 * rho * l * U * (cl * vx + cd * vcv) * (lcp - lcm)
        # eq 4.15 rotational drag torque
        tauR = self.tauR_k * w * abs(w)
        ct, st = math.cos(th), math.sin(th)
        tauB = self.tauB_k * ct
        wd = (tauT + tauR + tauB) / (self.I + self.Ia)                               # eq 4.9
        m, m22, mp, g = self.m, self.m22, self.mp, self.g
        vxd = ((m + m22) * w * vy - m22 * w * w * lcm + Lx + Dx - mp * g * st) / m   # eq 4.7 (m11 = 0)
        vyd = (-m * w * vx + m22 * wd * lcm + Ly + Dy - mp * g * ct) / (m + m22)     # eq 4.8 (m11 = 0)
        return (vx * ct - vy * st, vx * st + vy * ct, w, vxd, vyd, wd)               # eqs 4.4, 4.5, 4.6

    def step(self, s, dt):
        d = self.deriv
        k1 = d(s)
        k2 = d(tuple(s[i] + 0.5 * dt * k1[i] for i in range(6)))
        k3 = d(tuple(s[i] + 0.5 * dt * k2[i] for i in range(6)))
        k4 = d(tuple(s[i] + dt * k3[i] for i in range(6)))
        return tuple(s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(6))


# ------------------------------------------------------------------------------------------------ run + classifier
def run(plate, theta0, omega0, T, dt, vx0=0.0, vy0=0.0, sample=0.05):
    """Integrate (RK4, fixed step); return samples (t, theta, alpha, speed, y) every `sample` s, the final state and a blow-up flag.
    Windows for the classifier are cut from this record."""
    s = (0.0, 0.0, theta0, vx0, vy0, omega0)
    n = int(round(T / dt))
    k = max(1, int(round(sample / dt)))
    rec = []
    for i in range(n + 1):
        if i % k == 0:
            vcv = s[4] - s[5] * plate.lcm
            rec.append((i * dt, s[2], math.atan2(vcv, s[3]), math.hypot(s[3], vcv), s[1]))
        if i == n:
            break
        s = plate.step(s, dt)
        if s[0] != s[0] or abs(s[3]) > 1e5 or abs(s[4]) > 1e5 or abs(s[5]) > 1e5:
            return rec, s, True
    return rec, s, False


def window(rec, t0, t1):
    return [r for r in rec if t0 - 1e-9 <= r[0] <= t1 + 1e-9]


def _circ_mean_range(angles):
    """Circular mean and max |deviation| range of angles (rad)."""
    c = sum(math.cos(a) for a in angles) / len(angles)
    sn = sum(math.sin(a) for a in angles) / len(angles)
    mu = math.atan2(sn, c)
    dev = [math.atan2(math.sin(a - mu), math.cos(a - mu)) for a in angles]
    return mu, max(dev) - min(dev)


# classifier thresholds (frozen in research/tumble_model/README.md before the sweep)
TUMBLE_ROT = 2.0 * math.pi
STEADY_THETA_RANGE = math.radians(10.0)
STEADY_ALPHA_RANGE = math.radians(5.0)
STEADY_SPEED_RANGE = 0.10
DIVE_ALPHA = math.radians(2.0)


def classify(rec, blown=False):
    """-> (mode, stats).  Modes: TUMBLE, DIVE, GLIDE, BOUND/FLUTTER (and ERROR if the integration blew up)."""
    if blown or len(rec) < 5:
        return "ERROR", {}
    th = [r[1] for r in rec]
    al = [r[2] for r in rec]
    sp = [r[3] for r in rec]
    dth = th[-1] - th[0]
    th_range = max(th) - min(th)
    mu, a_range = _circ_mean_range(al)
    smean = sum(sp) / len(sp)
    s_range = (max(sp) - min(sp)) / smean if smean > 1e-9 else 1.0
    stats = dict(rot_net=dth / TUMBLE_ROT, theta_range_deg=math.degrees(th_range), alpha_mean_deg=math.degrees(mu), alpha_range_deg=math.degrees(a_range), speed_mean=smean, speed_range_frac=s_range)
    if abs(dth) >= TUMBLE_ROT:
        return "TUMBLE", stats
    if th_range < STEADY_THETA_RANGE and a_range < STEADY_ALPHA_RANGE and s_range < STEADY_SPEED_RANGE:
        return ("DIVE" if abs(mu) < DIVE_ALPHA else "GLIDE"), stats
    return "BOUND/FLUTTER", stats


MODE_RANK = {"TUMBLE": 0, "BOUND/FLUTTER": 1, "GLIDE": 2, "DIVE": 3}


def run_classify(plate, theta0, omega0, T, dt, win):
    """Terminal mode from the last `win` seconds of a T-second run."""
    rec, s, blown = run(plate, theta0, omega0, T, dt)
    return classify(window(rec, T - win, T), blown)


# ------------------------------------------------------------------------------------------------ Step A: Li et al. paper-plane case (section 6.3)
PP_L = 5.1                 # CITED: chord 2 in = 5.1 cm
PP_SPAN = 15.0             # CITED: span 6 in = 15 cm
PP_MP = 0.59               # CITED: bare paper mass 0.59 g
PP_RHO = 1.2e-3            # CITED: air, g/cm^3
PP_H_OVER_L = 0.002        # CITED: h/l = 0.002 (section 7)
PP_G = 981.0               # CALCULATED: cm/s^2
A_ATT = tuple(math.radians(d) for d in range(0, 360, 45))     # JUDGEMENT: 8 attitudes
A_RATE = (-10.0, 0.0, 10.0)                                    # JUDGEMENT: rad/s
A_T, A_DT, A_WIN = 15.0, 0.002, 5.0                            # JUDGEMENT


def paper_plane(lcm_over_l, variant="geometry"):
    """Paper + leading-edge tape.  ASSUMED: tape = line mass at the leading edge sized to put the CoM at l_CM; uniform paper; side fins ignored.
    variant 'geometry': I = paper (1/12 mp l^2 + mp lcm^2) + tape mw (l/2 - lcm)^2.  variant 'generic': I = m (l^2/12 + lcm^2) with the total mass (the paper's generic formula, section 4)."""
    l = PP_L
    lcm = lcm_over_l * l
    mp_ = PP_MP
    mw = mp_ * lcm / (0.5 * l - lcm) if lcm > 0 else 0.0
    m_tot = mp_ + mw
    if variant == "geometry":
        I3 = mp_ * (l * l / 12.0 + lcm * lcm) + mw * (0.5 * l - lcm) ** 2
    else:
        I3 = m_tot * (l * l / 12.0 + lcm * lcm)
    vol = PP_H_OVER_L * l * l * PP_SPAN                                  # cm^3, h * l * s
    return Plate(l, m_tot / PP_SPAN, I3 / PP_SPAN, lcm, PP_RHO, PP_G, vol / PP_SPAN), dict(tape_g=mw, mass_g=m_tot)


def _stepA_job(args):
    lcm_over_l, th0, w0, variant, dt = args
    pl, _ = paper_plane(lcm_over_l, variant)
    mode, st = run_classify(pl, th0, w0, A_T, dt, A_WIN)
    return (lcm_over_l, math.degrees(th0), w0, variant, dt, mode, st)


def summarise(rows, key_cols=("lcm_over_l",)):
    cells = {}
    for r in rows:
        cells.setdefault(r[0], {}).setdefault(r[5], 0)
        cells[r[0]][r[5]] += 1
    out = []
    for k in sorted(cells):
        c = cells[k]
        n = sum(c.values())
        top = max(c, key=lambda m: c[m])
        out.append(dict(lcm_over_l=k, n=n, counts=c, plurality=top, share=c[top] / n, clear=c[top] / n >= 0.8))
    return out


def judge(cells):
    by = {round(c["lcm_over_l"], 2): c for c in cells}
    res = []
    def chk(name, ok, detail):
        res.append(dict(rule=name, ok=bool(ok), detail=detail))
    for v in (0.0, 0.08):
        c = by[v]
        chk(f"1: {v:.2f} clear TUMBLE", c["plurality"] == "TUMBLE" and c["clear"], f"{c['plurality']} {100 * c['share']:.0f}% {c['counts']}")
    c = by[0.24]
    chk("2: 0.24 clear GLIDE", c["plurality"] == "GLIDE" and c["clear"], f"{c['plurality']} {100 * c['share']:.0f}% {c['counts']}")
    c = by[0.31]
    chk("3: 0.31 clear DIVE", c["plurality"] == "DIVE" and c["clear"], f"{c['plurality']} {100 * c['share']:.0f}% {c['counts']}")
    prev, viol = None, []
    for c in cells:
        if not c["clear"] or c["plurality"] not in MODE_RANK:
            continue
        r = MODE_RANK[c["plurality"]]
        if prev is not None and r < prev[1]:
            viol.append(f"{prev[0]:.2f}->{c['lcm_over_l']:.2f}: rank {prev[1]}->{r}")
        prev = (c["lcm_over_l"], r)
    chk("4: monotone rank (mixed cells skipped)", not viol, "; ".join(viol) or "no decreases")
    band = [c["lcm_over_l"] for c in cells if c["clear"] and c["plurality"] == "GLIDE"]
    contiguous = bool(band) and 0.24 in [round(b, 2) for b in band]
    chk("5: glide band contains 0.24", contiguous, f"clear GLIDE cells at {', '.join(f'{b:.2f}' for b in band) or 'none'}")
    return res


def stepA(pool):
    OUT.mkdir(parents=True, exist_ok=True)
    lcms = [round(0.01 * i, 2) for i in range(0, 36)]
    out_rows = {}
    for variant in ("geometry", "generic"):
        jobs = [(x, th, w, variant, A_DT) for x in lcms for th in A_ATT for w in A_RATE]
        rows = pool.map(_stepA_job, jobs, chunksize=4)
        out_rows[variant] = rows
        with (OUT / f"stepA_runs_{variant}.csv").open("w", newline="", encoding="utf-8") as fh:
            w_ = csv.writer(fh)
            w_.writerow(["lcm_over_l", "theta0_deg", "omega0_rad_s", "variant", "dt", "mode", "rot_net", "theta_range_deg", "alpha_mean_deg", "alpha_range_deg", "speed_mean_cm_s", "speed_range_frac"])
            for r in rows:
                st = r[6]
                w_.writerow([r[0], r[1], r[2], r[3], r[4], r[5]] + [f"{st.get(k, float('nan')):.4g}" for k in ("rot_net", "theta_range_deg", "alpha_mean_deg", "alpha_range_deg", "speed_mean", "speed_range_frac")])
        cells = summarise(rows)
        with (OUT / f"stepA_cells_{variant}.csv").open("w", newline="", encoding="utf-8") as fh:
            w_ = csv.writer(fh)
            w_.writerow(["lcm_over_l", "n", "plurality", "share", "clear", "TUMBLE", "BOUND/FLUTTER", "GLIDE", "DIVE", "ERROR"])
            for c in cells:
                w_.writerow([c["lcm_over_l"], c["n"], c["plurality"], f"{c['share']:.2f}", c["clear"]] + [c["counts"].get(m, 0) for m in ("TUMBLE", "BOUND/FLUTTER", "GLIDE", "DIVE", "ERROR")])
        verdicts = judge(cells)
        out_rows[variant + "_verdicts"] = verdicts
        out_rows[variant + "_cells"] = cells
        print(f"--- Step A, variant {variant}: " + ("PASS" if all(v["ok"] for v in verdicts) else "FAIL"))
        for v in verdicts:
            print(f"  [{'ok' if v['ok'] else 'FAIL'}] {v['rule']}: {v['detail']}")
        print("  plurality map: " + " ".join(f"{c['lcm_over_l']:.2f}:{c['plurality'][0]}{'' if c['clear'] else '?'}" for c in cells))
    # dt convergence check: same jobs at dt = 1 ms for the geometry variant at 6 values
    conv_x = (0.0, 0.08, 0.14, 0.24, 0.31, 0.35)
    jobs = [(x, th, w, "geometry", A_DT / 2) for x in conv_x for th in A_ATT for w in A_RATE]
    rows = pool.map(_stepA_job, jobs, chunksize=4)
    base = {(r[0], r[1], r[2]): r[5] for r in out_rows["geometry"]}
    same = sum(1 for r in rows if base[(r[0], r[1], r[2])] == r[5])
    print(f"--- dt convergence (dt 1 ms vs 2 ms, {len(rows)} starts): {same} identical modes")
    json.dump(dict(geometry=out_rows["geometry_verdicts"], generic=out_rows["generic_verdicts"], dt_identical=same, dt_total=len(rows)), (OUT / "stepA_verdict.json").open("w"), indent=1)
    return out_rows


# ------------------------------------------------------------------------------------------------ Step B: our gliders as 2-D strips
# STATEMENT: this is a 2-D model (flat rectangular plate, equations of Li et al. 2022) applied to 3-D swept, tailed, low-aspect-ratio wings.  The coefficient laws (5.1-5.7)
# were fitted to a rectangular plate of aspect ratio 8 in water (Re ~ 1e3-1e4) and are used here unchanged.  No tail bend / reflex is in the Li model: it is NOT added (not published).
B_ATT = tuple(math.radians(d) for d in range(0, 360, 10))        # JUDGEMENT: 36 attitudes, all 360 deg
B_RATE = (-30.0, -20.0, -10.0, -5.0, 0.0, 5.0, 10.0, 20.0, 30.0)  # JUDGEMENT: rad/s, both senses, 0 to 30
B_T, B_DT, B_WIN = 15.0, 0.001, 5.0                              # JUDGEMENT (dt halved vs Step A because the gliders are heavier per chord^2)
B_EARLY = (1.0, 2.5)                                             # JUDGEMENT: window for "a drop from under 10 m" (about 1-3 s of flight); same classifier
RHO_AIR_CGS = 1.2e-3                                             # ASSUMED: same air density as the paper-plane case, g/cm^3 (buoyancy neglected: vol_per_span = 0)


def poly_moments(poly):
    """Area, centroid x and second moment about the centroid of x for a simple polygon (same formula as pullout_sim.poly_moments, copied, not imported)."""
    A = cx = i2 = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        c = x0 * y1 - x1 * y0
        A += c / 2
        cx += (x0 + x1) * c / 6
        i2 += (x0 * x0 + x0 * x1 + x1 * x1) * c / 12
    cx /= A
    return A, cx, i2 - A * cx * cx


def _inertia_about(parts, cg):
    """parts: (mass_g, x_mm, own inertia g mm^2) -> mass g, I about cg in g mm^2."""
    return sum(p[0] for p in parts), sum(p[2] + p[0] * (p[1] - cg) ** 2 for p in parts)


def swift_parts(cg_mm):
    """Swift A foam 400 (GLIDE outline of bird_outline_v4_swift.py) at CG cg_mm (aft of the root LE): sheet + rail + ballast, ballast mass from the project's own size_numbers rule.
    CALCULATED from the project code (imported, not modified).  Returns (parts, geometry numbers) or (None, None) if the ballast cannot reach this CG."""
    import bird_outline_v4_swift as s4
    import bird_outline_v2 as v2
    g = s4.build_geometry4(s4.GLIDE)
    k = 400.0
    n = v2.size_numbers(g, s4.fake_res(cg_mm / k, 0.10), "foam", 5.99)
    if not n["reachable"]:
        return None, None
    poly = [(x * k, y * k) for x, y in g["full_outline"]]
    A, cx, i2 = poly_moments(poly)
    rail = n["rail"]
    parts = [(n["m_sheet"], cx, n["m_sheet"] * i2 / A),
             (rail["mass_g"], rail["cx"], rail["mass_g"] * (rail["saddle_end"] - rail["x_lo"]) ** 2 / 12.0),
             (n["ballast_g"], n["station"], 0.0)]
    geo = dict(span=k, mean_chord=g["S"] * k * k / k, mac=g["mac"] * k, mac_le=g["x_le_mac"] * k, sheet_chord=g["sheet_area"] * k, sheet_centroid=n["cx_sheet"], station=n["station"], ballast_g=n["ballast_g"])
    return parts, geo


def f45_parts(cg_mm):
    """Kit wing F45_span250 (0.6 mm printed plate + rib, flown rib-down), numbers from results/runs/rigid_glider/flat_test_v2/F45_span250/geometry.json (read only), ballast at the -22.5 mm nose hole
    sized for the CG (CALCULATED moment balance mb = M0 (cg0 - cg)/(cg - station))."""
    G_ = json.loads((ROOT / "results/runs/rigid_glider/flat_test_v2/F45_span250/geometry.json").read_text())
    v = lambda key: G_[key]["value"]
    th = G_["thickness_table"]["0.6"]
    span, cr, ct, tip_x, mac, mac_le = v("span_mm"), v("root_chord_mm"), v("tip_chord_mm"), v("tip_le_x_mm"), v("mac_mm"), v("mac_le_x_mm")
    nose = v("stick_nose_ahead_of_root_le_mm")
    poly = [(0, 0), (tip_x, span / 2), (tip_x + ct, span / 2), (cr, 0), (tip_x + ct, -span / 2), (tip_x, -span / 2)]
    A, cx, i2 = poly_moments(poly)
    mp_ = th["plate_mass_g"]["value"]
    sm = th["stick_mass_g"]["value"]
    parts = [(mp_, cx, mp_ * i2 / A),
             (th["rib_mass_g"]["value"], v("rib_x_centroid_mm"), 0.0),
             (th["fins_mass_g"]["value"], tip_x + ct / 2, 0.0),
             (sm, 0.5 * (-nose + cr), sm * (cr + nose) ** 2 / 12.0)]
    M0 = sum(p[0] for p in parts)
    cg0 = sum(p[0] * p[1] for p in parts) / M0
    station = -22.5                                                       # kit nose hole (ballast_holes_x_mm)
    mb = M0 * (cg0 - cg_mm) / (cg_mm - station)
    if mb < 0:
        return None, None
    parts.append((mb, station, 0.0))
    geo = dict(span=span, mean_chord=v("mean_chord_mm"), mac=mac, mac_le=mac_le, sheet_chord=v("mean_chord_mm"), sheet_centroid=v("wing_flat_plate_centroid_x_mm"), station=station, ballast_g=mb)
    return parts, geo


SPEC_DEFS = {   # key: (parts builder, design-mark CG mm, CG map offsets from the mark, mm)
    "Swift_A_foam400": (swift_parts, 37.0, range(-15, 16)),       # 37.0 mm = the build's CG at the 10 % static margin (task brief; reproduced by size_numbers: 9.76 g, 1.20 g ballast)
    "F45_span250": (f45_parts, 38.0, range(-8, 7)),               # 38.0 mm = 10 % MAC ahead of the AVL neutral point with fins (43.9 mm - 5.76 mm) : CALCULATED reference, the kit itself has no single mark
}
# variants: chord choice and reference-point choice.  PRIMARY = true mean chord S/b and the MAC mid-chord (the instruction for this task).
VARIANTS = {"primary": ("mean", "mac"), "chord_MAC": ("mac", "mac"), "chord_sheet": ("sheet", "mac"), "ref_centroid": ("mean", "centroid")}
SKIP = {("F45_span250", "chord_sheet"), ("F45_span250", "ref_centroid")}


def strip_plate(spec, variant, cg_mm):
    """-> (Plate, info) for the 2-D strip: per-unit-span mass and inertia (3-D value / span), chord and CG offset from the reference mid-chord, in cgs units."""
    parts, geo = SPEC_DEFS[spec][0](cg_mm)
    if parts is None:
        return None, None
    cg = sum(p[0] * p[1] for p in parts) / sum(p[0] for p in parts)
    M, I3 = _inertia_about(parts, cg)
    chord_key, ref_key = VARIANTS[variant]
    chord = {"mean": geo["mean_chord"], "mac": geo["mac"], "sheet": geo["sheet_chord"]}[chord_key]
    ref = geo["mac_le"] + 0.5 * geo["mac"] if ref_key == "mac" else geo["sheet_centroid"]
    lcm = ref - cg                                                          # mm, positive = CG ahead of the reference mid-chord
    s_cm = geo["span"] / 10.0
    pl = Plate(chord / 10.0, M / s_cm, I3 / 100.0 / s_cm, lcm / 10.0, RHO_AIR_CGS, PP_G, 0.0)
    return pl, dict(mass_g=M, I_g_mm2=I3, chord_mm=chord, lcm_mm=lcm, lcm_over_l=lcm / chord, cg_mm=cg, ref_mm=ref, ballast_g=geo["ballast_g"], station=geo["station"])


def _stepB_job(args):
    key, variant, d, th0, w0, plate_args = args
    pl = Plate(*plate_args)
    rec, s, blown = run(pl, th0, w0, B_T, B_DT)
    m_term, st = classify(window(rec, B_T - B_WIN, B_T), blown)
    m_early, _ = classify(window(rec, *B_EARLY), blown)
    y_early = next((r[4] for r in rec if r[0] >= B_EARLY[1] - 1e-9), float("nan")) / 100.0      # height change by the end of the early window, m (negative = fallen)
    return (key, variant, d, math.degrees(th0), w0, m_term, m_early, y_early, st)


MODES = ("TUMBLE", "BOUND/FLUTTER", "GLIDE", "DIVE", "ERROR")


def stepB(pool, quick=False):
    OUT.mkdir(parents=True, exist_ok=True)
    jobs, info = [], {}
    for key, (builder, mark, offs) in SPEC_DEFS.items():
        for variant in VARIANTS:
            if (key, variant) in SKIP:
                continue
            for d in offs:
                if quick and d % 3:
                    continue
                pl, inf = strip_plate(key, variant, mark + d)
                if pl is None:
                    continue
                info[(key, variant, d)] = inf
                pa = (pl.l, pl.m, pl.I, pl.lcm, pl.rho, pl.g, 0.0)
                for th in B_ATT:
                    for w in B_RATE:
                        jobs.append((key, variant, d, th, w, pa))
    print(f"Step B: {len(jobs)} runs")
    rows = pool.map(_stepB_job, jobs, chunksize=8)
    with (OUT / "stepB_runs_primary.csv").open("w", newline="", encoding="utf-8") as fh:
        w_ = csv.writer(fh)
        w_.writerow(["spec", "variant", "cg_offset_mm", "theta0_deg", "omega0_rad_s", "mode_terminal", "mode_early_1_2.5s", "height_change_by_2.5s_m", "rot_net_last5s", "alpha_mean_deg", "speed_mean_cm_s"])
        for r in rows:
            if r[1] == "primary":
                st = r[8]
                w_.writerow([r[0], r[1], r[2], r[3], r[4], r[5], r[6], f"{r[7]:.2f}", f"{st.get('rot_net', float('nan')):.3g}", f"{st.get('alpha_mean_deg', float('nan')):.3g}", f"{st.get('speed_mean', float('nan')):.4g}"])
    cells = {}
    for r in rows:
        c = cells.setdefault((r[0], r[1], r[2]), dict(n=0, term={m: 0 for m in MODES}, early={m: 0 for m in MODES}))
        c["n"] += 1
        c["term"][r[5]] += 1
        c["early"][r[6]] += 1
    with (OUT / "stepB_map.csv").open("w", newline="", encoding="utf-8") as fh:
        w_ = csv.writer(fh)
        w_.writerow(["spec", "variant", "cg_offset_mm", "cg_mm", "mass_g", "chord_mm", "lcm_mm", "lcm_over_l", "n_starts"] + [f"terminal_{m}" for m in MODES] + [f"early_{m}" for m in MODES])
        for k in sorted(cells):
            c, i = cells[k], info[k]
            w_.writerow([k[0], k[1], k[2], f"{i['cg_mm']:.2f}", f"{i['mass_g']:.2f}", f"{i['chord_mm']:.2f}", f"{i['lcm_mm']:.2f}", f"{i['lcm_over_l']:.3f}", c["n"]]
                        + [f"{c['term'][m] / c['n']:.3f}" for m in MODES] + [f"{c['early'][m] / c['n']:.3f}" for m in MODES])
    json.dump({f"{k[0]}|{k[1]}|{k[2]}": dict(info=info[k], n=cells[k]["n"], terminal=cells[k]["term"], early=cells[k]["early"]) for k in cells}, (OUT / "stepB_cells.json").open("w"), indent=1)
    write_index(cells, info)
    for k in sorted(cells):
        if k[1] == "primary" and k[2] in (-6, -3, 0, 3, 6):
            c, i = cells[k], info[k]
            print(f"{k[0]:16s} d={k[2]:+3d} mm CG {i['cg_mm']:.1f} m {i['mass_g']:.2f} g l_CM/l {i['lcm_over_l']:.3f}  terminal " + " ".join(f"{m[:4]} {100 * c['term'][m] / c['n']:.0f}%" for m in MODES[:4])
                  + "  | early " + " ".join(f"{m[:4]} {100 * c['early'][m] / c['n']:.0f}%" for m in MODES[:4]))
    return cells, info


COL = {"TUMBLE": "#e5534b", "BOUND/FLUTTER": "#e0a030", "GLIDE": "#4aa3ff", "DIVE": "#9d7bd8", "ERROR": "#888888"}


def _bars(cells, info, spec, variant, which):
    ks = sorted(k for k in cells if k[0] == spec and k[1] == variant)
    if not ks:
        return ""
    W, H, L, B = 760, 200, 46, 46
    bw = (W - L - 10) / len(ks)
    o = [f"<svg viewBox='0 0 {W} {H + B}' width='{W}' height='{H + B}' style='max-width:100%;background:#14181d;border:1px solid #333c46;border-radius:6px'>"]
    for j, k in enumerate(ks):
        c = cells[k][which]
        n = cells[k]["n"]
        y = 8.0
        for m in MODES:
            h = (H - 8) * c[m] / n
            if h > 0:
                o.append(f"<rect x='{L + j * bw:.1f}' y='{y:.1f}' width='{bw - 1:.1f}' height='{h:.1f}' fill='{COL[m]}'/>")
            y += h
        if (k[2] % 3 == 0) or len(ks) < 12:
            o.append(f"<text x='{L + j * bw + bw / 2:.1f}' y='{H + 14}' font-size='10' fill='#9aa7b4' text-anchor='middle'>{k[2]:+d}</text>")
            o.append(f"<text x='{L + j * bw + bw / 2:.1f}' y='{H + 27}' font-size='9' fill='#7d8996' text-anchor='middle'>{info[k]['lcm_over_l']:.2f}</text>")
        if k[2] == 0:
            o.append(f"<line x1='{L + j * bw + bw / 2:.1f}' x2='{L + j * bw + bw / 2:.1f}' y1='6' y2='{H}' stroke='#ffffff' stroke-dasharray='3 3'/>")
    o.append(f"<text x='4' y='14' font-size='10' fill='#9aa7b4'>100%</text><text x='4' y='{H}' font-size='10' fill='#9aa7b4'>0%</text>")
    o.append(f"<text x='{L}' y='{H + 41}' font-size='10' fill='#9aa7b4'>CG offset from the design mark, mm (+ aft); second row = l_CM/l (CG ahead of the reference mid-chord / strip chord)</text></svg>")
    return "".join(o)


def write_index(cells, info):
    import html as H_
    P = ["<!doctype html><html lang='en'><meta charset='utf-8'><title>Tumble simulation (Li et al. 2022 model)</title><style>body{background:#0f1317;color:#dde3ea;font-family:Segoe UI,Arial,sans-serif;margin:24px auto;max-width:900px;line-height:1.45}"
         "h1,h2,h3{color:#fff}table{border-collapse:collapse}td,th{border:1px solid #333c46;padding:3px 8px;font-size:13px}.sw{display:inline-block;width:12px;height:12px;margin:0 4px 0 12px}small{color:#9aa7b4}</style><body>",
         "<h1>Tumble simulation, Li, Goodwill, Wang &amp; Ristroph 2022 model</h1>",
         "<p><b>Not a flight result.</b> A 2-D flat-plate quasi-steady model (JFM 937 A6, eqs 4.4-4.15 and 5.1-5.7, all constants CITED) applied to 3-D swept, tailed, low-aspect-ratio gliders as a strip. "
         "The tail bend (reflex) is not in the model and is not represented. Step A was an implementation/consistency check, not a validation; this model is not usable for setting a CG. See README.</p>"]
    P.append("<p>" + "".join(f"<span class='sw' style='background:{COL[m]}'></span>{m}" for m in MODES[:4]) + "</p>")
    for spec in SPEC_DEFS:
        for variant in VARIANTS:
            for which, title in (("term", "terminal mode (last 5 s of a 15 s run)"), ("early", "mode in the 1.0-2.5 s window (a drop from under 10 m)")):
                svg = _bars(cells, info, spec, variant, which)
                if svg:
                    P.append(f"<h3>{H_.escape(spec)}, variant {variant}: {title}</h3>{svg}")
    P.append("<h2>Design-mark table (primary variant)</h2><table><tr><th>spec</th><th>CG offset mm</th><th>CG mm</th><th>l_CM/l</th><th>terminal T/B/G/D %</th><th>early T/B/G/D %</th></tr>")
    for k in sorted(cells):
        if k[1] == "primary" and k[2] in (-6, -3, 0, 3, 6):
            c = cells[k]
            fmt = lambda d: " / ".join(f"{100 * d[m] / c['n']:.0f}" for m in MODES[:4])
            P.append(f"<tr><td>{k[0]}</td><td>{k[2]:+d}</td><td>{info[k]['cg_mm']:.1f}</td><td>{info[k]['lcm_over_l']:.3f}</td><td>{fmt(c['term'])}</td><td>{fmt(c['early'])}</td></tr>")
    P.append("</table><p><small>Files: stepB_map.csv, stepB_runs_primary.csv, stepB_cells.json, stepA_*.csv. Script rigid_glider/tumble_sim.py.</small></p></body></html>")
    (OUT / "index.html").write_text("\n".join(P), encoding="utf-8")


if __name__ == "__main__":
    mp.freeze_support()
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "stepA":
        with mp.Pool(max(1, (mp.cpu_count() or 2) - 1)) as pool:
            stepA(pool)
    elif cmd in ("stepB", "stepB-quick"):
        with mp.Pool(max(1, (mp.cpu_count() or 2) - 1)) as pool:
            stepB(pool, quick=(cmd == "stepB-quick"))
    else:
        sys.exit(__doc__)
