"""Part 1 (v2): EXPLORATORY mass models -> wing loading and Re as functions of scale (analytic, no AVL).

Planform: straight-tapered trapezoid (taper 0.7 unless stated), S = b^2/AR, MAC from geometry.
 (a) shell/foam-like :  m = sigma*S + m_fixed            => W/S = sigma*g + m_fixed*g/S   (constant in span if m_fixed=0;
                                                           excess term ~ 1/b^2 at fixed AR)
 (b) solid/infill-like: m = rho_eff*(0.685*t/c)*S*MAC + m_fixed
                       (integral of section area 0.685 t c^2 over span equals 0.685 t S MAC, since MAC = int c^2 dy / S)
                                                        => W/S = g*rho_eff*0.685*(t/c)*MAC + m_fixed*g/S
                       with MAC ~ b/AR the first term is PROPORTIONAL to span at fixed AR.
rho_eff values are labelled assumptions (1240 = solid PLA, lower = foamed PLA / sparse infill). sigma values are
planform areal masses of a built-up/foam shell wing. Neither is a measurement.
py -B rigid_glider/pure_wing_v2_mass.py
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
from pathlib import Path

import pure_wing_model as m
from pure_wing_study import svg_chart

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/pure_wing_v2"
G = 9.80665
RHO, NU = m.RHO, m.NU
SPANS = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60]
ARS = [3, 4.5, 6]
CLS = [0.2, 0.3, 0.4, 0.5]
SIGMAS = [0.3, 0.5, 0.8, 1.2]
RHOS = [100, 250, 500, 1240]
MFIX_G = [0, 5, 10]
TCS = [0.06, 0.08, 0.10]
BANDS = [20000, 40000, 80000]
DB_RE = (10000, 160000)
SECTION_AREA_COEF = 0.685


def planform(b: float, ar: float, taper: float = 0.7) -> dict:
    S = b * b / ar
    cr = 2 * S / (b * (1 + taper))
    mac = 2 * cr * (1 + taper + taper ** 2) / (3 * (1 + taper))
    return dict(S=S, cr=cr, ct=taper * cr, mac=mac)


def mass_shell(pf, sigma, mfix_g) -> float:
    return sigma * pf["S"] + mfix_g / 1000.0


def mass_infill(pf, rho_eff, tc, mfix_g) -> float:
    return rho_eff * SECTION_AREA_COEF * tc * pf["S"] * pf["mac"] + mfix_g / 1000.0


def ws_of(mass_kg, pf) -> float:
    return mass_kg * G / pf["S"]


def band_fractions(pf, V, taper=0.7, n=1000) -> dict:
    bands = [0] + BANDS + [1e12]
    fr = [0.0] * 4
    inside = 0.0
    for i in range(n):
        eta = (i + 0.5) / n
        c = pf["cr"] + (pf["ct"] - pf["cr"]) * eta
        re = V * c / NU
        w = c / ((pf["cr"] + pf["ct"]) / 2 * n)
        fr[next(j for j in range(4) if bands[j] <= re < bands[j + 1])] += w
        inside += w * (DB_RE[0] <= re <= DB_RE[1])
    return dict(frac_lt20k=fr[0], frac_20_40k=fr[1], frac_40_80k=fr[2], frac_gt80k=fr[3], frac_in_database=inside)


def row(model, params, b, ar, cl, taper=0.7):
    pf = planform(b, ar, taper)
    if model == "shell":
        mass = mass_shell(pf, params["sigma"], params["mfix_g"])
    else:
        mass = mass_infill(pf, params["rho_eff"], params["tc"], params["mfix_g"])
    ws = ws_of(mass, pf)
    V = m.glide_speed(ws, cl)
    r = dict(model=model, **params, span_m=b, ar=ar, cl=cl, area_m2=pf["S"], mac_m=pf["mac"], mass_g=1000 * mass,
             ws_n_m2=ws, v_m_s=V, re_root=V * pf["cr"] / NU, re_mac=V * pf["mac"] / NU, re_tip=V * pf["ct"] / NU,
             re_mac_below_db_floor=int(V * pf["mac"] / NU < DB_RE[0]))
    r.update(band_fractions(pf, V, taper))
    return r


def all_rows() -> list[dict]:
    rows = []
    for b in SPANS:
        for ar in ARS:
            for cl in CLS:
                for s in SIGMAS:
                    for mf in MFIX_G:
                        rows.append(row("shell", dict(sigma=s, rho_eff=None, tc=None, mfix_g=mf), b, ar, cl))
                for rho in RHOS:
                    for tc in TCS:
                        for mf in MFIX_G:
                            rows.append(row("infill", dict(sigma=None, rho_eff=rho, tc=tc, mfix_g=mf), b, ar, cl))
    return rows


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = all_rows()
    with (OUT / "p1_mass_models_table.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})
    sel = lambda **kw: [r for r in rows if all(r[k] == v for k, v in kw.items())]
    ser = []
    for s in (0.3, 0.8, 1.2):
        for mf in (0, 10):
            ser.append(dict(label=f"sigma {s} kg/m2, m_fixed {mf} g", pts=[(r["span_m"], r["ws_n_m2"]) for r in sel(model="shell", sigma=s, mfix_g=mf, ar=4.5, cl=0.4)], marker="circle", dash=bool(mf)))
    svg_chart(OUT / "fig_p1a_ws_vs_span_shell.svg", "Shell/foam mass model: W/S vs span (AR 4.5, taper 0.7)", "span (m)", "wing loading W/S (N/m2)", ser,
              "m = sigma S + m_fixed. W/S = sigma g (constant in span) + m_fixed g/S (rises as span shrinks, ~1/b^2). EXPLORATORY.", logy=True)
    ser = []
    for rho in (100, 250, 500, 1240):
        for mf in (0, 10):
            ser.append(dict(label=f"rho_eff {rho}, m_fixed {mf} g", pts=[(r["span_m"], r["ws_n_m2"]) for r in sel(model="infill", rho_eff=rho, tc=0.08, mfix_g=mf, ar=4.5, cl=0.4)], marker="circle", dash=bool(mf)))
    svg_chart(OUT / "fig_p1b_ws_vs_span_infill.svg", "Solid/infill mass model: W/S vs span (AR 4.5, t/c 0.08)", "span (m)", "wing loading W/S (N/m2)", ser,
              "m = rho_eff 0.685 (t/c) S MAC + m_fixed. First term W/S ~ MAC ~ span (proportional); m_fixed term ~ 1/b^2. rho_eff 1240 = solid PLA.", logy=True)
    ser = []
    pick = [("shell", dict(sigma=0.5, mfix_g=0), "shell sigma 0.5, 0 g"), ("shell", dict(sigma=0.8, mfix_g=10), "shell sigma 0.8, +10 g"),
            ("shell", dict(sigma=1.2, mfix_g=10), "shell sigma 1.2, +10 g"),
            ("infill", dict(rho_eff=250, tc=0.08, mfix_g=5), "infill rho 250, +5 g"), ("infill", dict(rho_eff=500, tc=0.08, mfix_g=5), "infill rho 500, +5 g"),
            ("infill", dict(rho_eff=1240, tc=0.08, mfix_g=5), "infill rho 1240 (solid PLA), +5 g")]
    for mod, kw, lab in pick:
        ser.append(dict(label=lab, pts=[(r["span_m"], r["re_mac"]) for r in sel(model=mod, ar=4.5, cl=0.4, **kw)], marker="circle"))
    svg_chart(OUT / "fig_p1c_re_mac_vs_span_models.svg", "MAC Reynolds number vs span under the mass models (AR 4.5, CL 0.4)", "span (m)", "Re at MAC", ser,
              "Dashed: polar-database floor 10k (below = no section data), band limits 20k/40k/80k, database ceiling 160k.", logy=True,
              hlines=[(10000, "database floor 10k"), (20000, "20k"), (40000, "40k"), (80000, "80k"), (160000, "database ceiling 160k")], ylim=(3000, 400000))
    # compact summary: W/S range at span 0.40, AR 4.5
    summ = {}
    for mod, kw, lab in pick:
        r = sel(model=mod, ar=4.5, cl=0.4, span_m=0.40, **kw)[0]
        summ[lab] = dict(mass_g=r["mass_g"], ws=r["ws_n_m2"], v=r["v_m_s"], re_mac=r["re_mac"], frac_in_db=r["frac_in_database"])
    (OUT / "p1_summary_span040_ar4p5.json").write_text(json.dumps(summ, indent=1))
    print(json.dumps(summ, indent=1))


if __name__ == "__main__":
    main()
