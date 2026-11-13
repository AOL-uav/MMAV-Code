"""Part E (build numbers) and figures for the architecture comparison.  Reads arch_study outputs only; writes to
results/runs/rigid_glider/arch_compare/.   py -B rigid_glider/arch_report.py build | figs | all
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import statistics
from pathlib import Path

import arch_model as A
import arch_study as S
import pure_wing_study as st

OUT = A.OUT
B = S.B
CHOSEN = ("P07", "NACA_0006")
COL = {"pure": "#1769aa", "lip": "#c55127", "tail": "#28834f"}        # consistent architecture colours
COL_L = {"pure": "#7fb0d8", "lip": "#e8a27f", "tail": "#8cc9a5"}
LIP_CHOICES = [(0.20, "outer", 10.0), (0.15, "outer", 8.0), (0.20, "outer", 5.0)]
TAIL_CHOICES = [(0.10, 2.0, -3.0), (0.08, 1.5, -3.0)]


def geometry_sheet(models, wing):
    Mp = models[S.mkey("pure", wing)]
    cr, ct = Mp["root_chord"] * B, Mp["tip_chord"] * B
    tip_x = Mp["tip_x_le"] * B
    half = B / 2
    return dict(span_mm=1000 * B, area_cm2=1e4 * Mp["area"] * B * B, root_chord_mm=1000 * cr, tip_chord_mm=1000 * ct, mac_mm=1000 * Mp["mac"] * B,
                x_lemac_mm=1000 * Mp["x_lemac"] * B, xnp_pure_mm=1000 * Mp["xnp"] * B, le_sweep_deg=math.degrees(math.atan(tip_x / half)),
                te_sweep_deg=math.degrees(math.atan((tip_x + ct - cr) / half)), qc_sweep_deg=S.WINGS[wing][1], tip_le_x_mm=1000 * tip_x,
                tip_te_x_mm=1000 * (tip_x + ct), ar=S.WINGS[wing][0], taper=S.WINGS[wing][2])


def lip_rise_table(models, wing):
    Mp = models[S.mkey("pure", wing)]
    cr, ct = Mp["root_chord"] * B, Mp["tip_chord"] * B
    ch = lambda eta: cr + (ct - cr) * eta
    rows = []
    for cf in S.LIP_CFS:
        for ang in S.LIP_ANGLES[cf]:
            for pl, eta0 in (("outer", S.OUTER_SPLIT if hasattr(S, "OUTER_SPLIT") else 0.4), ("full", 0.0)):
                rows.append(dict(lip=f"{int(cf*100)}/{ang:g}", placement=pl, hinge_x_over_c=1 - cf, eta_inboard=eta0,
                                 lip_chord_inboard_mm=1000 * cf * ch(eta0), lip_chord_tip_mm=1000 * cf * ct,
                                 te_rise_inboard_mm=1000 * cf * ch(eta0) * math.sin(math.radians(ang)),
                                 te_rise_tip_mm=1000 * cf * ct * math.sin(math.radians(ang))))
    return rows


def tail_fin_sheet(models, wing):
    Mp = models[S.mkey("pure", wing)]
    cr = Mp["root_chord"] * B
    Sref = Mp["area"] * B * B
    tails = []
    for ta in S.TAIL_AREAS:
        for stn in S.TAIL_STATIONS:
            M = models[S.mkey("tail", wing, ta, stn)]
            g = A.tail_geometry(M, B)
            g.update(area_frac=ta, station_root_chords=stn, xnp_mm=1000 * M["xnp"] * B, xnp_shift_mm=1000 * (M["xnp"] - Mp["xnp"]) * B,
                     xnp_shift_pct_mac=100 * (M["xnp"] - Mp["xnp"]) / Mp["mac"], tail_re_at_7ms=7.0 * g["chord_mm"] / 1000 / A.NU)
            tails.append(g)
    fc = 0.5 * cr
    fins = []
    for a, x in ((0.06, 1.5), (0.04, 2.0), (0.06, 2.0), (0.04, 1.5)):
        h = a * Sref / fc
        fins.append(dict(area_frac=a, area_cm2=1e4 * a * Sref, chord_mm=1000 * fc, height_mm=1000 * h, x_qc_mm=1000 * x * cr, x_qc_root_chords=x))
    return tails, fins


def stage_build():
    models, recs, rows = S.load_windows_rows()
    recidx = S.rec_index(recs)
    ridx = S.row_index(rows)
    tabs = A.ArchTables()
    wing, sec = CHOSEN
    out = dict(chosen=CHOSEN, geometry=geometry_sheet(models, wing))
    out["lip_rise"] = lip_rise_table(models, wing)
    out["tails"], out["fins"] = tail_fin_sheet(models, wing)
    noms = []
    cfgs = [("pure", None, 0.0)] + [("lip", (cf, pl), ang) for cf, pl, ang in LIP_CHOICES] + [("tail", (ta, stn), inc) for ta, stn, inc in TAIL_CHOICES]
    for ws in (6, 12, 24):
        for kind, cfg, u in cfgs:
            noms.append(S.nominal(models, recidx, tabs, ridx, kind, wing, sec, cfg, u, ws))
    out["nominal"] = noms
    # CG envelope (windows N5/N9 any-evidence, panels 160, + AVL-only) over all W/S and the rail that covers every architecture
    env = {}
    for kind, cfg, u in cfgs:
        lo = hi = None
        for ws in S.WS_LIST:
            for tier, form, pan in (("N5", "coupled", 160), ("N9", "coupled", 160), ("avl", "avl_only", None)):
                r = ridx.get((kind, wing, sec, cfg[0] if cfg else None, cfg[1] if cfg else None, u, ws, form, pan, tier, "any"))
                if r and r["n_points"]:
                    lo = r["cg_mm_lo"] if lo is None else min(lo, r["cg_mm_lo"])
                    hi = r["cg_mm_hi"] if hi is None else max(hi, r["cg_mm_hi"])
        name = kind if not cfg else f"{kind} {cfg}/{u:g}"
        env[name] = (lo, hi)
    out["cg_envelope_mm"] = env
    allv = [v for v in env.values() if v[0] is not None]
    out["cg_rail_all_architectures_mm"] = (min(v[0] for v in allv), max(v[1] for v in allv))
    (OUT / "E_build_numbers.json").write_text(json.dumps(out, indent=1, default=str))
    write_build_text(out, models)
    print("build numbers written")
    return out


def write_build_text(o, models):
    g = o["geometry"]
    L = [f"E. BUILD NUMBERS  ({o['chosen'][0]} {o['chosen'][1]}, b = 400 mm)", "",
         f"Wing: AR {g['ar']}, taper {g['taper']}, area {g['area_cm2']:.1f} cm2, root chord {g['root_chord_mm']:.1f} mm, tip chord {g['tip_chord_mm']:.1f} mm, MAC {g['mac_mm']:.1f} mm "
         f"(LE of MAC at x = {g['x_lemac_mm']:.1f} mm), c/4 sweep {g['qc_sweep_deg']} deg, LE sweep {g['le_sweep_deg']:.1f} deg, TE sweep {g['te_sweep_deg']:.1f} deg, "
         f"tip LE x = {g['tip_le_x_mm']:.1f} mm, tip TE x = {g['tip_te_x_mm']:.1f} mm.  Pure-wing neutral point {g['xnp_pure_mm']:.1f} mm from root LE.", ""]
    L.append("CG envelope over the tested W/S (6..24), N5/N9 coupled windows and AVL-only windows, mm aft of root LE:")
    for k, v in o["cg_envelope_mm"].items():
        L.append(f"  {k}: {v[0]:.1f} .. {v[1]:.1f}" if v[0] is not None else f"  {k}: none")
    L.append(f"=> one CG rail covering all architectures: {o['cg_rail_all_architectures_mm'][0]:.1f} .. {o['cg_rail_all_architectures_mm'][1]:.1f} mm (add +-3 mm margin)")
    L += ["", "Trailing-edge lip: hinge at 1 - cf of the local chord; outboard placement = eta >= 0.4 semispan (split at y = 80 mm); TE rise = cf*c*sin(angle):"]
    for r in o["lip_rise"]:
        L.append(f"  {r['lip']:>5} {r['placement']:<5} hinge {100*r['hinge_x_over_c']:.0f}% c  lip chord {r['lip_chord_inboard_mm']:.1f} mm (inboard) / {r['lip_chord_tip_mm']:.1f} mm (tip)  TE rise {r['te_rise_inboard_mm']:.1f} / {r['te_rise_tip_mm']:.1f} mm")
    L += ["", "Tail (horizontal, span 35% b = 140 mm, thin symmetric section, inviscid):"]
    for t in o["tails"]:
        L.append(f"  area {100*t['area_frac']:.0f}% = {t['area_cm2']:.1f} cm2  chord {t['chord_mm']:.1f} mm  quarter chord at {t['station_root_chords']:g} root chords = {t['x_qc_mm']:.0f} mm; "
                 f"tail arm from the (with-tail) NP {t['arm_from_np_mm']:.0f} mm; NP shift {t['xnp_shift_mm']:+.1f} mm ({t['xnp_shift_pct_mac']:+.1f}% MAC); Re of tail at 7 m/s {t['tail_re_at_7ms']/1000:.1f}k")
    L += ["", "Fins (vertical, chord = 0.5 root chord, above the wing plane):"]
    for f in o["fins"]:
        L.append(f"  {100*f['area_frac']:.0f}% of wing area = {f['area_cm2']:.1f} cm2: chord {f['chord_mm']:.1f} mm x height {f['height_mm']:.1f} mm, quarter chord at {f['x_qc_mm']:.0f} mm ({f['x_qc_root_chords']:g} root chords)")
    L += ["", "Predicted nominal trim (centre of the both-Ncrit coupled window, panels 160, any evidence):"]
    for n in o["nominal"]:
        name = n["kind"] if not n["cfg"] else f"{n['kind']} {n['cfg']} {n['u']:g}"
        head = f" W/S {n['ws']:>2} ({n['mass_g']:.0f} g) {name}: windows mm AVL-only {n['win_avl_mm']:.1f} N5 {n['win_N5_any_mm']:.1f} N9 {n['win_N9_any_mm']:.1f} both {n['win_both_any_mm']:.1f}"
        if n.get("sm_nominal") is None:
            L.append(head + " ; NO WINDOW")
            continue
        L.append(head + f" ; first CG {n['cg_mm']:.1f} mm ({n['cg_pct_mac']:.1f}% MAC, SM {100*n['sm_nominal']:+.1f}%)")
        for nc in ("N5", "N9"):
            d = n.get(nc)
            if d:
                mm10 = 0.1 / d["cl_per_mm"] if d.get("cl_per_mm") else None
                L.append(f"      {nc}: CL {d['cl']:.2f} alpha {d['alpha_deg']:.1f} deg V {d['v']:.1f} m/s Re {d['re_root']/1000:.0f}-{d['re_tip']/1000:.0f}k ; "
                         f"dCL/mm {d['cl_per_mm']:.3f}" + (f" (0.1 in CL = {mm10:.1f} mm)" if mm10 else ""))
            else:
                L.append(f"      {nc}: no valid trim at this CG")
    L += ["", "Moving the CG: trim CL = (nose-up moment)/(static margin). CG AFT raises trim CL (slower, higher alpha, stall side); CG FORWARD lowers trim CL (faster, dive side).",
          "If it dives / flies fast: move the CG aft in steps of ~ 0.1/(dCL/mm) mm (listed above, 3-7 mm per 0.1 of CL); if it pitches up / stalls: move it forward by the same."]
    (OUT / "E_build_numbers.txt").write_text("\n".join(L), encoding="utf-8")


# ------------------------------------------------------------------ figures
def svg_bars(path, title, panels, caption, w=980, h=640):
    """panels: list of (label, unit, [(name, value, colour)]) -> small-multiple bar charts."""
    from html import escape
    cols = 3
    rows = math.ceil(len(panels) / cols)
    pw, ph = (w - 40) / cols, (h - 110) / rows
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" font-family="Arial">', '<rect width="100%" height="100%" fill="white"/>',
         f'<text x="{w/2}" y="24" text-anchor="middle" font-size="17">{escape(title)}</text>']
    for i, (label, unit, bars) in enumerate(panels):
        x0, y0 = 20 + (i % cols) * pw, 44 + (i // cols) * ph
        vmax = max([abs(v) for _, v, _ in bars if v is not None] + [1e-9])
        o.append(f'<text x="{x0+pw/2}" y="{y0+14}" text-anchor="middle" font-size="13" font-weight="bold">{escape(label)}</text>')
        o.append(f'<text x="{x0+pw/2}" y="{y0+28}" text-anchor="middle" font-size="10" fill="#555">{escape(unit)}</text>')
        base = y0 + ph - 34
        o.append(f'<line x1="{x0+18}" y1="{base}" x2="{x0+pw-18}" y2="{base}" stroke="#222"/>')
        bw = (pw - 60) / len(bars)
        for j, (nm, v, c) in enumerate(bars):
            bx = x0 + 30 + j * bw
            if v is None:
                o.append(f'<text x="{bx+bw/2-6}" y="{base-6}" text-anchor="middle" font-size="10" fill="#777">n/a</text>')
            else:
                hh = (ph - 90) * abs(v) / vmax
                o.append(f'<rect x="{bx:.1f}" y="{base-hh:.1f}" width="{bw-14:.1f}" height="{hh:.1f}" fill="{c}"/>')
                o.append(f'<text x="{bx+(bw-14)/2:.1f}" y="{base-hh-4:.1f}" text-anchor="middle" font-size="10">{v:.3g}</text>')
            o.append(f'<text x="{bx+(bw-14)/2:.1f}" y="{base+13}" text-anchor="middle" font-size="10">{escape(nm)}</text>')
    for j, ln in enumerate([caption[k:k + 150] for k in range(0, len(caption), 150)][:3]):
        o.append(f'<text x="24" y="{h-36+j*13}" font-size="11" fill="#333">{escape(ln)}</text>')
    o.append("</svg>")
    path.write_text("\n".join(o), encoding="utf-8")


def stage_figs():
    models, recs, rows = S.load_windows_rows()
    recidx = S.rec_index(recs)
    ridx = S.row_index(rows)
    tabs = A.ArchTables()
    wing, sec = CHOSEN
    nom = json.loads((OUT / "E_build_numbers.json").read_text())
    ws0 = 12
    lipc, tailc = (0.20, "outer", 10.0), (0.10, 2.0, -3.0)
    cfgs = {"pure": (None, 0.0), "lip": ((lipc[0], lipc[1]), lipc[2]), "tail": ((tailc[0], tailc[1]), tailc[2])}
    label = {"pure": "pure", "lip": "lip 20% outer 10 deg", "tail": "tail 10% @2.0cr inc -3"}
    # ---- Fig 1: trim CL vs CG
    ser = []
    for kind, (cfg, u) in cfgs.items():
        M = S.cfg_models(models, kind, wing, cfg)
        mac_mm, xnp_mm = M["mac"] * B * 1000, M["xnp"] * B * 1000
        sms = [0.005 * k for k in range(1, 41)]
        if kind == "pure":
            ser.append(dict(label="pure AVL-only (CL=0)", pts=[(xnp_mm - sm * mac_mm, 0.0) for sm in (0.0, 0.1, 0.2)], color=COL[kind], dash=True))
        else:
            cm0u = M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"]
            ser.append(dict(label=f"{kind} AVL-only", pts=[(xnp_mm - sm * mac_mm, u * cm0u / sm) for sm in sms if 0 < u * cm0u / sm <= 1.2], color=COL[kind], dash=True))
        for nc, mk, colr in ((5, "circle", COL[kind]), (9, "x", COL_L[kind])):
            rec = recidx[(kind, wing, sec, json.dumps(cfg), u, ws0, "coupled", nc, 160)]
            ser.append(dict(label=f"{kind} coupled Ncrit {nc} (stable, CL 0.2-0.5, cov>=0.9)", pts=[(xnp_mm - p[0] * mac_mm, p[1]) for p in rec["pts"]], color=colr, marker=mk, line=False))
    st.svg_chart(OUT / "fig1_trim_cl_vs_cg.svg", f"Trim CL vs CG on {wing} {sec}, W/S {ws0} N/m2 (b=0.40 m)", "CG aft of root leading edge (mm)", "trim CL", ser,
                 "Dashed: AVL-only. Markers: coupled Level 1 stable roots, Ncrit 5 (dark circles) and Ncrit 9 (light x) = the Ncrit band. Tail section is inviscid (Re ~10k, below the polar floor).", ylim=(0, 0.7))
    # ---- Fig 2: window by architecture and W/S
    ser = []
    for kind, (cfg, u) in cfgs.items():
        for tier, form, pan, mode, dash, nm in (("both", "coupled", 160, "any", False, "coupled both Ncrit"), ("both", "coupled", 160, "offti", True, "coupled both, off-TI evidence only")):
            pts = []
            for ws in S.WS_LIST:
                r = ridx.get((kind, wing, sec, cfg[0] if cfg else None, cfg[1] if cfg else None, u, ws, form, pan, tier, mode))
                pts.append((ws, r["width_mm"] if r and r["n_points"] else 0.0))
            ser.append(dict(label=f"{kind}: {nm}", pts=pts, color=COL[kind], dash=dash, marker="circle"))
        if kind != "pure":
            pts = []
            for ws in S.WS_LIST:
                r = ridx.get((kind, wing, sec, cfg[0], cfg[1], u, ws, "avl_only", None, "avl", "any"))
                pts.append((ws, r["width_mm"] if r and r["n_points"] else 0.0))
            ser.append(dict(label=f"{kind}: AVL-only", pts=pts, color=COL_L[kind], marker="x"))
    st.svg_chart(OUT / "fig2_cg_window_by_architecture.svg", f"CG window (trim CL 0.2-0.5) vs W/S, {wing} {sec}", "wing loading W/S (N/m2)", "stable-trim CG window (mm)", ser,
                 "Width = (n-1)*0.5% MAC of the widest contiguous run. Threshold in this study ~3 mm (exploratory). Dotted-dash = off-transition-interval evidence only; AVL-only ignores section viscosity.")
    # ---- Fig 3: lip moment retained vs Re
    lm = list(csv.DictReader((A.ROOT / "results/runs/rigid_glider/reflex_small/polars/lip_moment_table.csv").open(encoding="utf-8")))
    ser = []
    pal = ["#c55127", "#a33b17", "#e08a5e", "#793da4", "#ad850d", "#555555"]
    for i, (cf, ang) in enumerate(S.LIP_ORDER):
        pts = []
        for re in (15000, 20000, 30000, 40000, 60000, 80000, 100000):
            v = [float(r["fraction_of_inviscid"]) for r in lm if r["lip"] == f"{int(round(cf*100))}/{ang:g}" and r["cl"] == "0.25" and r["ncrit"] == "5" and r["panels"] == "160"
                 and int(r["re"]) == re and r["fraction_of_inviscid"] not in ("", None)]
            if v:
                pts.append((re / 1000, statistics.mean(v)))
        ser.append(dict(label=f"{int(round(cf*100))}/{ang:g}", pts=pts, color=pal[i], marker="circle"))
    Mp = models[S.mkey("pure", wing)]
    cr, ct = Mp["root_chord"] * B, Mp["tip_chord"] * B
    ch04 = cr + (ct - cr) * 0.4
    vlo, vhi = (2 * 6 / (1.225 * 0.3)) ** 0.5, (2 * 24 / (1.225 * 0.3)) ** 0.5
    lo, hi = vlo * ct / A.NU / 1000, vhi * ch04 / A.NU / 1000
    ser.append(dict(label=f"article lip-strip Re band {lo:.0f}k-{hi:.0f}k (W/S 6-24, CL 0.3)", pts=[(lo, 0.0), (lo, 0.65), (hi, 0.65), (hi, 0.0)], color="#999999", dash=True, marker=None))
    st.svg_chart(OUT / "fig3_lip_moment_retained_vs_re.svg", "Fraction of inviscid lip moment retained vs Re (Cl 0.25, Ncrit 5, 160 panels; mean of 0006/0008)", "Re (thousands)", "viscous / inviscid lip moment", ser,
                 "From reflex_small lip_moment_table.csv. Dashed box = Re range of the outboard lip strips of the chosen wing across W/S 6-24 (tip chord at W/S 6 to lip root chord at W/S 24). Retained fraction collapses below ~40k.", ylim=(0, 0.7))
    # ---- Fig 4: Cm vs CL at one CG each (W/S 12)
    ser = []
    for kind, (cfg, u) in cfgs.items():
        n = [x for x in nom["nominal"] if x["kind"] == kind and x["ws"] == ws0 and (kind == "pure" or (tuple(x["cfg"]) == cfg and x["u"] == u))][0]
        sm = n.get("sm_nominal")
        if sm is None:
            continue
        M = S.cfg_models(models, kind, wing, cfg)
        uu = u if kind != "pure" else 0.0
        cls = [c for c in A.GRID if c <= 0.8]
        ser.append(dict(label=f"{kind} AVL-only (CG {n['cg_mm']:.0f} mm)", pts=[(c, A.cm_np_avl(M, uu, c) - sm * c) for c in cls], color=COL[kind], dash=True))
        for nc, colr in ((5, COL[kind]), (9, COL_L[kind])):
            clean = tabs.clean(sec, nc, 160)
            cfgx = A.Cfg(clean, tabs.bent(sec, cfg[0], u, nc, 160), tabs.inv_curve(sec, cfg[0], u, 160)) if kind == "lip" else A.Cfg(clean)
            pts = []
            for c in cls:
                r = A.cm_extra_arch(M, B, uu, cfgx, c, ws=ws0, form="coupled")
                if r["coverage"] >= 0.9:
                    pts.append((c, r["cm_extra"] - sm * c))
            ser.append(dict(label=f"{kind} coupled Ncrit {nc}", pts=pts, color=colr))
    st.svg_chart(OUT / "fig4_cm_vs_cl_architectures.svg", f"Cm vs CL at the nominal CG of each architecture, {wing} {sec}, W/S {ws0}", "CL (glide line: V and Re vary with CL)", "total Cm about CG", ser,
                 "Zero crossings with negative slope = stable trims. Each architecture at its own nominal CG (centre of its both-Ncrit window). Points with coverage < 0.9 omitted.", ylim=(-0.06, 0.06))
    # ---- Fig 5: lateral
    lat = list(csv.DictReader((OUT / "lateral.csv").open(encoding="utf-8")))
    fl = lambda x: float(x) if x not in ("", None) else None
    opts = [("pure", "none", "0"), ("pure", "fin6@1.5cr", "0"), ("lip", "none", "0"), ("lip", "fin6@1.5cr", "0"), ("tail", "none", "0"), ("tail", "boomfin4@tail", "0"), ("tail", "boomfin6@tail", "0"),
            ("pure", "none", "3"), ("pure", "fin6@1.5cr", "3"), ("lip", "fin6@1.5cr", "3"), ("tail", "boomfin6@tail", "3")]
    for key, nm, yl in (("cn_beta_CL0.3", "Cn_beta", "Cn_beta (per rad)"), ("cl_beta_CL0.3", "Cl_beta", "Cl_beta (per rad)")):
        ser = []
        for i, (ar_, fin, dih) in enumerate(opts):
            r = [x for x in lat if x["wing"] == wing and x["arch"] == ar_ and x["fin"] == fin and x["dihedral"] == dih][0]
            ser.append(dict(label=f"{i+1}: {ar_} dih{dih} {fin}", pts=[(i + 1, fl(r[key]))], color=COL[ar_], marker="circle" if dih == "0" else "x", line=False))
        st.svg_chart(OUT / f"fig5_{nm}_by_architecture.svg", f"{nm} at CL 0.3 by architecture, dihedral and fin ({wing})", "option number (see legend)", yl, ser,
                     "AVL, stability axes, CG at SM +5%. Circles dihedral 0, crosses dihedral 3 deg. Exploratory Cn_beta target 0.01-0.03 /rad." if nm == "Cn_beta" else "AVL, stability axes, CL 0.3, CG at SM +5%. Circles dihedral 0, crosses dihedral 3 deg.",
                     hlines=[(0.01, "0.01"), (0.03, "0.03")] if nm == "Cn_beta" else [])
    # ---- Fig 6: comparison chart (W/S 12 + 16 averaged where defined)
    panels = []
    sel = {k: [x for x in nom["nominal"] if x["kind"] == k and x["ws"] == ws0 and (k == "pure" or (tuple(x["cfg"]) == cfgs[k][0] and x["u"] == cfgs[k][1]))][0] for k in cfgs}
    colr = lambda k: COL[k]
    mk = lambda f: [(k, f(sel[k]), colr(k)) for k in cfgs]
    panels.append(("CG window, AVL-only", "mm", mk(lambda n: n["win_avl_mm"])))
    panels.append(("CG window, coupled both Ncrit", "mm (any evidence)", mk(lambda n: n["win_both_any_mm"])))
    panels.append(("CG window, strict (off-TI) both", "mm", mk(lambda n: n["win_both_offti_mm"])))
    panels.append(("Trim sensitivity to CG", "|dCL| per mm (N5)", mk(lambda n: (n.get("N5") or {}).get("cl_per_mm"))))
    def lat_v(k, key):
        arch = k
        fin = {"pure": "fin6@1.5cr", "lip": "fin6@1.5cr", "tail": "boomfin6@tail"}[k]
        return fl([x for x in lat if x["wing"] == wing and x["arch"] == arch and x["fin"] == fin and x["dihedral"] == "0"][0][key])
    panels.append(("Cn_beta with std fin", "per rad (CL 0.3)", [(k, lat_v(k, "cn_beta_CL0.3"), colr(k)) for k in cfgs]))
    panels.append(("Roll per deg wing-incidence mismatch", "|Cl'| per deg", [(k, abs(lat_v(k, "roll_per_deg_incidence_mismatch")), colr(k)) for k in cfgs]))
    svg_bars(OUT / "fig6_architecture_comparison.svg", f"Architecture comparison, {wing} {sec}, W/S {ws0} (lip 20% outer 10 deg; tail 10% @2.0cr inc -3)", panels,
             "Bars from table D / lateral.csv. Pure = no moment feature; AVL-only window for pure is 0 (trims at CL=0). Strict window uses only strips off any transition interval. No winner is declared by this chart.")
    print("figures written")


def _pair_sens(recidx, kind, wing, sec, cfg, ws, nc, u1, u2):
    r1 = recidx.get((kind, wing, sec, json.dumps(cfg), u1, ws, "coupled", nc, 160))
    r2 = recidx.get((kind, wing, sec, json.dumps(cfg), u2, ws, "coupled", nc, 160))
    if not r1 or not r2:
        return []
    return [(p[1] - q[1]) / (u1 - u2) for p in r1["pts"] for q in r2["pts"] if abs(p[0] - q[0]) < 1e-9]


def stage_extras():
    models, recs, rows = S.load_windows_rows()
    recidx = S.rec_index(recs)
    ridx = S.row_index(rows)
    tabs = A.ArchTables()
    L = []
    # ---- sensitivity per degree of lip angle / tail incidence (medians over wings, sections, common CGs)
    sens = []
    for cf in S.LIP_CFS:
        a1, a2 = S.LIP_ANGLES[cf]
        for pl in S.PLACEMENTS:
            for ws in S.WS_LIST:
                for nc in S.NCRITS:
                    v = [x for wing in S.WINGS for sec in S.SECTIONS for x in _pair_sens(recidx, "lip", wing, sec, [cf, pl], ws, nc, a2, a1)]
                    avl = statistics.median(
                        [(M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"]) / 0.10 for M in (S.load_models()[S.mkey("lip", w_, cf, pl)] for w_ in S.WINGS)])
                    sens.append(dict(kind="lip", chord=cf, placement=pl, angles=f"{a1:g}->{a2:g}", ws=ws, ncrit=nc, n=len(v),
                                     coupled_dCL_per_deg_median=statistics.median(v) if v else None, avl_only_dCL_per_deg_at_SM0p10=avl))
    for ta in S.TAIL_AREAS:
        for stn in S.TAIL_STATIONS:
            for ws in S.WS_LIST:
                for nc in S.NCRITS:
                    v = [x for wing in S.WINGS for sec in S.SECTIONS for x in _pair_sens(recidx, "tail", wing, sec, [ta, stn], ws, nc, -4.0, -2.0)]
                    sens.append(dict(kind="tail", chord=ta, placement=stn, angles="-2->-4", ws=ws, ncrit=nc, n=len(v),
                                     coupled_dCL_per_deg_median=statistics.median(v) if v else None, avl_only_dCL_per_deg_at_SM0p10=None))
    S._write_csv(OUT / "sensitivity_per_degree.csv", sens)
    # ---- lift cost: alpha for the same CL, lip vs clean (AVL part, and total with viscous correction at CL 0.3, W/S 12)
    lc = []
    wing, sec = CHOSEN
    for cf in S.LIP_CFS:
        for pl in S.PLACEMENTS:
            M = models[S.mkey("lip", wing, cf, pl)]
            for ang in S.LIP_ANGLES[cf]:
                row = dict(wing=wing, lip=f"{int(cf*100)}/{ang:g}", placement=pl, d_cl_per_deg=M["d_cl"],
                           alpha_cost_avl_deg_at_CL0p3=-M["d_cl"] * ang / M["a_cl"], cl_at_zero_alpha=M["d_cl"] * ang)
                for nc in S.NCRITS:
                    clean = tabs.clean(sec, nc, 160)
                    cfgx = A.Cfg(clean, tabs.bent(sec, cf, ang, nc, 160), tabs.inv_curve(sec, cf, ang, 160))
                    a_l = A.trim_state(M, S.B, ang, cfgx, 0.3, 12)
                    a_c = A.trim_state(models[S.mkey("pure", wing)], S.B, 0.0, A.Cfg(clean), 0.3, 12)
                    row[f"alpha_cost_total_deg_N{nc}"] = a_l["alpha_deg"] - a_c["alpha_deg"]
                    row[f"coverage_N{nc}"] = a_l["coverage"]
                lc.append(row)
    S._write_csv(OUT / "lip_lift_cost.csv", lc)
    # ---- overall table D (median over the 3 wings of the best-config nominal points)
    nom = json.loads((OUT / "nominal_points.json").read_text())
    ov = []
    for ws in (8, 16):
        for kind in ("pure", "lip", "tail"):
            ns = [n for n in nom if n["kind"] == kind and n["ws"] == ws]
            g = lambda key: statistics.median([n[key] for n in ns]) if all(key in n for n in ns) else None
            row = dict(ws=ws, kind=kind, mass_g_median=g("mass_g"), win_avl_mm=g("win_avl_mm"), win_N5_mm=g("win_N5_any_mm"), win_N9_mm=g("win_N9_any_mm"),
                       win_both_mm=g("win_both_any_mm"), win_both_offti_mm=g("win_both_offti_mm"), win_both_clean_mm=g("win_both_clean_mm"),
                       win240_both_mm=g("win240_both_any_mm"))
            for nc in ("N5", "N9"):
                vals = [n[nc] for n in ns if n.get(nc)]
                if vals:
                    row[f"{nc}_cl"] = statistics.median(v["cl"] for v in vals); row[f"{nc}_v"] = statistics.median(v["v"] for v in vals)
                    row[f"{nc}_ld"] = statistics.median(v["ld"] for v in vals if v["ld"]); row[f"{nc}_cl_per_mm"] = statistics.median(v["cl_per_mm"] for v in vals if v["cl_per_mm"])
                    row[f"{nc}_margin_glide"] = statistics.median(v["margin_glide"] for v in vals)
                    row[f"{nc}_re_lo"] = min(v["re_tip"] for v in vals); row[f"{nc}_re_hi"] = max(v["re_root"] for v in vals)
            ov.append(row)
    S._write_csv(OUT / "table_D_overall.csv", ov)
    # ---- lateral summary for the chosen wing
    lat = list(csv.DictReader((OUT / "lateral.csv").open(encoding="utf-8")))
    S._write_csv(OUT / "lateral_chosen_wing.csv", [r for r in lat if r["wing"] == wing])
    print("extras written")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    fn = {"build": stage_build, "extras": stage_extras, "figs": stage_figs}
    for s in (list(fn) if what == "all" else [what]):
        fn[s]()


if __name__ == "__main__":
    main()
