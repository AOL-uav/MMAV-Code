"""Diagnosis for the swift-like outline (BS) of bird_outline_v2: why a broad inner wing and a thin outer wing, and which criterion binds.
Read-mostly: imports bird_outline_v2 / bird_outline (never edits them, never re-runs the v2 pipeline) and writes ONLY to
results/runs/rigid_glider/bird_outline_v4_swift/diagnosis/ (a few dozen serial AVL runs at the v2 panelling).
    py -B rigid_glider/bird_outline_v4_diag.py [--out DIR] [--no-panel]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import math
from pathlib import Path

import bird_outline as bo
import bird_outline_v2 as v2
import flat_test as ft

ROOT = ft.ROOT
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/bird_outline_v4_swift/diagnosis"
SPAN = 250.0
FINAL_ON = {"curved_te": 1, "root_blend": 1, "tip_round": 1, "neck": 1, "nose": 1}          # the v2 BS final
BEND = 5.0
SM_TARGETS = (0.08, 0.10)
FULL_LAW = ((0.0, 1.0), (0.5, 0.92), (0.8, 0.75), (0.95, 0.5), (1.0, 0.2))                    # a fuller hand-chord law (the BC-like rounded hand)
VARIANTS = [   # name, description, params update, feature set (None = final features)
    ("A_v2_final", "v2 BS final outline (reference)", {}, None),
    ("B_v1_outline", "v1 outline: all v2 features off", {}, {}),
    ("C_no_arm_taper", "arm chord constant (wrist chord = root chord)", dict(cw_over_croot=1.0), None),
    ("D_narrow_root", "narrower inner chord: wrist chord 1.15 x root chord", dict(cw_over_croot=1.15), None),
    ("E_fuller_hand", "fuller hand chord law, tip chord 20 % of wrist chord", dict(hand_stations=FULL_LAW), None),
    ("F_hand_sweep30", "hand sweep 30 deg instead of 50", dict(hand_sweep_deg=30.0), None),
    ("G_short_arm", "arm/hand split at 25 % of the semi-span instead of 40 %", dict(wrist_frac=0.25), None),
    ("H_even", "even planform: constant arm chord + fuller hand law + hand sweep 30", dict(cw_over_croot=1.0, hand_stations=FULL_LAW, hand_sweep_deg=30.0), None),
    ("I_even_wide_hand", "even planform with a wider hand wing: root 0.9 x, wrist chord 1.0 x root, fuller law, sweep 30, short arm 0.3", dict(cw_over_croot=1.0, hand_stations=((0.0, 1.0), (0.5, 0.98), (0.8, 0.9), (0.95, 0.7), (1.0, 0.3)), hand_sweep_deg=30.0, wrist_frac=0.3), None),
]


def chord_at(g, y):
    ys, le, te = g["y_nodes"], g["le_nodes"], g["te_nodes"]
    y = min(max(y, 0.0), ys[-1])
    for i in range(len(ys) - 1):
        if ys[i] <= y <= ys[i + 1]:
            t = (y - ys[i]) / (ys[i + 1] - ys[i])
            return (te[i] + t * (te[i + 1] - te[i])) - (le[i] + t * (le[i + 1] - le[i]))
    return 0.0


def area_between(g, y0, y1, n=2000):
    h = (y1 - y0) / n
    return 2 * sum(chord_at(g, y0 + (i + 0.5) * h) for i in range(n)) * h


def planform_numbers(g):
    k = SPAN
    half, yw = g["half_span"], g["y_wrist"]
    mean = g["S"] / g["b"] * k
    pts = {"root (centreline)": 0.0, "arm/hand junction (wrist)": yw, "mid-hand": yw + 0.5 * (half - yw), "near tip (95 % of the hand)": yw + 0.95 * (half - yw)}
    rows = []
    for nm, y in pts.items():
        c = chord_at(g, y) * k
        rows.append((nm, y * k, c, c / mean))
    S = g["S"]
    inner = area_between(g, 0, yw)
    outer = area_between(g, yw, half)
    extra = g["sheet_area"] - S - g["tail_area"]
    return dict(rows=rows, mean_mm=mean, inner_frac=inner / S, outer_frac=outer / S, S_mm2=S * k * k, tail_mm2=g["tail_area"] * k * k, sheet_mm2=g["sheet_area"] * k * k,
                extra_mm2=extra * k * k, split=yw / half, hand_sweep=g["p"]["hand_sweep_deg"], arm_sweep=g["p"]["arm_sweep_deg"], inner_area_mm2=inner * k * k, outer_area_mm2=outer * k * k,
                tail_over_S=g["tail_area"] / S)


def evaluate(name, desc, upd, feats, workdir: Path, v1_tip):
    on = FINAL_ON if feats is None else feats
    p = v2.feature_params("BS", on)
    p.update(upd)
    g = v2.build_geometry2(p)
    r0 = v2.run_case2(g, SPAN, 0.0, workdir / name / "b0", name + "b0")
    r5 = v2.run_case2(g, SPAN, BEND, workdir / name / "b5", name + "b5")
    mac = g["mac"]
    cm0pd = r5["cm0"] / BEND
    out = dict(name=name, desc=desc, g=g, plan=planform_numbers(g), np_pct=r5["np_pct_mac"], cm0_per_deg=cm0pd, cn_beta=r5["cn_beta"], cl_beta=r5["cl_beta"], tip_loading=r5["tip_loading"],
               tip_loading_b0=r0["tip_loading"], span_eff=r5["span_eff"], sm_at_5deg=r5["cm0"] / 0.3, np_span=r5["np_span"], mac=mac)
    for s in SM_TARGETS:
        cg_span = r5["np_span"] - s * mac
        res = {"trim": {c: dict(x_cg_span=cg_span, static_margin=s) for c in v2.TRIM_CLS}}
        for sk in ("printed", "foam"):
            n = v2.size_numbers(g, res, sk, BEND)
            out[f"ballast_frac_{sk}_sm{int(s * 100)}"] = n["ballast_frac"]
            out[f"ballast_g_{sk}_sm{int(s * 100)}"] = n["ballast_g"]
            out[f"M_{sk}_sm{int(s * 100)}"] = n["M"]
        out[f"bend_for_sm{int(s * 100)}"] = s * 0.3 / cm0pd
        out[f"cg_mm_sm{int(s * 100)}"] = cg_span * SPAN
        out[f"bare_cg_mm_{int(s * 100)}"] = n["cg0"]
    out["bare_cg_printed_mm"] = v2.size_numbers(g, {"trim": {c: dict(x_cg_span=r5["np_span"] - 0.10 * mac, static_margin=0.1) for c in v2.TRIM_CLS}}, "printed", BEND)["cg0"]
    out["bare_cg_pct_mac_aft_of_np"] = 100 * (out["bare_cg_printed_mm"] / SPAN - r5["np_span"]) / mac
    out["sheet_mass_printed_g"] = v2.sheet_mass_g(g, "printed")
    out["tip_chord_95_mm"] = out["plan"]["rows"][3][2]
    out["x_h_mm"] = g["x_h"] * SPAN
    out["tail_lever_mm"] = (g["tail_centroid_x"] - g["x_ref"]) * SPAN
    fails = []
    if out["bend_for_sm8"] > 10.0:
        fails.append(f"needs {out['bend_for_sm8']:.0f} deg bend for an 8% margin")
    for sk in ("printed", "foam"):
        fr = out[f"ballast_frac_{sk}_sm8"]
        if not (fr < v2.BALLAST_MAX_FRAC):
            fails.append("ballast " + ("unreachable" if not math.isfinite(fr) else f"{100 * fr:.0f}%") + f" ({sk})")
    if r5["cn_beta"] < v2.CN_BETA_MIN:
        fails.append(f"Cn_beta {r5['cn_beta']:+.4f}")
    if v1_tip and r5["tip_loading"] > v1_tip * (1 + v2.TIP_TOL):
        fails.append("tip loading above v1")
    out["fails"] = fails
    out["tip_ok"] = r5["tip_loading"] <= v1_tip * (1 + v2.TIP_TOL) if v1_tip else None
    return out


def panel_noise(workdir: Path, name, upd, feats):
    on = FINAL_ON if feats is None else feats
    p = v2.feature_params("BS", on)
    p.update(upd)
    g = v2.build_geometry2(p)
    keep = (v2.AVL_NCHORD, v2.AVL_NSPAN)
    rows = []
    try:
        for nc, ns in ((8, 64), (10, 80), (16, 128)):
            v2.AVL_NCHORD, v2.AVL_NSPAN = nc, ns
            r = v2.run_case2(g, SPAN, BEND, workdir / f"panel_{name}_{nc}", f"p{nc}")
            rows.append((name, nc, ns, r["np_pct_mac"], r["cm0"], r["cn_beta"], r["tip_loading"]))
    finally:
        v2.AVL_NCHORD, v2.AVL_NSPAN = keep
    return rows


def write_csv(path: Path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def run(out: Path = OUT_DEFAULT, panel=True, log=print):
    out.mkdir(parents=True, exist_ok=True)
    work = out / "avl"
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    # v1 tip-loading reference at the same bend (the v2 criterion compares with the v1 outline at the same bend)
    v1p = v2.feature_params("BS", {})
    v1g = v2.build_geometry2(v1p)
    v1r = v2.run_case2(v1g, SPAN, BEND, work / "v1ref", "v1ref")
    v1_tip = v1r["tip_loading"]
    results = []
    for nm, desc, upd, feats in VARIANTS:
        log("  " + nm)
        results.append(evaluate(nm, desc, upd, feats, work, v1_tip))
    base = results[0]
    hdr = ["variant", "description", "root chord mm", "wrist chord mm", "mid-hand chord mm", "near-tip chord mm (95% of hand)", "mean chord mm", "inner wing area %", "outer wing area %", "NP % MAC", "CM0 per deg of bend",
           "bend for SM 8% deg", "bend for SM 10% deg", "SM at 5 deg bend %", "CG for SM 10% mm", "bare CG mm (SM 10% set-up)", "bare CG aft of NP % MAC", "ballast % printed SM8", "ballast % foam SM8", "ballast % printed SM10",
           "ballast % foam SM10", "hinge x mm", "tail lever mm (tail centroid minus CM reference)", "criteria not met (8 % margin, bend up to 10 deg)", "Cn_beta no fin", "Cl_beta", "tip loading at 5 deg", f"tip loading v1 ref ({v1_tip:.3f}) +1%: ok", "span efficiency near-field (trend only)"]
    rows = []
    for r in results:
        pl = r["plan"]
        c = [x[2] for x in pl["rows"]]
        rows.append([r["name"], r["desc"], f"{c[0]:.1f}", f"{c[1]:.1f}", f"{c[2]:.1f}", f"{c[3]:.1f}", f"{pl['mean_mm']:.1f}", f"{100 * pl['inner_frac']:.1f}", f"{100 * pl['outer_frac']:.1f}", f"{r['np_pct']:.1f}",
                     f"{r['cm0_per_deg']:.5f}", f"{r['bend_for_sm8']:.2f}", f"{r['bend_for_sm10']:.2f}", f"{100 * r['sm_at_5deg']:.1f}", f"{r['cg_mm_sm10']:.1f}", f"{r['bare_cg_printed_mm']:.1f}", f"{r['bare_cg_pct_mac_aft_of_np']:+.1f}",
                     f"{100 * r['ballast_frac_printed_sm8']:.1f}", f"{100 * r['ballast_frac_foam_sm8']:.1f}", f"{100 * r['ballast_frac_printed_sm10']:.1f}", f"{100 * r['ballast_frac_foam_sm10']:.1f}",
                     f"{r['x_h_mm']:.1f}", f"{r['tail_lever_mm']:.1f}", "; ".join(r["fails"]) or "none", f"{r['cn_beta']:+.5f}", f"{r['cl_beta']:+.5f}", f"{r['tip_loading']:.3f}", "yes" if r["tip_ok"] else "NO", f"{r['span_eff']:.3f}"])
    write_csv(out / "variants.csv", hdr, rows)
    pn = []
    if panel:
        for nm, upd, feats in (("A_v2_final", {}, None), ("H_even", VARIANTS[7][2], None)):
            pn += panel_noise(work, nm, upd, feats)
        write_csv(out / "panelling.csv", ["variant", "chordwise", "spanwise", "NP % MAC", "CM0 at 5 deg", "Cn_beta", "tip loading"], [[a, b, c, f"{d:.2f}", f"{e:.5f}", f"{f:.5f}", f"{g_:.3f}"] for a, b, c, d, e, f, g_ in pn])
    # planform table of the reference
    pl = base["plan"]
    write_csv(out / "planform_v2_BS.csv", ["station", "y mm from centreline", "chord mm", "chord / mean chord"], [[a, f"{b:.1f}", f"{c:.1f}", f"{d:.2f}"] for a, b, c, d in pl["rows"]])
    page(out, results, v1_tip, pn)
    return results, v1_tip, pn


def page(out, results, v1_tip, pn):
    A = results[0]
    pl = A["plan"]
    B = next(r for r in results if r["name"] == "B_v1_outline")
    h = html.escape
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Why the v2 swift outline looks like that</title><style>{ft.CSS}\n.scroll{{overflow-x:auto}}table.t td,table.t th{{padding:4px 7px;font-size:13px;vertical-align:top}}</style></head><body><main>"]
    P.append("<h1>Why the v2 swift-like outline has a broad inner wing and a thin outer wing</h1>")
    P.append("<p class='note'>Diagnosis only; nothing is rebuilt. AVL outline model (inviscid, body ignored) plus arithmetic. Known caveats: AVL neutral point uncertain by up to about 5% MAC with the panelling when the tail sits in the wake plane; real flat plates lift 8 to 16% above AVL and their aerodynamic centre moves; v2 span efficiency used near-field induced drag (trend only).</p>")
    P.append("<section><h2>The planform, in numbers (v2 BS final, span 250 mm)</h2><div class='scroll'><table class='t'><tr><th>station</th><th>y, mm</th><th>chord, mm</th><th>chord / mean chord</th></tr>" +
             "".join(f"<tr><td>{a}</td><td class='n'>{b:.1f}</td><td class='n'>{c:.1f}</td><td class='n'>{d:.2f}</td></tr>" for a, b, c, d in pl["rows"]) + "</table></div>")
    P.append(f"<p>Mean chord {pl['mean_mm']:.1f} mm (span 250 / aspect ratio 9.5). Arm/hand split at {100 * pl['split']:.0f}% of the semi-span ({pl['split'] * 125:.0f} mm); arm leading-edge sweep {pl['arm_sweep']:g} deg, hand sweep {pl['hand_sweep']:g} deg. "
             f"Wing area {pl['S_mm2']:,.0f} mm&sup2;: inner (arm) wing {100 * pl['inner_frac']:.0f}%, outer (hand) wing {100 * pl['outer_frac']:.0f}%. Fan tail {pl['tail_mm2']:,.0f} mm&sup2; ({100 * pl['tail_over_S']:.0f}% of the wing area). "
             f"Sheet area {pl['sheet_mm2']:,.0f} mm&sup2;; the nose and the root blend add {pl['extra_mm2']:,.0f} mm&sup2; ({100 * pl['extra_mm2'] / pl['sheet_mm2']:.0f}% of the sheet) beyond wing plus tail "
             f"(v1 outline without blend and with the short nose: {B['plan']['extra_mm2']:,.0f} mm&sup2;). The tail neck is {A['g']['wt0'] * SPAN:.0f} mm wide, the hinge sits {A['g']['x_h'] * SPAN:.1f} mm aft of the root leading edge, so the fan starts exactly where the inner wing's trailing edge is; the tail joins the inner wing across {A['g']['wt0'] * SPAN:.0f} mm.</p></section>")
    P.append("<section><h2>What caused it</h2><h3>1. Parametrisation: the recipe, not the search</h3><ul>"
             "<li><b>The chord law was fixed by hand and never searched.</b> The v2 search varied seven switches (m_shape, curved_te, root_blend, tip_round, neck, spread, nose) and the tail bend. The base planform, wrist at 40% of the semi-span, hand leading-edge sweep 50 deg, wrist chord 0.85 x root chord, hand chord falling 1.00, 0.78, 0.50, 0.25, 0.06 of the wrist chord at 0, 40, 75, 93, 100% of the hand, is the v1 BS recipe (all JUDGEMENT; only the aspect ratio 9.5 and the tail spread are CITED). It was never an optimum of anything.</li>"
             "<li><b>The recipe can only express a chord that falls monotonically from the root to a point</b> (a scythe): there is no way to ask for an even or growing chord, or for a different hand chord law other than by editing the station table. With the aspect ratio held at 9.5 the root chord is solved to fill the area, so a pointed hand forces a wide inner wing.</li>"
             "<li><b>The hand sweep of 50 deg is a style choice.</b> The only source in the bird notes is a 60 deg swift hand-wing model in a water tunnel, secondary context, not a planform. The notes give no arm/hand split and no chord law.</li>"
             "<li><b>The fan tail attaches along the inner wing's trailing edge at the root</b> with an 8% span neck (20 mm), so the inner wing is also where the tail, the hinge and the rail saddle live; nothing in the recipe lets the tail attach elsewhere.</li>"
             "<li>The root blend and a 9% nose add area at the root only: a small contribution (see the numbers above).</li></ul>")
    P.append("<h3>2. Criteria: what the acceptance rules asked for</h3><div class='scroll'><table class='t'><tr><th>criterion</th><th>threshold (v2)</th><th>origin</th><th>binds the planform?</th></tr>"
             f"<tr><td>static margin at CL 0.3</td><td>{100 * v2.SM_WINDOW[0]:g} to {100 * v2.SM_WINDOW[1]:g}% MAC</td><td>JUDGEMENT (ours; v1 used 5 to 15%)</td><td>Sets the bend/CG pairing, not the planform.</td></tr>"
             f"<tr><td>nose ballast / all-up mass</td><td>below {100 * v2.BALLAST_MAX_FRAC:g}% at both sizes</td><td>JUDGEMENT</td><td>BS reaches {100 * A['ballast_frac_printed_sm8']:.0f}% printed, {100 * A['ballast_frac_foam_sm8']:.0f}% foam at an 8% margin: not binding for BS (it was binding for the crow outline).</td></tr>"
             f"<tr><td>tip loading indicator</td><td>not above the v1 value at the same bend (+{100 * v2.TIP_TOL:g}%)</td><td>JUDGEMENT (a relative rule, v1 = {v1_tip:.2f})</td><td>Prefers outlines that do not load the tip harder than v1; the indicator is not a stall prediction.</td></tr>"
             f"<tr><td>Cn_beta with no fin</td><td>at least {v2.CN_BETA_MIN} per rad</td><td>JUDGEMENT</td><td>BS passes by a small margin ({A['cn_beta']:+.4f}); see the variations.</td></tr>"
             "<tr><td>tail neck width</td><td>stress-based minimum or 4% of the span</td><td>ARITHMETIC on ASSUMED strength</td><td>Sets the neck width (20 mm), hence how wide the inner wing must be at the hinge.</td></tr>"
             "<tr><td>aspect ratio 9.5</td><td>fixed</td><td>CITED (swift b&sup2;/S)</td><td>Fixes area and mean chord (26.3 mm) at span 250 mm.</td></tr></table></div>"
             "<p>None of these thresholds comes from the literature. They are our own, and none selects a broad inner wing directly. The search could not even try another chord law.</p>")
    P.append("<h3>3. Physics and project constraints</h3><ul>"
             "<li><b>Real:</b> at aspect ratio 9.5 and 250 mm span the mean chord is only 26 mm; a 0.6 mm printed sheet and a 5 mm foam sheet are 2 to 19% of the chord, so any outer chord of a few millimetres is hard to cut or print and bends easily. The near-tip chord of the v2 BS is "
             f"{A['tip_chord_95_mm']:.1f} mm at 95% of the hand. A minimum buildable chord is a real constraint of this project (flat sheet, hand-cut or printed); its value (about 8 to 10 mm is a guess) is JUDGEMENT and not tested here.</li>"
             "<li><b>Real:</b> the fan tail and the hinge need a root region wide enough to take the hinge strip; the tail load (0.4 mm strip) sets the neck width.</li>"
             "<li><b>Inherited/arbitrary:</b> pointed swept hand, 50 deg: copied from the idea of a swift silhouette, not required by passive pitch stability, by the nose-down drop or by a flat sheet. A taper to a point is also a classic way to reduce induced drag at a given span; AVL's span-efficiency numbers here are near-field and only a trend.</li>"
             "<li><b>Not tested by anything here:</b> later folding. A folding wing prefers a moderately wide, simple inner panel, but no folding geometry has been defined.</li></ul></section>")
    # variations
    P.append("<section><h2>What the variation runs show</h2><p>Same span, same aspect ratio 9.5 (so the same area), same tail, same nose and neck; only the chord distribution or the split changes. AVL at 10 x 80 panels, tail bend 5 deg, one run per bend 0 and 5. "
             "Ballast is at the trim CG that gives a static margin of 8% and 10% (CG = neutral point minus the margin x MAC), v2 mass arithmetic.</p><div class='scroll'><table class='t'><tr><th>variant</th><th>chord mm: root / wrist / mid-hand / near tip</th><th>inner area %</th><th>NP % MAC</th>"
             "<th>hinge x / tail lever, mm</th><th>CM0 per deg</th><th>bend for SM 10%, deg</th><th>ballast printed / foam at SM 8%</th><th>at SM 10%</th><th>bare CG aft of NP, % MAC</th><th>Cn_beta</th><th>Cl_beta</th><th>tip loading</th><th>criteria not met (8% margin, bend up to 10 deg)</th></tr>")
    for r in results:
        pl2 = r["plan"]
        c = [x[2] for x in pl2["rows"]]
        P.append(f"<tr><td><b>{r['name']}</b><br><small>{h(r['desc'])}</small></td><td class='n'>{c[0]:.0f} / {c[1]:.0f} / {c[2]:.0f} / {c[3]:.0f}</td><td class='n'>{100 * pl2['inner_frac']:.0f}</td><td class='n'>{r['np_pct']:.1f}</td><td class='n'>{r['x_h_mm']:.0f} / {r['tail_lever_mm']:.0f}</td><td class='n'>{r['cm0_per_deg']:.4f}</td>"
                 f"<td class='n'>{r['bend_for_sm10']:.1f}</td><td class='n'>{100 * r['ballast_frac_printed_sm8']:.0f}% / {100 * r['ballast_frac_foam_sm8']:.0f}%</td><td class='n'>{100 * r['ballast_frac_printed_sm10']:.0f}% / {100 * r['ballast_frac_foam_sm10']:.0f}%</td>"
                 f"<td class='n'>{r['bare_cg_pct_mac_aft_of_np']:+.0f}</td><td class='n'>{r['cn_beta']:+.4f}</td><td class='n'>{r['cl_beta']:+.4f}</td><td class='n'>{r['tip_loading']:.2f}{'' if r['tip_ok'] else ' (above v1)'}</td><td>{h('; '.join(r['fails']) or 'none')}</td></tr>")
    P.append(f"</table></div><p>Tip loading v1 reference {v1_tip:.3f} at 5 deg. Full table: <a href='variants.csv'>variants.csv</a>.</p>")
    R = {r["name"]: r for r in results}
    E, F, C, D, G, H, I = (R[k] for k in ("E_fuller_hand", "F_hand_sweep30", "C_no_arm_taper", "D_narrow_root", "G_short_arm", "H_even", "I_even_wide_hand"))
    P.append("<h3>Findings</h3><ul>"
             f"<li><b>Free to change:</b> a fuller hand chord law with the same 50 deg sweep (E). Near-tip chord {E['tip_chord_95_mm']:.0f} mm instead of {A['tip_chord_95_mm']:.1f} mm, tip loading {E['tip_loading']:.2f} instead of {A['tip_loading']:.2f}, Cn_beta {E['cn_beta']:+.4f} against {A['cn_beta']:+.4f}, ballast {100 * E['ballast_frac_printed_sm8']:.0f}% / {100 * E['ballast_frac_foam_sm8']:.0f}% against {100 * A['ballast_frac_printed_sm8']:.0f}% / {100 * A['ballast_frac_foam_sm8']:.0f}% at an 8% margin. "
             f"Cost seen: the tail has less authority ({E['cm0_per_deg']:.4f} against {A['cm0_per_deg']:.4f} CM0 per degree, so about {E['bend_for_sm10']:.1f} deg of bend for a 10% margin against {A['bend_for_sm10']:.1f}), and the neutral point moves forward ({E['np_pct']:.0f}% against {A['np_pct']:.0f}% MAC, partly because the MAC itself moves).</li>"
             f"<li><b>Costs something real: less hand sweep (F, H, I).</b> At 30 deg Cn_beta falls to {F['cn_beta']:+.4f} to {H['cn_beta']:+.4f} (the 0.004 floor fails) and the bare CG sits {F['bare_cg_pct_mac_aft_of_np']:+.0f}% MAC behind the neutral point (against {A['bare_cg_pct_mac_aft_of_np']:+.0f}%), so ballast rises to {100 * F['ballast_frac_printed_sm8']:.0f}% printed. Sweep is doing real work against our own yaw-without-a-fin and ballast criteria; it is not just styling. Both criteria are our JUDGEMENT, and a fin or another fix was not tried here.</li>"
             f"<li><b>Costs something real: a narrower inner wing (C, D).</b> The hinge sits at the trailing edge of the root chord, so a narrower root pulls the tail forward (hinge {A['x_h_mm']:.0f} mm to {D['x_h_mm']:.0f} mm; tail lever {A['tail_lever_mm']:.0f} to {D['tail_lever_mm']:.0f} mm) and the tail loses authority ({D['cm0_per_deg']:.4f} CM0 per degree, a bend of {D['bend_for_sm10']:.0f} deg for a 10% margin). That coupling is a recipe artefact: tail length and attachment point were not independent parameters.</li>"
             f"<li><b>Costs a lot: moving the arm/hand split inboard (G).</b> Margin at 5 deg falls to {100 * G['sm_at_5deg']:.0f}%, a 10% margin needs {G['bend_for_sm10']:.0f} deg of bend, tip loading rises to {G['tip_loading']:.2f}, and the ballast station is out of reach.</li>"
             f"<li><b>An even planform (H, I) gives the lowest tip loading</b> ({H['tip_loading']:.2f}, {I['tip_loading']:.2f} against {A['tip_loading']:.2f}) but fails the yaw floor ({H['cn_beta']:+.4f}, {I['cn_beta']:+.4f}) and the ballast limit ({100 * H['ballast_frac_printed_sm8']:.0f}%, {100 * I['ballast_frac_printed_sm8']:.0f}% printed) because of the 30 deg sweep chosen with it. The same planform at 50 deg sweep was not run.</li>"
             "<li><b>Which criteria bound the v2 outline?</b> Not ballast (12% printed, 7% foam at an 8% margin). The tip-loading rule favoured rounding the tip. Cn_beta passes by a small margin. The static-margin window sets the bend. The search did not vary the chord law at all, so no criterion chose the broad inner wing; the recipe did.</li>"
             "<li><b>Why the inner wing looks massive:</b> the inner (arm) wing is "
             f"{100 * pl['inner_frac']:.0f}% of the wing area in {100 * pl['split']:.0f}% of the semi-span, with chord {pl['rows'][0][3]:.2f} times the mean at the root, and the visible inner mass also includes the nose and root blend ({pl['extra_mm2']:,.0f} mm&sup2;) and the fan tail ({100 * pl['tail_over_S']:.0f}% of the wing area) joined to it through a {A['g']['wt0'] * SPAN:.0f} mm neck.</li></ul>"
             "<h3>Suggested degrees of freedom for a v4 recipe (not built)</h3><ol>"
             "<li>Hand chord law as two or three free numbers (tip chord fraction, mid-hand fullness) instead of a pointed fixed table, with a stated minimum buildable chord.</li>"
             "<li>Wrist chord / root chord allowed to exceed 1, and the arm/hand split free (not tied to 40%).</li>"
             "<li>Hinge position and tail length independent of the root chord (tail attachment point, length, spread and neck as separate numbers), so the tail lever is not a side effect of the inner chord.</li>"
             "<li>Leading-edge sweep and trailing-edge shape defined separately (so sweep for yaw can be kept without a pointed tip); hand sweep as a searched parameter with the yaw and ballast criteria visible.</li>"
             "<li>Body width, nose and root blend independent of the wing, and counted separately in mass and area.</li>"
             "<li>Keep the criteria as explicit, separately reported numbers; ask for each threshold (margin window, ballast limit, Cn_beta floor, tip loading) whether it is really needed or only inherited.</li></ol>")
    if pn:
        P.append("<h3>Numerical noise</h3><div class='scroll'><table class='t'><tr><th>variant</th><th>panels</th><th>NP % MAC</th><th>CM0</th><th>Cn_beta</th><th>tip loading</th></tr>" + "".join(
            f"<tr><td>{a}</td><td>{b} x {c}</td><td class='n'>{d:.1f}</td><td class='n'>{e:+.4f}</td><td class='n'>{f:+.4f}</td><td class='n'>{g_:.2f}</td></tr>" for a, b, c, d, e, f, g_ in pn) + "</table></div>"
                 "<p>Differences between variants smaller than these spreads are not results.</p>")
    P.append("</section></main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--no-panel", action="store_true")
    a = ap.parse_args()
    results, v1, pn = run(a.out, not a.no_panel)
    for r in results:
        print(r["name"], f"NP {r['np_pct']:.1f} cm0/deg {r['cm0_per_deg']:.5f} ballast sm8 {100 * r['ballast_frac_printed_sm8']:.0f}/{100 * r['ballast_frac_foam_sm8']:.0f} sm10 {100 * r['ballast_frac_printed_sm10']:.0f}/{100 * r['ballast_frac_foam_sm10']:.0f} Cn {r['cn_beta']:+.4f} tip {r['tip_loading']:.2f}")


if __name__ == "__main__":
    main()
