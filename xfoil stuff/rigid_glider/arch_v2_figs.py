"""Stage-3 figures (SVG).  py -B rigid_glider/arch_v2_figs.py"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import re
from html import escape

import arch_v2_items as I
import arch_v2_model as V2
import arch_v2_study as T
import pure_wing_study as st

OUT = V2.OUT
COL = {"pure": "#1769aa", "lip": "#c55127", "tail": "#28834f"}
LIP_FORM_COL = {"a": "#c55127", "b": "#e8a27f", "c": "#7a2a0a"}
TAIL_COL = {"avl": "#28834f", "half": "#8cc9a5"}
WING, SEC = "P07", "NACA_0006"


def fig_windows():
    ser = []
    pts = lambda kind, cfg, u, form: [(ws, I.wmm(I.W("final", kind, WING, SEC, cfg, u, ws, form, 160, "both"))) for ws in I.WS]
    ser.append(dict(label="pure", pts=pts("pure", None, 0.0, "coupled"), color=COL["pure"], marker="circle"))
    for form, tag, mk in (("coupled", "a", "circle"), ("replace", "b", "x"), ("replace_k", "c", "hollow")):
        ser.append(dict(label=f"lip 20/10 outboard, form ({tag})", pts=pts("lip", I.lipcfg(0.20), 10.0, form), color=LIP_FORM_COL[tag], marker=mk, dash=(tag != "a")))
    ser.append(dict(label="tail 10% @2.0cr, AVL effectiveness", pts=pts("tail", I.tailcfg(0.10), -3.0, "coupled"), color=TAIL_COL["avl"], marker="circle"))
    ser.append(dict(label="tail, 0.5 effectiveness", pts=pts("tail", I.tailcfg(0.05), -3.0, "coupled"), color=TAIL_COL["half"], marker="x", dash=True))
    ser.append(dict(label="lip AVL-only", pts=[(ws, I.wmm(I.W("final", "lip", WING, SEC, I.lipcfg(0.20), 10.0, ws, "avl_only", None, "avl"))) for ws in I.WS], color="#999999", marker=None, dash=True))
    st.svg_chart(OUT / "fig1_uncensored_window_by_architecture.svg", f"Uncensored CG window (trim CL 0.2-0.5, both Ncrit) vs W/S, {WING} {SEC}, final deck",
                 "wing loading W/S (N/m2)", "stable-trim CG window (mm)", ser,
                 "SM scan -0.15..+0.60 (no window touches an edge). Lip forms (a) increment, (b) replace, (c) replace + lift redistribution scaled by k form the lip band; tail at AVL and 0.5 lift effectiveness.")


def svg_hbars(path, title, groups, xlabel, caption, xlim, vlines=()):
    """groups: [(group label, [(bar label, lo, hi, colour)])]"""
    W_, rowh = 980, 15
    nrows = sum(len(g[1]) for g in groups) + len(groups)
    H = 90 + nrows * rowh + 70
    L, R_ = 230, 30
    sx = lambda v: L + (v - xlim[0]) / (xlim[1] - xlim[0]) * (W_ - L - R_)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W_}" height="{H}" viewBox="0 0 {W_} {H}" font-family="Arial">', '<rect width="100%" height="100%" fill="white"/>',
         f'<text x="{W_/2}" y="24" text-anchor="middle" font-size="16">{escape(title)}</text>']
    for k in range(6):
        v = xlim[0] + k * (xlim[1] - xlim[0]) / 5
        o += [f'<line x1="{sx(v):.1f}" y1="44" x2="{sx(v):.1f}" y2="{H-60}" stroke="#e5e9ed"/>', f'<text x="{sx(v):.1f}" y="{H-46}" text-anchor="middle" font-size="11">{v:.0f}</text>']
    y = 52
    for gl, bars in groups:
        o.append(f'<text x="8" y="{y+11}" font-size="12" font-weight="bold">{escape(gl)}</text>')
        y += rowh
        for bl, lo, hi, col in bars:
            o.append(f'<text x="{L-6}" y="{y+10}" text-anchor="end" font-size="10">{escape(bl)}</text>')
            if lo is not None:
                o.append(f'<rect x="{sx(lo):.1f}" y="{y+2}" width="{max(2.0, sx(hi)-sx(lo)):.1f}" height="{rowh-4}" fill="{col}"/>')
            else:
                o.append(f'<text x="{sx(xlim[0])+4:.1f}" y="{y+10}" font-size="10" fill="#777">no window</text>')
            y += rowh
    for v, lab in vlines:
        o += [f'<line x1="{sx(v):.1f}" y1="44" x2="{sx(v):.1f}" y2="{H-60}" stroke="#555" stroke-dasharray="5 3"/>', f'<text x="{sx(v)+3:.1f}" y="56" font-size="10" fill="#444">{escape(lab)}</text>']
    o.append(f'<text x="{(L+W_-R_)/2}" y="{H-30}" text-anchor="middle" font-size="12">{escape(xlabel)}</text>')
    for j, ln in enumerate([caption[i:i + 160] for i in range(0, len(caption), 160)][:2]):
        o.append(f'<text x="12" y="{H-14+j*12}" font-size="10" fill="#333">{escape(ln)}</text>')
    o.append("</svg>")
    path.write_text("\n".join(o), encoding="utf-8")


def fig_centre_extent():
    groups = []
    for ws in I.WS:
        bars = []
        def add(label, kind, cfg, u, form, col):
            r = I.W("final", kind, WING, SEC, cfg, u, ws, form, 160, "both")
            bars.append((label, (r["cg_mm_lo"], r["cg_mm_hi"]) [0] if r and r["n_points"] else None, r["cg_mm_hi"] if r and r["n_points"] else None, col))
        add("pure", "pure", None, 0.0, "coupled", COL["pure"])
        add("lip 20/10 (a)", "lip", I.lipcfg(0.20), 10.0, "coupled", LIP_FORM_COL["a"])
        add("lip 20/10 (b)", "lip", I.lipcfg(0.20), 10.0, "replace", LIP_FORM_COL["b"])
        add("lip 20/10 (c)", "lip", I.lipcfg(0.20), 10.0, "replace_k", LIP_FORM_COL["c"])
        add("tail 10% AVL eff.", "tail", I.tailcfg(0.10), -3.0, "coupled", TAIL_COL["avl"])
        add("tail 10% 0.5 eff.", "tail", I.tailcfg(0.05), -3.0, "coupled", TAIL_COL["half"])
        groups.append((f"W/S {ws} N/m2", bars))
    Mp = I.mdl("final", "pure", WING, None)
    svg_hbars(OUT / "fig2_window_centre_extent_P07.svg", f"Where each both-Ncrit stable-trim window sits on the CG rail, {WING} {SEC} (final deck)", groups,
              "CG aft of root leading edge (mm)", "Bars = window start to end; neutral point of the pure wing marked (dashed). Tail windows sit near the pure-wing NP; lip windows sit 10-20 mm forward, and form (c) sits aft of (a) and (b).",
              (35, 95), vlines=[(Mp["xnp"] * T.B * 1000, "pure NP")])


def fig_mass():
    R = list(csv.DictReader((OUT / "item6_mass_cg_budget.csv").open(encoding="utf-8")))
    rows = [r for r in R if r["wing"] == WING and r["section"] == SEC and r["ballast_station_root_chords"] == "-0.25"]
    cats = []
    parts_col = {"wing": "#7f7f7f", "pod": "#aaaaaa", "boom": "#cccccc", "fin": "#8c564b", "tail": "#28834f", "ballast": "#222222"}
    W_, H = 980, 520
    L, Bt, Tp = 70, 90, 50
    ymax = max(float(r["all_up_mass_g"]) for r in rows) * 1.1
    sy = lambda v: H - Bt - v / ymax * (H - Bt - Tp)
    bw = (W_ - L - 30) / len(rows)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W_}" height="{H}" viewBox="0 0 {W_} {H}" font-family="Arial">', '<rect width="100%" height="100%" fill="white"/>',
         f'<text x="{W_/2}" y="24" text-anchor="middle" font-size="16">Mass and nose-ballast budget, {WING} {SEC}, b = 0.40 m (exploratory)</text>',
         f'<line x1="{L}" y1="{Tp}" x2="{L}" y2="{H-Bt}" stroke="#222"/>', f'<line x1="{L}" y1="{H-Bt}" x2="{W_-30}" y2="{H-Bt}" stroke="#222"/>',
         f'<text x="16" y="{(H-Bt+Tp)/2}" transform="rotate(-90 16 {(H-Bt+Tp)/2})" text-anchor="middle" font-size="12">mass (g)</text>']
    for k in range(6):
        v = ymax * k / 5
        o += [f'<line x1="{L}" y1="{sy(v):.1f}" x2="{W_-30}" y2="{sy(v):.1f}" stroke="#e5e9ed"/>', f'<text x="{L-6}" y="{sy(v)+4:.1f}" text-anchor="end" font-size="11">{v:.0f}</text>']
    order = {"low": 0, "high": 1}
    rows.sort(key=lambda r: (order[r["struct_case"]], ["pure", "lip 20/10", "lip 15/8", "tail 10%@2.0cr"].index(r["arch"])))
    for i, r in enumerate(rows):
        x0 = L + i * bw + 10
        y0 = 0.0
        comps = [(m_.group(1), float(m_.group(2))) for m_ in re.finditer(r"(\w+) ([\d.]+)g@", r["parts"])]
        comps.append(("ballast", float(r["ballast_g"] or 0.0)))
        for name, mass in comps:
            h = sy(y0) - sy(y0 + mass)
            o.append(f'<rect x="{x0:.1f}" y="{sy(y0+mass):.1f}" width="{bw-20:.1f}" height="{h:.1f}" fill="{parts_col.get(name, "#bbb")}"/>')
            y0 += mass
        ws_ = r["ws_n_m2"]
        o.append(f'<text x="{x0+(bw-20)/2:.1f}" y="{sy(y0)-5:.1f}" text-anchor="middle" font-size="10">{y0:.0f} g, W/S {float(ws_):.1f}</text>' if ws_ else f'<text x="{x0:.1f}" y="{sy(y0)-5:.1f}" font-size="10">unreachable</text>')
        o.append(f'<text x="{x0+(bw-20)/2:.1f}" y="{H-Bt+14}" text-anchor="middle" font-size="10">{r["struct_case"]} {escape(r["arch"])}</text>')
    lx = L
    for name, c in parts_col.items():
        o += [f'<rect x="{lx}" y="{H-40}" width="12" height="12" fill="{c}"/>', f'<text x="{lx+16}" y="{H-30}" font-size="11">{name}</text>']
        lx += 100
    o.append(f'<text x="12" y="{H-10}" font-size="10" fill="#333">Ballast at 0.25 root chords ahead of the root LE brings the CG to each architecture\'s target (window centre, W/S 12). Low: shell 0.5 kg/m2, pod 3 g, boom 1.5 g/m, 0.3 kg/m2 tail/fin. High: infill 250, 6 g, 3 g/m, 0.6 kg/m2.</text>')
    o.append("</svg>")
    (OUT / "fig3_mass_ballast_budget.svg").write_text("\n".join(o), encoding="utf-8")


def main():
    fig_windows(); fig_centre_extent(); fig_mass()
    print("figures written")


if __name__ == "__main__":
    main()
