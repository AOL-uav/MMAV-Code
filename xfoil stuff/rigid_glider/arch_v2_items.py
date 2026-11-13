"""Stage-3 items 1-9 and corrected table D.  Reads arch_v2_study outputs.   py -B rigid_glider/arch_v2_items.py i1 i2 i3 ...
Writes only to results/runs/rigid_glider/arch_compare_v2/."""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import pickle
import statistics
from collections import defaultdict
from pathlib import Path

import arch_model as A
import arch_study as S
import arch_v2_model as V2
import arch_v2_study as T
import pure_wing_model as m
import pure_wing_v2_model as v2
import study_geometry as sg
from avl import run_avl

OUT = V2.OUT
B = T.B
G = 9.80665
WINGS = T.WINGS
SECS = T.SECTIONS
WS = T.WS_LIST
LIPS_OUT = [(0.10, 3.0), (0.15, 5.0), (0.10, 6.0), (0.20, 5.0), (0.15, 8.0), (0.20, 10.0)]     # ordered by inviscid lip moment
CHOSEN_LIP = [(0.20, 10.0), (0.15, 8.0)]
_cache = {}


def models():
    if "m" not in _cache:
        _cache["m"] = T.load_models()
    return _cache["m"]


def recs():
    if "r" not in _cache:
        _cache["r"] = json.loads((OUT / "scans_v2.json").read_text())
    return _cache["r"]


def rows():
    if "rows" not in _cache:
        _cache["rows"] = V2.window_rows2(recs(), models())
    return _cache["rows"]


def ridx():
    if "ridx" not in _cache:
        _cache["ridx"] = {(r["deck"], r["kind"], r["wing"], r["section"], r["cfg1"], r["cfg2"], r["u"], r["ws"], r["form"], r["panels"], r["tier"], r["mode"]): r for r in rows()}
    return _cache["ridx"]


def recidx():
    if "recidx" not in _cache:
        _cache["recidx"] = {(r["deck"], r["kind"], r["wing"], r["section"], json.dumps(r["cfg"]), r["u"], r["ws"], r["form"], r["ncrit"], r["panels"]): r for r in recs()}
    return _cache["recidx"]


def tabs():
    if "tabs" not in _cache:
        _cache["tabs"] = A.ArchTables()
    return _cache["tabs"]


def cfgvals(kind, cfg):
    return (None, None) if cfg is None else (cfg[0], cfg[1])


def W(deck, kind, wing, sec, cfg, u, ws, form, pan, tier, mode="any"):
    c1, c2 = cfgvals(kind, cfg)
    if form == "avl_only":
        pan, tier = None, "avl"
    return ridx().get((deck, kind, wing, sec, c1, c2, u, ws, form, pan, tier, mode))


def wmm(r):
    return r["width_mm"] if (r and r["n_points"]) else 0.0


def lipcfg(cf):
    return [cf, "outer"]


def tailcfg(a):
    return [a, T.TAIL_STN]


def mdl(deck, kind, wing, cfg):
    return models()[deck][V2.model_key(kind, wing, cfg)]


def pt_at(pts, sm):
    return S.pt_at(pts, sm)


def trim_info(deck, kind, wing, sec, cfg, u, ws, form, nc, pan, cl):
    M = mdl(deck, kind, wing, cfg)
    ev, ti, avl = V2.evaluators(M, B, u, kind, tabs(), sec, form, nc, pan, cfg, ws)
    r = ev(cl, None)
    V = r["V"]
    mac = M["mac"] * B
    st = M["strips"]
    cdi = M["k_cdi"] * cl ** 2
    out = dict(cl=cl, V=V, alpha_deg=r["alpha_avl"] + (r.get("dalpha") or 0.0), alpha_avl_deg=r["alpha_avl"], coverage=r["coverage"], jag=r["jag"], cdp=r.get("cdp"),
               ld=(cl / (cdi + r["cdp"]) if r.get("cdp") else None), re_root=V * st[0]["c"] * B / A.NU, re_tip=V * st[-1]["c"] * B / A.NU, re_mac=V * mac / A.NU)
    if "k" in r:
        out["k"] = r["k"]
        out["converged"] = r["converged"]
    return out


def centre_idx(r):
    return round(round(r["sm_centre"] / V2.STEP) * V2.STEP, 4)


# ------------------------------------------------------------------ item 1
def item1():
    R = [r for r in rows() if r["deck"] == "legacy"]
    V2_rows = [r for r in R if r["n_points"]]
    write = lambda name, rr: T.write_csv(OUT / name, rr)
    write("item1_windows_uncensored_legacy_deck.csv", R)
    summ = {}
    for kind, form in (("pure", "coupled"), ("lip", "coupled"), ("lip", "replace"), ("lip", "replace_k"), ("tail", "coupled"), ("lip", "avl_only"), ("tail", "avl_only")):
        for tier in (("avl",) if form == "avl_only" else ("N5", "N9", "both")):
            for mode in (("any",) if form == "avl_only" else V2.MODES):
                sel = [r for r in R if r["kind"] == kind and r["form"] == form and r["tier"] == tier and r["mode"] == mode and r["n_points"]]
                tot = len([r for r in R if r["kind"] == kind and r["form"] == form and r["tier"] == tier and r["mode"] == mode])
                edge = [r for r in sel if r["touches_lo_edge"] or r["touches_hi_edge"]]
                w = [r["width_mm"] for r in sel]
                summ[f"{kind}|{form}|{tier}|{mode}"] = dict(cases=tot, with_window=len(sel), touching_edge=len(edge), touching_lo=sum(r["touches_lo_edge"] for r in sel),
                                                           touching_hi=sum(r["touches_hi_edge"] for r in sel), median_mm=statistics.median(w) if w else None,
                                                           max_mm=max(w) if w else None)
    # analytic untruncated AVL-only windows
    ana = []
    for kind, cfgs in (("lip", [(cf, ang) for cf, ang in LIPS_OUT]), ("tail", [(a, -3.0) for a in T.TAIL_AREAS2])):
        for wing in WINGS:
            for cf, ang in cfgs:
                cfg = lipcfg(cf) if kind == "lip" else tailcfg(cf)
                M = mdl("legacy", kind, wing, cfg)
                cm0u = M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"]
                x = ang * cm0u
                if x <= 0:
                    continue
                lo, hi = x / 0.5, x / 0.2
                mac_mm = M["mac"] * B * 1000
                sr = W("legacy", kind, wing, "NACA_0006", cfg, ang, 8, "avl_only", None, "avl")
                ana.append(dict(kind=kind, wing=wing, cfg=json.dumps(cfg), u=ang, sm_lo=lo, sm_hi=hi, sm_centre=0.5 * (lo + hi), width_mm=(hi - lo) * mac_mm,
                                centre_cg_mm=M["xnp"] * B * 1000 - 0.5 * (lo + hi) * mac_mm, inside_scan=int(lo >= V2.SM_MIN and hi <= V2.SM_MAX),
                                scan_width_mm=sr["width_mm"] if sr and sr["n_points"] else None))
    write("item1_avl_only_analytic.csv", ana)
    # window centre / CL range summary for both-Ncrit coupled windows
    (OUT / "item1_summary.json").write_text(json.dumps(summ, indent=1))
    return summ


# ------------------------------------------------------------------ item 2
def item2():
    dec = []
    for deck in ("legacy", "final"):
        for cf in S.LIP_CFS:
            M = mdl(deck, "lip", "P07", lipcfg(cf))
            d = V2.lip_decomposition(M)
            for ang in LIP_ANGLES_OF(cf):
                dec.append(dict(deck=deck, lip=f"{int(cf*100)}/{ang:g}", cm0_total=ang * d["cm0u_total"], cm0_couple=ang * d["couple"], cm0_redistribution=ang * d["redistribution"],
                                frac_couple=d["frac_couple"], frac_redistribution=d["frac_redistribution"], independent_redistribution=ang * d["redistribution_independent"],
                                closure_error_frac=d["closure_error"], dcl_check=d["d_cl_check"]))
    T.write_csv(OUT / "item2_cm0_decomposition_P07.csv", dec)
    # forms a, b, c on P07 and k statistics
    out = []
    for deck in ("legacy",):
        for sec in SECS:
            for cf, ang in LIPS_OUT:
                for ws in WS:
                    row = dict(deck=deck, section=sec, lip=f"{int(cf*100)}/{ang:g}", ws=ws)
                    for form, tag in (("coupled", "a"), ("replace", "b"), ("replace_k", "c")):
                        r = W(deck, "lip", "P07", sec, lipcfg(cf), ang, ws, form, 160, "both")
                        row[f"win_{tag}_mm"] = wmm(r)
                        row[f"centre_{tag}_mm"] = r["cg_mm_centre"] if r and r["n_points"] else None
                    for nc in (5, 9):
                        rc = recidx().get((deck, "lip", "P07", sec, json.dumps(lipcfg(cf)), ang, ws, "replace_k", nc, 160))
                        row[f"k_mean_N{nc}"] = rc["k_mean_band"] if rc else None
                        row[f"nonconv_N{nc}"] = rc["nonconv"] if rc else None
                    out.append(row)
    T.write_csv(OUT / "item2_forms_abc_P07.csv", out)
    return dec, out


def LIP_ANGLES_OF(cf):
    return T.LIP_ANGLES[cf]


# ------------------------------------------------------------------ item 3
def item3(deck):
    out = []
    for cf, ang in LIPS_OUT:
        for ws in WS:
            rec = dict(deck=deck, lip=f"{int(cf*100)}/{ang:g}", ws=ws)
            for form, tag in (("coupled", "a"), ("replace", "b"), ("replace_k", "c")):
                w = [wmm(W(deck, "lip", wing, sec, lipcfg(cf), ang, ws, form, 160, "both")) for wing in WINGS for sec in SECS]
                rec.update({f"{tag}_median": statistics.median(w), f"{tag}_min": min(w), f"{tag}_max": max(w), f"{tag}_n_ge3": sum(1 for x in w if x >= T.THRESH),
                            f"{tag}_meets_4of6": int(sum(1 for x in w if x >= T.THRESH) >= 4), f"{tag}_n_borderline_2p5_3p5": sum(1 for x in w if 2.5 <= x <= 3.5)})
            out.append(rec)
    T.write_csv(OUT / f"item3_threshold_table_{deck}.csv", out)
    return out


# ------------------------------------------------------------------ item 4
def item4i():
    out = []
    for deck in ("legacy", "final"):
        for wing in WINGS:
            for base, fac in ((0.08, 1.0), (0.06, 0.75), (0.04, 0.5), (0.10, 1.0), (0.075, 0.75), (0.05, 0.5)):
                nominal = 0.08 if base in (0.08, 0.06, 0.04) else 0.10
                M = mdl(deck, "tail", wing, tailcfg(base))
                M0 = mdl(deck, "tail", wing, tailcfg(nominal))
                for sec in SECS:
                    for ws in WS:
                        row = dict(deck=deck, wing=wing, tail_nominal=nominal, tail_area_frac=base, effectiveness=fac, section=sec, ws=ws,
                                   d_cl_ratio=M["d_cl"] / M0["d_cl"], np_shift_ratio=(M["xnp"] - mdl(deck, "pure", wing, None)["xnp"]) / (M0["xnp"] - mdl(deck, "pure", wing, None)["xnp"]))
                        row["win_avl_mm"] = wmm(W(deck, "tail", wing, sec, tailcfg(base), T.TAIL_INC, ws, "avl_only", None, "avl"))
                        for tier in ("N5", "N9", "both"):
                            r = W(deck, "tail", wing, sec, tailcfg(base), T.TAIL_INC, ws, "coupled", 160, tier)
                            row[f"win_{tier}_mm"] = wmm(r)
                            if tier == "both":
                                row["cg_centre_mm"] = r["cg_mm_centre"] if r and r["n_points"] else None
                                row["cl_lo"], row["cl_hi"] = (r["cl_lo"], r["cl_hi"]) if r and r["n_points"] else (None, None)
                                row["touches_edge"] = int(bool(r and r["n_points"] and (r["touches_lo_edge"] or r["touches_hi_edge"])))
                                row["strict_offti_both_mm"] = wmm(W(deck, "tail", wing, sec, tailcfg(base), T.TAIL_INC, ws, "coupled", 160, "both", "offti"))
                        out.append(row)
    T.write_csv(OUT / "item4i_tail_effectiveness.csv", out)
    return out


def item4ii():
    """Neutral point and window of the P07 10 % tail vs panel density and tail height above the wing plane (final deck definition)."""
    ar, sw, tp = WINGS["P07"]
    wk = OUT / "avl_panel"
    out = []
    pure_ref = {}
    for nc_, ns_ in ((8, 24), (12, 32), (20, 32), (32, 64)):
        w = V2.final_wing(ar, sw, tp, 3)
        Mp = V2.build_pure_final(ar, sw, tp, wk / f"pure_{nc_}_{ns_}", nchord=nc_, nspan=ns_) if nc_ >= 12 else None
        for z in (0.0, 0.05, 0.10):
            M = V2.build_tail_final(ar, sw, tp, 0.10, wk / f"tail_{nc_}_{ns_}_{z}", nchord=nc_, nspan=ns_, tail_z_cr=z)
            row = dict(nchord=nc_, nspan=ns_, tail_z_over_root_chord=z, xnp_mm=1000 * M["xnp"] * B, d_cl_per_deg=M["d_cl"], d_cm_per_deg=M["d_cm"],
                       cm0u_per_deg=M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"], cla_per_rad=M["cla_per_rad"],
                       avl_only_window_mm_cl0p2_0p5=abs(T.TAIL_INC * (M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"])) * 3 * M["mac"] * B * 1000)
            # coupled window at W/S 12, NACA 0006
            for nc in (5, 9):
                ev, ti, avl = V2.evaluators(M, B, T.TAIL_INC, "tail", tabs(), "NACA_0006", "coupled", nc, 160, [0.10, 2.0], 12)
                curve, _ = V2.build_curve_ev(ev, ti)
                sets, pts = V2.scan_curve2(curve)
                row[f"_sets{nc}"] = set(sets["any"])
                row[f"win_N{nc}_mm"] = (v2.contiguous_window(sets["any"], V2.STEP)["width"] or 0.0) * M["mac"] * B * 1000 if sets["any"] else 0.0
            w_b = v2.contiguous_window(sorted(row.pop("_sets5") & row.pop("_sets9")), V2.STEP)
            row["win_both_mm"] = (w_b["width"] or 0.0) * M["mac"] * B * 1000 if w_b["n"] else 0.0
            row["win_both_sm_lo"], row["win_both_sm_hi"] = w_b["sm_lo"], w_b["sm_hi"]
            out.append(row)
    ref = [r for r in out if r["nchord"] == 20 and r["tail_z_over_root_chord"] == 0.0][0]
    for r in out:
        r["xnp_minus_20x32_z0_mm"] = r["xnp_mm"] - ref["xnp_mm"]
    T.write_csv(OUT / "item4ii_tail_panel_height.csv", out)
    return out


# ------------------------------------------------------------------ item 5
def item5():
    out = []
    wk = OUT / "avl_np"
    for wing, (ar, sw, tp) in WINGS.items():
        w0 = m._wing(ar, sw, tp, 0)
        w3 = m._wing(ar, sw, tp, 3)
        cr = w0.root_chord
        fin = dict(area_frac=0.06, x_qc=1.5 * cr)
        tail_kw = lambda x: dict(area_frac=0.10, x_qc=x * cr, incidence=0.0)
        variants = [
            ("pure 8x24 planar (sg.deck; pitch models stage 2)", w0, sg.deck(w0, "v")),
            ("pure 20x32 planar (lip/lateral deck family)", w0, A.lat_deck(w0, "v", nchord=20, nspan=32)),
            ("pure 20x32 dihedral 3", w3, A.lat_deck(w3, "v", nchord=20, nspan=32)),
            ("pure 20x32 planar + 6% centre fin", w0, A.lat_deck(w0, "v", fin=fin, nchord=20, nspan=32)),
            ("pure 20x32 dihedral 3 + 6% centre fin (FINAL)", w3, V2.deck_full(w3, "v", fin=fin)),
            ("lip(u=0) 20x32 planar (reflex deck)", w0, A.lat_deck(w0, "v", lip=(0.2, "outer"), nchord=20, nspan=32)),
            ("tail 10% @2.0cr 8x24 planar (sg.deck; pitch models stage 2)", w0, sg.deck(w0, "v", tail_area_fraction=0.10, tail_incidence_deg=0.0, tail_ac_x_m=2.0 * cr, tail_span_fraction=0.35)),
            ("tail 10% @2.0cr 20x32 planar", w0, V2.deck_full(w0, "v", tail=tail_kw(2.0))),
            ("tail 10% @2.0cr 20x32 dihedral 3 + 6% boom fin (FINAL)", w3, V2.deck_full(w3, "v", tail=tail_kw(2.0), fin=dict(area_frac=0.06, x_qc=2.0 * cr))),
        ]
        base = None
        for name, w, deck in variants:
            xnp, cla, p = V2.xnp_of(w, deck, wk / f"{wing}_{len(out)}")
            if base is None:
                base = xnp
            out.append(dict(wing=wing, variant=name, xnp_mm=1000 * xnp * B, xnp_over_mac_pct=100 * (xnp * B - w.x_le(w.mac_y) * B) / (w.mac * B), cla_per_rad=cla * 180 / math.pi,
                            diff_vs_first_variant_mm=1000 * (xnp - base) * B))
    T.write_csv(OUT / "item5_neutral_point_by_deck.csv", out)
    return out


# ------------------------------------------------------------------ item 6 (mass budget)
STRUCT = {"low": dict(wing=("shell", 0.5), pod_g=3.0, boom_g_per_m=1.5, areal=0.3), "high": dict(wing=("infill", 250.0), pod_g=6.0, boom_g_per_m=3.0, areal=0.6)}
ARCHS6 = [("pure", None, 0.0, "pure"), ("lip", [0.20, "outer"], 10.0, "lip 20/10"), ("lip", [0.15, "outer"], 8.0, "lip 15/8"), ("tail", [0.10, 2.0], -3.0, "tail 10%@2.0cr")]


def target_cg_mm(wing, sec, kind, cfg, u, ws=12):
    if kind == "lip":
        rw = robust_window("final", wing, sec, cfg[0], u, ws)
        if rw:
            return 0.5 * (rw[0] + rw[1]), "centre of the window common to lip forms (a),(b),(c) (W/S 12)"
    r = W("final", kind, wing, sec, cfg, u, ws, "coupled", 160, "both")
    if r and r["n_points"]:
        return r["cg_mm_centre"], "coupled both-Ncrit window centre (W/S 12)"
    r = W("final", kind, wing, sec, cfg, u, ws, "avl_only", None, "avl")
    if r and r["n_points"]:
        return r["cg_mm_centre"], "AVL-only window centre (no coupled window)"
    return None, "none"


def item6():
    out = []
    for wing in WINGS:
        M0 = mdl("final", "pure", wing, None)
        cr, ct = M0["root_chord"] * B, M0["tip_chord"] * B
        Sw, mac = M0["area"] * B * B, M0["mac"] * B
        xw = V2.planform_centroid_x(cr, ct, M0["tip_x_le"] * B, B / 2) * 1000
        for sec in SECS:
            tc = int(sec[-2:]) / 100
            for case, P in STRUCT.items():
                mw = V2.wing_mass(P["wing"][0], Sw, mac, sigma=P["wing"][1], rho_eff=P["wing"][1], tc=tc) * 1000
                for kind, cfg, u, label in ARCHS6:
                    parts = [("wing", mw, xw)]
                    pod_x = 0.25 * cr * 1000                     # pod from -0.5 to +1.0 root chords, CG at mid-length
                    parts.append(("pod", P["pod_g"], pod_x))
                    fin_a = 0.06 * Sw
                    tail_a = 0.10 * Sw
                    if kind == "tail":
                        x_t = 2.0 * cr * 1000
                        boom_len = (2.0 - 1.0) * cr
                        parts += [("boom", P["boom_g_per_m"] * boom_len, 0.5 * (1.0 + 2.0) * cr * 1000), ("tail", P["areal"] * tail_a * 1000, x_t), ("fin", P["areal"] * fin_a * 1000, x_t)]
                    else:
                        boom_len = (1.5 - 1.0) * cr
                        parts += [("boom", P["boom_g_per_m"] * boom_len, 0.5 * (1.0 + 1.5) * cr * 1000), ("fin", P["areal"] * fin_a * 1000, 1.5 * cr * 1000)]
                    ms = sum(p[1] for p in parts)
                    xs = sum(p[1] * p[2] for p in parts) / ms
                    tgt, how = target_cg_mm(wing, sec, kind, cfg, u)
                    for xb_cr in (-0.25, -0.5):
                        xb = xb_cr * cr * 1000
                        mb = V2.ballast_for_target([(p[1], p[2]) for p in parts], tgt, xb) if tgt is not None else None
                        mt = ms + (mb or 0.0)
                        out.append(dict(wing=wing, section=sec, struct_case=case, arch=label, ballast_station_root_chords=xb_cr, ballast_station_mm=xb, target_cg_mm=tgt, target_basis=how,
                                        wing_mass_g=mw, wing_cg_mm=xw, structure_mass_g=ms, structure_cg_mm=xs, ballast_g=mb, all_up_mass_g=mt,
                                        ws_n_m2=(mt / 1000 * G / Sw if mb is not None and mb >= 0 else None), ballast_fraction=(mb / mt if mb is not None else None),
                                        reachable_with_nose_ballast=int(mb is not None and mb >= 0), ballast_gt_25pct=int(mb is not None and mb / mt > 0.25),
                                        parts="; ".join(f"{n} {mm_:.1f}g@{x:.0f}mm" for n, mm_, x in parts)))
    T.write_csv(OUT / "item6_mass_cg_budget.csv", out)
    return out


# ------------------------------------------------------------------ item 7 and table D
ARCH7 = [("pure", None, 0.0, "coupled", "pure"), ("lip", lipcfg(0.20), 10.0, "coupled", "lip 20/10 (a) increment"), ("lip", lipcfg(0.20), 10.0, "replace", "lip 20/10 (b) replace"),
         ("lip", lipcfg(0.20), 10.0, "replace_k", "lip 20/10 (c) replace+k"), ("tail", tailcfg(0.10), -3.0, "coupled", "tail 10% @2.0cr, AVL effectiveness"),
         ("tail", tailcfg(0.05), -3.0, "coupled", "tail 10% @2.0cr, 0.5 effectiveness")]


def item7():
    out = []
    for wing in WINGS:
        for sec in SECS:
            for kind, cfg, u, form, label in ARCH7:
                for ws in WS:
                    rb = W("final", kind, wing, sec, cfg, u, ws, form, 160, "both")
                    for tier in ("N5", "N9", "both"):
                        r = W("final", kind, wing, sec, cfg, u, ws, form, 160, tier)
                        row = dict(wing=wing, section=sec, arch=label, ws=ws, tier=tier)
                        if not r or not r["n_points"]:
                            row.update(window="none")
                            out.append(row)
                            continue
                        row.update(cg_start_mm=round(r["cg_mm_lo"]), cg_end_mm=round(r["cg_mm_hi"]), width_mm=round(r["width_mm"]), window_edge=int(r["touches_lo_edge"] or r["touches_hi_edge"]))
                        c = centre_idx(r)
                        for nc in ((5, 9) if tier == "both" else (int(tier[1:]),)):
                            rr = recidx().get(("final", kind, wing, sec, json.dumps(cfg), u, ws, form, nc, 160))
                            p = pt_at(rr["pts"], c) if rr else None
                            if p:
                                ti = trim_info("final", kind, wing, sec, cfg, u, ws, form, nc, 160, p[1])
                                row.update({f"N{nc}_cl": p[1], f"N{nc}_alpha_deg": ti["alpha_deg"], f"N{nc}_v_m_s": ti["V"]})
                        out.append(row)
    T.write_csv(OUT / "item7_trim_table.csv", out)
    return out


def tableD():
    lines = ["CORRECTED TABLE D (final deck: 20x32, dihedral 3, 6% centre fin / tail + boom fin; SM scan -0.15..+0.60; NACA 0006; both-Ncrit = N5 & N9 intersection; 160 panels).",
             "mm values are windows at trim CL 0.2-0.5 (any evidence); strict = off transition interval; CG mm from root LE rounded to 1 mm.", ""]
    flat = []
    for wing in WINGS:
        lines.append(f"=== {wing} ===")
        for ws in (8, 16):
            lines.append(f" W/S {ws}")
            for kind, cfg, u, form, label in ARCH7:
                sec = "NACA_0006"
                ra = W("final", kind, wing, sec, cfg, u, ws, "avl_only", None, "avl") if kind != "pure" else None
                rr = {t: W("final", kind, wing, sec, cfg, u, ws, form, 160, t) for t in ("N5", "N9", "both")}
                rs = W("final", kind, wing, sec, cfg, u, ws, form, 160, "both", "offti")
                r240 = W("final", kind, wing, sec, cfg, u, ws, form, 240, "both")
                row = dict(wing=wing, ws=ws, arch=label, win_avl_mm=wmm(ra) if ra else 0.0, win_N5_mm=wmm(rr["N5"]), win_N9_mm=wmm(rr["N9"]), win_both_mm=wmm(rr["both"]),
                           win_both_strict_mm=wmm(rs), win_both_240_mm=wmm(r240))
                seg = rr["both"] if rr["both"] and rr["both"]["n_points"] else None
                txt = f"  {label}: AVL-only {row['win_avl_mm']:.0f} | N5 {row['win_N5_mm']:.0f} | N9 {row['win_N9_mm']:.0f} | both {row['win_both_mm']:.0f} mm (strict {row['win_both_strict_mm']:.0f}, 240-panel both {row['win_both_240_mm']:.0f})"
                if seg:
                    row.update(cg_start_mm=round(seg["cg_mm_lo"]), cg_end_mm=round(seg["cg_mm_hi"]), edge=int(seg["touches_lo_edge"] or seg["touches_hi_edge"]))
                    c = centre_idx(seg)
                    txt += f"; CG {row['cg_start_mm']}-{row['cg_end_mm']} mm"
                    for nc in (5, 9):
                        rr_ = recidx().get(("final", kind, wing, sec, json.dumps(cfg), u, ws, form, nc, 160))
                        p = pt_at(rr_["pts"], c) if rr_ else None
                        if p:
                            ti = trim_info("final", kind, wing, sec, cfg, u, ws, form, nc, 160, p[1])
                            row.update({f"N{nc}_cl": p[1], f"N{nc}_alpha": ti["alpha_deg"], f"N{nc}_v": ti["V"], f"N{nc}_margin_glide": -p[2], f"N{nc}_margin_fixed": -p[3]})
                            txt += f"; N{nc}: CL {p[1]:.2f} a {ti['alpha_deg']:.1f} V {ti['V']:.1f} marg {-p[2]:+.2f}/{-p[3]:+.2f}"
                lines.append(txt)
                flat.append(row)
    (OUT / "tableD_corrected.txt").write_text("\n".join(lines), encoding="utf-8")
    T.write_csv(OUT / "tableD_corrected.csv", flat)
    return flat


# ------------------------------------------------------------------ item 8
def item8():
    out = []
    for wing, (ar, sw, tp) in WINGS.items():
        M = mdl("final", "pure", wing, None)
        for span in (0.25, 0.30, 0.40):
            cr, ct = M["root_chord"] * span, M["tip_chord"] * span
            for sec in SECS:
                tc = int(sec[-2:]) / 100
                for ws in (6, 8, 12):
                    row = dict(wing=wing, span_m=span, section=sec, ws=ws, root_chord_mm=1000 * cr, tip_chord_mm=1000 * ct, root_thickness_mm=1000 * cr * tc,
                               tip_thickness_mm=1000 * ct * tc, mass_g=1000 * ws * M["area"] * span * span / G, mac_mm=1000 * M["mac"] * span)
                    for pan in (160, 240):
                        sets = {}
                        pts = {}
                        for nc in (5, 9):
                            cfg = A.Cfg(tabs().clean(sec, nc, pan))
                            curve = A.build_curve_arch(M, span, 0.0, cfg, ws, "coupled")
                            s_, p_ = V2.scan_curve2(curve)
                            sets[nc] = {md: set(s_[md]) for md in V2.MODES}
                            pts[nc] = p_
                        for mode in ("any", "offti"):
                            for tier, ss in (("N5", sets[5][mode]), ("N9", sets[9][mode]), ("both", sets[5][mode] & sets[9][mode])):
                                wd = v2.contiguous_window(sorted(ss), V2.STEP)
                                row[f"p{pan}_{mode}_{tier}_mm"] = (wd["width"] or 0.0) * M["mac"] * span * 1000 if wd["n"] else 0.0
                                if mode == "any" and tier == "both":
                                    row[f"p{pan}_sm_lo"], row[f"p{pan}_sm_hi"] = wd["sm_lo"], wd["sm_hi"]
                                    row[f"p{pan}_edge"] = int(bool(wd["n"] and (wd["sm_lo"] <= V2.SM_MIN + 1e-9 or wd["sm_hi"] >= V2.SM_MAX - 1e-9)))
                    out.append(row)
    T.write_csv(OUT / "item8_pure_trade.csv", out)
    return out


# ------------------------------------------------------------------ item 9
def item9():
    out = []
    wk = OUT / "avl_tipfin"
    for wing, (ar, sw, tp) in WINGS.items():
        for dih in (0, 3):
            w = V2.final_wing(ar, sw, tp, dih)
            Mp = mdl("final", "pure", wing, None)
            xcg = Mp["xnp"] - 0.05 * Mp["mac"]
            w = m._wing(ar, sw, tp, dih, xref=xcg)
            cr = w.root_chord
            opts = [("bare", {}), ("tip fin 4% sym", dict(tip_fin=dict(area_frac=0.04, mode="sym"))), ("tip fin 4% above", dict(tip_fin=dict(area_frac=0.04, mode="above"))),
                    ("centre fin 6% @1.5cr", dict(fin=dict(area_frac=0.06, x_qc=1.5 * cr)))]
            for name, kw in opts:
                deck = V2.deck_full(w, "t", **kw)
                alphas = [round((cl - Mp["cl0"]) / Mp["a_cl"], 3) for cl in (0.15, 0.3, 0.45, 0.6)]
                pts = run_avl(w, "t", alphas, wk / f"{wing}_{dih}_{name.replace(' ', '_').replace('%', 'p').replace('@', '_')}", A.AVL_EXE, deck_text=deck)
                c = [p.cl for p in pts]

                def fit(v):
                    n = len(c); mx, my = sum(c) / n, sum(v) / n
                    sl = sum((x - mx) * (y - my) for x, y in zip(c, v)) / sum((x - mx) ** 2 for x in c)
                    return my - sl * mx, sl
                a0, a1 = fit([p.clb_per_rad for p in pts]); n0, n1 = fit([p.cnb_per_rad for p in pts])
                out.append(dict(wing=wing, dihedral=dih, option=name, cl_beta_CL0p3=a0 + a1 * 0.3, cn_beta_CL0p3=n0 + n1 * 0.3, xnp_mm=1000 * sum(p.xnp_m for p in pts) / len(pts) * B))
    T.write_csv(OUT / "item9_tipfins.csv", out)
    return out


# ------------------------------------------------------------------ extra: form-robust lip window and lip-vs-tail ordering
def robust_window(deck, wing, sec, cf, ang, ws, tier="both", pan=160, mode="any"):
    """Intersection (mm from root LE) of the lip windows of forms (a), (b), (c); None if empty."""
    los, his = [], []
    for form in ("coupled", "replace", "replace_k"):
        r = W(deck, "lip", wing, sec, lipcfg(cf), ang, ws, form, pan, tier, mode)
        if not r or not r["n_points"]:
            return None
        los.append(r["cg_mm_lo"]); his.append(r["cg_mm_hi"])
    lo, hi = max(los), min(his)
    return (lo, hi) if hi >= lo else None


def item2b():
    out = []
    for deck in ("final",):
        for cf, ang in CHOSEN_LIP:
            for wing in WINGS:
                for sec in SECS:
                    for ws in WS:
                        rw = robust_window(deck, wing, sec, cf, ang, ws)
                        row = dict(deck=deck, lip=f"{int(cf*100)}/{ang:g}", wing=wing, section=sec, ws=ws)
                        for form, tag in (("coupled", "a"), ("replace", "b"), ("replace_k", "c")):
                            r = W(deck, "lip", wing, sec, lipcfg(cf), ang, ws, form, 160, "both")
                            row[f"{tag}_cg_lo"], row[f"{tag}_cg_hi"] = (round(r["cg_mm_lo"]), round(r["cg_mm_hi"])) if r and r["n_points"] else (None, None)
                        row["robust_lo_mm"], row["robust_hi_mm"] = (round(rw[0]), round(rw[1])) if rw else (None, None)
                        row["robust_width_mm"] = (rw[1] - rw[0]) if rw else 0.0
                        out.append(row)
    T.write_csv(OUT / "item2b_form_robust_lip_windows.csv", out)
    return out


def order():
    """Lip (a)/(b)/(c) vs tail (AVL eff, 0.5 eff) both-Ncrit windows per (wing, section, W/S) on the final deck (lip 20/10 outboard; tail 10% @2.0cr -3)."""
    out = []
    for wing in WINGS:
        for sec in SECS:
            for ws in WS:
                row = dict(wing=wing, section=sec, ws=ws)
                for form, tag in (("coupled", "lip_a"), ("replace", "lip_b"), ("replace_k", "lip_c")):
                    row[tag] = wmm(W("final", "lip", wing, sec, lipcfg(0.20), 10.0, ws, form, 160, "both"))
                row["tail_avl_eff"] = wmm(W("final", "tail", wing, sec, tailcfg(0.10), -3.0, ws, "coupled", 160, "both"))
                row["tail_half_eff"] = wmm(W("final", "tail", wing, sec, tailcfg(0.05), -3.0, ws, "coupled", 160, "both"))
                row["pure"] = wmm(W("final", "pure", wing, sec, None, 0.0, ws, "coupled", 160, "both"))
                out.append(row)
    T.write_csv(OUT / "order_lip_vs_tail.csv", out)
    cmp_ = {}
    for a, b in (("lip_a", "tail_avl_eff"), ("lip_b", "tail_avl_eff"), ("lip_c", "tail_avl_eff"), ("lip_a", "tail_half_eff"), ("lip_b", "tail_half_eff"), ("lip_c", "tail_half_eff")):
        cmp_[f"{a} > {b}"] = sum(1 for r in out if r[a] > r[b] + 0.5)
        cmp_[f"{a} < {b}"] = sum(1 for r in out if r[a] < r[b] - 0.5)
        cmp_[f"{a} ~ {b} (within 0.5 mm)"] = sum(1 for r in out if abs(r[a] - r[b]) <= 0.5)
    cmp_["cases"] = len(out)
    (OUT / "order_summary.json").write_text(json.dumps(cmp_, indent=1))
    return cmp_


def main():
    what = sys.argv[1:] or ["all"]
    fn = {"i1": item1, "i2": item2, "i3": lambda: (item3("legacy"), item3("final")), "i4": lambda: (item4i(), item4ii()), "i5": item5, "i6": item6, "i7": item7,
          "d": tableD, "i8": item8, "i9": item9, "i2b": item2b, "order": order}
    for w in (list(fn) if what == ["all"] else what):
        print("running", w)
        fn[w]()


if __name__ == "__main__":
    main()
