"""Crow-like proportion rework (v3).  Question: can the crow-like (BC) outline meet ALL the v2 selection criteria, including nose ballast under 20% of the all-up
mass, if only the tail length, the tail spread, the nose length ahead of the wing (= the wing's fore-aft position on the body) and the hand sweep are varied, with the
outer tail edges folded up as fins allowed in place of the ventral fin?  Aspect ratio stays 5 (CITED: magpie 4.97, jackdaw about 6).

Imports (never edits) bird_outline_v2.py, bird_outline.py, flat_test.py.  Writes ONLY to --out (default results/runs/rigid_glider/bird_outline_v3_crow/).
Screening AVL uses the v2 panelling (10 x 80); the decision and every reported number use the finer panelling (16 x 128); the panelling sensitivity of the
chosen outline is reported.  AVL runs are serial.  Same honesty rules as v2: AVL is inviscid, outline-only, body ignored; nothing here says an outline flies.
    py -B rigid_glider/bird_outline_v3_crow.py [--out DIR] [--no-pdf] [--quick]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import itertools
import json
import math
import subprocess
from collections import Counter
from pathlib import Path

import bird_outline as bo
import bird_outline_v2 as v2
import flat_test as ft

ROOT = ft.ROOT
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/bird_outline_v3_crow"
FAM = "BC"
HELD_ON = {"curved_te": 1, "root_blend": 1, "tip_round": 1, "neck": 1}          # the v2 BC final's other features, held fixed (none of them is a proportion)
BEFORE = dict(tail_len_frac=0.20, tail_spread_frac=0.48, nose_frac=0.11, hand_sweep_deg=20.0)   # the v2 BC final's values of the four varied proportions
BEFORE_ON = dict(HELD_ON, spread=1, nose=1)
BEFORE_BEND, BEFORE_FIN = 2.0, 0.03
TAIL_LENS = (0.06, 0.08, 0.12, 0.16, 0.20)
SPREADS = (0.20, 0.30, 0.40, 0.48, 0.56)
SWEEPS = (20.0, 30.0, 40.0, 50.0)
NOSES = (0.07, 0.11, 0.16, 0.22, 0.30)
EDGES = (None, (v2.EDGE_FIN_FRAC, 30.0), (v2.EDGE_FIN_FRAC, 60.0))
BENDS_SCREEN = (1.0, 1.5, 2.0, 3.0, 4.0, 5.0)
PANEL_SCREEN = (10, 80)
PANEL_FINE = (16, 128)
PANEL_CHECK = ((8, 64), (10, 80), (16, 128))
N_EXACT_MAX = 14
N_EXACT_WANT = 3


def set_panels(nc, ns):
    v2.AVL_NCHORD, v2.AVL_NSPAN = nc, ns


def params(tl, sp, hs, nz, held=True):
    p = v2.feature_params(FAM, HELD_ON if held else {})
    p.update(tail_len_frac=tl, tail_spread_frac=sp, hand_sweep_deg=hs, nose_frac=nz)
    return p


class Runner:
    def __init__(self, workdir: Path, avl=True):
        self.workdir = Path(workdir)
        self.avl = avl
        self.path = self.workdir / "cache_v3.json"
        self.cache, self.n = {}, 0
        if self.path.is_file():
            try:
                d = json.loads(self.path.read_text())
                if d.get("version") == "v3.1":
                    self.cache = {k: v2.Runner._revive(v) for k, v in d["items"].items()}
            except Exception:
                self.cache = {}

    def save(self):
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(dict(version="v3.1", items=self.cache)))

    def run(self, tl, sp, hs, edge, bend, vfin=None, panels=PANEL_SCREEN, nz=0.11, held=True):
        key = json.dumps([tl, sp, hs, edge, bend, vfin, panels, held])
        if key in self.cache:
            return self.cache[key]
        if not self.avl:
            raise RuntimeError("AVL disabled, not cached: " + key)
        set_panels(*panels)
        g = v2.build_geometry2(params(tl, sp, hs, nz, held))
        vf = bo.vfin_for(g, vfin) if vfin else None
        tag = "k%05d" % (len(self.cache) + 1)
        r = v2.run_case2(g, v2.RUN_SPAN_MM, bend, self.workdir / tag, tag, vf, edge)
        ftp = self.workdir / tag / "cl03" / "point_00.ft"
        r["e_far"] = None
        if ftp.is_file():
            m = [l for l in ftp.read_text(errors="replace").splitlines() if "CYff" in l]
            if m and "e =" in m[0]:
                r["e_far"] = float(m[0].split("e =")[1].split()[0])
        self.n += 1
        r = v2.Runner._revive(json.loads(json.dumps(r)))
        self.cache[key] = r
        if self.n % 10 == 0:
            self.save()
        return r


def make_row(p, on, bend, res, g, tip_base, n_fins, label):
    row = v2.candidate_row(FAM, on, bend, res, g)
    row.update(n_features=len(on), n_fins=n_fins, n_justified=0, appearance_cost_frac=0.0, label=label)
    row["fails"] = v2.criteria_failures(row, tip_base)
    ok = math.isfinite(row["ballast_frac_printed"]) and math.isfinite(row["ballast_frac_foam"])
    row["margin"] = v2.margin_of(row, tip_base) if ok else -9.0
    row["ballast_max"] = max(row["ballast_frac_printed"], row["ballast_frac_foam"]) if ok else 9.0
    return row


def violation(r):
    """Sum of normalised criterion violations (0 = passes everything); used to pick the CLOSEST candidate when nothing passes."""
    v = max(0.0, r["ballast_max"] - v2.BALLAST_MAX_FRAC) / v2.BALLAST_MAX_FRAC
    v += max(0.0, v2.SM_WINDOW[0] - r["sm03"]) / v2.SM_WINDOW[0] + max(0.0, r["sm03"] - v2.SM_WINDOW[1]) / v2.SM_WINDOW[1]
    v += max(0.0, v2.CN_BETA_MIN - r["cn_beta"]) / v2.CN_BETA_MIN + max(0.0, r["cl_beta"] - v2.CL_BETA_MAX) / abs(v2.CL_BETA_MAX)
    v += 0.0 if r["neck_ok"] else 1.0
    v += 0.0 if r["reachable_all"] else 1.0
    v += 0.0 if r["valid_outline"] else 1.0
    v += 1.0 if r["x_cg03_span"] < r["nose_x_span"] + v2.NOSE_CLEAR_SPAN else 0.0
    return v


def tradeoff(rows):
    """The two corners that show which criteria block: (A) fail ONLY ballast: the least ballast reached; (B) fail ONLY the static-margin window: the largest margin reached with ballast ok."""
    def only(r, key):
        return r["fails"] and all(f.startswith(key) for f in r["fails"])
    A = [r for r in rows if only(r, "ballast")]
    B = [r for r in rows if only(r, "static")]
    return (min(A, key=lambda r: r["ballast_max"]) if A else None, max(B, key=lambda r: r["sm03"]) if B else None, len(A), len(B))


def edge_name(e):
    return "none" if e is None else f"tail edges up {e[1]:g} deg"


def screening(R, quick=False):
    base_p = v2.base_params(FAM)
    b0 = R.run(0.20, 0.38, 20.0, None, 0.0, held=False)                                  # v1 BC outline (tail length 0.20, spread 0.38, sweep 20): the tip-loading reference
    b5 = R.run(0.20, 0.38, 20.0, None, 5.0, held=False)
    # note: the v1 baseline has no v2 features; its tip loading is the 'v1 value' of the criterion, as in v2
    tip_base = {b: v2.interp_bend(b0, b5, b)["tip_loading"] for b in BENDS_SCREEN}
    tls, sps, hss = (TAIL_LENS, SPREADS, SWEEPS) if not quick else ((0.08, 0.20), (0.20, 0.48), (20.0, 40.0))
    rows = []
    for tl, sp, hs, edge in itertools.product(tls, sps, hss, EDGES):
        r0, r5 = R.run(tl, sp, hs, edge, 0.0), R.run(tl, sp, hs, edge, 5.0)
        for nz in NOSES:
            p = params(tl, sp, hs, nz)
            g = v2.build_geometry2(p)
            for bend in BENDS_SCREEN:
                res = v2.retrim(v2.interp_bend(r0, r5, bend), g["mac"])
                row = make_row(p, BEFORE_ON, bend, res, g, tip_base[bend], 0 if edge is None else 1, "screening")
                row.update(tl=tl, sp=sp, hs=hs, nz=nz, edge=edge)
                rows.append(row)
    R.save()
    return rows, tip_base


def exact_stage(R, ranked, log=print):
    out, done = [], []
    seen = set()
    for r in ranked:
        key = (r["tl"], r["sp"], r["hs"], r["nz"], r["edge"], r["bend"])
        if key in seen:
            continue
        seen.add(key)
        if len(done) >= N_EXACT_MAX or len(out) >= N_EXACT_WANT:
            break
        res = R.run(r["tl"], r["sp"], r["hs"], r["edge"], r["bend"], panels=PANEL_FINE)
        b0 = R.run(0.20, 0.38, 20.0, None, r["bend"], panels=PANEL_FINE, held=False)
        g = r["g"]
        row = make_row(None, BEFORE_ON, r["bend"], res, g, b0["tip_loading"], r["n_fins"], "exact (16x128)")
        row.update(tl=r["tl"], sp=r["sp"], hs=r["hs"], nz=r["nz"], edge=r["edge"])
        done.append(row)
        log(f"  exact {key}: {'PASS' if not row['fails'] else row['fails']}")
        if not row["fails"]:
            out.append(row)
    return out, done


def overlay_svg(g_before, g_after, k=250.0):
    w, h = 760, 420
    sx = 1.35
    x0 = 380
    ytop = 40
    def poly(g, color, fill):
        pts = " ".join(f"{x0 + y * k * sx:.1f},{ytop + (x * k + 40) * sx:.1f}" for x, y in g["full_outline"])
        return f"<polygon points='{pts}' fill='{fill}' fill-opacity='0.35' stroke='{color}' stroke-width='1.6'/>"
    return (f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {w} {h}' width='{w}' height='{h}' style='max-width:100%;background:#14181d;border:1px solid #333c46;border-radius:6px'>"
            + poly(g_before, "#9aa7b4", "#9aa7b4") + poly(g_after, "#e89a55", "#e89a55")
            + f"<text x='14' y='24' font-family='Segoe UI,Arial' font-size='13' fill='#9aa7b4'>before (v2 BC final) grey, after (v3) orange; top view, nose up, 250 mm span, mm scale 1.35 px/mm</text></svg>")


def run(out: Path = OUT_DEFAULT, avl=True, pdf=True, quick=False, log=print):
    out.mkdir(parents=True, exist_ok=True)
    R = Runner(out / "avl_work", avl)
    log("screening (10x80) ...")
    rows, tip_base = screening(R, quick)
    ranked, failers = v2.select(rows)
    cnt = Counter(f.split(" ")[0] for r in rows for f in r["fails"])
    log(f"{len(rows)} candidates, {len(ranked)} pass screening; failure counts {dict(cnt)}")
    finals, done = exact_stage(R, ranked, log) if ranked else ([], [])
    R.save()
    winner = None
    if finals:
        finals.sort(key=v2.rank_key)
        winner = finals[0]
        log(f"WINNER: tail {winner['tl']} spread {winner['sp']} nose {winner['nz']} sweep {winner['hs']} edge {winner['edge']} bend {winner['bend']}")
        close = winner
    else:
        pool = done + rows
        close = min(pool, key=lambda r: (violation(r), len(r["fails"])))
        log(f"NOTHING PASSES. closest: {close['features']} tl {close['tl']} sp {close['sp']} nz {close['nz']} hs {close['hs']} edge {close['edge']} bend {close['bend']}: {close['fails']}")
        # re-run the closest at the fine panelling so its numbers are comparable
        res = R.run(close["tl"], close["sp"], close["hs"], close["edge"], close["bend"], panels=PANEL_FINE)
        b0 = R.run(0.20, 0.38, 20.0, None, close["bend"], panels=PANEL_FINE, held=False)
        c2 = make_row(None, BEFORE_ON, close["bend"], res, close["g"], b0["tip_loading"], close["n_fins"], "exact (16x128)")
        c2.update(tl=close["tl"], sp=close["sp"], hs=close["hs"], nz=close["nz"], edge=close["edge"])
        close = c2
    # ---- before (v2 BC final) at the fine panelling
    set_panels(*PANEL_FINE)
    g_b = v2.build_geometry2(v2.feature_params(FAM, BEFORE_ON))
    res_b = R.run(BEFORE["tail_len_frac"], BEFORE["tail_spread_frac"], BEFORE["hand_sweep_deg"], None, BEFORE_BEND, vfin=BEFORE_FIN, panels=PANEL_FINE)
    b0 = R.run(0.20, 0.38, 20.0, None, BEFORE_BEND, panels=PANEL_FINE, held=False)
    before = make_row(None, BEFORE_ON, BEFORE_BEND, res_b, g_b, b0["tip_loading"], 1, "before (v2 final, 3% ventral fin)")
    # ---- one-at-a-time reversal: what each changed proportion buys (fine panelling)
    subj = close
    bs = R.run(0.20, 0.38, 20.0, None, subj["bend"], panels=PANEL_FINE, held=False)
    rev = []
    for name, key_ in (("tail length", "tl"), ("tail spread", "sp"), ("hand sweep", "hs"), ("nose length", "nz"), ("edge fold", "edge")):
        args = dict(tl=subj["tl"], sp=subj["sp"], hs=subj["hs"], nz=subj["nz"], edge=subj["edge"])
        bef = dict(tl=BEFORE["tail_len_frac"], sp=BEFORE["tail_spread_frac"], hs=BEFORE["hand_sweep_deg"], nz=BEFORE["nose_frac"], edge=None)
        if args[key_] == bef[key_]:
            continue
        args[key_] = bef[key_]
        res = R.run(args["tl"], args["sp"], args["hs"], args["edge"], subj["bend"], panels=PANEL_FINE)
        g = v2.build_geometry2(params(args["tl"], args["sp"], args["hs"], args["nz"]))
        row = make_row(None, BEFORE_ON, subj["bend"], res, g, bs["tip_loading"], 0 if args["edge"] is None else 1, "reverted " + name)
        rev.append((name, bef[key_], subj[key_] if key_ != "edge" else edge_name(subj["edge"]), row))
    # ---- panelling sensitivity of the chosen / closest outline and of the before outline
    panel = []
    for lab, (tl, sp, hs, edge, vf, bend, nz) in (("after", (subj["tl"], subj["sp"], subj["hs"], subj["edge"], None, subj["bend"], subj["nz"])),
                                                 ("before", (0.20, 0.48, 20.0, None, BEFORE_FIN, BEFORE_BEND, 0.11))):
        for nc, ns in PANEL_CHECK:
            r = R.run(tl, sp, hs, edge, bend, vfin=vf, panels=(nc, ns), nz=nz)
            panel.append(dict(outline=lab, bend=bend, nc=nc, ns=ns, np=r["np_pct_mac"], cm0=r["cm0"], cn=r["cn_beta"], cl=r["cl_beta"], e=r["e_far"], tail=r["tail_lift_frac"]))
    R.save()
    # ---- builds (winner only)
    info = None
    files = []
    pdf_name = None
    if winner is not None:
        a = _assemble(R, winner)
        orig = bo.drawing_svg_bird
        bo.drawing_svg_bird = lambda g, k, label, *aa: orig(g, k, label.replace(" v2", " v3"), *aa)
        try:
            info = v2.build_final(FAM, a, out)
        finally:
            bo.drawing_svg_bird = orig
        files = info["files"]
        if pdf and info["template_pages"]:
            pdf_name = "bird_v3_crow_templates.pdf"
            subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(out / pdf_name)] + [str(p) for p in info["template_pages"]], check=True)
    _write_csv(out, rows, done, before, rev, panel, close, winner)
    _page(out, before, close, winner, rev, panel, rows, cnt, info, files, pdf_name, g_b, quick)
    return dict(winner=winner, close=close, before=before, rows=rows, panel=panel, info=info, pdf=pdf_name)


def _assemble(R, w):
    """Minimal 'analysis' dict so v2.build_final can be reused: exact rows at the selection bends for the winner's geometry."""
    bends = {}
    base_g = w["g"]
    for b in v2.SELECTION_BENDS:
        res = R.run(w["tl"], w["sp"], w["hs"], w["edge"], b, panels=PANEL_FINE)
        b0 = R.run(0.20, 0.38, 20.0, None, b, panels=PANEL_FINE, held=False)
        row = make_row(None, BEFORE_ON, b, res, base_g, b0["tip_loading"], w["n_fins"], "exact")
        bends[("final", b)] = row
    final = bends[("final", w["bend"])]
    return dict(final=final, bends=bends, waive=())


def _write_csv(out, rows, done, before, rev, panel, close, winner):
    def line(r):
        return [r["label"], r.get("tl"), r.get("sp"), r.get("nz"), r.get("hs"), edge_name(r.get("edge")), r["bend"], f"{r['cn_beta']:.5f}", f"{r['cl_beta']:.5f}", f"{100 * r['sm03']:.2f}",
                f"{r['x_cg03_mm']:.2f}", f"{r['ballast_frac_printed']:.4f}", f"{r['ballast_frac_foam']:.4f}", f"{r['tip_loading']:.4f}", r["neck_ok"], "yes" if not r["fails"] else "no", " | ".join(r["fails"])]
    hdr = ["stage", "tail_len_frac", "tail_spread_frac", "nose_frac", "hand_sweep_deg", "tail_edges", "bend_deg", "Cn_beta", "Cl_beta", "static_margin_pct", "trim_CG_mm_250", "ballast_frac_250", "ballast_frac_400",
           "tip_loading", "neck_ok", "passes", "failures"]
    v2.write_csv(out / "candidates.csv", hdr, [line(r) for r in rows])
    v2.write_csv(out / "exact_checks.csv", hdr, [line(r) for r in done] + [line(before)])
    v2.write_csv(out / "reversals.csv", ["changed proportion", "before value", "after value", "result of reverting it alone: Cn_beta", "Cl_beta", "SM %", "ballast frac 250", "ballast frac 400", "tip loading", "fails"],
                 [[n, b, a, f"{r['cn_beta']:.5f}", f"{r['cl_beta']:.5f}", f"{100 * r['sm03']:.2f}", f"{r['ballast_frac_printed']:.4f}", f"{r['ballast_frac_foam']:.4f}", f"{r['tip_loading']:.4f}", " | ".join(r["fails"])] for n, b, a, r in rev])
    v2.write_csv(out / "panelling_check.csv", ["outline", "bend_deg", "chordwise", "spanwise", "NP_pct_MAC", "CM0", "Cn_beta", "Cl_beta", "span_eff_far_field", "tail_lift_over_weight"],
                 [[p["outline"], p["bend"], p["nc"], p["ns"], f"{p['np']:.2f}", f"{p['cm0']:.5f}", f"{p['cn']:.5f}", f"{p['cl']:.5f}", p["e"], f"{p['tail']:.4f}"] for p in panel])


def _page(out, before, close, winner, rev, panel, rows, cnt, info, files, pdf_name, g_b, quick):
    ranked_s = [r for r in rows if not r['fails']]
    h = html.escape
    sub = winner or close
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Crow-like outline, proportion rework (v3)</title><style>{ft.CSS}\n.scroll{{overflow-x:auto}}table.t td,table.t th{{padding:4px 8px;font-size:13.5px;vertical-align:top}}"
         f".warn{{background:#2c2418;border:2px solid #e89a55;padding:10px 16px}}</style></head><body><main>"]
    P.append("<h1>Crow-like outline: can the proportions meet every criterion?</h1>")
    P.append("<p class='note'>Outline-only AVL model plus arithmetic. Nothing here is a flight result. Tags as in version 2: GEOMETRY, AVL, ARITHMETIC, JUDGEMENT, ASSUMED, CITED. AVL is inviscid and ignores the body; it says nothing about stall, Reynolds number, or whether a build pulls out. "
             "Aspect ratio is held at 5 (CITED: magpie 4.97, jackdaw about 6). Varied: tail length, tail spread, nose length ahead of the wing, hand sweep, and the tail edges folded up in place of the ventral fin. All numbers below are at the finer panelling (16 x 128 per half wing); screening used 10 x 80."
             + (" <b>QUICK MODE: reduced search.</b>" if quick else "") + "</p>")
    if winner:
        P.append("<p class='warn'><b>One outline passes every criterion, including nose ballast under 20% at both sizes.</b> A proposal for Kai, not a decision.</p>")
    else:
        P.append(f"<p class='warn'><b>No outline in the search passes every criterion.</b> Closest candidate: tail length {close['tl']:g}, spread {close['sp']:g}, nose {close['nz']:g}, hand sweep {close['hs']:g}, {edge_name(close['edge'])}, bend {close['bend']:g}. It fails: {h('; '.join(close['fails']))}.</p>")
    A_, B_, nA, nB = tradeoff(rows)
    if not winner:
        def one(r):
            return f"tail length {r['tl']:g}, spread {r['sp']:g}, nose {r['nz']:g}, sweep {r['hs']:g}, {edge_name(r['edge'])}, bend {r['bend']:g}"
        P.append("<h2>Which criteria block</h2><p>"
                 + (f"<b>Ballast against static margin.</b> {nA} candidates fail only the ballast limit; the least ballast any of them reaches is {100 * A_['ballast_max']:.1f}% of the all-up mass ({one(A_)}; static margin {100 * A_['sm03']:.1f}%, screening values). " if A_ else "No candidate fails only the ballast limit. ")
                 + (f"{nB} candidates meet ballast and every other criterion but fail the static-margin window; the largest margin any of them reaches is {100 * B_['sm03']:.1f}% against the {100 * v2.SM_WINDOW[0]:g}% floor ({one(B_)}). " if B_ else "")
                 + "The reason in the model: a margin of 8% or more needs a tail bend, the bend moves the trim CG forward of the neutral point, and the sheet's own CG is behind it, so the difference is ballast. Within the searched ranges the two limits meet at about an 8% margin and about 21% ballast. "
                 + (f"{len(ranked_s)} candidates passed everything at the screening panelling; none survived the re-run at the finer panelling. " if ranked_s is not None else "")
                 + "The gap that remains (about one point of ballast, a few per cent of the tip-loading indicator) is smaller than the panelling spread in the table below, so read this as 'no outline passes in this search', not as a proof that none exists.</p>")
    P.append("<h2>Before and after</h2>" + overlay_svg(g_b, sub["g"]))
    rowsx = []

    def both(label, fn):
        rowsx.append(f"<tr><td>{label}</td><td class='n'>{fn(before)}</td><td class='n'>{fn(sub)}</td></tr>")
    both("tail length / span", lambda r: f"{r['g']['Lt']:.3f}")
    both("tail spread / span", lambda r: f"{r['g']['St']:.3f}")
    both("nose ahead of root LE, mm (250)", lambda r: f"{-r['g']['nose_x'] * 250:.1f}")
    both("hand sweep, deg", lambda r: f"{r['g']['p']['hand_sweep_deg']:g}")
    both("yaw fix", lambda r: "ventral fin 3%" if r is before else edge_name(r.get("edge")))
    both("tail bend, deg", lambda r: f"{r['bend']:g}")
    both("aspect ratio", lambda r: f"{r['ar']:.2f}")
    both("neutral point, % MAC [AVL]", lambda r: f"{r['np_pct_mac']:.1f}")
    both("static margin at CL 0.3, % [AVL]", lambda r: f"{100 * r['sm03']:.1f}")
    both("trim CG, mm aft of root LE (250) [AVL]", lambda r: f"{r['x_cg03_mm']:.1f}")
    both("Cn_beta / Cl_beta per rad [AVL]", lambda r: f"{r['cn_beta']:+.4f} / {r['cl_beta']:+.4f}")
    both("tip loading indicator [AVL, indicator only]", lambda r: f"{r['tip_loading']:.2f}")
    both("all-up mass, ballast: printed 250 [ARITHMETIC]", lambda r: f"{r['sizes']['printed']['M']:.1f} g, {r['sizes']['printed']['ballast_g']:.1f} g ({100 * r['ballast_frac_printed']:.0f}%)")
    both("all-up mass, ballast: foam 400 [ARITHMETIC]", lambda r: f"{r['sizes']['foam']['M']:.1f} g, {r['sizes']['foam']['ballast_g']:.1f} g ({100 * r['ballast_frac_foam']:.0f}%)")
    both("wing loading N/m&sup2; (250) [ARITHMETIC]", lambda r: f"{r['ws_printed']:.1f}")
    both("criteria not met", lambda r: h("; ".join(r["fails"]) or "none"))
    P.append("<div class='scroll'><table class='t'><tr><th></th><th>before: v2 crow-like final</th><th>after: " + ("v3 winner" if winner else "closest candidate (no winner)") + "</th></tr>" + "".join(rowsx) + "</table></div>")
    P.append("<h2>What changed and why</h2><p>Each row reverts ONE proportion of the " + ("winner" if winner else "closest candidate") + " to its before value and re-runs AVL (finer panelling) at the same bend, so the effect of that change alone is visible.</p>")
    rr = [f"<tr><td>{n}</td><td>{h(str(b))}</td><td>{h(str(a))}</td><td class='n'>{r['cn_beta']:+.4f}</td><td class='n'>{100 * r['sm03']:.1f}%</td><td class='n'>{100 * r['ballast_frac_printed']:.0f}% / {100 * r['ballast_frac_foam']:.0f}%</td><td>{h('; '.join(r['fails']) or 'passes')}</td></tr>" for n, b, a, r in rev]
    P.append("<div class='scroll'><table class='t'><tr><th>changed</th><th>before</th><th>after</th><th>Cn_beta if reverted</th><th>SM if reverted</th><th>ballast 250 / 400 if reverted</th><th>what the reverted outline fails</th></tr>" + "".join(rr) + "</table></div>")
    P.append("<h2>How the search went</h2><p>" + f"{len(rows)} candidates ({len(TAIL_LENS)} tail lengths x {len(SPREADS)} spreads x {len(SWEEPS)} sweeps x {len(EDGES)} yaw options x {len(NOSES)} nose lengths x {len(BENDS_SCREEN)} bends; the bends other than 0 and 5 come from the model's linearity in tail incidence, and the best candidates were re-run exactly). Failures per criterion, counting every failed criterion of every candidate: "
             + ", ".join(f"{k} {v}" for k, v in cnt.most_common()) + ". Every candidate with its reasons: <a href='candidates.csv'>candidates.csv</a>; the exact re-runs: <a href='exact_checks.csv'>exact_checks.csv</a>.</p>")
    P.append("<h2>Panelling sensitivity of this outline</h2>")
    P.append("<div class='scroll'><table class='t'><tr><th>outline</th><th>bend</th><th>panels (chord x span)</th><th>NP % MAC</th><th>CM0</th><th>Cn_beta</th><th>Cl_beta</th><th>e (far-field, AVL)</th><th>tail lift / weight</th></tr>"
             + "".join(f"<tr><td>{p['outline']}</td><td>{p['bend']:g}</td><td>{p['nc']} x {p['ns']}</td><td class='n'>{p['np']:.1f}</td><td class='n'>{p['cm0']:+.4f}</td><td class='n'>{p['cn']:+.4f}</td><td class='n'>{p['cl']:+.4f}</td><td class='n'>{p['e']}</td><td class='n'>{p['tail']:+.3f}</td></tr>" for p in panel) + "</table></div>")
    bp = {p["nc"]: p for p in panel if p["outline"] == "before"}
    P.append(f"<p><b>The before outline moved too.</b> At the finer panelling its neutral point is {bp[16]['np']:.1f}% MAC against {bp[10]['np']:.1f}% at the version-2 panelling (10 x 80), so its ballast is {100 * before['ballast_frac_printed']:.0f}% here against 36% in the version-2 report. The comparison in the table above uses the finer panelling for both columns.</p>")
    if not winner:
        P.append(f"<p class='warn'><b>The closest candidate is not very crow-like.</b> Its nose is {-sub['g']['nose_x'] * 250:.0f} mm ahead of the wing root at 250 mm span (the largest value in the search, {max(NOSES):g} of the span), its tail is short ({sub['g']['Lt'] * 250:.0f} mm) and broad, and its hand sweep is {sub['hs']:g} deg. The best candidates sit at the edges of the searched ranges, so a wider search might do better; I did not run one.</p>")
    P.append("<p>Same warning as version 2: the tail lies in the wing's own wake plane, so the neutral point and the tail lift move with the panelling while CM0, Cn_beta and Cl_beta hold. The static margin used in the criteria comes from CM0 and the neutral point, so it inherits part of that spread.</p>")
    if info:
        P.append("<h2>Builds</h2><ul>" + "".join(f"<li>{x}</li>" for x in files) + "</ul>" + (f"<p>Templates (1:1 Letter): <a href='{pdf_name}'>{pdf_name}</a></p>" if pdf_name else ""))
        if winner and winner["edge"]:
            hs_ = winner["g"]["St"] * 250 / 2
            P.append(f"<p class='warn'><b>Not in the files:</b> the winner folds the outer {100 * v2.EDGE_FIN_FRAC:g}% of each side of the fan up by {winner['edge'][1]:g} deg. The fold line runs parallel to the centreline at {hs_ * (1 - v2.EDGE_FIN_FRAC):.1f} mm from it (250 mm size; scale by span/250). The templates, DXF and STL do not draw this fold and the STL has no scored hinge there; add one by hand.</p>")
    P.append("<p>Files: <a href='reversals.csv'>reversals.csv</a>, <a href='panelling_check.csv'>panelling_check.csv</a>. Unverified: nothing here was flown or built; the structure numbers are screening arithmetic as in version 2.</p></main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--no-pdf", action="store_true")
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    R = run(a.out, True, not a.no_pdf, a.quick)
    w = R["winner"]
    print("WINNER" if w else "NO WINNER", {k: R["close"][k] for k in ("tl", "sp", "nz", "hs", "edge", "bend")}, R["close"]["fails"])


if __name__ == "__main__":
    main()
