"""AVL stages of the reflex extension study. Output: results/runs/rigid_glider/reflex_ext/.

Run:  py rigid_glider/reflex_ext_run.py [repro item1 item2 item3 item4 item6 | all] [--force]
Stages are cached as JSON; --force re-runs. Never touches results/runs/.../reflex/.
"""

from __future__ import annotations

import csv
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path

from geometry import Wing
from reflex_geometry import Reflex
from reflex_ext_avl import (OUT, ROOT, case, load_configs, neutral_point,
                            wing_at_margin, write_csv, CSV_KEYS)

BAND_CL = (0.2, 0.5)
BAND_SM = (0.05, 0.15)
OLD = ROOT / "results/runs/rigid_glider/reflex"
WIDEN = None  # set in main from config


def tag(x: float) -> str:
    return f"{x:g}".replace(".", "p").replace("-", "m")


def run_tasks(tasks: list[dict], flight, workers: int) -> list[dict]:
    """tasks: dict(name, wing, feature, meta). Returns rows (meta merged), same order."""
    def one(t):
        r = case(t["name"], t["wing"], t["feature"], flight, OUT, widen_probe=WIDEN)
        r.update(t.get("meta", {}))
        return r
    with ThreadPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(one, tasks))
    return rows


def save(stage: str, rows: list[dict], extra: dict | None = None):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{stage}.json").write_text(json.dumps({"rows": rows, **(extra or {})}, indent=1))
    write_csv(OUT / f"{stage}.csv", rows, CSV_KEYS + [k for k in ("group", "tol_kind", "tol_error") if any(k in r for r in rows)])


def load(stage: str):
    p = OUT / f"{stage}.json"
    return json.loads(p.read_text()) if p.is_file() else None


def passes(r: dict) -> bool:
    return (r.get("trim_cl") is not None and BAND_CL[0] <= r["trim_cl"] <= BAND_CL[1]
            and BAND_SM[0] <= r["static_margin_mac"] <= BAND_SM[1])


def bases(base_cfg, ext, wing, flight):
    """Neutral point per sweep (same probe as the original study)."""
    xnp = {}
    for sweep in ext["item1"]["sweeps_deg"]:
        w = replace(wing, quarter_chord_sweep_deg=sweep)
        xnp[sweep] = (w, neutral_point(w, flight))
    return xnp


def stage_repro(ctx):
    base_cfg, ext, wing, flight, xnp = ctx
    w0, x0 = xnp[25]
    w = wing_at_margin(w0, x0, 0.10)
    tasks = [{"name": "repro_s25_full_c10_a3", "wing": w, "feature": Reflex(.1, 3, "full")},
             {"name": "repro_s25_outer_c10_a4", "wing": w, "feature": Reflex(.1, 4, "outer")}]
    rows = run_tasks(tasks, flight, 2)
    old = {r["name"]: r for r in csv.DictReader((OLD / "static.csv").open())}
    pairs = {"repro_s25_full_c10_a3": "s25_full_c10_a3", "repro_s25_outer_c10_a4": "s25_outer_c10_a4"}
    keymap = {"trim_cl": "trim_cl", "trim_alpha_deg": "trim_alpha_deg", "static_margin_mac": "static_margin_mac",
              "cma_per_rad": "cma_per_rad", "clb_per_rad": "clb_per_rad", "cnb_per_rad": "cnb_per_rad",
              "cg_x_m": "cg_x_m", "speed_m_s": "speed_m_s", "cdi_avl": "cdi_avl", "trim_cl_roll": "trim_cl_roll"}
    comp = []
    for r in rows:
        o = old[pairs[r["name"]]]
        for k in keymap:
            comp.append({"case": pairs[r["name"]], "field": k, "stored": float(o[k]), "ext": r[k],
                         "abs_diff": abs(r[k] - float(o[k]))})
    save("repro", rows, {"comparison": comp})
    with (OUT / "repro_comparison.csv").open("w", newline="") as s:
        wr = csv.DictWriter(s, fieldnames=["case", "field", "stored", "ext", "abs_diff"])
        wr.writeheader(); wr.writerows(comp)
    for c in comp:
        print(c)
    # stored CSV values are rounded by AVL (5-6 significant digits); fail on larger differences
    ok = all(c["abs_diff"] <= max(2e-5, 1e-5 * abs(c["stored"])) for c in comp)
    print("REPRODUCTION", "OK" if ok else "FAILED")
    return ok


def stage_item1(ctx):
    base_cfg, ext, wing, flight, xnp = ctx
    c = ext["item1"]
    tasks = []
    for margin in ext["static_margins"]:
        for sweep in c["sweeps_deg"]:
            w0, x0 = xnp[sweep]
            w = wing_at_margin(w0, x0, margin)
            placements = ["full"] + (["outer"] if sweep in c["outer_sweeps_deg"] else [])
            for pl in placements:
                for cf in c["chord_fractions"]:
                    for ang in c["angles_deg"]:
                        name = f"m{int(round(margin*1000)):03d}_s{sweep}_{pl}_c{int(round(cf*100)):02d}_a{ang}"
                        tasks.append({"name": name, "wing": w,
                                      "feature": Reflex(cf, ang, pl, c["outer_split"]),
                                      "meta": {"target_margin": margin}})
    rows = run_tasks(tasks, flight, ext["workers"])
    save("item1_margin_sweep", rows)
    # smallest tested angle giving trim CL >= 0.2 per group
    groups = {}
    for r in rows:
        groups.setdefault((r["target_margin"], r["sweep_deg"], r["placement"], r["chord_fraction"]), []).append(r)
    summ = []
    for key in sorted(groups):
        rs = sorted(groups[key], key=lambda r: r["angle_deg"])
        hit = next((r for r in rs if r.get("trim_cl") is not None and r["trim_cl"] >= 0.2), None)
        summ.append({"target_margin": key[0], "sweep_deg": key[1], "placement": key[2], "chord_fraction": key[3],
                     "smallest_angle_cl_ge_0p2": hit["angle_deg"] if hit else None,
                     "trim_cl": hit["trim_cl"] if hit else None,
                     "trim_alpha_deg": hit["trim_alpha_deg"] if hit else None,
                     "static_margin_mac": hit["static_margin_mac"] if hit else None,
                     "that_case_exceeds_0p5": bool(hit and hit["trim_cl"] > 0.5),
                     "angles_with_cl_gt_0p5": [r["angle_deg"] for r in rs if r.get("trim_cl") and r["trim_cl"] > 0.5],
                     "angles_no_trim": [r["angle_deg"] for r in rs if r.get("trim_cl") is None],
                     "angles_widened_probe": [r["angle_deg"] for r in rs if r.get("alpha_range_extended")]})
    (OUT / "item1_smallest_angle.json").write_text(json.dumps(summ, indent=1))
    with (OUT / "item1_smallest_angle.csv").open("w", newline="") as s:
        wr = csv.DictWriter(s, fieldnames=list(summ[0]))
        wr.writeheader(); wr.writerows(summ)
    print(f"item1: {len(rows)} cases, errors={sum(bool(r['error']) for r in rows)}")


def stage_item2(ctx):
    base_cfg, ext, wing, flight, xnp = ctx
    c = ext["item2"]
    w0, x0 = xnp[c["sweep_deg"]]
    w = wing_at_margin(w0, x0, c["static_margin"])
    variants = [("full", 0.55)] + [("outer", s) for s in c["outboard_starts"]] + [("inner", s) for s in c["inboard_ends"]]
    tasks = []
    for pl, split in variants:
        for cf in c["chord_fractions"]:
            for ang in c["angles_deg"]:
                name = f"x_{pl}{int(round(split*100)):02d}_c{int(round(cf*100)):02d}_a{ang}" if pl != "full" else \
                       f"x_full_c{int(round(cf*100)):02d}_a{ang}"
                tasks.append({"name": name, "wing": w, "feature": Reflex(cf, ang, pl, split),
                              "meta": {"target_margin": c["static_margin"]}})
    rows = run_tasks(tasks, flight, ext["workers"])
    for r in rows:
        if r.get("trim_cl") is not None:
            r["trim_cl_per_lip_area_frac"] = r["trim_cl"] / r["lip_area_frac"]  # CL per (A_lip/S)
            r["trim_cl_per_lip_area_frac_per_deg"] = r["trim_cl_per_lip_area_frac"] / r["angle_deg"]
            r["trim_cl_per_m2_lip"] = r["trim_cl"] / r["lip_area_m2"]
    keys = CSV_KEYS + ["trim_cl_per_lip_area_frac", "trim_cl_per_lip_area_frac_per_deg", "trim_cl_per_m2_lip"]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "item2_extent.json").write_text(json.dumps({"rows": rows}, indent=1))
    write_csv(OUT / "item2_extent.csv", rows, keys)
    print(f"item2: {len(rows)} cases, errors={sum(bool(r['error']) for r in rows)}")


def stage_item3(ctx):
    base_cfg, ext, wing, flight, xnp = ctx
    c = ext["item3"]
    w0, x0 = xnp[c["sweep_deg"]]
    w = wing_at_margin(w0, x0, c["static_margin"])
    tasks = []
    for k in c["cases"]:
        for m in c["mismatch_deg"]:
            f = Reflex(k["chord_fraction"], k["angle_deg"], k["placement"], k["split"], mismatch_deg=m)
            tasks.append({"name": f"mm_{k['placement']}_c{int(round(k['chord_fraction']*100)):02d}_a{k['angle_deg']}_m{tag(m)}",
                          "wing": w, "feature": f, "meta": {"target_margin": c["static_margin"]}})
    rows = run_tasks(tasks, flight, ext["workers"])
    for r in rows:
        ref = next((q for q in rows if q["placement"] == r["placement"] and q["mismatch_deg"] == 0), None)
        if ref and r.get("trim_cl") is not None and ref.get("trim_cl") is not None:
            r["d_roll_vs_zero_mismatch"] = r["trim_cl_roll"] - ref["trim_cl_roll"]
            r["d_yaw_vs_zero_mismatch"] = r["trim_cn_yaw"] - ref["trim_cn_yaw"]
    keys = CSV_KEYS + ["d_roll_vs_zero_mismatch", "d_yaw_vs_zero_mismatch"]
    (OUT / "item3_mismatch.json").write_text(json.dumps({"rows": rows}, indent=1))
    write_csv(OUT / "item3_mismatch.csv", rows, keys)
    print(f"item3: {len(rows)} cases, errors={sum(bool(r['error']) for r in rows)}")


def tolerance_tasks(label, w_nom, f_nom, ext):
    """One-at-a-time config perturbations (as reflex_study.main) + dense limit scans."""
    errs = json.loads((Path(__file__).with_name("reflex_config.json")).read_text())["exploratory_errors_not_measured_tolerances"]
    c = ext["item4"]
    t = []

    def add(kind, err, w, f, grp):
        t.append({"name": f"t4_{label}_{kind}_{tag(err)}", "wing": w, "feature": f,
                  "meta": {"group": f"{label}:{grp}", "tol_kind": kind, "tol_error": err,
                           "tol_case": label}})
    for mm in errs["cg_mm"]:
        add("cg_mm", mm, replace(w_nom, cg_x_m=w_nom.cg_x_m + mm / 1000), f_nom, "oat")
    for d in errs["reflex_angle_deg"]:
        add("angle_deg", d, w_nom, replace(f_nom, angle_deg=f_nom.angle_deg + d), "oat")
    for d in errs["left_right_reflex_mismatch_deg"]:
        add("mismatch_deg", d, w_nom, replace(f_nom, mismatch_deg=d), "oat")
    for s in c["chord_scale"]:  # relative chord error: 0.8/1.2 of nominal (=0.08/0.12 for the 10% lip)
        add("chord_scale", s, w_nom, replace(f_nom, chord_fraction=f_nom.chord_fraction * s), "oat")
    for d in errs["incidence_mismatch_deg"]:
        add("incidence_mismatch_deg", d, w_nom, replace(f_nom, left_incidence_deg=-d / 2, right_incidence_deg=d / 2), "oat")
    for mm in c["limit_scan_cg_mm"]:
        add("cg_mm", mm, replace(w_nom, cg_x_m=w_nom.cg_x_m + mm / 1000), f_nom, "scan")
    for d in c["limit_scan_angle_deg"]:
        add("angle_deg", d, w_nom, replace(f_nom, angle_deg=f_nom.angle_deg + d), "scan")
    for t_ in t:
        t_["name"] = t_["name"] + ("_s" if t_["meta"]["group"].endswith("scan") else "_o")
    return t


def limit(scan: list[tuple[float, dict]], direction: int):
    """Largest error in 'direction' (+1/-1) from 0 before CL or SM leaves its band."""
    pts = sorted([p for p in scan if p[0] * direction >= 0], key=lambda p: abs(p[0]))
    if not pts or not passes(pts[0][1]):
        return None, "nominal fails"
    prev = pts[0]
    for cur in pts[1:]:
        if passes(cur[1]):
            prev = cur
            continue
        if cur[1].get("trim_cl") is None:
            return prev[0], "no trim beyond"
        best = None
        for metric, bounds in (("trim_cl", BAND_CL), ("static_margin_mac", BAND_SM)):
            a, b = prev[1][metric], cur[1][metric]
            for bound, side in ((bounds[0], "lower"), (bounds[1], "upper")):
                if (b < bound <= a or b > bound >= a) and a != b:
                    frac = (bound - a) / (b - a)
                    if best is None or frac < best[0]:
                        best = (frac, f"{metric}={bound:g}")
        if best is None:
            return prev[0], "unresolved"
        return prev[0] + best[0] * (cur[0] - prev[0]), best[1]
    return pts[-1][0], "not reached within scan"


def stage_item4(ctx):
    base_cfg, ext, wing, flight, xnp = ctx
    c = ext["item4"]
    w0, x0 = xnp[c["sweep_deg"]]
    cases = [dict(k) for k in c["cases"]]
    # smallest-lip passing case at 5% margin from item 1 (25 deg sweep)
    i1 = load("item1_margin_sweep")["rows"]
    cand = [r for r in i1 if r["sweep_deg"] == 25 and r["target_margin"] == 0.05 and passes(r)]
    cand.sort(key=lambda r: (r["lip_area_m2"], r["angle_deg"]))
    chosen = []
    if cand:
        chosen.append(("area", cand[0]))
        full = sorted([r for r in cand if r["placement"] == "full"], key=lambda r: (r["chord_fraction"], r["angle_deg"]))
        if full and full[0]["name"] != cand[0]["name"]:
            chosen.append(("full_smallest_chord_then_angle", full[0]))
    for how, r in chosen:
        cases.append({"label": f"minlip_{'area' if how == 'area' else 'fullspan'}_{r['placement']}_c{int(round(r['chord_fraction']*100)):02d}_a{r['angle_deg']:g}",
                      "placement": r["placement"], "chord_fraction": r["chord_fraction"],
                      "angle_deg": r["angle_deg"], "margin": 0.05, "selection": how, "from_item1": r["name"]})
    all_tasks, spec = [], {}
    for k in cases:
        w = wing_at_margin(w0, x0, k["margin"])
        f = Reflex(k["chord_fraction"], k["angle_deg"], k["placement"], 0.55)
        spec[k["label"]] = (w, f, k)
        all_tasks += tolerance_tasks(k["label"], w, f, ext)
    # combined corners on nominal full 10%/3 deg
    wn, fn, _ = spec["nominal_full_c10_a3"]
    corner_tasks = []
    for dcg in c["corner_cg_mm"]:
        for dang in c["corner_angle_deg"]:
            for dm in c["corner_mismatch_deg"]:
                corner_tasks.append({"name": f"corner_cg{tag(dcg)}_ang{tag(dang)}_mm{tag(dm)}",
                                     "wing": replace(wn, cg_x_m=wn.cg_x_m + dcg / 1000),
                                     "feature": replace(fn, angle_deg=fn.angle_deg + dang, mismatch_deg=dm),
                                     "meta": {"group": "corner", "cg_err_mm": dcg, "angle_err_deg": dang,
                                              "mismatch_deg_err": dm}})
    rows = run_tasks(all_tasks + corner_tasks, flight, ext["workers"])
    tol = [r for r in rows if r["group"] != "corner"]
    corners = [r for r in rows if r["group"] == "corner"]
    limits = []
    for label, (w, f, k) in spec.items():
        rec = {"case": label, "placement": k["placement"], "chord_fraction": k["chord_fraction"],
               "angle_deg": k["angle_deg"], "margin_target": k["margin"]}
        for kind, unit in (("cg_mm", "mm"), ("angle_deg", "deg")):
            scan = [(r["tol_error"], r) for r in tol if r["tol_case"] == label and r["tol_kind"] == kind
                    and r["group"].endswith("scan")]
            lo, lo_why = limit(scan, -1)
            hi, hi_why = limit(scan, +1)
            rec.update({f"{kind}_neg_limit": lo, f"{kind}_neg_binding": lo_why,
                        f"{kind}_pos_limit": hi, f"{kind}_pos_binding": hi_why})
        nom = next(r for r in tol if r["tol_case"] == label and r["tol_kind"] == "cg_mm" and r["tol_error"] == 0 and r["group"].endswith("oat"))
        rec.update(nominal_trim_cl=nom.get("trim_cl"), nominal_sm=nom.get("static_margin_mac"))
        limits.append(rec)
    (OUT / "item4_tolerances.json").write_text(json.dumps({"rows": tol, "corners": corners, "limits": limits,
                                                            "cases": cases}, indent=1))
    keys = CSV_KEYS + ["group", "tol_case", "tol_kind", "tol_error"]
    write_csv(OUT / "item4_tolerances.csv", tol, keys)
    write_csv(OUT / "item4_corners.csv", corners, CSV_KEYS + ["cg_err_mm", "angle_err_deg", "mismatch_deg_err"])
    with (OUT / "item4_limits.csv").open("w", newline="") as s:
        wr = csv.DictWriter(s, fieldnames=list(limits[0]))
        wr.writeheader(); wr.writerows(limits)
    print(f"item4: {len(rows)} cases ({len(corners)} corners), errors={sum(bool(r['error']) for r in rows)}")


def stage_item6(ctx):
    base_cfg, ext, wing, flight, xnp = ctx
    old = json.loads((OLD / "static.json").read_text())["cases"]
    i1 = load("item1_margin_sweep")["rows"]
    out = []
    for o in old:
        if "trim_cl" in o:
            continue
        sw = [p["alpha_deg"] for p in o["alpha_sweep"]]
        cm = [p["cm"] for p in o["alpha_sweep"]]
        slope = (cm[-1] - cm[-2]) / (sw[-1] - sw[-2])
        est = sw[-1] - cm[-1] / slope if slope < 0 else None
        new = next((r for r in i1 if r["target_margin"] == 0.10 and r["sweep_deg"] == o["wing"]["quarter_chord_sweep_deg"]
                    and r["placement"] == "full" and r["chord_fraction"] == 0.15 and r["angle_deg"] == 5), None)
        out.append({"old_case": o["name"], "old_cm_at_alpha_range": dict(zip(sw, cm)),
                    "old_last_cm_alpha8": cm[-1], "old_cm_slope_per_deg_last_pair": slope,
                    "old_linear_extrapolated_crossing_alpha_deg": est,
                    "ext_trim_alpha_deg": new and new.get("trim_alpha_deg"),
                    "ext_trim_cl": new and new.get("trim_cl"),
                    "ext_static_margin": new and new.get("static_margin_mac"),
                    "ext_alpha_range_extended": new and new.get("alpha_range_extended"),
                    "ext_name": new and new["name"], "ext_error": new and new["error"]})
    (OUT / "item6_notrim.json").write_text(json.dumps(out, indent=1))
    for o in out:
        print(o)


def main():
    global WIDEN
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    force = "--force" in sys.argv
    base_cfg, ext, wing, flight = load_configs()
    WIDEN = ext["item6"]["widened_probe_alpha_deg"]
    OUT.mkdir(parents=True, exist_ok=True)
    stages = {"repro": stage_repro, "item1": stage_item1, "item2": stage_item2,
              "item3": stage_item3, "item4": stage_item4, "item6": stage_item6}
    wanted = list(stages) if (not args or "all" in args) else args
    ctx = (base_cfg, ext, wing, flight, bases(base_cfg, ext, wing, flight))
    for name in wanted:
        done = {"repro": "repro", "item1": "item1_margin_sweep", "item2": "item2_extent",
                "item3": "item3_mismatch", "item4": "item4_tolerances", "item6": "item6_notrim"}[name]
        if not force and (OUT / f"{done}.json").is_file() and name != "repro":
            print("cached:", name)
            continue
        print("== stage", name, flush=True)
        res = stages[name](ctx)
        if name == "repro" and res is False:
            print("Reproduction failed; stopping.")
            sys.exit(1)


if __name__ == "__main__":
    main()
