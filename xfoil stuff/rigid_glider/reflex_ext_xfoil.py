"""XFOIL stages of the reflex extension study (item 5 + reproduction check).

Wraps reflex_xfoil.run() by re-pointing its module-level OUT to
results/runs/rigid_glider/reflex_ext/xfoil at run time (no edit of that file).
Clean polars are only READ from reflex/xfoil/.

Run: py rigid_glider/reflex_ext_xfoil.py [repro|item5|all]
"""

from __future__ import annotations

import csv
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import reflex_xfoil as rx
from reflex_ext_avl import OUT as EXT_OUT, ROOT, load_configs

XOUT = EXT_OUT / "xfoil"
OLD_X = ROOT / "results/runs/rigid_glider/reflex/xfoil"
rx.OUT = XOUT  # run() resolves OUT at call time


def old_polar(section, re_, chord, angle):
    label = f"{section}_Re{re_}_c{int(chord*100):02d}_a{angle:g}".replace(".", "p")
    return rx.read_polar(OLD_X / label / "polar.dat")


def by_alpha(rows):
    return {round(r["alpha_deg"], 3): r for r in rows}


def stage_repro():
    new = rx.run("NACA_0008", 80000, 0.10, 3.0)
    stored = old_polar("NACA_0008", 80000, 0.10, 3.0)
    n, s = by_alpha(new["rows"]), by_alpha(stored)
    out = []
    for a in (0.0, 2.0, 4.0):
        for k in ("cl", "cd", "cm"):
            ov, nv = s.get(a, {}).get(k), n.get(a, {}).get(k)
            out.append({"alpha_deg": a, "quantity": k, "stored": ov, "regenerated": nv,
                        "abs_diff": None if ov is None or nv is None else abs(nv - ov)})
    full = {"new_count": len(new["rows"]), "stored_count": len(stored),
            "new_alphas": sorted(n), "stored_alphas": sorted(s), "rows": out,
            "max_abs_diff_all_common_alphas": max(
                (abs(n[a][k] - s[a][k]) for a in set(n) & set(s) for k in ("cl", "cd", "cm")), default=None),
            "n_common_alphas": len(set(n) & set(s))}
    (EXT_OUT / "xfoil_repro.json").write_text(json.dumps(full, indent=1))
    for r in out:
        print(r)
    print({k: v for k, v in full.items() if k != "rows"})


def stage_item5():
    base_cfg, ext, _, _ = load_configs()
    c = ext["item5"]
    res_list = base_cfg["xfoil_reynolds"]
    jobs = []
    for sec in c["sections"]:
        jobs.append((sec, 80000, 0.10, 0.0, True))  # clean inviscid
        for chord, ang in c["lips"]:
            jobs.append((sec, 80000, chord, float(ang), True))  # bent inviscid
            for re_ in res_list:
                jobs.append((sec, re_, chord, float(ang), False))
    jobs = list(dict.fromkeys(jobs))
    with ThreadPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(lambda j: rx.run(j[0], j[1], j[2], j[3], inviscid=j[4]), jobs))
    XOUT.mkdir(parents=True, exist_ok=True)
    (XOUT / "item5_summary.json").write_text(json.dumps(results, indent=1))

    # counts
    counts = []
    for r in results:
        counts.append({"section": r["section"], "inviscid": r["inviscid"], "reynolds": r["reynolds"],
                       "chord_fraction": r["chord_fraction"], "angle_deg": r["angle_deg"],
                       "requested": r["expected_count"], "returned": r["converged_count"],
                       "exit_code": r["exit_code"],
                       "missing_alphas": sorted(set(round(-3 + 0.5 * i, 1) for i in range(23))
                                                - {round(x["alpha_deg"], 1) for x in r["rows"]})})
    _csv(XOUT / "item5_counts.csv", counts)

    clean = {}
    for sec in c["sections"]:
        for re_ in res_list:
            clean[(sec, re_)] = old_polar(sec, re_, 0.10, 0.0)
    bent = {(r["section"], r["chord_fraction"], r["angle_deg"], r["reynolds"]): r["rows"]
            for r in results if not r["inviscid"]}

    deltas, cds, inv = [], [], []
    for sec in c["sections"]:
        for chord, ang in c["lips"]:
            for re_ in res_list:
                b, cl_ = by_alpha(bent[(sec, chord, float(ang), re_)]), by_alpha(clean[(sec, re_)])
                for a in c["delta_alphas_deg"]:
                    bb, cc = b.get(float(a)), cl_.get(float(a))
                    deltas.append({"section": sec, "chord_fraction": chord, "angle_deg": ang, "reynolds": re_,
                                   "alpha_deg": a,
                                   "bent_cl": bb and bb["cl"], "clean_cl": cc and cc["cl"],
                                   "delta_cl": bb and cc and bb["cl"] - cc["cl"],
                                   "bent_cm": bb and bb["cm"], "clean_cm": cc and cc["cm"],
                                   "delta_cm": bb and cc and bb["cm"] - cc["cm"],
                                   "note": "" if bb and cc else "missing point(s); not filled"})
            # Cd at matched Cl at the requested Re
            re_ = c["cd_re"]
            sb = rx.at_cl(bent[(sec, chord, float(ang), re_)], c["cd_at_cl"])
            sc = rx.at_cl(clean[(sec, re_)], c["cd_at_cl"])
            cds.append({"section": sec, "chord_fraction": chord, "angle_deg": ang, "reynolds": re_,
                        "target_cl": c["cd_at_cl"],
                        "bent_cd": sb and sb["cd"], "clean_cd": sc and sc["cd"],
                        "delta_cd": sb and sc and sb["cd"] - sc["cd"],
                        "bent_alpha": sb and sb["alpha_deg"], "clean_alpha": sc and sc["alpha_deg"],
                        "bent_cm": sb and sb["cm"], "clean_cm": sc and sc["cm"],
                        "note": "" if sb and sc else "Cl target not bracketed by adjacent returned points"})
        # inviscid alpha=0 check
        cl0 = next(r for r in results if r["inviscid"] and r["section"] == sec and r["angle_deg"] == 0.0)
        c0 = by_alpha(cl0["rows"]).get(0.0)
        for chord, ang in c["lips"]:
            rr = next(r for r in results if r["inviscid"] and r["section"] == sec
                      and r["chord_fraction"] == chord and r["angle_deg"] == float(ang))
            b0 = by_alpha(rr["rows"]).get(0.0)
            inv.append({"section": sec, "chord_fraction": chord, "angle_deg": ang,
                        "inviscid_bent_cl_a0": b0 and b0["cl"], "inviscid_clean_cl_a0": c0 and c0["cl"],
                        "inviscid_delta_cl_a0": b0 and c0 and b0["cl"] - c0["cl"],
                        "inviscid_bent_cm_a0": b0 and b0["cm"], "inviscid_clean_cm_a0": c0 and c0["cm"],
                        "inviscid_delta_cm_a0": b0 and c0 and b0["cm"] - c0["cm"],
                        "points_returned": rr["converged_count"]})
    _csv(XOUT / "item5_delta_cl_cm.csv", deltas)
    _csv(XOUT / "item5_cd_at_cl0p3_re80k.csv", cds)
    _csv(XOUT / "item5_inviscid_alpha0.csv", inv)
    visc = [x for x in counts if not x["inviscid"]]
    print(f"item5: {len(results)} xfoil runs; viscous polars {len(visc)}: requested "
          f"{sum(x['requested'] for x in visc)}, returned {sum(x['returned'] for x in visc)}; "
          f"inviscid polars {len(counts)-len(visc)}: requested "
          f"{sum(x['requested'] for x in counts if x['inviscid'])}, returned "
          f"{sum(x['returned'] for x in counts if x['inviscid'])}")


def _csv(path, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    which = sys.argv[1:] or ["all"]
    XOUT.mkdir(parents=True, exist_ok=True)
    if "repro" in which or "all" in which:
        stage_repro()
    if "item5" in which or "all" in which:
        stage_item5()
