"""Stage-3 driver: uncensored scans, lip bound, deck consistency, mass budget, trim tables, trades.  Writes only to
results/runs/rigid_glider/arch_compare_v2/.   py -B rigid_glider/arch_v2_study.py models | scan | items | all
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import statistics
import time
from collections import defaultdict
from multiprocessing import Pool
from pathlib import Path

import arch_model as A
import arch_study as S
import arch_v2_model as V2
import pure_wing_model as m
import pure_wing_v2_model as v2

OUT = V2.OUT
B = 0.40
WINGS = S.WINGS
SECTIONS = S.SECTIONS
WS_LIST = S.WS_LIST
LIP_ANGLES = S.LIP_ANGLES
LIP_ORDER = S.LIP_ORDER
TAIL_AREAS2 = [0.04, 0.05, 0.06, 0.075, 0.08, 0.10]          # 0.04/0.05/0.06/0.075 = effectiveness-scaled versions of the 8 % and 10 % tails
TAIL_STN = 2.0
TAIL_INC = -3.0
THRESH = 3.0
G = 9.80665


def write_csv(path, rows):
    S._write_csv(path, rows)


def mk(kind, wing, *cfg):
    return V2.model_key(kind, wing, list(cfg) if cfg else None)


# ------------------------------------------------------------------ models
def stage_models():
    t = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    legacy = json.loads((S.OUT / "avl_models.json").read_text())
    wk = OUT / "avl"
    for wid, (ar, sw, tp) in WINGS.items():
        for a in TAIL_AREAS2:
            key = mk("tail", wid, a, TAIL_STN)
            if key not in legacy:
                legacy[key] = A.build_tail(ar, sw, tp, a, TAIL_STN, wk / f"legacy_tail_{wid}_{a:g}")
    (OUT / "models_legacy.json").write_text(json.dumps(legacy))
    final = {}
    for wid, (ar, sw, tp) in WINGS.items():
        final[mk("pure", wid)] = V2.build_pure_final(ar, sw, tp, wk / f"final_pure_{wid}")
        for cf in S.LIP_CFS:
            final[mk("lip", wid, cf, "outer")] = V2.build_lip_final(ar, sw, tp, cf, wk / f"final_lip_{wid}_{int(cf*100)}")
        for a in TAIL_AREAS2:
            final[mk("tail", wid, a, TAIL_STN)] = V2.build_tail_final(ar, sw, tp, a, wk / f"final_tail_{wid}_{a:g}")
    (OUT / "models_final.json").write_text(json.dumps(final))
    bad = max(abs(M["lin_check"]["cl_avl"] - M["lin_check"]["cl_pred"]) for d in (legacy, final) for M in d.values())
    print(f"models: legacy {len(legacy)}, final {len(final)} in {time.time()-t:.0f}s; max linearity error in CL {bad:.5f}")


def load_models():
    return {"legacy": json.loads((OUT / "models_legacy.json").read_text()), "final": json.loads((OUT / "models_final.json").read_text())}


# ------------------------------------------------------------------ scan worker
_S = {}


def _init():
    _S["models"] = load_models()
    _S["tabs"] = A.ArchTables()


def forms_for(kind):
    return ["avl_only", "coupled"] + (["replace", "replace_k"] if kind == "lip" else [])


def work(job):
    deck, kind, wing, sec, cfg, u = job
    M = _S["models"][deck][V2.model_key(kind, wing, cfg)]
    tabs = _S["tabs"]
    recs = []
    for ws in WS_LIST:
        for form in forms_for(kind):
            combos = [(None, None)] if form == "avl_only" else [(nc, pan) for nc in (5, 9) for pan in (160, 240)]
            for nc, pan in combos:
                ev, ti_fn, avl = V2.evaluators(M, B, u, kind, tabs, sec, form, nc, pan, cfg, ws)
                curve, nonconv = V2.build_curve_ev(ev, ti_fn, avl_only=avl)
                sets, pts = V2.scan_curve2(curve)
                ks = [p["k"] for p in curve if p.get("k") is not None and 0.2 <= p["cl"] <= 0.5 and p["cov"] > 0]
                recs.append(dict(deck=deck, kind=kind, wing=wing, section=sec, cfg=cfg, u=u, ws=ws, form=form, ncrit=nc, panels=pan, sets=sets, pts=pts,
                                 nonconv=nonconv, k_mean_band=(statistics.mean(ks) if ks else None)))
    return recs


def all_jobs():
    jobs = []
    for deck in ("legacy", "final"):
        for wid in WINGS:
            for sec in SECTIONS:
                jobs.append((deck, "pure", wid, sec, None, 0.0))
                for cf in S.LIP_CFS:
                    for ang in LIP_ANGLES[cf]:
                        jobs.append((deck, "lip", wid, sec, [cf, "outer"], ang))
                for a in TAIL_AREAS2:
                    jobs.append((deck, "tail", wid, sec, [a, TAIL_STN], TAIL_INC))
    return jobs


def stage_scan():
    t = time.time()
    jobs = all_jobs()
    with Pool(8, initializer=_init) as pool:
        res = pool.map(work, jobs, chunksize=1)
    recs = [r for x in res for r in x]
    (OUT / "scans_v2.json").write_text(json.dumps(recs))
    nc = sum(r["nonconv"] for r in recs if r["form"] == "replace_k")
    print(f"scan: {len(jobs)} jobs, {len(recs)} records in {time.time()-t:.0f}s; form-(c) non-converged CL points: {nc}")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    fn = {"models": stage_models, "scan": stage_scan}
    for s in (list(fn) if what == "all" else [what]):
        fn[s]()


if __name__ == "__main__":
    main()
