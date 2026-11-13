"""Drop-window scoring of the tumble model (Li et al. 2022 strip): share of starts that make >= 1 rotation before falling H metres, and share still rotating at that depth.
Definitions are frozen in research/tumble_model/README.md (section DROP-WINDOW SCORING).  Uses tumble_sim.py unchanged.  Standard library only.
    py -B rigid_glider/tumble_drop_window.py
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import multiprocessing as mp
from pathlib import Path

import tumble_sim as ts

OUT = ts.OUT
HEIGHTS_M = (5.0, 10.0)
ROT = 2.0 * math.pi
W_THRESH = 5.0            # rad/s, JUDGEMENT (frozen in README)
T_MAX = 15.0
CASES = [("mark, primary (area/span chord)", "primary", 0), ("mark, chord = MAC", "chord_MAC", 0), ("mark, chord = sheet area/span", "chord_sheet", 0),
         ("mark, reference = sheet centroid", "ref_centroid", 0), ("+3 mm, primary", "primary", 3)]
SPEC = "Swift_A_foam400"


def drop_job(args):
    pa, th0, w0 = args
    pl = ts.Plate(*pa)
    dt = ts.B_DT
    s = (0.0, 0.0, th0, 0.0, 0.0, w0)
    n = int(round(T_MAX / dt))
    maxrot = 0.0
    out = {}
    pending = list(HEIGHTS_M)
    for i in range(n):
        s = pl.step(s, dt)
        if s[0] != s[0]:
            break
        d = abs(s[2] - th0)
        if d > maxrot:
            maxrot = d
        while pending and s[1] <= -pending[0] * 100.0:
            H = pending.pop(0)
            dth = s[2] - th0
            still = abs(s[5]) >= W_THRESH and (s[5] * dth > 0) and abs(dth) >= math.pi
            out[H] = (maxrot >= ROT, still, (i + 1) * dt)
        if not pending:
            break
    return th0, w0, out


def main():
    rows = []
    summary = {}
    with mp.Pool(max(1, (mp.cpu_count() or 2) - 1)) as pool:
        for label, variant, d in CASES:
            pl, info = ts.strip_plate(SPEC, variant, 37.0 + d)
            pa = (pl.l, pl.m, pl.I, pl.lcm, pl.rho, pl.g, 0.0)
            jobs = [(pa, th, w) for th in ts.B_ATT for w in ts.B_RATE]
            res = pool.map(drop_job, jobs, chunksize=8)
            summary[label] = dict(info=info, H={})
            for H in HEIGHTS_M:
                ok = [(th, w, o[H]) for th, w, o in res if H in o]
                n_all = len(res)
                n = len(ok)
                rot = sum(1 for _, _, o in ok if o[0])
                still = sum(1 for _, _, o in ok if o[1])
                tmean = sum(o[2] for _, _, o in ok) / max(1, n)
                by_rate = {}
                for w in ts.B_RATE:
                    sub = [o for _, ww, o in ok if ww == w]
                    by_rate[w] = (len(sub), sum(1 for o in sub if o[0]), sum(1 for o in sub if o[1]))
                quad = {}
                for q in range(4):
                    sub = [o for th, _, o in ok if int(math.degrees(th) // 90) == q]
                    quad[q] = (len(sub), sum(1 for o in sub if o[0]))
                att = {}
                for th in ts.B_ATT:
                    sub = [o for t2, _, o in ok if t2 == th]
                    att[round(math.degrees(th))] = (len(sub), sum(1 for o in sub if o[0]))
                fr = [v[1] / v[0] for v in att.values() if v[0]]
                summary[label]["H"][H] = dict(n_all=n_all, n_reached=n, rot=rot, still=still, t_mean=tmean, by_rate=by_rate, quad=quad, att_min=min(fr) if fr else None, att_max=max(fr) if fr else None)
                for w, (nn, r, s_) in by_rate.items():
                    rows.append([label, H, w, nn, r, s_])
    with (OUT / "dropwindow_by_rate.csv").open("w", newline="", encoding="utf-8") as fh:
        w_ = csv.writer(fh)
        w_.writerow(["case", "depth_m", "omega0_rad_s", "n_reached_depth", "n_ge1_rotation", "n_still_rotating_at_depth"])
        w_.writerows(rows)
    json.dump({k: dict(info=v["info"], H={str(h): dict(x, by_rate={str(a): b for a, b in x["by_rate"].items()}, quad={str(a): b for a, b in x["quad"].items()}) for h, x in v["H"].items()}) for k, v in summary.items()},
              (OUT / "dropwindow_summary.json").open("w"), indent=1)
    for label, v in summary.items():
        print(f"== {label}: l_CM/l {v['info']['lcm_over_l']:.3f}")
        for H, x in v["H"].items():
            n = max(1, x["n_reached"])
            print(f"  H {H:g} m: reached {x['n_reached']}/{x['n_all']} (mean fall time {x['t_mean']:.2f} s)  >=1 rotation {100 * x['rot'] / n:.0f}%  still rotating {100 * x['still'] / n:.0f}%")
            print("     by rate " + " ".join(f"{w:+.0f}:{100 * b[1] / max(1, b[0]):.0f}/{100 * b[2] / max(1, b[0]):.0f}" for w, b in x["by_rate"].items()))
            print("     by quadrant " + " ".join(f"{q}:{100 * b[1] / max(1, b[0]):.0f}" for q, b in x["quad"].items()) + f"  per-attitude min/max {100 * x['att_min']:.0f}/{100 * x['att_max']:.0f}")


if __name__ == "__main__":
    mp.freeze_support()
    main()
