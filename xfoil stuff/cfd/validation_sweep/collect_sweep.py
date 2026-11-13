#!/usr/bin/env python3
"""Collect the angle-of-attack sweep of the Ananda AR3 flat-plate CFD cases into sweep_results.csv,
next to the measured values interpolated to the same angle. Standard library only.

usage:  python collect_sweep.py [--runs RUNS_DIR] [--measured CSV] [--out CSV]

RUNS_DIR defaults to \\\\wsl.localhost\\MMAV-CFD\\home\\cfd\\run on Windows (this wakes the distro) and to
/home/cfd/run when run inside WSL. Cases read: vp_a4_coarse (alpha 4, already finished) and vp_sweep_aNN.

Averaging window (chord passages, t = c/U):
  endTime 20 (alpha 0-8): t = 8 .. 20, the same window used for the alpha-4 coarse case.
  endTime 40 (alpha 10, 12, separated/unsteady): last half, t = 20 .. 40.
  A case that has not reached its endTime is reported as 'partial' with the last half of what exists.
Mean and sd are time-weighted (adaptive dt); sd is the standard deviation of the fluctuation about the mean.

Measured data: research/plate_data/ananda2012_flatplate_AR3_taper1.csv, run 0383ar (Re 59999.4), AR 3, taper 1.
The run has an increasing-alpha leg and a decreasing-alpha leg. Each leg is linearly interpolated in alpha at the
CFD angle (a straight line between the two bracketing table points; no extrapolation), and the mean of the two
legs is also given. Measured alpha is wall-corrected; CFD is free air with the plate rotated by exactly alpha.
CM is about the quarter chord, nose-up positive, in both.
"""
import argparse, csv, glob, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MEAS = os.path.normpath(os.path.join(HERE, "..", "..", "research", "plate_data",
                                             "ananda2012_flatplate_AR3_taper1.csv"))
RUN_ID = "0383ar"


def default_runs():
    if os.name == "nt":
        return r"\\wsl.localhost\MMAV-CFD\home\cfd\run"
    return "/home/cfd/run"


def read_coeffs(case):
    """Return list of (t, Cd, Cl, CmPitch) from postProcessing/forceCoeffs1/*/coefficient.dat."""
    files = glob.glob(os.path.join(case, "postProcessing", "forceCoeffs1", "*", "coefficient.dat"))
    files.sort(key=lambda p: float(os.path.basename(os.path.dirname(p))))
    seen, rows = set(), []
    for f in files:
        with open(f) as fh:
            for ln in fh:
                if ln.startswith("#") or not ln.strip():
                    continue
                c = ln.split()
                t = float(c[0])
                if t in seen:
                    continue
                seen.add(t)
                rows.append((t, float(c[1]), float(c[4]), float(c[7])))
    rows.sort()
    return rows


def end_time(case):
    try:
        with open(os.path.join(case, "system", "controlDict")) as fh:
            m = re.search(r"^\s*endTime\s+([0-9.eE+-]+)\s*;", fh.read(), re.M)
        return float(m.group(1))
    except Exception:
        return None


def weighted(rows, t0, t1):
    sel = [r for r in rows if t0 <= r[0] <= t1]
    if len(sel) < 3:
        return None
    ts = [r[0] for r in sel]
    # weights = local time step (central differences, one-sided at the ends)
    w = []
    for i, t in enumerate(ts):
        lo = ts[i - 1] if i > 0 else t
        hi = ts[i + 1] if i < len(ts) - 1 else t
        w.append((hi - lo) / (2.0 if 0 < i < len(ts) - 1 else 1.0))
    sw = sum(w)
    out = {"n": len(sel), "t0": ts[0], "t1": ts[-1]}
    for k, name in ((2, "CL"), (1, "CD"), (3, "CM")):
        y = [r[k] for r in sel]
        mean = sum(a * b for a, b in zip(w, y)) / sw
        var = sum(a * (b - mean) ** 2 for a, b in zip(w, y)) / sw
        out[name] = (mean, var ** 0.5)
    return out


def load_measured(path):
    legs = {"increasing": [], "decreasing": []}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["run"] == RUN_ID and r["sweep"] in legs:
                legs[r["sweep"]].append((float(r["alpha_deg"]), float(r["CL"]), float(r["CD"]), float(r["CM_c4"])))
    for k in legs:
        legs[k].sort()
    return legs


def interp(leg, a, col):
    for (a0, *v0), (a1, *v1) in zip(leg, leg[1:]):
        if a0 <= a <= a1 and a1 > a0:
            f = (a - a0) / (a1 - a0)
            return v0[col] + f * (v1[col] - v0[col])
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default=default_runs())
    ap.add_argument("--measured", default=DEFAULT_MEAS)
    ap.add_argument("--out", default=os.path.join(HERE, "sweep_results.csv"))
    a = ap.parse_args()

    cases = [(4.0, os.path.join(a.runs, "vp_a4_coarse"))]
    for d in glob.glob(os.path.join(a.runs, "vp_sweep_a*")):
        m = re.search(r"vp_sweep_a(\d+)$", d)
        if m:
            cases.append((float(m.group(1)), d))
    cases.sort()
    legs = load_measured(a.measured)

    hdr = ["alpha_deg", "status", "t_end", "window_start", "window_end", "n_samples",
           "CL_mean", "CL_sd", "CD_mean", "CD_sd", "CM_mean", "CM_sd",
           "meas_CL_up", "meas_CL_down", "meas_CL_mean", "meas_CD_up", "meas_CD_down", "meas_CD_mean",
           "meas_CM_up", "meas_CM_down", "meas_CM_mean", "dCL_cfd_minus_meas", "dCD_cfd_minus_meas", "dCM_cfd_minus_meas"]
    rows_out = []
    for alpha, case in cases:
        if not os.path.isdir(case):
            continue
        rows = read_coeffs(case)
        if os.path.exists(os.path.join(case, "FAILED")):
            status = "FAILED"
        elif os.path.exists(os.path.join(case, "DONE")) or case.endswith("vp_a4_coarse"):
            status = "done"
        else:
            status = "running/partial"
        if not rows:
            rows_out.append([alpha, status] + [""] * (len(hdr) - 2))
            continue
        tl = rows[-1][0]
        endt = end_time(case) or tl
        if status == "done" or tl >= endt - 1e-6:
            w0 = 8.0 if endt <= 20.5 else endt / 2.0
            w1 = endt
        else:
            w0, w1 = tl / 2.0, tl
            status = status if status == "FAILED" else "partial"
        st = weighted(rows, w0, w1)
        meas = []
        for col in range(3):  # CL, CD, CM
            u = interp(legs["increasing"], alpha, col)
            d = interp(legs["decreasing"], alpha, col)
            m = (u + d) / 2.0 if u is not None and d is not None else None
            meas += [u, d, m]
        mm = {"CL": meas[2], "CD": meas[5], "CM": meas[8]}
        diffs = [(st[k][0] - mm[k]) if (st and mm[k] is not None) else None for k in ("CL", "CD", "CM")]
        r = [alpha, status, round(tl, 3), w0, w1, st["n"] if st else 0]
        for k in ("CL", "CD", "CM"):
            r += [st[k][0], st[k][1]] if st else ["", ""]
        r += meas + diffs
        rows_out.append(["" if v is None else (round(v, 5) if isinstance(v, float) else v) for v in r])
    with open(a.out, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(hdr)
        wr.writerows(rows_out)
    print("wrote", a.out)
    print("%6s %-16s %6s | %8s %7s | %8s %7s | %8s %8s | CFD CL, CD, CM vs measured mean (legs interpolated)" %
          ("alpha", "status", "t_end", "CL", "sd", "CD", "sd", "meas CL", "meas CD"))
    for r in rows_out:
        d = dict(zip(hdr, r))
        if d["CL_mean"] == "":
            print("%6s %-16s (no coefficients yet)" % (d["alpha_deg"], d["status"]))
            continue
        print("%6.1f %-16s %6s | %8.4f %7.4f | %8.4f %7.4f | %8s %8s | CM %8.5f vs %s" %
              (d["alpha_deg"], d["status"], d["t_end"], d["CL_mean"], d["CL_sd"], d["CD_mean"], d["CD_sd"],
               d["meas_CL_mean"], d["meas_CD_mean"], d["CM_mean"], d["meas_CM_mean"]))


if __name__ == "__main__":
    sys.exit(main())
