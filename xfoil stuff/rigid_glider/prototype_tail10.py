"""10% horizontal-tail inviscid AVL study (new; reuses prototype modules unmodified).

Run: py rigid_glider/prototype_tail10.py
Refuses to run if the output folder already exists (never overwrites).
Steps: 0 repro gate, 1 CG scan, 2 tail incidence sweep, 3 fin map, 4 tolerances.
Everything is inviscid AVL; missing/failed points are reported, never filled.
"""

from __future__ import annotations

import csv
import json
import sys
from dataclasses import replace
from math import pi
from pathlib import Path

import robustness_study as rs
from geometry import FlightCondition, Wing
from robustness_study import cg_for_margin, evaluate

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(__file__).with_name("prototype_tail10_config.json")
PROTO_CONFIG = Path(__file__).with_name("prototype_config.json")
OUT = ROOT / "results/runs/rigid_glider/prototype_tail10"
OLD = ROOT / "results/runs/rigid_glider/prototype"


# ---------------- pure helpers (unit-tested) ----------------
def frange(start: float, stop: float, step: float) -> list[float]:
    n = round((stop - start) / step)
    if n < 0 or abs(start + n * step - stop) > 1e-9:
        raise ValueError("range does not land on stop")
    return [round(start + i * step, 9) for i in range(n + 1)]


def cg_values_mm(cfg: dict) -> list[float]:
    s = cfg["cg_scan"]
    vals = set(frange(s["start_mm"], s["stop_mm"], s["step_mm"])) | set(s["extra_mm"])
    return sorted(vals)


def tail_chord(wing_area: float, wing_span: float, fraction: float, span_fraction: float) -> float:
    return fraction * wing_area / (span_fraction * wing_span)


def slope(xs: list, ys: list):
    """Least-squares slope; None if fewer than two valid points (no filling)."""
    pts = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    if len(pts) < 2:
        return None
    mx = sum(x for x, _ in pts) / len(pts)
    my = sum(y for _, y in pts) / len(pts)
    den = sum((x - mx) ** 2 for x, _ in pts)
    return None if den == 0 else sum((x - mx) * (y - my) for x, y in pts) / den


def equivalent_beta_deg(cn_yaw, cnb_per_rad):
    """Sideslip (deg) whose weathervane moment equals the given yaw moment."""
    if cn_yaw is None or not cnb_per_rad:
        return None
    return cn_yaw / cnb_per_rad * 180 / pi


def row_flags(row: dict, widened: bool) -> str:
    flags = []
    if row.get("status") != "positive-lift trim":
        flags.append("NO_POSITIVE_LIFT_TRIM")
    if widened:
        flags.append("WIDENED_ALPHA_PROBE")
    return ";".join(flags)


def compare(name: str, got: float, expected: float, tol: float) -> dict:
    return {"quantity": name, "new": got, "existing": expected, "diff": got - expected,
            "ok": abs(got - expected) <= tol}


# ---------------- runner ----------------
def write_csv(path: Path, rows: list[dict]) -> None:
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


class Study:
    def __init__(self) -> None:
        self.cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
        proto = json.loads(PROTO_CONFIG.read_text(encoding="utf-8"))
        self.wing = Wing(**proto["main_wing"])
        self.flight = FlightCondition(**proto["flight"])
        self.t = self.cfg["tail"]
        self.f = self.cfg["fin"]

    def tail_re(self, speed, fraction):
        if speed is None:
            return None
        c = tail_chord(self.wing.area, self.wing.span_m, fraction,
                       self.t["span_fraction_of_main_wing"])
        return speed * c / self.flight.nu_m2_s

    def run(self, name, cg_m, *, tail_inc=None, fin_fraction=None, **extra) -> dict:
        wing = replace(self.wing, cg_x_m=cg_m)
        opts = {"tail_area_fraction": self.t["area_fraction"],
                "tail_incidence_deg": self.t["incidence_deg"] if tail_inc is None else tail_inc,
                "tail_ac_x_m": self.t["ac_x_m"],
                "tail_span_fraction": self.t["span_fraction_of_main_wing"]}
        ff = self.f["area_fraction"] if fin_fraction is None else fin_fraction
        if ff:
            opts.update(fin_area_fraction=ff, fin_ac_x_m=self.f["ac_x_m"], fin_type=self.f["type"])
        opts.update(extra)
        widened = False
        row = evaluate(name, wing, self.flight, output=OUT, **dict(opts))
        if row["status"] == "no Cm=0 in screened alpha range":
            saved = rs.ALPHAS
            rs.ALPHAS = [float(a) for a in self.cfg["wide_alpha_probe_deg"]]
            try:
                row = evaluate(name + "_wide", wing, self.flight, output=OUT, **dict(opts))
            finally:
                rs.ALPHAS = saved
            widened = True
        out = {k: row.get(k) for k in ("name", "status", "trim_alpha_deg", "trim_cl", "cm_residual",
                                       "static_margin_mac", "cma_per_rad", "clb_per_rad", "cnb_per_rad",
                                       "cl_roll", "cn_yaw", "speed_m_s", "xnp_m")}
        out.update(name=name, cg_x_m=cg_m, cg_mm=round(cg_m * 1000, 6),
                   tail_incidence_deg=opts["tail_incidence_deg"],
                   fin_area_fraction=ff, includes_fin=bool(ff),
                   tail_re=self.tail_re(row.get("speed_m_s"), opts["tail_area_fraction"]),
                   widened_probe=widened, flags=row_flags(row, widened))
        return out


def read_old(path: Path) -> dict:
    with path.open(newline="", encoding="utf-8") as f:
        return {r["name"]: r for r in csv.DictReader(f)}


def repro_gate(s: Study) -> list[dict]:
    old_tail = read_old(OLD / "tail_map.csv")
    old_tol = read_old(OLD / "tolerances.csv")
    rows = []
    # 10% tail at x=0.44, NO fin (as in tail_map): balanced CG, then +-5 mm
    opts = {"tail_area_fraction": 0.1, "tail_incidence_deg": -3.0, "tail_ac_x_m": 0.44,
            "tail_span_fraction": 0.35}
    bal = cg_for_margin("repro_t10", s.wing, 0.10, output=OUT, **opts)
    for mm, key in ((0, "tail_a0.1_x0.44_cg+0"), (-5, "tail_a0.1_x0.44_cg-5")):
        r = evaluate(f"repro_t10_cg{mm:+g}", replace(bal, cg_x_m=bal.cg_x_m + mm / 1000), s.flight,
                     output=OUT, **dict(opts))
        o = old_tail[key]
        for q in ("trim_alpha_deg", "trim_cl", "static_margin_mac", "xnp_m"):
            rows.append({"case": key, **compare(q, r[q], float(o[q]), 1e-5)})
        rows.append({"case": key, **compare("cg_x_m", bal.cg_x_m + mm / 1000, float(o["cg_x_m"]), 1e-6)})
    # 8% tail + 3% fin tail-incidence perturbation rows
    o8 = {"tail_area_fraction": 0.08, "tail_incidence_deg": -3.0, "tail_ac_x_m": 0.44,
          "tail_span_fraction": 0.35}
    bal8 = cg_for_margin("repro_t08", s.wing, 0.10, output=OUT, **o8)
    for err in (-1, 1):
        key = f"tol_tail_inc_{err:+g}"
        r = evaluate(f"repro_{key}", bal8, s.flight, output=OUT,
                     **{**o8, "tail_incidence_deg": -3.0 + err, "fin_area_fraction": 0.03,
                        "fin_ac_x_m": 0.48, "fin_type": "center"})
        for q in ("trim_alpha_deg", "trim_cl", "static_margin_mac", "cnb_per_rad"):
            rows.append({"case": key, **compare(q, r[q], float(old_tol[key][q]), 1e-5)})
    return rows


def main() -> int:
    if OUT.exists():
        print(f"{OUT} exists; refusing to overwrite", file=sys.stderr)
        return 2
    OUT.mkdir(parents=True)
    s = Study()
    cfg = s.cfg
    gate = repro_gate(s)
    write_csv(OUT / "repro_gate.csv", gate)
    bad = [g for g in gate if not g["ok"]]
    print(f"repro gate: {len(gate) - len(bad)}/{len(gate)} quantities within tolerance")
    if bad:
        print("REPRO GATE FAILED; stopping", file=sys.stderr)
        for g in bad:
            print(g, file=sys.stderr)
        return 1

    summary: dict = {"note": "inviscid AVL; all tail-on cases include the 3% centre fin unless includes_fin is False",
                     "tail_chord_m": tail_chord(s.wing.area, s.wing.span_m, 0.10, 0.35)}

    # Step 1
    cg_rows = [s.run(f"cg_{mm:g}", mm / 1000) for mm in cg_values_mm(cfg)]
    write_csv(OUT / "cg_scan.csv", cg_rows)
    ok = [r for r in cg_rows if r["status"] == "positive-lift trim"]
    summary["cg_scan_slopes_per_mm"] = {
        k: slope([r["cg_mm"] for r in ok], [r[k] for r in ok])
        for k in ("trim_alpha_deg", "trim_cl", "static_margin_mac", "cma_per_rad", "speed_m_s",
                  "clb_per_rad", "cnb_per_rad", "tail_re")}

    # Step 2
    inc_rows = []
    sweep = cfg["incidence_sweep"]
    for cg in sweep["cg_mm"]:
        for inc in frange(sweep["start_deg"], sweep["stop_deg"], sweep["step_deg"]):
            inc_rows.append(s.run(f"inc_{inc:+g}_cg{cg:g}", cg / 1000, tail_inc=inc))
    write_csv(OUT / "tail_incidence_sweep.csv", inc_rows)
    summary["incidence_slopes_per_deg"] = {}
    for cg in sweep["cg_mm"]:
        sub = [r for r in inc_rows if abs(r["cg_mm"] - cg) < 1e-6 and r["status"] == "positive-lift trim"]
        summary["incidence_slopes_per_deg"][f"cg{cg:g}"] = {
            k: slope([r["tail_incidence_deg"] for r in sub], [r[k] for r in sub])
            for k in ("trim_alpha_deg", "trim_cl", "static_margin_mac", "speed_m_s")}

    # Step 3
    fin_rows = []
    chord = 0.5 * s.wing.mac
    for cg in cfg["fin_map"]["cg_mm"]:
        for ff in cfg["fin_map"]["fin_area_fractions"]:
            r = s.run(f"fin_{ff:g}_cg{cg:g}", cg / 1000, fin_fraction=ff)
            arm = s.f["ac_x_m"] - cg / 1000
            r.update(fin_chord_m=chord if ff else None,
                     fin_height_m=ff * s.wing.area / chord if ff else None,
                     fin_arm_from_cg_m=arm if ff else None,
                     vertical_tail_volume=ff * arm / s.wing.span_m if ff else None)
            fin_rows.append(r)
    write_csv(OUT / "fin_map.csv", fin_rows)

    # Step 4
    tol = cfg["tolerances"]
    tol_rows = []
    for cg in tol["cg_mm"]:
        nominal = s.run(f"tol_nominal_cg{cg:g}", cg / 1000)
        nominal.update(perturbation="nominal", value=0.0)
        tol_rows.append(nominal)
        for v in tol["wing_incidence_mismatch_deg"]:
            r = s.run(f"tol_wing_inc_{v:+g}_cg{cg:g}", cg / 1000, incidence_mismatch_deg=v)
            r.update(perturbation="wing_incidence_mismatch_deg", value=v)
            tol_rows.append(r)
        for v in tol["fin_yaw_alignment_error_deg"]:
            r = s.run(f"tol_fin_yaw_{v:+g}_cg{cg:g}", cg / 1000, fin_alignment_deg=v)
            r.update(perturbation="fin_yaw_alignment_error_deg", value=v,
                     equivalent_beta_deg=equivalent_beta_deg(r["cn_yaw"], nominal["cnb_per_rad"]))
            tol_rows.append(r)
        for v in tol["tail_incidence_error_deg"]:
            r = s.run(f"tol_tail_inc_{v:+g}_cg{cg:g}", cg / 1000, tail_inc=-3.0 + v)
            r.update(perturbation="tail_incidence_error_deg", value=v)
            tol_rows.append(r)
    write_csv(OUT / "tolerances.csv", tol_rows)

    summary["files"] = ["repro_gate.csv", "cg_scan.csv", "tail_incidence_sweep.csv", "fin_map.csv",
                        "tolerances.csv"]
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    flagged = [r["name"] for r in cg_rows + inc_rows + fin_rows + tol_rows if r["flags"]]
    print(f"cg {len(cg_rows)}, incidence {len(inc_rows)}, fin {len(fin_rows)}, tol {len(tol_rows)}; "
          f"flagged: {flagged}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
