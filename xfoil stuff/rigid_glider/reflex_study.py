"""Bounded AVL study of fixed localized reflex. Run: py rigid_glider/reflex_study.py.

No global objective or ranking by L/D. Outputs are screening evidence only.
The prior fixed-tail branch is read for comparison, never modified.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, replace
from math import sqrt
from pathlib import Path

from avl import run_avl
from geometry import FlightCondition, Wing
from reflex_geometry import Reflex, deck


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/reflex"
AVL = ROOT / "tools/avl/avl352.exe"
CONFIG = Path(__file__).with_name("reflex_config.json")


def case(name: str, wing: Wing, feature: Reflex, flight: FlightCondition,
         *, alpha_probe: tuple[float, ...] = (-2, 0, 2, 4, 6, 8)) -> dict:
    controls = [f"D1 D1 {-feature.angle_deg+feature.mismatch_deg/2:.6f}",
                f"D2 D2 {-feature.angle_deg-feature.mismatch_deg/2:.6f}"]
    points = run_avl(wing, name, list(alpha_probe), OUT / "raw" / name, AVL,
                     deck_text=deck(wing, name, feature),
                     oper_prefix=controls)
    zero = next(p for p in points if p.alpha_deg == 0)
    result = {"name": name, "wing": asdict(wing), "reflex": asdict(feature),
              "xnp_m": zero.xnp_m, "cg_x_m": wing.cg_x_m,
              "static_margin_mac": (zero.xnp_m-wing.cg_x_m)/wing.mac,
              "cma_per_rad": zero.cma_per_rad, "clb_per_rad": zero.clb_per_rad,
              "cnb_per_rad": zero.cnb_per_rad,
              "cm0": zero.cm, "cl0": zero.cl,
              "alpha_sweep": [{"alpha_deg": p.alpha_deg, "cl": p.cl, "cm": p.cm,
                               "cdi": p.cdi, "cl_roll": p.cl_roll, "cn_yaw": p.cn_yaw}
                              for p in points]}
    for a, b in zip(points, points[1:]):
        if a.cm == 0 or a.cm*b.cm <= 0:
            alpha = a.alpha_deg if a.cm == 0 else a.alpha_deg - a.cm*(b.alpha_deg-a.alpha_deg)/(b.cm-a.cm)
            p = run_avl(wing, name+"_trim", [alpha], OUT / "raw" / (name+"_trim"), AVL,
                        deck_text=deck(wing, name+"_trim", feature),
                        oper_prefix=controls)[0]
            result.update(trim_alpha_deg=alpha, trim_cl=p.cl, cdi_avl=p.cdi,
                          cm_residual=p.cm, trim_cl_roll=p.cl_roll,
                          trim_cn_yaw=p.cn_yaw, trim_strip_file=str(p.strip_file),
                          xnp_m=p.xnp_m,
                          static_margin_mac=(p.xnp_m-wing.cg_x_m)/wing.mac,
                          cma_per_rad=p.cma_per_rad, clb_per_rad=p.clb_per_rad,
                          cnb_per_rad=p.cnb_per_rad)
            if p.cl > 0:
                speed = sqrt(2*flight.wing_loading(wing)/(flight.rho_kg_m3*p.cl))
                result.update(speed_m_s=speed, re_root=speed*wing.root_chord/flight.nu_m2_s,
                              re_tip=speed*wing.tip_chord/flight.nu_m2_s)
            break
    return result


def save_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    keys = ["name", "sweep_deg", "chord_fraction", "angle_deg", "placement",
            "cg_x_m", "static_margin_mac", "trim_cl", "trim_alpha_deg",
            "speed_m_s", "re_root", "re_tip", "cdi_avl", "cm0",
            "cma_per_rad", "clb_per_rad", "cnb_per_rad", "trim_cl_roll"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (r["wing"]["quarter_chord_sweep_deg"] if k == "sweep_deg" else
                                 r["reflex"][k] if k in {"chord_fraction", "angle_deg", "placement"} else
                                 r.get(k)) for k in keys})


def main() -> None:
    cfg = json.loads(CONFIG.read_text())
    OUT.mkdir(parents=True, exist_ok=True)
    wing = Wing(**cfg["reference"])
    flight = FlightCondition(**cfg["flight"])
    screen = cfg["screen"]
    rows = []
    for sweep in screen["sweeps_deg"]:
        test_wing = replace(wing, quarter_chord_sweep_deg=sweep)
        probe = case(f"np_s{sweep}", test_wing, Reflex(.1, 0), flight)
        test_wing = replace(test_wing, cg_x_m=probe["xnp_m"]-screen["static_margin"]*wing.mac)
        for placement in screen["placements"]:
            for chord in screen["chord_fractions"]:
                for angle in screen["angles_deg"]:
                    name = f"s{sweep}_{placement}_c{int(chord*100)}_a{angle}"
                    rows.append(case(name, test_wing, Reflex(chord, angle, placement), flight))
    # Nominally feasible cases, for tolerances: choose least angular geometry
    # within each placement rather than an arbitrary blended objective.
    low, high = screen["trim_cl_band"]
    good = [r for r in rows if low <= r.get("trim_cl", -1) <= high and
            screen["margin_band"][0] <= r["static_margin_mac"] <= screen["margin_band"][1]]
    good.sort(key=lambda r: (r["reflex"]["angle_deg"], r["reflex"]["chord_fraction"]))
    nominal = next((r for r in good if r["name"] == "s25_full_c10_a3"),
                   next((r for r in good if r["wing"]["quarter_chord_sweep_deg"] == 25),
                        good[0] if good else None))
    tolerances = []
    if nominal:
        base_wing = Wing(**nominal["wing"])
        base = Reflex(**nominal["reflex"])
        errors = cfg["exploratory_errors_not_measured_tolerances"]
        for mm in errors["cg_mm"]:
            tolerances.append({"kind": "cg_mm", "error": mm,
                               **case(f"tol_cg_{mm:+g}", replace(base_wing, cg_x_m=base_wing.cg_x_m+mm/1000), base, flight)})
        for delta in errors["reflex_angle_deg"]:
            tolerances.append({"kind": "reflex_angle_deg", "error": delta,
                               **case(f"tol_angle_{delta:+g}", base_wing, replace(base, angle_deg=base.angle_deg+delta), flight)})
        for delta in errors["left_right_reflex_mismatch_deg"]:
            tolerances.append({"kind": "left_right_reflex_mismatch_deg", "error": delta,
                               **case(f"tol_mismatch_{delta:+g}", base_wing, replace(base, mismatch_deg=delta), flight)})
        for fraction in errors["reflex_chord_fraction"]:
            tolerances.append({"kind": "reflex_chord_fraction", "error": fraction-base.chord_fraction,
                               **case(f"tol_chord_{fraction:g}", base_wing, replace(base, chord_fraction=fraction), flight)})
        for delta in errors["incidence_mismatch_deg"]:
            tolerances.append({"kind": "incidence_mismatch_deg", "error": delta,
                               **case(f"tol_incidence_{delta:+g}", base_wing,
                                      replace(base, left_incidence_deg=-delta/2, right_incidence_deg=delta/2), flight)})
    (OUT / "static.json").write_text(json.dumps({"config": cfg, "cases": rows,
                                                  "tolerances": tolerances,
                                                  "nominal": nominal}, indent=2))
    save_csv(OUT / "static.csv", rows)
    save_csv(OUT / "tolerances.csv", tolerances)
    print(f"{len(rows)} static cases; {len(good)} within exploratory bands; "
          f"{len(tolerances)} tolerance cases. Output: {OUT}")
    if nominal:
        print("Nominal study case:", nominal["name"], "CL", nominal["trim_cl"],
              "SM", nominal["static_margin_mac"])


if __name__ == "__main__":
    main()
