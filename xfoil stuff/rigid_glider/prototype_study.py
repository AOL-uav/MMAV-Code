"""Bounded tail-area/arm, center-fin, and test-article tolerance maps.

Run from any directory: py <repository>/rigid_glider/prototype_study.py
No optimizer, CFD, or invented mass/inertia properties.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, replace
from pathlib import Path

from geometry import FlightCondition, Wing
from plot_svg import plot
from polars import SectionPolars
from robustness_study import cg_for_margin, evaluate


ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(__file__).with_name("prototype_config.json")
OUT = ROOT / "results/runs/rigid_glider/prototype"


def tail_data(wing: Wing, fraction: float, tail_ac_x: float, speed: float | None,
              flight: FlightCondition, tail_span_fraction: float,
              cg_offset_mm: float = 0) -> dict:
    tail_span = tail_span_fraction * wing.span_m
    tail_chord = fraction * wing.area / tail_span
    arm = tail_ac_x - wing.cg_x_m
    return {"tail_area_fraction": fraction, "tail_ac_x_m": tail_ac_x,
            "tail_span_m": tail_span, "tail_chord_m": tail_chord,
            "tail_arm_from_cg_m": arm, "horizontal_tail_volume": fraction * arm / wing.mac,
            "tail_re": speed * tail_chord / flight.nu_m2_s if speed is not None else None,
            "cg_offset_mm": cg_offset_mm}


def compact(row: dict, **extra) -> dict:
    keys = ("name", "status", "trim_alpha_deg", "trim_cl", "static_margin_mac", "cma_per_rad",
            "clb_per_rad", "cnb_per_rad", "cl_roll", "cn_yaw", "speed_m_s",
            "re_root", "re_tip", "cdi_avl", "cd_profile_xfoil", "ld_estimate", "xnp_m")
    return {**{key: row.get(key) for key in keys}, **extra}


def write_csv(path: Path, rows: list[dict]) -> None:
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    wing = Wing(**config["main_wing"])
    flight = FlightCondition(**config["flight"])
    tail = config["tail"]
    fin = config["fin"]
    ref = config["reference_for_tolerance_study_not_final_design"]
    OUT.mkdir(parents=True, exist_ok=True)
    polars = SectionPolars(ROOT / "data/polars/Currentmegapolar.dat", wing.section)

    tail_rows = []
    for fraction in tail["area_fractions"]:
        for ac_x in tail["aerodynamic_center_x_m_from_root_le"]:
            label = f"tail_a{fraction:g}_x{ac_x:g}"
            options = {"tail_area_fraction": fraction,
                       "tail_incidence_deg": tail["incidence_deg"], "tail_ac_x_m": ac_x,
                       "tail_span_fraction": tail["span_fraction_of_main_wing"]}
            balanced = cg_for_margin(label, wing, tail["target_nominal_static_margin_mac"],
                                     output=OUT, **options)
            for mm in tail["cg_offsets_mm"]:
                actual = replace(balanced, cg_x_m=balanced.cg_x_m + mm / 1000)
                result = evaluate(f"{label}_cg{mm:+g}", actual, flight, output=OUT, **options)
                tail_rows.append(compact(result, cg_x_m=actual.cg_x_m,
                                         **tail_data(actual, fraction, ac_x,
                                                     result.get("speed_m_s"), flight,
                                                     tail["span_fraction_of_main_wing"], mm)))

    ref_options = {"tail_area_fraction": ref["tail_area_fraction"],
                   "tail_incidence_deg": tail["incidence_deg"], "tail_ac_x_m": ref["tail_ac_x_m"],
                   "tail_span_fraction": tail["span_fraction_of_main_wing"]}
    balanced = cg_for_margin("reference", wing, tail["target_nominal_static_margin_mac"],
                             output=OUT, **ref_options)
    reference_no_fin = evaluate("reference_no_fin", balanced, flight, polars,
                                output=OUT, **ref_options)
    fin_rows = []
    for fraction in fin["area_fractions"]:
        for ac_x in fin["aerodynamic_center_x_m_from_root_le"]:
            options = {**ref_options, "fin_area_fraction": fraction,
                       "fin_ac_x_m": ac_x, "fin_type": "center"}
            label = f"fin_a{fraction:g}_x{ac_x:g}"
            result = evaluate(label, balanced, flight, output=OUT, **options)
            arm = ac_x - balanced.cg_x_m
            fin_rows.append(compact(result, fin_area_fraction=fraction,
                                    fin_ac_x_m=ac_x, fin_arm_from_cg_m=arm,
                                    vertical_tail_volume=fraction * arm / balanced.span_m,
                                    fin_chord_m=.5 * balanced.mac,
                                    fin_height_m=fraction * balanced.area / (.5 * balanced.mac)))

    tol = config["exploratory_tolerances_not_measured"]
    fin_options = {**ref_options, "fin_area_fraction": ref["fin_area_fraction"],
                   "fin_ac_x_m": ref["fin_ac_x_m"], "fin_type": "center"}
    tolerance = []
    for mm in tol["cg_mm"]:
        actual = replace(balanced, cg_x_m=balanced.cg_x_m + mm / 1000)
        row = evaluate(f"tol_cg_{mm:+g}", actual, flight, output=OUT, **fin_options)
        tolerance.append(compact(row, perturbation="cg_mm", value=mm))
    for error in tol["tail_incidence_error_deg"]:
        opts = {**fin_options, "tail_incidence_deg": tail["incidence_deg"] + error}
        row = evaluate(f"tol_tail_inc_{error:+g}", balanced, flight, output=OUT, **opts)
        tolerance.append(compact(row, perturbation="tail_incidence_error_deg", value=error))
    for error in tol["wing_incidence_mismatch_deg"]:
        row = evaluate(f"tol_wing_inc_{error:+g}", balanced, flight, output=OUT,
                       incidence_mismatch_deg=error, **fin_options)
        tolerance.append(compact(row, perturbation="wing_incidence_mismatch_deg", value=error))
    for error in tol["wing_sweep_mismatch_deg"]:
        # The split-wing representation changes a coplanar tail's nominal
        # interference result: interpret only deltas from this split baseline.
        row = evaluate(f"tol_wing_sweep_{error:+g}", balanced, flight, output=OUT,
                       left_sweep_error_deg=error / 2 if error else 1e-8,
                       right_sweep_error_deg=-error / 2, **fin_options)
        tolerance.append(compact(row, perturbation="wing_sweep_mismatch_deg_split_model", value=error))
    for error in tol["fin_yaw_alignment_error_deg"]:
        row = evaluate(f"tol_fin_yaw_{error:+g}", balanced, flight, output=OUT,
                       fin_alignment_deg=error, **fin_options)
        tolerance.append(compact(row, perturbation="fin_yaw_alignment_error_deg", value=error))

    payload = {"configuration": config, "reference_wing": asdict(balanced),
               "reference_no_fin": reference_no_fin,
               "tail_map": tail_rows, "fin_map": fin_rows, "tolerances": tolerance}
    (OUT / "prototype_study.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    for group, rows in (("tail_map", tail_rows), ("fin_map", fin_rows), ("tolerances", tolerance)):
        write_csv(OUT / f"{group}.csv", rows)

    nominal = [r for r in tail_rows if r["cg_offset_mm"] == 0]
    plot(OUT / "tail_cl_vs_arm.svg", "Tail sizing: trim CL at nominal 10% margin",
         "tail AC arm from CG (m)", "trim CL",
         [(f"S_t/S={f:g}", [(r["tail_arm_from_cg_m"], r["trim_cl"])
                            for r in nominal if r["tail_area_fraction"] == f])
          for f in tail["area_fractions"]], True)
    plot(OUT / "tail_cl_vs_volume.svg", "Tail sizing: trim CL versus horizontal tail volume",
         "V_H about CG", "trim CL",
         [(f"S_t/S={f:g}", [(r["horizontal_tail_volume"], r["trim_cl"])
                            for r in nominal if r["tail_area_fraction"] == f])
          for f in tail["area_fractions"]], True)
    plot(OUT / "fin_cnb_vs_volume.svg", "Center-fin sizing: Cn_beta versus vertical tail volume",
         "V_V about CG", "Cn_beta per rad",
         [(f"S_v/S={f:g}", [(r["vertical_tail_volume"], r["cnb_per_rad"])
                            for r in fin_rows if r["fin_area_fraction"] == f])
          for f in fin["area_fractions"]], True)
    for kind, key, ylabel in (("cg_mm", "trim_cl", "trim CL"),
                              ("wing_incidence_mismatch_deg", "cl_roll", "Cl' roll"),
                              ("fin_yaw_alignment_error_deg", "cn_yaw", "Cn' yaw")):
        group = [r for r in tolerance if r["perturbation"] == kind]
        plot(OUT / f"tol_{kind}.svg", f"Exploratory {kind}", kind, ylabel,
             [(ylabel, [(r["value"], r[key]) for r in group])], True)
    print(f"Wrote {len(tail_rows)} tail/CG, {len(fin_rows)} fin, and "
          f"{len(tolerance)} tolerance cases to {OUT}")


if __name__ == "__main__":
    main()
