"""Independently rerun and check the tail/axis claims before prototype sizing."""

from __future__ import annotations

import json
from dataclasses import replace
from math import pi
from pathlib import Path

from avl import run_avl
from geometry import FlightCondition, Wing
from robustness_study import cg_for_margin, evaluate
from study_geometry import deck


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/prototype/verification"
EXE = ROOT / "tools/avl/avl352.exe"


def run(name: str, wing: Wing, alpha: float, geometry: str, prefix: list[str] | None = None):
    return run_avl(wing, name, [alpha], OUT / name, EXE,
                   deck_text=geometry, oper_prefix=prefix)[0]


def main() -> None:
    baseline = json.loads((ROOT / "rigid_glider/baseline.json").read_text(encoding="utf-8"))
    flight = FlightCondition(**baseline["flight"])
    plain = Wing(**baseline["wing"])
    plain = replace(plain, washout_deg=0, dihedral_deg=0)
    result = {"axis_note": "geometry x aft/z up; AVL output standard x forward/z down; Cl' and Cn' stability axes"}
    verified_cases = {}
    for fraction, label in ((.05, "tail_05"), (.10, "tail_10")):
        options = {"tail_area_fraction": fraction, "tail_incidence_deg": -3}
        wing = cg_for_margin(label, plain, .10, output=OUT, **options)
        screened = evaluate(label, wing, flight, output=OUT, **options)
        verified_cases[label] = (wing, screened)
        alpha = screened["trim_alpha_deg"]
        geometry = deck(wing, label, tail_area_fraction=fraction, tail_incidence_deg=-3)
        p = run(label, wing, alpha, geometry)
        span_t = .35 * wing.span_m
        chord_t = fraction * wing.area / span_t
        x_ac_t = wing.root_chord + 1.15 * wing.mac + chord_t / 4
        margin_np = (p.xnp_m - wing.cg_x_m) / wing.mac
        margin_deriv = -p.cma_per_rad / p.cla_per_rad
        expected_cl = .18008 if label == "tail_05" else .31041
        assert abs(p.cl - expected_cl) < .0005
        assert abs(p.cm) < .001
        assert abs(margin_np - margin_deriv) < .001
        result[label] = {"cl": p.cl, "cm_residual": p.cm, "alpha_deg": alpha,
                         "cg_x_m": wing.cg_x_m, "cg_z_m": wing.cg_z_m,
                         "sref_main_wing_m2": wing.area, "cref_main_wing_mac_m": wing.mac,
                         "bref_main_wing_m": wing.span_m,
                         "tail_span_m": span_t, "tail_chord_m": chord_t,
                         "tail_area_m2": span_t * chord_t,
                         "tail_ac_x_m": x_ac_t, "tail_arm_from_cg_m": x_ac_t - wing.cg_x_m,
                         "tail_volume_about_cg": fraction * (x_ac_t - wing.cg_x_m) / wing.mac,
                         "xnp_m": p.xnp_m, "static_margin_np_mac": margin_np,
                         "static_margin_derivative_mac": margin_deriv,
                         "cma_per_rad": p.cma_per_rad, "cla_per_rad": p.cla_per_rad,
                         "cnb_per_rad": p.cnb_per_rad}

    # Finite difference uses AVL stability-axis Cn' at alpha fixed. AVL's
    # derivative is per RADIAN even though the operating beta is in degrees.
    wing, tail10 = verified_cases["tail_10"]
    alpha = tail10["trim_alpha_deg"]
    for fin_fraction in (0.0, .04):
        name = f"beta_fin_{fin_fraction:g}"
        geom = deck(wing, name, tail_area_fraction=.10, tail_incidence_deg=-3,
                    fin_area_fraction=fin_fraction, fin_type="center")
        minus = run(name + "_minus", wing, alpha, geom, ["B B -1"])
        zero = run(name + "_zero", wing, alpha, geom, ["B B 0"])
        plus = run(name + "_plus", wing, alpha, geom, ["B B 1"])
        finite = (plus.cn_yaw - minus.cn_yaw) / (2 * pi / 180)
        assert abs(finite - zero.cnb_per_rad) < .001
        result[name] = {"cnprime_beta_minus_1deg": minus.cn_yaw,
                        "cnprime_beta_plus_1deg": plus.cn_yaw,
                        "finite_difference_cnb_per_rad": finite,
                        "avl_cnb_per_rad": zero.cnb_per_rad}

    geom = deck(wing, "incidence", tail_area_fraction=.10, tail_incidence_deg=-3,
                incidence_control=True)
    left = run("incidence_minus_1", wing, alpha, geom, ["D1 D1 -0.5"])
    right = run("incidence_plus_1", wing, alpha, geom, ["D1 D1 0.5"])
    assert abs(left.cl_roll + right.cl_roll) < .0002
    speed = (2 * flight.wing_loading(wing) / (flight.rho_kg_m3 * tail10["trim_cl"])) ** .5
    torque = .5 * flight.rho_kg_m3 * speed**2 * wing.area * wing.span_m * abs(left.cl_roll)
    result["incidence_mismatch_1deg"] = {"clprime_roll_minus": left.cl_roll,
                                          "clprime_roll_plus": right.cl_roll,
                                          "nominal_speed_m_s": speed,
                                          "roll_torque_magnitude_n_m": torque}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "verification.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
