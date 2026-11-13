"""Bounded architecture/robustness study, deliberately not an optimizer.

Run: py rigid_glider/robustness_study.py
Results: results/runs/rigid_glider/robustness/
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, replace
from math import sqrt
from pathlib import Path

from avl import read_strips, run_avl
from geometry import FlightCondition, Wing
from plot_svg import plot
from polars import PolarCoverageError, SectionPolars
from study_geometry import deck


ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
OUT = ROOT / "results/runs/rigid_glider/robustness"
EXE = ROOT / "tools/avl/avl352.exe"
POLARS = ROOT / "data/polars/Currentmegapolar.dat"
ALPHAS = [-4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0]


def point(name: str, wing: Wing, alpha: float, *, output: Path = OUT, **options):
    reflex = "reflex_deflection_deg" in options
    prefix = [f"D1 D1 {options.pop('reflex_deflection_deg')}"] if reflex else None
    incidence = options.pop("incidence_mismatch_deg", None)
    fin_yaw = options.pop("fin_alignment_deg", None)
    if fin_yaw is not None:
        if prefix is not None or incidence is not None:
            raise ValueError("fin alignment is a separate control test")
        prefix = [f"D1 D1 {fin_yaw}"]
    if incidence is not None:
        if prefix is not None:
            raise ValueError("reflex and differential incidence controls are separate tests")
        prefix = [f"D1 D1 {incidence / 2}"]
    geometry = deck(wing, name, reflex=reflex,
                    incidence_control=incidence is not None,
                    fin_yaw_control=fin_yaw is not None, **options)
    return run_avl(wing, name, [alpha], output / "raw" / name, EXE,
                   deck_text=geometry, oper_prefix=prefix)[0]


def points(name: str, wing: Wing, alphas: list[float], *, output: Path = OUT, **options):
    reflex = "reflex_deflection_deg" in options
    prefix = [f"D1 D1 {options.pop('reflex_deflection_deg')}"] if reflex else None
    incidence = options.pop("incidence_mismatch_deg", None)
    fin_yaw = options.pop("fin_alignment_deg", None)
    if fin_yaw is not None:
        if prefix is not None or incidence is not None:
            raise ValueError("fin alignment is a separate control test")
        prefix = [f"D1 D1 {fin_yaw}"]
    if incidence is not None:
        if prefix is not None:
            raise ValueError("reflex and differential incidence controls are separate tests")
        prefix = [f"D1 D1 {incidence / 2}"]
    geometry = deck(wing, name, reflex=reflex,
                    incidence_control=incidence is not None,
                    fin_yaw_control=fin_yaw is not None, **options)
    return run_avl(wing, name, alphas, output / "raw" / name, EXE,
                   deck_text=geometry, oper_prefix=prefix)


def cg_for_margin(name: str, wing: Wing, margin: float, *, output: Path = OUT, **options) -> Wing:
    probe = point(name + "_np", wing, 0, output=output, **options)
    return replace(wing, cg_x_m=probe.xnp_m - margin * wing.mac)


def trim_alpha(sample) -> float | None:
    for a, b in zip(sample, sample[1:]):
        if abs(a.cm) < 1e-7:
            return a.alpha_deg
        if a.cm * b.cm < 0:
            return a.alpha_deg - a.cm * (b.alpha_deg - a.alpha_deg) / (b.cm - a.cm)
    return sample[-1].alpha_deg if abs(sample[-1].cm) < 1e-7 else None


def evaluate(name: str, wing: Wing, flight: FlightCondition, polars: SectionPolars | None = None,
             *, output: Path = OUT, **options) -> dict:
    sweep = points(name + "_alpha", wing, ALPHAS, output=output, **options)
    alpha = trim_alpha(sweep)
    row = {"name": name, "wing": asdict(wing), "options": options, "area_m2": wing.area,
           "mac_m": wing.mac, "wing_loading_n_m2": flight.wing_loading(wing),
           "cm_at_0": next(p.cm for p in sweep if p.alpha_deg == 0),
           "cl_at_4": next(p.cl for p in sweep if p.alpha_deg == 4),
           "alpha_sweep": [{"alpha_deg": p.alpha_deg, "cl": p.cl, "cm": p.cm} for p in sweep]}
    if alpha is None:
        row["status"] = "no Cm=0 in screened alpha range"
        return row
    p = point(name + "_trim", wing, alpha, output=output, **options)
    margin = (p.xnp_m - wing.cg_x_m) / wing.mac
    row.update({"trim_alpha_deg": alpha, "trim_cl": p.cl, "cm_residual": p.cm,
                "static_margin_mac": margin, "cma_per_rad": p.cma_per_rad,
                "clb_per_rad": p.clb_per_rad, "cnb_per_rad": p.cnb_per_rad,
                "clp_per_rad": p.clp_per_rad, "cmq_per_rad": p.cmq_per_rad,
                "cnr_per_rad": p.cnr_per_rad, "cl_roll": p.cl_roll,
                "cn_yaw": p.cn_yaw, "xnp_m": p.xnp_m, "cdi_avl": p.cdi,
                "status": "positive-lift trim" if p.cl > 0 else "nonpositive-lift Cm=0"})
    if p.cl <= 0:
        return row
    speed = sqrt(2 * flight.wing_loading(wing) / (flight.rho_kg_m3 * p.cl))
    row.update({"speed_m_s": speed, "re_root": speed * wing.root_chord / flight.nu_m2_s,
                "re_tip": speed * wing.tip_chord / flight.nu_m2_s})
    if "reflex_deflection_deg" in options:
        row["polar_note"] = "no XFOIL polar for the deflected trailing-edge section; drag/L/D omitted"
    if polars is not None and options.get("fin_area_fraction", 0) == 0 and "reflex_deflection_deg" not in options:
        try:
            strips = read_strips(p.strip_file)
            # Tail, when present, uses the same NACA 0008 section as the wing.
            total = 0.0
            for strip in strips:
                reynolds = speed * strip.chord_m / flight.nu_m2_s
                _, cd = polars.drag_at_local_cl(reynolds, strip.cl, alpha)
                total += strip.area_m2 * cd
            row["cd_profile_xfoil"] = total / wing.area
            row["ld_estimate"] = p.cl / (p.cdi + row["cd_profile_xfoil"])
        except PolarCoverageError as error:
            row["polar_note"] = str(error)
    return row


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((HERE / "robustness_config.json").read_text(encoding="utf-8"))
    base_cfg = json.loads((HERE / "baseline.json").read_text(encoding="utf-8"))
    base = Wing(**base_cfg["wing"])
    flight = FlightCondition(**base_cfg["flight"])
    polars = SectionPolars(POLARS, "NACA_0008")
    planar = []
    screen = cfg["planar_screen"]
    for sweep in screen["sweep_deg"]:
        for ar in screen["aspect_ratio"]:
            for taper in screen["taper"]:
                name = f"planar_s{sweep}_ar{ar}_t{taper:g}"
                wing = replace(base, quarter_chord_sweep_deg=sweep, aspect_ratio=ar,
                               taper=taper, washout_deg=0, dihedral_deg=0)
                wing = cg_for_margin(name, wing, screen["target_static_margin_mac"])
                planar.append(evaluate(name, wing, flight))

    # Common untwisted 25-degree wing. CG is reset for 10% static margin for
    # each added aerodynamic surface, exposing the needed CG placement.
    simple = replace(base, washout_deg=0, dihedral_deg=0)
    additions = [
        ("plain", {}), ("washout_2", {"washout_deg": -2}),
        ("washout_4", {"washout_deg": -4}),
        ("reflex_1", {"reflex_deflection_deg": -1}),
        ("reflex_2", {"reflex_deflection_deg": -2}),
        ("tail_05", {"tail_area_fraction": .05, "tail_incidence_deg": -3}),
        ("tail_10", {"tail_area_fraction": .10, "tail_incidence_deg": -3}),
    ]
    fixes = []
    candidate_wings = {}
    for name, kwargs in additions:
        wing = replace(simple, washout_deg=kwargs.get("washout_deg", 0))
        options = {key: value for key, value in kwargs.items() if key != "washout_deg"}
        wing = cg_for_margin(name, wing, .10, **options)
        candidate_wings[name] = wing
        fixes.append(evaluate(name, wing, flight, polars, **options))

    # The reference robust candidate is an untwisted wing with one small fixed
    # horizontal surface, not the peak-L/D case. Other candidates are retained.
    candidate = candidate_wings["tail_10"]
    tail_options = {"tail_area_fraction": .10, "tail_incidence_deg": -3}
    tol = cfg["exploratory_perturbations_not_measured_tolerances"]
    perturbations = []
    for mm in tol["cg_error_mm"]:
        perturbations.append(evaluate(f"cg_{mm:+g}mm", replace(candidate, cg_x_m=candidate.cg_x_m + mm / 1000),
                                      flight, **tail_options))
    for mult in tol["wing_loading_multiplier"]:
        condition = replace(flight, wing_loading_n_m2=flight.wing_loading(candidate) * mult)
        perturbations.append(evaluate(f"loading_{mult:g}", candidate, condition, **tail_options))
    for mismatch in tol["incidence_mismatch_deg"]:
        perturbations.append(evaluate(f"incidence_{mismatch:+g}", candidate, flight,
                                      incidence_mismatch_deg=mismatch, **tail_options))
    for mismatch in tol["sweep_mismatch_deg"]:
        # Use the wing-only washed-out candidate here: a split-wing deck with a
        # separate coplanar tail has a sizable AVL wake-interaction artifact.
        sweep_candidate = candidate_wings["washout_4"]
        perturbations.append(evaluate(f"sweep_error_{mismatch:+g}", sweep_candidate, flight,
                                      left_sweep_error_deg=mismatch / 2 if mismatch else 1e-8,
                                      right_sweep_error_deg=-mismatch / 2))
    for error in tol["dihedral_error_deg"]:
        perturbations.append(evaluate(f"dihedral_error_{error:+g}",
                                      replace(candidate, dihedral_deg=candidate.dihedral_deg + error),
                                      flight, **tail_options))
    for angle in (-4, -2):
        perturbations.append(evaluate(f"tail_incidence_{angle:+g}", candidate, flight,
                                      tail_area_fraction=.10, tail_incidence_deg=angle))
    for factor in (0.8, 1.2):
        perturbations.append(evaluate(f"tail_area_{factor:g}", candidate, flight,
                                      tail_area_fraction=.10 * factor, tail_incidence_deg=-3))

    fins = []
    for kind in ("center", "tip"):
        for fraction in cfg["fin_total_area_over_main_wing_area"]:
            name = f"{kind}_fin_{fraction:g}"
            fins.append(evaluate(name, candidate, flight, tail_area_fraction=.10,
                                 tail_incidence_deg=-3, fin_area_fraction=fraction, fin_type=kind))

    section_results = []
    sections = ("NACA_0004", "NACA_0006", "NACA_0008", "NACA_0010", "NACA_0012",
                "NACA_0015", "NACA_1208", "NACA_2408")
    for section in sections:
        polar = SectionPolars(POLARS, section)
        item = {"section": section, "xfoil": {}}
        for reynolds in (80000, 130000):
            item["xfoil"][str(reynolds)] = {}
            for cl in (.2, .4):
                try:
                    alpha, cd = polar.drag_at_local_cl(reynolds, cl, 2)
                    item["xfoil"][str(reynolds)][str(cl)] = {"alpha_deg": alpha, "cd": cd}
                except PolarCoverageError as error:
                    item["xfoil"][str(reynolds)][str(cl)] = {"unavailable": str(error)}
        wing = replace(simple, section=section)
        wing = cg_for_margin(f"section_{section}", wing, .10)
        item["avl_plain"] = evaluate(f"section_{section}", wing, flight)
        section_results.append(item)

    payload = {"configuration": cfg, "planar": planar, "fixes": fixes,
               "perturbations": perturbations, "fins": fins, "sections": section_results,
               "reference_candidate": {"wing": asdict(candidate), "tail": tail_options}}
    (OUT / "study.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    for group in ("planar", "fixes", "perturbations", "fins"):
        rows = payload[group]
        columns = ("name", "status", "trim_alpha_deg", "trim_cl", "static_margin_mac", "cma_per_rad",
                   "clb_per_rad", "cnb_per_rad", "cl_roll", "cn_yaw", "speed_m_s", "re_root", "re_tip",
                   "cd_profile_xfoil", "cdi_avl", "ld_estimate", "cm_at_0", "cl_at_4")
        with (OUT / f"{group}.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
    planar_plot = [row for row in planar if row["wing"]["aspect_ratio"] == 7
                   and row["wing"]["taper"] == .7]
    plot(OUT / "planar_cm.svg", "Untwisted symmetric wing: Cm vs alpha at 10% static margin",
         "alpha (deg)", "Cm about CG",
         [(f"sweep {r['wing']['quarter_chord_sweep_deg']:g} deg",
           [(p["alpha_deg"], p["cm"]) for p in r["alpha_sweep"]]) for r in planar_plot], True)
    plot(OUT / "fixes_cm.svg", "Simple pitch fixes: Cm vs alpha at 10% static margin",
         "alpha (deg)", "Cm about CG",
         [(r["name"], [(p["alpha_deg"], p["cm"]) for p in r["alpha_sweep"]])
          for r in fixes if r["name"] in ("plain", "washout_4", "reflex_2", "tail_05", "tail_10")], True)
    cg_rows = [r for r in perturbations if r["name"].startswith("cg_")]
    for key, label in (("trim_cl", "Trim CL"), ("static_margin_mac", "Static margin / MAC"),
                       ("speed_m_s", "Trim speed (m/s)")):
        plot(OUT / f"cg_{key}.svg", f"Fixed-tail candidate: {label} vs CG error",
             "CG error (mm)", label,
             [(label, [(float(r["name"].split("_")[1].removesuffix("mm")), r.get(key)) for r in cg_rows])])
    inc_rows = [r for r in perturbations if r["name"].startswith("incidence_")]
    plot(OUT / "incidence_roll.svg", "Fixed-tail candidate: residual roll moment from incidence mismatch",
         "left-right incidence mismatch (deg)", "Cl roll at pitch trim",
         [("Cl", [(float(r["name"].split("_")[1]), r.get("cl_roll")) for r in inc_rows])], True)
    plot(OUT / "fin_cnb.svg", "Fixed-tail candidate: yaw stability vs vertical area",
         "total vertical area / main-wing area", "Cn_beta per rad",
         [(kind, [(r["options"]["fin_area_fraction"], r.get("cnb_per_rad"))
                  for r in fins if r["options"]["fin_type"] == kind])
          for kind in ("center", "tip")], True)
    print(f"Wrote {len(planar)} planar, {len(fixes)} fixes, {len(perturbations)} perturbations, "
          f"{len(fins)} fin, and {len(section_results)} section cases to {OUT}")


if __name__ == "__main__":
    main()
