"""One flying-wing baseline and a small manual sensitivity study.

Run from any directory: py <project>/rigid_glider/run.py
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, replace
from math import degrees, sqrt
from pathlib import Path

from avl import AvlPoint, read_strips, run_avl
from geometry import FlightCondition, Wing
from plot_svg import plot
from polars import PolarCoverageError, SectionPolars


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = Path(__file__).with_name("baseline.json")
DEFAULT_OUTPUT = ROOT / "results" / "runs" / "rigid_glider"
DEFAULT_AVL = ROOT / "tools" / "avl" / "avl352.exe"
DEFAULT_POLARS = ROOT / "data" / "polars" / "Currentmegapolar.dat"


def trim_bracket(points: list[AvlPoint]) -> float | None:
    for a, b in zip(points, points[1:]):
        if a.cm == 0:
            return a.alpha_deg
        if a.cm * b.cm < 0:
            return a.alpha_deg - a.cm * (b.alpha_deg - a.alpha_deg) / (b.cm - a.cm)
    return points[-1].alpha_deg if points[-1].cm == 0 else None


def profile_drag(point: AvlPoint, wing: Wing, flight: FlightCondition,
                 polars: SectionPolars, speed: float) -> dict:
    strips = read_strips(point.strip_file)
    area_sum = sum(strip.area_m2 for strip in strips)
    if abs(area_sum / wing.area - 1) > 0.02:
        raise ValueError(f"AVL strip area {area_sum:.6f} inconsistent with S={wing.area:.6f}")
    numerator = 0.0
    reynolds = []
    alpha2d = []
    for strip in strips:
        y_fraction = 2 * abs(strip.y_m) / wing.span_m
        local_re = speed * strip.chord_m / flight.nu_m2_s
        twist = wing.washout_deg * y_fraction
        alpha_hint = point.alpha_deg + twist - degrees(strip.induced_angle_rad)
        alpha, cd = polars.drag_at_local_cl(local_re, strip.cl, alpha_hint)
        numerator += strip.area_m2 * cd
        reynolds.append(local_re)
        alpha2d.append(alpha)
    return {"cd_profile_xfoil": numerator / wing.area,
            "re_local_min": min(reynolds), "re_local_max": max(reynolds),
            "alpha_2d_min_deg": min(alpha2d), "alpha_2d_max_deg": max(alpha2d)}


def analyze(name: str, wing: Wing, flight: FlightCondition, alphas: list[float],
            output: Path, avl_exe: Path, polars: SectionPolars) -> dict:
    case_dir = output / name
    points = run_avl(wing, name, alphas, case_dir / "sweep", avl_exe)
    with (case_dir / "alpha_sweep.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("alpha_deg", "cl_avl", "cdi_avl", "cm_about_cg", "xnp_m", "cma_per_rad", "clb_per_rad", "cnb_per_rad"))
        for point in points:
            writer.writerow((point.alpha_deg, point.cl, point.cdi, point.cm, point.xnp_m,
                             point.cma_per_rad, point.clb_per_rad, point.cnb_per_rad))
    result = {"name": name, "wing": asdict(wing), "flight": asdict(flight),
              "derived": {"area_m2": wing.area, "aspect_ratio": wing.ar,
                          "root_chord_m": wing.root_chord, "tip_chord_m": wing.tip_chord,
                          "mac_m": wing.mac, "tip_x_le_m": wing.tip_x_le, "tip_z_m": wing.tip_z,
                          "wing_loading_n_m2": flight.wing_loading(wing),
                          "reference_mass_kg": flight.reference_mass(wing)},
              "alpha_sweep": [{"alpha_deg": p.alpha_deg, "cl": p.cl, "cm": p.cm} for p in points],
              "screen_alpha_range_deg": [min(alphas), max(alphas)]}
    alpha_trim = trim_bracket(points)
    if alpha_trim is None:
        result["trim"] = None
        result["note"] = "no Cm=0 crossing in screened alpha range"
        return result
    trim = run_avl(wing, name + "_trim", [alpha_trim], case_dir / "trim", avl_exe)[0]
    static_margin = (trim.xnp_m - wing.cg_x_m) / wing.mac
    derivative_margin = -trim.cma_per_rad / trim.cla_per_rad
    if abs(static_margin - derivative_margin) > 0.015:
        raise ValueError(f"AVL neutral-point and derivative static margins disagree: {static_margin}, {derivative_margin}")
    info = {"alpha_deg": alpha_trim, "cl_avl": trim.cl, "cm_about_cg": trim.cm,
            "cdi_avl": trim.cdi, "xnp_m": trim.xnp_m,
            "static_margin_mac": static_margin, "cma_per_rad": trim.cma_per_rad,
            "cla_per_rad": trim.cla_per_rad, "clb_per_rad": trim.clb_per_rad,
            "cnb_per_rad": trim.cnb_per_rad}
    result["trim"] = info
    if trim.cl <= 0:
        result["note"] = "Cm=0, but CL is nonpositive; no positive-lift glide condition"
        return result
    loading = flight.wing_loading(wing)
    speed = sqrt(2 * loading / (flight.rho_kg_m3 * trim.cl))
    info.update({"speed_m_s": speed, "re_root": speed * wing.root_chord / flight.nu_m2_s,
                 "re_tip": speed * wing.tip_chord / flight.nu_m2_s,
                 "required_cl": 2 * loading / (flight.rho_kg_m3 * speed**2)})
    if abs(info["required_cl"] - trim.cl) > 1e-8:
        raise ValueError("weight balance and AVL CL disagree")
    try:
        info.update(profile_drag(trim, wing, flight, polars, speed))
        info["cd_total_estimate"] = trim.cdi + info["cd_profile_xfoil"]
        info["ld_estimate"] = trim.cl / info["cd_total_estimate"]
    except PolarCoverageError as error:
        info["profile_drag_unavailable"] = str(error)
    return result


def study_cases(baseline: Wing) -> list[tuple[str, str, float, Wing]]:
    # One variable changes at a time. This is a diagnostic, not an optimizer.
    cases = []
    for value in (0.0, 10.0, 20.0, 30.0):
        cases.append((f"sweep_{int(value)}", "sweep_deg", value, replace(baseline, quarter_chord_sweep_deg=value)))
    for value in (0.0, -2.0, -4.0):
        cases.append((f"washout_{int(value)}", "washout_deg", value, replace(baseline, washout_deg=value)))
    for value in (0.4, 0.7, 1.0):
        cases.append((f"taper_{value:.1f}", "taper", value, replace(baseline, taper=value)))
    for value in (0.135, 0.155, 0.175):
        cases.append((f"cg_{value:.3f}", "cg_x_m", value, replace(baseline, cg_x_m=value)))
    return cases


def write_study_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as file:
        columns = ("case", "parameter", "value", "trim_alpha_deg", "trim_cl", "trim_speed_m_s",
                   "static_margin_mac", "cma_per_rad", "clb_per_rad", "cnb_per_rad",
                   "cd_profile_xfoil", "cdi_avl", "ld_estimate", "note")
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            trim = row["result"]["trim"] or {}
            writer.writerow({"case": row["name"], "parameter": row["parameter"], "value": row["value"],
                             "trim_alpha_deg": trim.get("alpha_deg"), "trim_cl": trim.get("cl_avl"),
                             "trim_speed_m_s": trim.get("speed_m_s"),
                             "static_margin_mac": trim.get("static_margin_mac"),
                             "cma_per_rad": trim.get("cma_per_rad"), "clb_per_rad": trim.get("clb_per_rad"),
                             "cnb_per_rad": trim.get("cnb_per_rad"),
                             "cd_profile_xfoil": trim.get("cd_profile_xfoil"),
                             "cdi_avl": trim.get("cdi_avl"), "ld_estimate": trim.get("ld_estimate"),
                             "note": row["result"].get("note", trim.get("profile_drag_unavailable", ""))})


def write_report(path: Path, baseline: dict, rows: list[dict]) -> None:
    wing, flight, derived = baseline["wing"], baseline["flight"], baseline["derived"]
    trim = baseline["trim"]
    lines = ["# Rigid flying-wing baseline screening", "",
             "Generated by `py rigid_glider/run.py`. This is a low-order screening case, not a flight prediction.", "",
             "## Geometry and flight condition", "",
             f"- Symmetric {wing['section']}; span {wing['span_m']:.3f} m, area {derived['area_m2']:.6f} m², AR {derived['aspect_ratio']:.2f}.",
             f"- Taper {wing['taper']:.2f}; root/tip chords {derived['root_chord_m']:.4f}/{derived['tip_chord_m']:.4f} m; MAC {derived['mac_m']:.4f} m.",
             f"- Quarter-chord sweep {wing['quarter_chord_sweep_deg']:.1f}°, tip twist {wing['washout_deg']:.1f}° (washout negative), dihedral {wing['dihedral_deg']:.1f}°.",
             f"- CG x={wing['cg_x_m']:.4f} m aft of root leading edge; vertical surfaces OFF.",
             f"- W/S={derived['wing_loading_n_m2']:.2f} N/m² (equivalent mass {derived['reference_mass_kg']:.3f} kg); rho={flight['rho_kg_m3']:.3f} kg/m³; nu={flight['nu_m2_s']:.2g} m²/s.", "",
             "## Baseline trim and stability", ""]
    if trim is None:
        lines.append("No Cm=0 crossing in screened alpha range.")
    else:
        lines += [f"- AVL Cm=0 at alpha={trim['alpha_deg']:.3f}°, CL={trim['cl_avl']:.4f}, Cm residual={trim['cm_about_cg']:+.5f}.",
                  f"- Neutral point x={trim['xnp_m']:.4f} m; static margin={trim['static_margin_mac']:.3f} MAC; Cm_alpha={trim['cma_per_rad']:.3f}/rad.",
                  f"- Cl_beta={trim['clb_per_rad']:+.5f}/rad; Cn_beta={trim['cnb_per_rad']:+.5f}/rad. Positive Cn_beta is the conventional restoring-yaw sign; near zero is inconclusive."]
        if "speed_m_s" in trim:
            lines += [f"- Weight-balanced speed {trim['speed_m_s']:.3f} m/s; root/tip Re={trim['re_root']:.0f}/{trim['re_tip']:.0f}.",
                      f"- AVL CDi={trim['cdi_avl']:.5f}."]
        if "ld_estimate" in trim:
            lines.append(f"- XFOIL area-weighted profile CD={trim['cd_profile_xfoil']:.5f}; additive total CD≈{trim['cd_total_estimate']:.5f}; L/D≈{trim['ld_estimate']:.1f} (screening only).")
        elif "profile_drag_unavailable" in trim:
            lines.append(f"- Profile drag unavailable: {trim['profile_drag_unavailable']}.")
    lines += ["", "## One-at-a-time manual sensitivity", "",
              "| Case | Cm=0 alpha (deg) | CL | Static margin (MAC) | Speed (m/s) | Cn_beta (/rad) | Polar drag? |",
              "|---|---:|---:|---:|---:|---:|---|"]
    for row in rows:
        t = row["result"]["trim"] or {}
        fmt = lambda key, digits: f"{t[key]:.{digits}f}" if key in t else "—"
        drag = "yes" if "cd_profile_xfoil" in t else "no"
        lines.append(f"| {row['name']} | {fmt('alpha_deg', 2)} | {fmt('cl_avl', 4)} | "
                     f"{fmt('static_margin_mac', 3)} | {fmt('speed_m_s', 2)} | "
                     f"{fmt('cnb_per_rad', 5)} | {drag} |")
    lines += ["", "## Interpretation and limits", "",
              "- Positive static margin and positive-lift AVL trim coexist for the baseline, but static equilibrium alone does not establish a stable straight drop glide.",
              "- Zero sweep and 10° sweep are not viable *at this fixed CG/washout*: the former is strongly pitch-unstable and trims at negligible CL, the latter trims at negative CL. The zero-sweep speed is a formal near-zero-CL artifact, not a credible flight prediction. This is not a general zero-sweep impossibility result.",
              "- The baseline Cn_beta is nearly zero/slightly negative, so directional stability is not established. Dynamic stability, control-free launch transients, stall, and low-Re separation remain untested.",
              "- AVL is inviscid: its CL/Cm/derivatives do not vary with Reynolds number here. W/S and trimmed CL determine speed/Re exactly; XFOIL at those local Re supplies only profile CD. This does **not** couple viscous section lift/moment back into trim.",
              "- XFOIL profile CD is interpolated on an attached-flow branch from local AVL strip cl and chord/Re, with no extrapolation outside the stored polar range. Missing drag entries are intentionally blank.",
              "- L/D omits fuselage/winglet/roughness/interference drag; the weight balance uses lift≈weight, accurate to small glide-angle order.",
              "- Sweep, taper, and CG rows change only one input each; identical CG x is not identical fraction of MAC across geometry variants.",
              "", "Raw AVL files, CSV, JSON, and SVG plots in this directory permit independent inspection.", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def make_plots(output: Path, baseline: dict, rows: list[dict]) -> None:
    sweep = baseline["alpha_sweep"]
    marker = baseline["trim"]["alpha_deg"] if baseline["trim"] else None
    plot(output / "baseline_cm.svg", "Baseline: pitch moment about CG", "alpha (deg)", "Cm",
         [("baseline", [(p["alpha_deg"], p["cm"]) for p in sweep])], True, marker)
    plot(output / "baseline_cl.svg", "Baseline: AVL lift", "alpha (deg)", "CL",
         [("baseline", [(p["alpha_deg"], p["cl"]) for p in sweep])], True, marker)
    sweep_rows = [r for r in rows if r["parameter"] == "sweep_deg"]
    plot(output / "sweep_cm.svg", "Sweep sensitivity: Cm vs alpha", "alpha (deg)", "Cm",
         [(f"{r['value']:g} deg", [(p["alpha_deg"], p["cm"]) for p in r["result"]["alpha_sweep"]])
          for r in sweep_rows], True)
    for parameter, label in (("sweep_deg", "Quarter-chord sweep (deg)"),
                             ("washout_deg", "Tip washout (deg)"),
                             ("taper", "Taper ratio"), ("cg_x_m", "CG x from root LE (m)")):
        group = sorted((r for r in rows if r["parameter"] == parameter), key=lambda r: r["value"])
        for key, title, axis in (("static_margin_mac", "Static margin", "MAC fraction"),
                                 ("cl_avl", "Trim CL", "CL"), ("speed_m_s", "Trim speed", "m/s")):
            values = [(r["value"], (r["result"]["trim"] or {}).get(key)) for r in group]
            plot(output / f"{parameter}_{key}.svg", f"{title} vs {parameter}", label, axis,
                 [(title, values)], zero_line=(key != "speed_m_s"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--avl", type=Path, default=DEFAULT_AVL)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    wing = Wing(**config["wing"])
    flight = FlightCondition(**config["flight"])
    alphas = sorted(float(value) for value in config["alpha_grid_deg"])
    if len(alphas) < 2 or len(set(alphas)) != len(alphas):
        raise ValueError("alpha_grid_deg must contain distinct values")
    if not args.avl.is_file():
        raise FileNotFoundError(f"AVL missing: {args.avl}; install official MIT AVL 3.52 for Windows")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    polars = SectionPolars(DEFAULT_POLARS, wing.section)
    baseline = analyze("baseline", wing, flight, alphas, output, args.avl.resolve(), polars)
    rows = []
    for name, parameter, value, variant in study_cases(wing):
        result = analyze(name, variant, flight, alphas, output, args.avl.resolve(), polars)
        rows.append({"name": name, "parameter": parameter, "value": value, "result": result})
    (output / "baseline.json").write_text(json.dumps(baseline, indent=2), encoding="utf-8")
    (output / "study.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    write_study_csv(output / "study.csv", rows)
    write_report(output / "report.md", baseline, rows)
    make_plots(output, baseline, rows)
    print(f"Baseline: {json.dumps(baseline['trim'], indent=2)}")
    print(f"Wrote baseline, {len(rows)} manual variants, CSV, raw AVL outputs, and SVG plots to {output}")


if __name__ == "__main__":
    main()
