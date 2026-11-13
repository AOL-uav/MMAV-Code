"""AVL helpers for the reflex extension study (writes only under reflex_ext/).

Re-implements reflex_study.case() with a configurable output directory,
per-case error capture and an optional widened alpha probe. The trim method
(coarse alpha probe, first Cm sign change, linear interpolation, re-run at the
interpolated alpha) is identical to the original. reflex_geometry.deck() and
avl.run_avl() are reused unchanged.
"""

from __future__ import annotations

import csv
import json
from dataclasses import replace
from math import sqrt
from pathlib import Path

from avl import run_avl
from geometry import FlightCondition, Wing
from reflex_geometry import Reflex, deck

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/reflex_ext"
AVL = ROOT / "tools/avl/avl352.exe"
BASE_CONFIG = Path(__file__).with_name("reflex_config.json")
EXT_CONFIG = Path(__file__).with_name("reflex_ext_config.json")
DEFAULT_PROBE = (-2, 0, 2, 4, 6, 8)


def load_configs():
    base = json.loads(BASE_CONFIG.read_text())
    ext = json.loads(EXT_CONFIG.read_text())
    return base, ext, Wing(**base["reference"]), FlightCondition(**base["flight"])


def lip_area(wing: Wing, f: Reflex) -> float:
    """Planform area (both halves) of the aft chord_fraction*c strip that is active."""
    if f.placement == "full":
        y1, y2 = 0.0, 1.0
    elif f.placement == "inner":
        y1, y2 = 0.0, f.split_fraction
    else:
        y1, y2 = f.split_fraction, 1.0
    half = wing.span_m / 2
    c1, c2 = wing.chord(y1 * half), wing.chord(y2 * half)
    return 2 * f.chord_fraction * (y2 - y1) * half * (c1 + c2) / 2


def _controls(f: Reflex) -> list[str]:
    # Identical to reflex_study.case: right-minus-left TE-up angle = mismatch_deg.
    return [f"D1 D1 {-f.angle_deg + f.mismatch_deg / 2:.6f}",
            f"D2 D2 {-f.angle_deg - f.mismatch_deg / 2:.6f}"]


def _avl(wing, name, alphas, f, out_dir):
    return run_avl(wing, name, list(alphas), out_dir / "raw" / name, AVL,
                   deck_text=deck(wing, name, f), oper_prefix=_controls(f))


def case(name: str, wing: Wing, f: Reflex, flight: FlightCondition,
         out_dir: Path = OUT, widen_probe=None) -> dict:
    """Return a flat result dict. Never raises for AVL failure; sets 'error'.

    widen_probe: if the default probe has no Cm sign change, retry once with
    this wider alpha list (flagged by alpha_range_extended=True).
    """
    row = {"name": name, "sweep_deg": wing.quarter_chord_sweep_deg,
           "chord_fraction": f.chord_fraction, "angle_deg": f.angle_deg,
           "placement": f.placement, "split_fraction": f.split_fraction,
           "mismatch_deg": f.mismatch_deg,
           "incidence_mismatch_deg": f.right_incidence_deg - f.left_incidence_deg,
           "lip_area_m2": lip_area(wing, f), "wing_area_m2": wing.area,
           "cg_x_m": wing.cg_x_m, "alpha_range_extended": False, "error": None}
    row["lip_area_frac"] = row["lip_area_m2"] / wing.area
    try:
        for probe in (DEFAULT_PROBE, tuple(widen_probe) if widen_probe else None):
            if probe is None:
                break
            pts = _avl(wing, name, probe, f, out_dir)
            zero = next(p for p in pts if p.alpha_deg == 0)
            row.update(xnp_m=zero.xnp_m, static_margin_mac=(zero.xnp_m - wing.cg_x_m) / wing.mac,
                       cma_per_rad=zero.cma_per_rad, clb_per_rad=zero.clb_per_rad,
                       cnb_per_rad=zero.cnb_per_rad, cm0=zero.cm, cl0=zero.cl,
                       probe_alphas=list(probe),
                       probe_cm=[p.cm for p in pts], probe_cl=[p.cl for p in pts])
            row["alpha_range_extended"] = probe != DEFAULT_PROBE
            for a, b in zip(pts, pts[1:]):
                if a.cm == 0 or a.cm * b.cm <= 0:
                    alpha = a.alpha_deg if a.cm == 0 else a.alpha_deg - a.cm * (b.alpha_deg - a.alpha_deg) / (b.cm - a.cm)
                    p = _avl(wing, name + "_trim", [alpha], f, out_dir)[0]
                    row.update(trim_alpha_deg=alpha, trim_cl=p.cl, cdi_avl=p.cdi,
                               cm_residual=p.cm, trim_cl_roll=p.cl_roll, trim_cn_yaw=p.cn_yaw,
                               trim_cy=p.cy, trim_strip_file=str(p.strip_file), xnp_m=p.xnp_m,
                               static_margin_mac=(p.xnp_m - wing.cg_x_m) / wing.mac,
                               cma_per_rad=p.cma_per_rad, clb_per_rad=p.clb_per_rad,
                               cnb_per_rad=p.cnb_per_rad)
                    if p.cl > 0:
                        v = sqrt(2 * flight.wing_loading(wing) / (flight.rho_kg_m3 * p.cl))
                        row.update(speed_m_s=v, re_root=v * wing.root_chord / flight.nu_m2_s,
                                   re_tip=v * wing.tip_chord / flight.nu_m2_s)
                    return row
        row["error"] = "no Cm sign change in probe range " + str(row.get("probe_alphas"))
    except Exception as exc:  # timeout, AVL failure, parse failure: record, do not retry
        row["error"] = f"{type(exc).__name__}: {exc}"
    return row


def neutral_point(wing: Wing, flight: FlightCondition, out_dir: Path = OUT) -> float:
    """Same probe as reflex_study.main(): Reflex(.1, 0) full span, Xnp from the run."""
    r = case(f"np_s{wing.quarter_chord_sweep_deg:g}", wing, Reflex(.1, 0), flight, out_dir)
    if r["error"]:
        raise RuntimeError(r["error"])
    return r["xnp_m"]


def wing_at_margin(wing: Wing, xnp: float, margin: float) -> Wing:
    return replace(wing, cg_x_m=xnp - margin * wing.mac)


CSV_KEYS = ["name", "sweep_deg", "chord_fraction", "angle_deg", "placement", "split_fraction",
            "mismatch_deg", "target_margin", "cg_x_m", "static_margin_mac", "xnp_m", "trim_cl",
            "trim_alpha_deg", "speed_m_s", "re_root", "re_tip", "cdi_avl", "cm0",
            "cma_per_rad", "clb_per_rad", "cnb_per_rad", "trim_cl_roll", "trim_cn_yaw",
            "lip_area_m2", "lip_area_frac", "alpha_range_extended", "error"]


def write_csv(path: Path, rows: list[dict], keys=None):
    keys = keys or CSV_KEYS
    with path.open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})
