"""Small AVL geometry variants used only by the robustness study.

Sref/Cref/Bref and CG remain those of the main wing. Added stabilizers are
explicit surfaces; they are not hidden in an empirical moment correction.
"""

from __future__ import annotations

from geometry import Wing


def _section(x: float, y: float, z: float, chord: float, incidence: float,
             section: str, reflex: bool = False) -> str:
    result = (f"SECTION\n{x:.9f} {y:.9f} {z:.9f} {chord:.9f} {incidence:.9f}\n"
              f"NACA\n{section[-4:]}\n")
    if reflex:
        result += "CONTROL\nfixed_reflex 1.0 0.80 0.0 1.0 0.0 1.0\n"
    return result


def deck(wing: Wing, name: str, *, left_incidence_deg: float = 0.0,
         right_incidence_deg: float = 0.0, left_sweep_error_deg: float = 0.0,
         right_sweep_error_deg: float = 0.0, tail_area_fraction: float = 0.0,
         tail_incidence_deg: float = -3.0, tail_ac_x_m: float | None = None,
         tail_span_fraction: float = 0.35,
         fin_area_fraction: float = 0.0, fin_ac_x_m: float | None = None,
         fin_type: str = "tip", reflex: bool = False,
         incidence_control: bool = False, fin_yaw_control: bool = False) -> str:
    if tail_area_fraction < 0 or fin_area_fraction < 0:
        raise ValueError("surface area fractions cannot be negative")
    if not 0 < tail_span_fraction < 1:
        raise ValueError("tail span fraction must be between 0 and 1")
    if fin_type not in {"tip", "center"}:
        raise ValueError("fin_type must be tip or center")
    text = (f"{name}\n0.0\n0 0 0.0\n"
            f"{wing.area:.9f} {wing.mac:.9f} {wing.span_m:.9f}\n"
            f"{wing.cg_x_m:.9f} 0.0 {wing.cg_z_m:.9f}\n")
    asymmetric = any(abs(v) > 1e-10 for v in (left_incidence_deg - right_incidence_deg,
                                               left_sweep_error_deg - right_sweep_error_deg))
    if asymmetric:
        # AVL panel orientation must run from left tip to root, then root to
        # right tip; do not mirror the section order for the left surface.
        from math import radians, tan
        left_x = wing.tip_x_le + wing.span_m / 2 * (tan(radians(wing.quarter_chord_sweep_deg + left_sweep_error_deg))
                                                   - tan(radians(wing.quarter_chord_sweep_deg)))
        right_x = wing.tip_x_le + wing.span_m / 2 * (tan(radians(wing.quarter_chord_sweep_deg + right_sweep_error_deg))
                                                    - tan(radians(wing.quarter_chord_sweep_deg)))
        text += "SURFACE\nLeftWing\n8 1.0 24 -2.0\nCOMPONENT\n1\n"
        text += _section(left_x, -wing.span_m / 2, wing.tip_z, wing.tip_chord,
                         wing.washout_deg + left_incidence_deg, wing.section, reflex)
        text += _section(0, 0, 0, wing.root_chord, left_incidence_deg, wing.section, reflex)
        text += "SURFACE\nRightWing\n8 1.0 24 -2.0\nCOMPONENT\n1\n"
        text += _section(0, 0, 0, wing.root_chord, right_incidence_deg, wing.section, reflex)
        text += _section(right_x, wing.span_m / 2, wing.tip_z, wing.tip_chord,
                         wing.washout_deg + right_incidence_deg, wing.section, reflex)
    else:
        text += "SURFACE\nMainWing\n8 1.0 24 -2.0\nYDUPLICATE\n0.0\n"
        text += _section(0, 0, 0, wing.root_chord, right_incidence_deg, wing.section, reflex)
        if incidence_control:
            text += "CONTROL\ndiffinc 1.0 0.0 0.0 1.0 0.0 -1.0\n"
        text += _section(wing.tip_x_le, wing.span_m / 2, wing.tip_z, wing.tip_chord,
                         wing.washout_deg + right_incidence_deg, wing.section, reflex)
        if incidence_control:
            text += "CONTROL\ndiffinc 1.0 0.0 0.0 1.0 0.0 -1.0\n"
    if tail_area_fraction:
        tail_span = tail_span_fraction * wing.span_m
        tail_chord = tail_area_fraction * wing.area / tail_span
        tail_x = (tail_ac_x_m - tail_chord / 4 if tail_ac_x_m is not None
                  else wing.root_chord + 1.15 * wing.mac)
        text += "SURFACE\nFixedTail\n6 1.0 10 -2.0\nYDUPLICATE\n0.0\n"
        text += _section(tail_x, 0, 0, tail_chord, tail_incidence_deg, "NACA_0008")
        text += _section(tail_x, tail_span / 2, 0, tail_chord, tail_incidence_deg, "NACA_0008")
    if fin_area_fraction:
        n = 2 if fin_type == "tip" else 1
        chord = 0.5 * wing.mac
        height = fin_area_fraction * wing.area / (n * chord)
        if fin_type == "tip":
            x, y, z = wing.tip_x_le + 0.25 * wing.tip_chord, wing.span_m / 2, wing.tip_z
            text += "SURFACE\nTipFins\n5 1.0 8 -2.0\nYDUPLICATE\n0.0\n"
        else:
            x, y, z = wing.root_chord + 0.75 * wing.mac, 0, 0
            text += "SURFACE\nCenterFin\n5 1.0 8 -2.0\n"
        if fin_ac_x_m is not None:
            x = fin_ac_x_m - 0.30 * chord
        # Top to bottom is AVL's recommended vertical-surface section order.
        text += _section(x + 0.1 * chord, y, z + height, chord, 0, "NACA_0008")
        if fin_yaw_control:
            text += "CONTROL\nfinyaw 1.0 0.0 0.0 0.0 1.0 1.0\n"
        text += _section(x, y, z, chord, 0, "NACA_0008")
        if fin_yaw_control:
            text += "CONTROL\nfinyaw 1.0 0.0 0.0 0.0 1.0 1.0\n"
    return text
