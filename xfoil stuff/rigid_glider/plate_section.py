"""Flat-plate section geometry: constant thickness, 5:1 elliptical leading edge, 10:1 elliptical trailing edge.

Geometry of Ananda, Sukumar & Selig (2015), generated here from the brief (semi-major axis of the leading-edge
ellipse = 5 x half-thickness, trailing-edge ellipse = 10 x half-thickness, unit chord).  Pure functions, no I/O.

Contour order and lip convention follow reflex_xfoil.coordinates: upper trailing edge first, around the leading
edge, lower surface back to the trailing edge (the first and last points coincide at (1, 0)); a lip rigidly
rotates everything aft of the hinge x = 1 - chord_fraction about the hinge, TE-UP positive.
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import math

LE_RATIO = 5.0
TE_RATIO = 10.0
N_LE = 60           # points per surface on the leading-edge ellipse
N_TE = 80           # points per surface on the trailing-edge ellipse
N_FLAT = 40         # interior points per surface on the constant-thickness part

THICKNESS = {"PLATE_43": 0.043, "PLATE_30": 0.030}


def half_thickness_at(x: float, t: float) -> float:
    """Plate half-height y(x) of the unrotated section."""
    h = t / 2
    a_le, a_te = LE_RATIO * h, TE_RATIO * h
    if x <= 0.0:
        return 0.0
    if x < a_le:
        return h * math.sqrt(max(0.0, 1 - ((a_le - x) / a_le) ** 2))
    if x > 1 - a_te:
        return h * math.sqrt(max(0.0, 1 - ((x - (1 - a_te)) / a_te) ** 2))
    return h


def upper_x_stations(t: float) -> list[float]:
    """x stations from LE to TE: elliptical-angle spacing on both ellipses, uniform on the flat part."""
    h = t / 2
    a_le, a_te = LE_RATIO * h, TE_RATIO * h
    if a_le + a_te >= 1:
        raise ValueError("thickness too large for the ellipse lengths")
    xs = [a_le * (1 - math.cos(0.5 * math.pi * i / N_LE)) for i in range(N_LE + 1)]
    x0, x1 = a_le, 1 - a_te
    xs += [x0 + (x1 - x0) * k / (N_FLAT + 1) for k in range(1, N_FLAT + 1)]
    xs += [x1 + a_te * math.sin(0.5 * math.pi * i / N_TE) for i in range(N_TE + 1)]
    return xs


def coordinates(t: float, chord_fraction: float = 0.10, angle_deg: float = 0.0) -> list[tuple[float, float]]:
    """Closed contour, upper TE first.  angle_deg = 0 gives the clean plate (chord_fraction then unused)."""
    xs = upper_x_stations(t)
    xh = 1 - chord_fraction
    ang = math.radians(angle_deg)
    upper = [(x, half_thickness_at(x, t)) for x in reversed(xs)]          # TE -> LE
    lower = [(x, -half_thickness_at(x, t)) for x in xs[1:]]               # after LE -> TE
    pts = []
    for x, y in upper + lower:
        if angle_deg != 0.0 and x > xh:
            dx = x - xh
            x, y = xh + dx * math.cos(ang) - y * math.sin(ang), dx * math.sin(ang) + y * math.cos(ang)
        pts.append((x, y))
    return pts


def section_coordinates(section: str, chord_fraction: float, angle_deg: float):
    return coordinates(THICKNESS[section], chord_fraction, angle_deg)
