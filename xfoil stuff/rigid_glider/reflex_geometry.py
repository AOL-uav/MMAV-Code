"""AVL screen of a localized, fixed trailing-edge-up feature.

The main section remains symmetric and untwisted. AVL CONTROL rotates the
panel normal aft of the hinge; it is *not* a viscous or structural lip model.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import radians, tan

from geometry import Wing


@dataclass(frozen=True)
class Reflex:
    chord_fraction: float
    angle_deg: float
    placement: str = "full"  # full, inner, outer
    split_fraction: float = 0.55
    mismatch_deg: float = 0.0  # right minus left trailing-edge-up angle
    left_incidence_deg: float = 0.0
    right_incidence_deg: float = 0.0

    def __post_init__(self):
        if not 0 < self.chord_fraction < 0.5 or self.placement not in {"full", "inner", "outer"}:
            raise ValueError("invalid reflex chord fraction or placement")
        if not 0 < self.split_fraction < 1:
            raise ValueError("split_fraction must lie in (0,1)")


def deck(wing: Wing, name: str, reflex: Reflex, nchord: int = 20, nspan: int = 32) -> str:
    if wing.washout_deg != 0 or wing.dihedral_deg != 0:
        raise ValueError("this screen requires a planar untwisted main wing")
    if nchord < 12:
        raise ValueError("localized control needs at least 12 chordwise panels")
    text = (f"{name}\n0.0\n0 0 0.0\n"
            f"{wing.area:.9f} {wing.mac:.9f} {wing.span_m:.9f}\n"
            f"{wing.cg_x_m:.9f} 0.0 {wing.cg_z_m:.9f}\n")
    # Separate halves permit independent fixed angles and incidence errors.
    # Left span runs tip -> root so both surface normals point upward.
    # AVL's documented partial-span controls are declared on the two
    # bounding SECTIONs only. A shared middle section may carry both names.
    stations = [0.0, reflex.split_fraction, 1.0]
    for side, side_stations, incidence in (
        ("Left", list(reversed(stations)), reflex.left_incidence_deg),
        ("Right", stations, reflex.right_incidence_deg),
    ):
        text += f"SURFACE\n{side}\n{nchord} 1.0 {nspan} -2.0\nCOMPONENT\n1\n"
        for fraction in side_stations:
            y = fraction * wing.span_m / 2 * (-1 if side == "Left" else 1)
            chord = wing.chord(abs(y))
            x = wing.x_le(abs(y))
            text += (f"SECTION\n{x:.9f} {y:.9f} 0.0 {chord:.9f} {incidence:.9f}\n"
                     f"NACA\n{wing.section[-4:]}\n")
            # Shared middle section terminates one controlled segment and
            # starts the other; no artificial taper across the inactive half.
            active = (reflex.placement == "full" or
                      (reflex.placement == "inner" and fraction <= reflex.split_fraction) or
                      (reflex.placement == "outer" and fraction >= reflex.split_fraction))
            if active:
                text += (f"CONTROL\nrefl{side[0]} 1.0 {1-reflex.chord_fraction:.6f} "
                         f"0.0 1.0 0.0 1.0\n")
    return text
