"""Geometry and atmosphere shared by the AVL and polar calculations."""

from dataclasses import dataclass
from math import radians, tan
import re


@dataclass(frozen=True)
class Wing:
    span_m: float
    taper: float
    quarter_chord_sweep_deg: float
    washout_deg: float
    dihedral_deg: float
    section: str
    cg_x_m: float
    aspect_ratio: float | None = None
    area_m2: float | None = None
    vertical_surfaces: bool = False
    cg_z_m: float = 0.0

    def __post_init__(self):
        if self.span_m <= 0 or self.taper <= 0 or self.taper > 1.5:
            raise ValueError("span and taper must be positive; taper must be <= 1.5")
        if (self.aspect_ratio is None) == (self.area_m2 is None):
            raise ValueError("specify exactly one of aspect_ratio or area_m2")
        if self.aspect_ratio is not None and self.aspect_ratio <= 0:
            raise ValueError("aspect_ratio must be positive")
        if self.area_m2 is not None and self.area_m2 <= 0:
            raise ValueError("area_m2 must be positive")
        if not re.fullmatch(r"NACA_\d{4}", self.section):
            raise ValueError("section must be a four-digit NACA name such as NACA_0008")
        if self.vertical_surfaces:
            raise ValueError("vertical surfaces are reserved for a later explicit geometry model")

    @property
    def area(self) -> float:
        return self.area_m2 if self.area_m2 is not None else self.span_m**2 / self.aspect_ratio

    @property
    def ar(self) -> float:
        return self.span_m**2 / self.area

    @property
    def root_chord(self) -> float:
        return 2 * self.area / (self.span_m * (1 + self.taper))

    @property
    def tip_chord(self) -> float:
        return self.taper * self.root_chord

    @property
    def mac(self) -> float:
        return 2 * self.root_chord * (1 + self.taper + self.taper**2) / (3 * (1 + self.taper))

    @property
    def mac_y(self) -> float:
        return self.span_m * (1 + 2 * self.taper) / (6 * (1 + self.taper))

    @property
    def tip_x_le(self) -> float:
        return self.span_m / 2 * tan(radians(self.quarter_chord_sweep_deg)) + (self.root_chord - self.tip_chord) / 4

    @property
    def tip_z(self) -> float:
        return self.span_m / 2 * tan(radians(self.dihedral_deg))

    def chord(self, y_m: float) -> float:
        if not 0 <= y_m <= self.span_m / 2:
            raise ValueError("span station outside right semi-span")
        return self.root_chord + (self.tip_chord - self.root_chord) * (2 * y_m / self.span_m)

    def x_le(self, y_m: float) -> float:
        return self.tip_x_le * (2 * y_m / self.span_m)


@dataclass(frozen=True)
class FlightCondition:
    rho_kg_m3: float = 1.225
    nu_m2_s: float = 1.5e-5
    g_m_s2: float = 9.80665
    wing_loading_n_m2: float | None = None
    mass_kg: float | None = None

    def __post_init__(self):
        if min(self.rho_kg_m3, self.nu_m2_s, self.g_m_s2) <= 0:
            raise ValueError("atmospheric and gravity values must be positive")
        if (self.wing_loading_n_m2 is None) == (self.mass_kg is None):
            raise ValueError("specify exactly one of wing_loading_n_m2 or mass_kg")
        if self.wing_loading_n_m2 is not None and self.wing_loading_n_m2 <= 0:
            raise ValueError("wing loading must be positive")
        if self.mass_kg is not None and self.mass_kg <= 0:
            raise ValueError("mass must be positive")

    def wing_loading(self, wing: Wing) -> float:
        return self.wing_loading_n_m2 if self.wing_loading_n_m2 is not None else self.mass_kg * self.g_m_s2 / wing.area

    def reference_mass(self, wing: Wing) -> float:
        return self.wing_loading(wing) * wing.area / self.g_m_s2
