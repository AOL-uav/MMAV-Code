"""Small, strict file interface to MIT AVL 3.52; no invented stability outputs."""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

from geometry import Wing


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][-+]?\d+)?"


@dataclass(frozen=True)
class AvlPoint:
    alpha_deg: float
    cl: float
    cdi: float
    cm: float
    xnp_m: float
    cla_per_rad: float
    cma_per_rad: float
    clb_per_rad: float
    cnb_per_rad: float
    strip_file: Path
    cl_roll: float = 0.0
    cn_yaw: float = 0.0
    cy: float = 0.0
    cmq_per_rad: float = 0.0
    clp_per_rad: float = 0.0
    cnr_per_rad: float = 0.0


@dataclass(frozen=True)
class AvlStrip:
    y_m: float
    chord_m: float
    area_m2: float
    cl: float
    induced_angle_rad: float


def read_strips(path: Path) -> list[AvlStrip]:
    strips = []
    inside = False
    for line in path.read_text(errors="replace").splitlines():
        if "Strip Forces referred to Strip Area, Chord" in line:
            inside = True
            continue
        if not inside:
            continue
        parts = line.split()
        if len(parts) >= 15 and parts[0].isdigit():
            strips.append(AvlStrip(float(parts[2]), float(parts[4]), float(parts[5]),
                                   float(parts[9]), float(parts[7])))
    if not strips:
        raise ValueError(f"no AVL strip rows in {path}")
    return strips


def geometry_text(wing: Wing, name: str, nchord: int = 8, nspan: int = 24) -> str:
    # Explicit full wing via YDUPLICATE (rather than a symmetry boundary) is
    # necessary for beta, roll, and yaw derivatives.
    if nchord < 2 or nspan < 2:
        raise ValueError("AVL panel counts must each be at least 2")
    return f"""{name}
0.0
0 0 0.0
{wing.area:.9f} {wing.mac:.9f} {wing.span_m:.9f}
{wing.cg_x_m:.9f} 0.0 {wing.cg_z_m:.9f}
SURFACE
MainWing
{nchord} 1.0 {nspan} -2.0
YDUPLICATE
0.0
SECTION
0.0 0.0 0.0 {wing.root_chord:.9f} 0.0
NACA
{wing.section[-4:]}
SECTION
{wing.tip_x_le:.9f} {wing.span_m / 2:.9f} {wing.tip_z:.9f} {wing.tip_chord:.9f} {wing.washout_deg:.9f}
NACA
{wing.section[-4:]}
"""


def _field(text: str, key: str) -> float:
    match = re.search(rf"\b{re.escape(key)}\s*=\s*({NUMBER})", text)
    if not match:
        raise ValueError(f"AVL output missing {key}")
    return float(match.group(1).replace("D", "E"))


def run_avl(wing: Wing, name: str, alphas: list[float], case_dir: Path, executable: Path,
            nchord: int = 8, nspan: int = 24, deck_text: str | None = None,
            oper_prefix: list[str] | None = None) -> list[AvlPoint]:
    case_dir.mkdir(parents=True, exist_ok=True)
    for index in range(len(alphas)):
        for extension in ("ft", "st", "fs"):
            (case_dir / f"point_{index:02d}.{extension}").unlink(missing_ok=True)
    deck = case_dir / f"{name}.avl"
    deck.write_text(deck_text if deck_text is not None else geometry_text(wing, name, nchord, nspan), encoding="ascii")
    commands = ["PLOP", "G", "", "OPER", *(oper_prefix or [])]
    for index, alpha in enumerate(alphas):
        commands += [f"A A {alpha:.8f}", "X", "FT", f"point_{index:02d}.ft", "ST", f"point_{index:02d}.st", "FS", f"point_{index:02d}.fs"]
    commands += ["", "QUIT"]
    script = "\n".join(commands) + "\n"
    (case_dir / "commands.in").write_text(script, encoding="ascii")
    result = subprocess.run([str(executable), deck.name], input=script, text=True, cwd=case_dir,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
    (case_dir / "avl.log").write_text(result.stdout, encoding="utf-8", errors="replace")
    if result.returncode:
        raise RuntimeError(f"AVL exited {result.returncode}; see {case_dir / 'avl.log'}")
    points = []
    for index, alpha in enumerate(alphas):
        ft_path = case_dir / f"point_{index:02d}.ft"
        st_path = case_dir / f"point_{index:02d}.st"
        strip_path = case_dir / f"point_{index:02d}.fs"
        if not (ft_path.is_file() and st_path.is_file() and strip_path.is_file()):
            raise RuntimeError(f"AVL did not write all outputs for alpha={alpha}; see {case_dir / 'avl.log'}")
        ft = ft_path.read_text(errors="replace")
        st = st_path.read_text(errors="replace")
        points.append(AvlPoint(alpha, _field(ft, "CLtot"), _field(ft, "CDind"), _field(ft, "Cmtot"),
                               _field(st, "Xnp"), _field(st, "CLa"), _field(st, "Cma"),
                               _field(st, "Clb"), _field(st, "Cnb"), strip_path,
                               _field(ft, "Cl'tot"), _field(ft, "Cn'tot"), _field(ft, "CYtot"),
                               _field(st, "Cmq"), _field(st, "Clp"), _field(st, "Cnr")))
    return points
