"""Loft two or more airfoil DAT sections into a solid STEP wing.

Run without arguments to answer prompts and save wing_settings.json.
Pass a JSON file to run directly from saved settings.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
import subprocess
import sys

from airfoil_dat_to_dxf import read_airfoil, scale_and_check


def finite_number(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be a number") from None
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def angle(value, name):
    result = finite_number(value, name)
    if abs(result) >= 85:
        raise ValueError(f"{name} must be between -85 and 85 degrees")
    return result


def airfoil_surfaces(path):
    contour = scale_and_check(read_airfoil(path), 1.0)
    nose = min(range(len(contour)), key=lambda i: contour[i][0])
    upper = contour[:nose + 1]
    lower = contour[nose:]
    if lower[-1][0] < upper[0][0]:
        # read_airfoil removes a repeated first/last point for closed DAT loops.
        lower.append(upper[0])
    if len(upper) < 3 or len(lower) < 3:
        raise ValueError(f"{path}: expected upper trailing edge -> nose -> lower trailing edge DAT order")
    if any(upper[i][0] <= upper[i + 1][0] for i in range(len(upper) - 1)):
        raise ValueError(f"{path}: upper surface x coordinates must decrease toward the nose")
    if any(lower[i][0] >= lower[i + 1][0] for i in range(len(lower) - 1)):
        raise ValueError(f"{path}: lower surface x coordinates must increase toward the trailing edge")
    return upper, lower


def sample_surface(surface, x_values):
    ascending = list(reversed(surface)) if surface[0][0] > surface[-1][0] else surface
    xs = [point[0] for point in ascending]
    result = []
    for x in x_values:
        i = max(1, min(bisect.bisect_left(xs, x), len(xs) - 1))
        x0, z0 = ascending[i - 1]
        x1, z1 = ascending[i]
        result.append((x, z0 + (z1 - z0) * (x - x0) / (x1 - x0)))
    return result


def load_config(path):
    config = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(config, dict):
        raise ValueError("Config must be a JSON object")
    sections = config.get("stations")
    if not isinstance(sections, list) or len(sections) < 2:
        raise ValueError("stations must contain at least two sections")

    sweep = angle(config.get("sweep_deg", 0), "sweep_deg")
    dihedral = angle(config.get("dihedral_deg", 0), "dihedral_deg")
    sweep_at = finite_number(config.get("sweep_reference_fraction", 0.25), "sweep_reference_fraction")
    twist_at = finite_number(config.get("twist_axis_fraction", 0.25), "twist_axis_fraction")
    if not (0 <= sweep_at <= 1 and 0 <= twist_at <= 1):
        raise ValueError("Reference fractions must be between 0 and 1")
    mirror = config.get("mirror", True)
    if not isinstance(mirror, bool):
        raise ValueError("mirror must be true or false")
    points_per_side = config.get("points_per_side", 61)
    if isinstance(points_per_side, bool) or not isinstance(points_per_side, int) or not 16 <= points_per_side <= 200:
        raise ValueError("points_per_side must be an integer from 16 to 200")

    parsed = []
    for i, section in enumerate(sections):
        if not isinstance(section, dict):
            raise ValueError(f"stations[{i}] must be a JSON object")
        span = finite_number(section.get("span_mm"), f"stations[{i}].span_mm")
        chord = finite_number(section.get("chord_mm"), f"stations[{i}].chord_mm")
        twist = angle(section.get("twist_deg", 0), f"stations[{i}].twist_deg")
        if chord <= 0 or span < 0:
            raise ValueError(f"stations[{i}] needs positive chord and nonnegative span")
        if i == 0 and span != 0:
            raise ValueError("First station must be at span_mm = 0")
        if parsed and span <= parsed[-1]["span"]:
            raise ValueError("Station span_mm values must increase strictly")
        dat = section.get("dat")
        if not isinstance(dat, str) or not dat.strip():
            raise ValueError(f"stations[{i}].dat must be a DAT file path")
        dat_path = Path(dat).expanduser()
        if not dat_path.is_absolute():
            dat_path = path.parent / dat_path
        parsed.append(dict(span=span, chord=chord, twist=twist, dat=dat_path))

    output_value = config.get("output", str(Path.home() / "Downloads" / f"{path.stem}.step"))
    if not isinstance(output_value, str) or not output_value.lower().endswith(('.step', '.stp')):
        raise ValueError("output must be a .step or .stp path")
    output = Path(output_value).expanduser()
    if not output.is_absolute():
        output = path.parent / output
    if output.resolve() in {path.resolve(), *(s["dat"].resolve() for s in parsed)}:
        raise ValueError("Output path must differ from config and DAT inputs")
    return parsed, output, sweep, dihedral, sweep_at, twist_at, mirror, points_per_side


def make_wing(config_path):
    try:
        import cadquery as cq
    except ImportError as exc:
        raise RuntimeError(
            f"CadQuery is missing from {sys.executable}. Run run_wing_step.cmd or install it with "
            f"\"{sys.executable}\" -m pip install cadquery"
        ) from exc

    stations, output, sweep, dihedral, sweep_at, twist_at, mirror, n = load_config(config_path)
    # All sections use the same cosine-spaced chordwise stations and edge layout.
    ascending_x = [0.5 * (1 - math.cos(math.pi * i / (n - 1))) for i in range(n)]
    airfoils = {}
    for station in stations:
        path = station["dat"]
        if path not in airfoils:
            upper, lower = airfoil_surfaces(path)
            airfoils[path] = (sample_surface(upper, reversed(ascending_x)),
                              sample_surface(lower, ascending_x))

    root_chord = stations[0]["chord"]

    def make_wire(station, side):
        span, chord, twist = station["span"], station["chord"], station["twist"]
        upper, lower = airfoils[station["dat"]]
        theta = math.radians(twist)
        c, s = math.cos(theta), math.sin(theta)
        x_le = sweep_at * (root_chord - chord) + span * math.tan(math.radians(sweep))
        z_base = span * math.tan(math.radians(dihedral))
        pivot = twist_at * chord

        def position(x, z):
            dx = x * chord - pivot
            local_z = z * chord
            return cq.Vector(x_le + pivot + dx * c + local_z * s,
                             side * span,
                             z_base - dx * s + local_z * c)

        upper_points = [position(x, z) for x, z in upper]
        lower_points = [position(x, z) for x, z in lower]
        edges = [cq.Edge.makeSpline(upper_points), cq.Edge.makeSpline(lower_points)]
        if (upper_points[0] - lower_points[-1]).Length > 1e-7:
            edges.append(cq.Edge.makeLine(lower_points[-1], upper_points[0]))
        return cq.Wire.assembleEdges(edges), len(edges)

    wings = []
    for side in (1, -1) if mirror else (1,):
        built = [make_wire(station, side) for station in stations]
        if len({edge_count for _, edge_count in built}) != 1:
            raise ValueError("Mix of open and closed trailing edges; use DAT files with matching trailing-edge types")
        wing = cq.Solid.makeLoft([wire for wire, _ in built], ruled=True)
        if not wing.isValid() or len(wing.Solids()) != 1 or wing.Volume() <= 0:
            raise RuntimeError("Loft did not produce one valid solid; check airfoil shapes and station settings")
        wings.append(wing)
    body = cq.Compound.makeCompound(wings) if mirror else wings[0]
    if not body.isValid() or len(body.Solids()) != len(wings):
        raise RuntimeError("STEP body does not contain the expected separate wing solids")
    output.parent.mkdir(parents=True, exist_ok=True)
    body.exportStep(str(output), unit="MM")
    return output, body.Volume(), 2 * stations[-1]["span"] if mirror else stations[-1]["span"], len(wings)


def prompt_config():
    folder = Path(__file__).parent
    settings_path = folder / "wing_settings.json"
    defaults_path = settings_path if settings_path.exists() else folder / "wing_example.json"
    defaults = json.loads(defaults_path.read_text(encoding="utf-8-sig"))
    if defaults_path != settings_path:
        defaults["output"] = str(Path.home() / "Downloads" / "wing_custom.step")

    def ask(label, default):
        answer = input(f"{label} [{default}]: ").strip()
        return answer.strip('"\'') if answer else default

    def number(label, default):
        return finite_number(ask(label, default), label)

    print("Wing settings: press Enter to keep the value in brackets.")
    print("Lengths are in mm. Span positions are measured from the centerline.")
    mirror_default = "Y" if defaults.get("mirror", True) else "N"
    mirror_answer = str(ask("Make both wings as separate STEP solids? Y/N", mirror_default)).upper()
    if mirror_answer not in {"Y", "N"}:
        raise ValueError("Make both wings must be Y or N")
    count = number("Number of airfoil stations (2 or more)", len(defaults["stations"]))
    if not count.is_integer() or count < 2:
        raise ValueError("Number of airfoil stations must be an integer of at least 2")
    count = int(count)

    old = defaults["stations"]
    sections = []
    for i in range(count):
        if count == len(old):
            base = old[i]
        else:
            fraction = i / (count - 1)
            base = {
                "span_mm": old[-1]["span_mm"] * fraction,
                "dat": old[-1]["dat"] if i == count - 1 else old[0]["dat"],
                "chord_mm": old[0]["chord_mm"] + (old[-1]["chord_mm"] - old[0]["chord_mm"]) * fraction,
                "twist_deg": old[0].get("twist_deg", 0) +
                             (old[-1].get("twist_deg", 0) - old[0].get("twist_deg", 0)) * fraction,
            }
        print(f"\nStation {i + 1} of {count}" + (" (root, at 0 mm)" if i == 0 else ""))
        section = {
            "span_mm": 0 if i == 0 else number("Span position from centerline", base["span_mm"]),
            "dat": ask("Airfoil DAT file", base["dat"]),
            "chord_mm": number("Chord", base["chord_mm"]),
            "twist_deg": number("Twist, positive = nose up", base.get("twist_deg", 0)),
        }
        sections.append(section)

    print("\nWhole-wing geometry")
    config = {
        "output": ask("STEP output file", defaults["output"]),
        "mirror": mirror_answer == "Y",
        "sweep_deg": number("Sweep angle, positive = aft", defaults.get("sweep_deg", 0)),
        "sweep_reference_fraction": number("Sweep reference (0 = leading edge, 0.25 = quarter chord)",
                                           defaults.get("sweep_reference_fraction", 0.25)),
        "dihedral_deg": number("Dihedral angle, positive = tips up", defaults.get("dihedral_deg", 0)),
        "twist_axis_fraction": number("Twist axis (0.25 = quarter chord)",
                                      defaults.get("twist_axis_fraction", 0.25)),
        "points_per_side": int(defaults.get("points_per_side", 61)),
        "stations": sections,
    }
    settings_path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    print(f"Saved settings to {settings_path}")
    return settings_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, nargs="?", help="JSON wing configuration; omit for prompts")
    args = parser.parse_args()
    try:
        import cadquery  # noqa: F401
    except ImportError:
        # Windows often gives `python` and `py` different installations.
        if sys.platform == "win32":
            try:
                probe = subprocess.run(["py", "-c", "import cadquery"],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if probe.returncode == 0:
                    parser.exit(subprocess.call(["py", "-B", str(Path(__file__)), *sys.argv[1:]]))
            except OSError:
                pass
    try:
        config_path = args.config if args.config is not None else prompt_config()
        output, volume, span, solids = make_wing(config_path)
    except EOFError:
        parser.exit(1, "Error: prompts need an interactive terminal. Pass a JSON config file instead.\n")
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        parser.exit(1, f"Error: {error}\n")
    print(f"Wrote {output} ({solids} separate solid{'s' if solids != 1 else ''}, "
          f"{span:g} mm total span, {volume:.3f} mm^3 total volume)")


if __name__ == "__main__":
    main()
