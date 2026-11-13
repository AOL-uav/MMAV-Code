"""Convert a Selig-style airfoil DAT coordinate loop to a closed, mm DXF sketch.

Example:
    py airfoil_dat_to_dxf.py <HOME>\\Downloads\\naca0006.dat --chord-mm 100
"""

import argparse
import math
from pathlib import Path
import sys


def read_airfoil(path: Path) -> list[tuple[float, float]]:
    points = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = line.split("#", 1)[0].split("!", 1)[0].strip()
        if not line:
            continue
        columns = line.replace(",", " ").split()
        try:
            x, y = float(columns[0]), float(columns[1])
        except (ValueError, IndexError):
            if not points:  # The first nonnumeric line may be the airfoil name.
                continue
            raise ValueError(f"Invalid coordinate on line {line_number}: {line}") from None
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"Nonfinite coordinate on line {line_number}")
        if not points or (x, y) != points[-1]:
            points.append((x, y))

    if len(points) > 1 and points[-1] == points[0]:
        points.pop()  # DXF's closed flag supplies this edge.
    if len(points) < 3 or len(set(points)) < 3:
        raise ValueError("At least three distinct contour points are required")
    return points


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a, b, c, d, tolerance):
    ab_c, ab_d = _cross(a, b, c), _cross(a, b, d)
    cd_a, cd_b = _cross(c, d, a), _cross(c, d, b)
    if ((ab_c > tolerance and ab_d < -tolerance or ab_c < -tolerance and ab_d > tolerance)
            and (cd_a > tolerance and cd_b < -tolerance or cd_a < -tolerance and cd_b > tolerance)):
        return True
    for p, u, v, cross in ((c, a, b, ab_c), (d, a, b, ab_d),
                           (a, c, d, cd_a), (b, c, d, cd_b)):
        if (abs(cross) <= tolerance
                and min(u[0], v[0]) - tolerance <= p[0] <= max(u[0], v[0]) + tolerance
                and min(u[1], v[1]) - tolerance <= p[1] <= max(u[1], v[1]) + tolerance):
            return True
    return False


def scale_and_check(points, chord_mm):
    if not math.isfinite(chord_mm) or chord_mm <= 0:
        raise ValueError("Chord must be a positive, finite number of millimeters")
    xmin = min(x for x, _ in points)
    xmax = max(x for x, _ in points)
    if xmax <= xmin:
        raise ValueError("Airfoil has no chordwise extent")
    scale = chord_mm / (xmax - xmin)
    contour = [((x - xmin) * scale, y * scale) for x, y in points]
    if not all(math.isfinite(v) for point in contour for v in point):
        raise ValueError("Scaled coordinate exceeds numeric range")

    n = len(contour)
    area2 = sum(_cross((0, 0), contour[i], contour[(i + 1) % n]) for i in range(n))
    if abs(area2) <= 1e-12 * chord_mm * chord_mm:
        raise ValueError("Contour encloses no usable area")

    tolerance = 1e-12 * chord_mm * chord_mm
    for i in range(n):
        a, b = contour[i], contour[(i + 1) % n]
        if math.dist(a, b) <= 1e-12 * chord_mm:
            raise ValueError(f"Zero-length edge near point {i + 1}")
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue  # Neighboring edges share a vertex by design.
            if _segments_intersect(a, b, contour[j], contour[(j + 1) % n], tolerance):
                raise ValueError(f"Contour crosses or touches itself at edges {i + 1} and {j + 1}")
    return contour


def write_dxf(path: Path, contour):
    # AutoCAD 2000 ASCII DXF: one LWPOLYLINE, with bit 1 of group 70 set.
    rows = [
        "0", "SECTION", "2", "HEADER", "9", "$ACADVER", "1", "AC1015",
        "9", "$INSUNITS", "70", "4",  # 4 = millimeters
        "9", "$MEASUREMENT", "70", "1",
        "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES",
        "0", "LWPOLYLINE", "8", "AIRFOIL", "90", str(len(contour)), "70", "1",
    ]
    for x, y in contour:
        rows.extend(("10", format(x, ".15g"), "20", format(y, ".15g")))
    rows.extend(("0", "ENDSEC", "0", "EOF"))
    path.write_text("\n".join(rows) + "\n", encoding="ascii")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dat", type=Path, help="Selig-style DAT file (upper surface, nose, lower surface)")
    parser.add_argument("--chord-mm", type=float, required=True, help="finished leading-to-trailing edge chord in mm")
    parser.add_argument("--output", type=Path, help="output DXF path (default: beside DAT, with chord in name)")
    if len(sys.argv) == 1:
        try:
            dat = input("DAT file path: ").strip().strip('"\'')
            chord = input("Chord in mm: ").strip()
        except EOFError:
            parser.exit(1, "Error: DAT path and chord are required.\n")
        args = parser.parse_args([dat, "--chord-mm", chord])
    else:
        args = parser.parse_args()
    try:
        contour = scale_and_check(read_airfoil(args.dat), args.chord_mm)
        chord_label = format(args.chord_mm, "g")
        output = args.output or args.dat.with_name(f"{args.dat.stem}_{chord_label}mm.dxf")
        if output.resolve() == args.dat.resolve():
            raise ValueError("Output path must differ from input DAT path")
        write_dxf(output, contour)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Error: {error}\n")
    print(f"Wrote {output} ({len(contour)} vertices, closed polyline, {chord_label} mm chord)")


if __name__ == "__main__":
    main()
