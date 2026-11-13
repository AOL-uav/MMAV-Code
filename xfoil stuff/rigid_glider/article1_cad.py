"""Article 1 CAD package: P07 planform at one or more spans, for the printed NACA 0006 wing and the sheet (planar) wing.

Per span, writes to results/runs/rigid_glider/article1_cad/span_<mm>/ :
  planform.dxf, small_parts.dxf, sections.dxf      2D, millimetres (DXF carries no units: import as mm)
  section_root_xyz.txt, section_strip_xyz.txt, section_tip_xyz.txt   3D loft curves (x aft, y spanwise, z up; mm; FLAT panel)
  naca0006_unit.dat                                 unit-chord section, Selig order
  stations.csv, summary.json, drawing.svg
and article1_cad/index.html + scale_table.csv.

Sections are STREAMWISE (parallel to the flight direction), as in the model. The panel is given flat; apply the 3 deg dihedral in
CAD by rotating each panel about its root chord line. Scaling is geometric only: the architecture comparison was run at 0.40 m,
so CG start points at other spans are scaled values, not model output.

Run:  py -B rigid_glider/article1_cad.py [span_mm ...]      (default 400 350)
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/article1_cad"
AR, TAPER, SWEEP, DIH, TC = 4.5, 0.5, 25.0, 3.0, 0.06
REF_SPAN = 400.0
RHO, NU, G = 1.225, 1.5e-5, 9.80665
CG_START_REF = {"pure": 64.0, "lip_20_10": 54.0, "lip_15_8": 56.0, "tail_10pct": 62.0}   # mm at 0.40 m (ARTICLE1.md)
NP_REF = {"pure_tip_fins": 71.11, "tail_10pct": 82.50}                                       # mm at 0.40 m (AVL)


def naca00(t: float, n: int = 81) -> list[tuple[float, float]]:
    """Half-thickness distribution, standard NACA 4-digit (open trailing edge, a4 = -0.1015), cosine spacing LE -> TE."""
    pts = []
    for i in range(n):
        x = 0.5 * (1 - math.cos(math.pi * i / (n - 1)))
        pts.append((x, 5 * t * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1015 * x ** 4)))
    return pts


HALF_T = naca00(TC)


def half_thickness(xc: float) -> float:
    return 5 * TC * (0.2969 * math.sqrt(xc) - 0.1260 * xc - 0.3516 * xc * xc + 0.2843 * xc ** 3 - 0.1015 * xc ** 4)


class Geo:
    def __init__(self, span: float):
        self.b, self.k = span, span / REF_SPAN
        self.S = span * span / AR
        self.cr = 2 * (self.S / span) / (1 + TAPER)
        self.ct = TAPER * self.cr
        self.half = span / 2
        self.tan = math.tan(math.radians(SWEEP))
        self.y_strip = 0.4 * self.half
        self.join = 12.0
        self.fin_h = 0.04 * self.S / self.ct
        self.tail_span, self.tail_chord, self.tail_qc = 140.0 * self.k, 25.0 * self.k, 2.0 * self.cr
        self.cfin_chord, self.cfin_h, self.cfin_qc = 36.0 * self.k, 59.0 * self.k, 1.5 * self.cr
        self.mac = 2 / 3 * self.cr * (1 + TAPER + TAPER ** 2) / (1 + TAPER)

    def c(self, y): return self.cr + (self.ct - self.cr) * abs(y) / self.half
    def le(self, y): return 0.25 * self.cr + abs(y) * self.tan - 0.25 * self.c(y)
    def te(self, y): return self.le(y) + self.c(y)

    def section_loop(self, y):                                  # (x aft, z up) closed loop: upper TE -> LE -> lower TE
        c, x0 = self.c(y), self.le(y)
        up = [(x0 + x * c, z * c) for x, z in reversed(HALF_T)]
        lo = [(x0 + x * c, -z * c) for x, z in HALF_T[1:]]
        return up + lo


# ------------------------------------------------------------------ DXF (R12 ASCII, entities only)
def dxf_poly(pts, layer, closed=False) -> str:
    s = f"0\nPOLYLINE\n8\n{layer}\n66\n1\n70\n{1 if closed else 0}\n"
    for x, y in pts:
        s += f"0\nVERTEX\n8\n{layer}\n10\n{x:.4f}\n20\n{y:.4f}\n30\n0.0\n"
    return s + "0\nSEQEND\n"


def dxf_file(entities: list[str]) -> str:
    return "0\nSECTION\n2\nENTITIES\n" + "".join(entities) + "0\nENDSEC\n0\nEOF\n"


def rect(x, y, w, h): return [(x, y), (x + w, y), (x + w, y - h), (x, y - h)]


def write_dxfs(g: Geo, out: Path) -> None:
    P = lambda y, x: (y, -x)                                     # DXF X = spanwise (right panel), Y = forward (nose up)
    e = [dxf_poly([P(0, 0), P(g.half, g.le(g.half)), P(g.half, g.te(g.half)), P(0, g.cr)], "OUTLINE", True),
         dxf_poly([P(0, 0.25 * g.cr), P(g.half, g.le(g.half) + 0.25 * g.ct)], "QUARTER_CHORD")]
    for frac, name in ((0.80, "HINGE_80"), (0.85, "HINGE_85")):
        e.append(dxf_poly([P(g.y_strip, g.le(g.y_strip) + frac * g.c(g.y_strip)), P(g.half, g.le(g.half) + frac * g.ct)], name))
    e.append(dxf_poly([P(g.y_strip, g.le(g.y_strip) + 0.80 * g.c(g.y_strip)), P(g.y_strip, g.te(g.y_strip))], "STRIP_START"))
    e.append(dxf_poly([P(g.join, g.le(g.join)), P(g.join, g.te(g.join))], "JOINER_LIMIT"))
    (out / "planform.dxf").write_text(dxf_file(e), encoding="ascii")

    e, y = [], 0.0
    for name, w, h in (("TIP_FIN", g.ct, g.fin_h), ("TIP_FIN", g.ct, g.fin_h), ("TAIL_10PCT", g.tail_span, g.tail_chord),
                       ("TAIL_8PCT", g.tail_span, g.tail_chord * 0.8), ("CENTRE_FIN", g.cfin_chord, g.cfin_h)):
        e.append(dxf_poly(rect(0, y, w, h), name, True))
        y -= h + 8
    (out / "small_parts.dxf").write_text(dxf_file(e), encoding="ascii")

    e, off = [], 0.0
    for name, ys in (("SECTION_ROOT", 0.0), ("SECTION_STRIP_START", g.y_strip), ("SECTION_TIP", g.half)):
        x0 = g.le(ys)
        e.append(dxf_poly([(x - x0, z + off) for x, z in g.section_loop(ys)], name, True))
        xh = 0.8 * g.c(ys)
        e.append(dxf_poly([(xh, off - 0.05 * g.cr), (xh, off + 0.05 * g.cr)], "HINGE_80"))
        off -= 0.16 * g.cr
    (out / "sections.dxf").write_text(dxf_file(e), encoding="ascii")


# ------------------------------------------------------------------ printable solid (STL) and print-mass estimate
TE_MIN = 0.4                                                     # mm, full trailing-edge thickness of the PRINT solid (a manufacturing change)
PRINT_CASES = {"foaming PLA, one 0.42 mm wall (assumed 0.55 g/cm3)": (0.42, 0.55), "foaming PLA, one 0.42 mm wall (assumed 0.70 g/cm3)": (0.42, 0.70),
               "standard PLA, one 0.42 mm wall (1.24 g/cm3)": (0.42, 1.24), "standard PLA, solid (1.24 g/cm3)": (None, 1.24)}


def print_loop(g: Geo, y: float) -> list[tuple[float, float, float]]:
    """Section loop for printing: NACA 0006 plus a linear thickening so the trailing edge is TE_MIN thick. Flat panel, mm."""
    c, x0 = g.c(y), g.le(y)
    add = TE_MIN / 2 - half_thickness(1.0) * c
    up = [(x0 + x * c, y, z * c + add * x) for x, z in reversed(HALF_T)]
    lo = [(x0 + x * c, y, -(z * c + add * x)) for x, z in HALF_T[1:]]
    return up + lo


def _sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def _dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def write_stl(g: Geo, path: Path) -> dict:
    import struct
    r, t = print_loop(g, 0.0), print_loop(g, g.half)
    n = len(r)
    cr_, ct_ = tuple(sum(p[i] for p in r) / n for i in range(3)), tuple(sum(p[i] for p in t) / n for i in range(3))
    skin = []
    for i in range(n):
        j = (i + 1) % n
        skin += [(r[i], r[j], t[j]), (r[i], t[j], t[i])]
    caps = [(cr_, r[i], r[(i + 1) % n]) for i in range(n)] + [(ct_, t[i], t[(i + 1) % n]) for i in range(n)]
    body = tuple((cr_[i] + ct_[i]) / 2 for i in range(3))
    tris, skin_area, vol = [], 0.0, 0.0
    for k, (a, b, c) in enumerate(skin + caps):
        nrm = _cross(_sub(b, a), _sub(c, a))
        cen = tuple((a[i] + b[i] + c[i]) / 3 for i in range(3))
        if _dot(nrm, _sub(cen, body)) < 0:                       # convex body: outward normal points away from the body centre
            b, c = c, b
            nrm = tuple(-v for v in nrm)
        mag = math.sqrt(_dot(nrm, nrm))
        if k < len(skin):
            skin_area += mag / 2
        vol += _dot(a, _cross(b, c)) / 6
        tris.append((tuple(v / mag for v in nrm) if mag else (0.0, 0.0, 0.0), a, b, c))
    with open(path, "wb") as f:
        f.write(b"MMAV Article 1 wing panel (right), mm, flat, NACA 0006 with 0.4 mm TE".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for nrm, a, b, c in tris:
            f.write(struct.pack("<12fH", *nrm, *a, *b, *c, 0))
    est = {}
    for name, (wall, rho) in PRINT_CASES.items():
        panel = (skin_area * wall if wall else vol) / 1000 * rho
        est[name] = dict(one_panel_g=round(panel, 1), both_panels_g=round(2 * panel, 1))
    return dict(triangles=len(tris), skin_area_cm2_one_panel=round(skin_area / 100, 1), volume_cm3_one_panel=round(vol / 1000, 1), te_min_mm=TE_MIN, print_mass=est)


# ------------------------------------------------------------------ tables
def stations(g: Geo) -> list[dict]:
    rows = []
    for name, y in (("root", 0.0), ("joiner limit", g.join), ("MAC", g.b / 6 * (1 + 2 * TAPER) / (1 + TAPER)), ("strip start", g.y_strip), ("tip", g.half)):
        c = g.c(y)
        rows.append(dict(station=name, y_mm=round(y, 2), le_x_mm=round(g.le(y), 2), chord_mm=round(c, 2), te_x_mm=round(g.te(y), 2),
                         max_thickness_mm=round(TC * c, 2), thickness_at_80pct_chord_mm=round(2 * half_thickness(0.8) * c, 2),
                         te_thickness_mm=round(2 * half_thickness(1.0) * c, 3), hinge80_x_mm=round(g.le(y) + 0.8 * c, 2), hinge85_x_mm=round(g.le(y) + 0.85 * c, 2)))
    return rows


def scale_row(g: Geo) -> dict:
    r = dict(span_mm=g.b, root_chord_mm=round(g.cr, 1), tip_chord_mm=round(g.ct, 1), area_cm2=round(g.S / 100, 1), mac_mm=round(g.mac, 1),
             root_thk_mm=round(TC * g.cr, 2), tip_thk_mm=round(TC * g.ct, 2), strip_thk_at_hinge_tip_mm=round(2 * half_thickness(0.8) * g.ct, 2),
             panel_box_mm=f"{g.half:.0f} x {g.te(g.half):.0f}")
    for ws in (6.0, 8.0, 12.0):
        v = math.sqrt(2 * ws / (RHO * 0.3))
        r[f"mass_g_ws{ws:.0f}"] = round(ws * g.S / 1e6 / G * 1000, 1)
        r[f"re_mac_ws{ws:.0f}"] = round(v * g.mac / 1000 / NU, -2)
        r[f"re_tip_ws{ws:.0f}"] = round(v * g.ct / 1000 / NU, -2)
    return r


# ------------------------------------------------------------------ drawing
def L(x1, y1, x2, y2, w=0.35, dash=None, col="#000"):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{col}" stroke-width="{w}"{d}/>'


def T(x, y, s, size=4.6, anchor="start", col="#000"):
    return f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}" fill="{col}" font-family="Arial">{s}</text>'


def PG(pts, w=0.6, fill="none", col="#000"):
    return f'<polygon points="{" ".join(f"{x:.2f},{y:.2f}" for x, y in pts)}" fill="{fill}" stroke="{col}" stroke-width="{w}"/>'


DARK_MAP = {"#000": "#dde3ea", "#444": "#9aa7b4", "#666": "#9aa7b4", "#eef2f6": "#27313c", "#eccfb4": "#6a4a2d", "#c2621a": "#e89a55"}


def dark_svg(svg: str) -> str:
    """On-screen drawing.svg only: dark background, light lines and labels."""
    svg = svg.replace('<rect width="100%" height="100%" fill="white"/>', '<rect width="100%" height="100%" fill="#14181d"/>')
    for a, b in DARK_MAP.items():
        svg = svg.replace(f'"{a}"', f'"{b}"')
    return svg


def drawing(g: Geo, st: list[dict]) -> str:
    W, H, ox, oy = 540.0, 470.0, 270.0, 50.0
    P = lambda y, x: (ox + y, oy + x)
    b = [T(10, 12, f"ARTICLE 1  -  P07 planform, span {g.b:.0f} mm  -  dimensions in mm  -  model-derived, unvalidated", 6.2),
         T(10, 21, "TOP VIEW (nose up).  Sections are streamwise.  Panel drawn flat; dihedral 3 deg per panel applied at the centre joint.", 4.4, col="#444")]
    b.append(PG([P(-g.half, g.le(g.half)), P(0, 0), P(g.half, g.le(g.half)), P(g.half, g.te(g.half)), P(0, g.cr), P(-g.half, g.te(g.half))], 0.7, "#eef2f6"))
    b.append(L(*P(-g.half, g.le(g.half) + 0.25 * g.ct), *P(0, 0.25 * g.cr), 0.25, "4 3", "#666"))
    b.append(L(*P(0, 0.25 * g.cr), *P(g.half, g.le(g.half) + 0.25 * g.ct), 0.25, "4 3", "#666"))
    for s in (1, -1):
        ys = s * g.y_strip
        b.append(PG([P(ys, g.te(ys) - 0.2 * g.c(ys)), P(s * g.half, g.te(g.half) - 0.2 * g.ct), P(s * g.half, g.te(g.half)), P(ys, g.te(ys))], 0.4, "#eccfb4"))
        b.append(L(*P(s * g.half, g.le(g.half)), *P(s * g.half, g.te(g.half)), 1.6))
    b.append(L(*P(0, -6), *P(0, g.cr + 6), 0.25, "8 2 1 2", "#666"))
    # dimensions
    b += [L(*P(-g.half, -22), *P(g.half, -22), 0.3), L(*P(-g.half, -26), *P(-g.half, -18), 0.3), L(*P(g.half, -26), *P(g.half, -18), 0.3), T(*P(0, -24.5), f"span {g.b:.0f}", 5, "middle")]
    b += [L(*P(-10, 0), *P(-10, g.cr), 0.3), T(*P(-13, g.cr * 0.55), f"root {g.cr:.1f}", 4.6, "end")]
    b += [L(*P(g.half + 6, g.le(g.half)), *P(g.half + 6, g.te(g.half)), 0.3), T(*P(g.half + 9, g.le(g.half) + g.ct * 0.55), f"tip {g.ct:.1f}", 4.6)]
    b += [L(*P(g.half + 6, 0), *P(g.half + 6, g.le(g.half)), 0.3, "2 2"), T(*P(g.half + 9, g.le(g.half) * 0.5), f"{g.le(g.half):.1f}", 4.6)]
    b += [L(*P(0, g.cr + 12), *P(g.y_strip, g.cr + 12), 0.3), L(*P(g.y_strip, g.te(g.y_strip)), *P(g.y_strip, g.cr + 15), 0.25, "2 2", "#c2621a"), T(*P(g.y_strip / 2, g.cr + 10), f"{g.y_strip:.0f}", 4.6, "middle")]
    b += [T(*P(g.half * 0.7, g.te(g.half * 0.7) + 10), "removable aft 20 % strip", 4.6, "middle", "#c2621a"), T(*P(-g.half * 0.55, g.le(g.half * 0.55) - 5), f"quarter-chord sweep {SWEEP:.0f} deg", 4.6, "middle", "#444")]
    b += [T(*P(-g.half, g.te(g.half) + 10), f"tip fin {g.ct:.1f} x {g.fin_h:.1f}", 4.6)]
    # front view, true scale
    fy = oy + g.te(g.half) + 62
    rise = g.half * math.tan(math.radians(DIH))
    b.append(T(10, fy - 26, "FRONT VIEW (true scale)", 5))
    for s in (1, -1):
        b.append(L(ox, fy, ox + s * g.half, fy - rise, 1.4))
        b.append(L(ox + s * g.half, fy - rise - g.fin_h / 2, ox + s * g.half, fy - rise + g.fin_h / 2, 1.4))
    b.append(L(ox - g.half, fy, ox + g.half, fy, 0.25, "3 3", "#666"))
    b.append(T(ox, fy + 12, f"3 deg per panel  -  tip rise {rise:.1f}  -  centre joint included angle {180 - 2 * DIH:.0f} deg  -  fins {g.fin_h / 2:.1f} above and below", 4.6, "middle"))
    # sections at 2x
    sy = fy + 52
    b.append(T(10, sy - 22, "SECTIONS, NACA 0006, drawn 2x.  Tick = 80 % chord hinge line.", 5))
    x0 = 14.0
    for name, key in (("root", "root"), ("strip start", "strip start"), ("tip", "tip")):
        r = next(q for q in st if q["station"] == key)
        c = r["chord_mm"]
        loop = [(x0 + 2 * (x - r["le_x_mm"]), sy - 2 * z) for x, z in g.section_loop(r["y_mm"])]
        b.append(PG(loop, 0.5, "#eef2f6"))
        b.append(L(x0 + 2 * 0.8 * c, sy - 6, x0 + 2 * 0.8 * c, sy + 6, 0.3, None, "#c2621a"))
        b.append(T(x0, sy + 16, f"{name}: chord {c:.1f}, max {r['max_thickness_mm']:.2f} thick, {r['thickness_at_80pct_chord_mm']:.2f} at hinge", 4.2))
        x0 += 2 * c + 16 if key == "root" else 2 * c + 12
        if x0 + 2 * g.ct > W - 10:
            x0, sy = 14.0, sy + 40
    # station table
    ty = H - 44
    b.append(T(10, ty, "STATIONS   y from centre | LE x | chord | TE x | max thickness      (x aft of root leading edge)", 4.4, col="#444"))
    for i, r in enumerate(st):
        b.append(T(10, ty + 7 + i * 6, f"{r['station']:<13} {r['y_mm']:>7.2f} | {r['le_x_mm']:>7.2f} | {r['chord_mm']:>7.2f} | {r['te_x_mm']:>7.2f} | {r['max_thickness_mm']:>5.2f}", 4.4).replace('font-family="Arial"', 'font-family="Consolas, monospace" xml:space="preserve"'))
    return f'<svg xmlns="http://www.w3.org/2000/svg" width="{W * 2:.0f}" height="{H * 2:.0f}" viewBox="0 0 {W} {H}">\n<rect width="100%" height="100%" fill="white"/>\n' + "\n".join(b) + "\n</svg>\n"


def build(span: float) -> dict:
    g = Geo(span)
    out = OUT / f"span_{span:.0f}"
    out.mkdir(parents=True, exist_ok=True)
    write_dxfs(g, out)
    for name, y in (("root", 0.0), ("strip", g.y_strip), ("tip", g.half)):
        (out / f"section_{name}_xyz.txt").write_text("".join(f"{x:.4f}\t{y:.4f}\t{z:.4f}\n" for x, z in g.section_loop(y)), encoding="ascii")
    unit = [(x, z) for x, z in reversed(HALF_T)] + [(x, -z) for x, z in HALF_T[1:]]
    (out / "naca0006_unit.dat").write_text("NACA 0006\n" + "".join(f"{x:.6f} {z:.6f}\n" for x, z in unit), encoding="ascii")
    st = stations(g)
    cols = list(st[0])
    (out / "stations.csv").write_text(",".join(cols) + "\n" + "\n".join(",".join(str(r[c]) for c in cols) for r in st) + "\n", encoding="utf-8")
    (out / "drawing.svg").write_text(dark_svg(drawing(g, st)), encoding="utf-8")
    summ = dict(span_mm=span, scale_vs_400=g.k, area_cm2=g.S / 100, root_chord_mm=g.cr, tip_chord_mm=g.ct, mac_mm=g.mac, tip_le_x_mm=g.le(g.half),
                tip_rise_mm=g.half * math.tan(math.radians(DIH)), strip_start_y_mm=g.y_strip, tip_fin_mm=[g.ct, g.fin_h],
                tail_mm=dict(span=g.tail_span, chord=g.tail_chord, quarter_chord_x=g.tail_qc), centre_fin_mm=dict(chord=g.cfin_chord, height=g.cfin_h, quarter_chord_x=g.cfin_qc),
                lip_rise_mm=dict(lip_20_10=[0.2 * g.c(g.y_strip) * math.sin(math.radians(10)), 0.2 * g.ct * math.sin(math.radians(10))],
                                 lip_15_8=[0.15 * g.c(g.y_strip) * math.sin(math.radians(8)), 0.15 * g.ct * math.sin(math.radians(8))]),
                cg_start_mm={k: round(v * g.k, 1) for k, v in CG_START_REF.items()}, avl_neutral_point_mm={k: round(v * g.k, 1) for k, v in NP_REF.items()},
                note="cg_start and neutral point are the 0.40 m values scaled geometrically; only the 0.40 m values are model output")
    summ["print_solid"] = write_stl(g, out / "wing_panel_right.stl")
    (out / "summary.json").write_text(json.dumps(summ, indent=1), encoding="utf-8")
    print(f"span {span:.0f}: print solid", json.dumps(summ["print_solid"]))
    return scale_row(g)


def main() -> None:
    spans = [float(a) for a in sys.argv[1:]] or [400.0, 350.0]
    OUT.mkdir(parents=True, exist_ok=True)
    rows = [build(s) for s in spans] + [scale_row(Geo(300.0))] * (300.0 not in spans)
    rows.sort(key=lambda r: -r["span_mm"])
    cols = list(rows[0])
    (OUT / "scale_table.csv").write_text(",".join(cols) + "\n" + "\n".join(",".join(str(r[c]) for c in cols) for r in rows) + "\n", encoding="utf-8")
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
