"""Article 1 (P07 planform, sheet flat-plate build): geometry, 1:1 cutting templates, overview drawing, ballast table.

Geometry is the P07 candidate of ARCH_COMPARISON.md (span 0.40 m, AR 4.5, taper 0.5, quarter-chord sweep 25 deg, 3 deg dihedral
per panel, as in the final AVL deck). Tip fins are the "tip fin 4% sym" option of arch_compare_v2/item9_tipfins.csv (each fin 4 % of
wing area, chord = tip chord, centred on the wing plane). Masses are ASSUMED sheet densities, not measurements.

Writes only to results/runs/rigid_glider/article1/.   Run:  py -B rigid_glider/article1_drawings.py
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/article1"

B, AR, TAPER, SWEEP, DIH = 400.0, 4.5, 0.5, 25.0, 3.0          # mm, -, -, deg, deg per panel
S_MM2 = B * B / AR
CR = 2 * (S_MM2 / B) / (1 + TAPER)
CT = TAPER * CR
HALF = B / 2
TAN = math.tan(math.radians(SWEEP))
RHO, NU, G = 1.225, 1.5e-5, 9.80665
STRIP_Y0, TAIL_SPAN, TAIL_CHORD, TAIL_QC = 80.0, 140.0, 25.0, 237.0
FIN_H = 0.04 * S_MM2 / CT                                      # tip fin height, each fin 4 % of wing area
JOIN = 12.0                                                    # panel root insertion into the centre piece (design choice)


def chord(y): return CR + (CT - CR) * abs(y) / HALF
def le(y): return 0.25 * CR + abs(y) * TAN - 0.25 * chord(y)
def te(y): return le(y) + chord(y)


def derived() -> dict:
    mac = 2 / 3 * CR * (1 + TAPER + TAPER ** 2) / (1 + TAPER)
    y_mac = B / 6 * (1 + 2 * TAPER) / (1 + TAPER)
    n = 2000
    ys = [(i + 0.5) * HALF / n for i in range(n)]
    area = sum(chord(y) for y in ys) * HALF / n
    x_cen = sum(chord(y) * (le(y) + chord(y) / 2) for y in ys) * HALF / n / area
    return dict(span_mm=B, area_cm2=S_MM2 / 100, root_chord_mm=CR, tip_chord_mm=CT, mac_mm=mac, y_mac_mm=y_mac, x_le_mac_mm=le(y_mac),
                le_tip_mm=le(HALF), te_tip_mm=te(HALF), panel_area_cm2=area / 100, wing_area_centroid_x_mm=x_cen,
                tip_rise_mm=HALF * math.tan(math.radians(DIH)), joiner_included_angle_deg=180 - 2 * DIH,
                tip_fin_height_mm=FIN_H, tip_fin_chord_mm=CT, strip_inboard_chord_mm=chord(STRIP_Y0),
                lip_20_10_rise_mm=[0.2 * chord(STRIP_Y0) * math.sin(math.radians(10)), 0.2 * CT * math.sin(math.radians(10))],
                lip_15_8_rise_mm=[0.15 * chord(STRIP_Y0) * math.sin(math.radians(8)), 0.15 * CT * math.sin(math.radians(8))],
                avl_neutral_point_mm=dict(pure_centre_fin_final_deck=70.72, pure_tip_fins_sym=71.11, tail10_final_deck=82.50))


SHEETS = {"foam 3 mm (assumed 40 kg/m3)": 0.040 * 3.0, "balsa 1.5 mm (assumed 160 kg/m3)": 0.160 * 1.5, "balsa 3 mm (assumed 120 kg/m3)": 0.120 * 3.0}  # kg/m2
CENTRE_G, CENTRE_X = 5.0, 35.0                                 # printed centre piece: ASSUMED mass and CG station


def ballast_rows(d: dict) -> list[dict]:
    rows = []
    for name, areal in SHEETS.items():
        m_w = areal * S_MM2 / 1e6 * 1000
        m_f = areal * 2 * FIN_H * CT / 1e6 * 1000
        x_f = le(HALF) + CT / 2
        for ws in (6.0, 8.0, 12.0):
            m_all = ws * S_MM2 / 1e6 / G * 1000
            m_b = m_all - m_w - m_f - CENTRE_G
            for cg in (45, 54, 62, 64, 70, 78):
                x_b = (m_all * cg - m_w * d["wing_area_centroid_x_mm"] - m_f * x_f - CENTRE_G * CENTRE_X) / m_b if m_b > 0 else float("nan")
                v = math.sqrt(2 * ws / (RHO * 0.3))
                rows.append(dict(sheet=name, ws_n_m2=ws, all_up_g=round(m_all, 1), wing_g=round(m_w, 1), fins_g=round(m_f, 2), centre_g=CENTRE_G,
                                 ballast_g=round(m_b, 1), target_cg_mm=cg, ballast_station_mm=round(x_b, 1), speed_cl0p3_m_s=round(v, 2),
                                 re_mac=round(v * d["mac_mm"] / 1000 / NU), re_tip=round(v * CT / 1000 / NU)))
    return rows


# ------------------------------------------------------------------ SVG helpers
def line(x1, y1, x2, y2, w=0.25, dash=None, col="#000"):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{col}" stroke-width="{w}"{d}/>'


def text(x, y, s, size=3.2, anchor="start", col="#000"):
    return f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}" fill="{col}" font-family="Arial">{s}</text>'


def poly(pts, w=0.35, fill="none", col="#000", dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<polygon points="{" ".join(f"{x:.2f},{y:.2f}" for x, y in pts)}" fill="{fill}" stroke="{col}" stroke-width="{w}"{d}/>'


def page(w_mm, h_mm, body, physical=True):
    size = f'width="{w_mm}mm" height="{h_mm}mm"' if physical else f'width="{w_mm * 2.2:.0f}" height="{h_mm * 2.2:.0f}"'
    return f'<svg xmlns="http://www.w3.org/2000/svg" {size} viewBox="0 0 {w_mm} {h_mm}">\n<rect width="100%" height="100%" fill="white"/>\n' + "\n".join(body) + "\n</svg>\n"


def check_bar(x, y):
    out = [line(x, y, x + 100, y, 0.4)] + [line(x + i * 10, y - (2.5 if i % 5 == 0 else 1.5), x + i * 10, y, 0.25) for i in range(11)]
    return out + [text(x + 50, y + 4.5, "CHECK: this bar must measure 100 mm. Print at 100 % scale (no fit-to-page).", 3.0, "middle")]


DARK_MAP = {"#000": "#dde3ea", "#666": "#9aa7b4", "#c2621a": "#e89a55", "#1f5fa8": "#7fb4ff", "#2f7d4f": "#6cc08b", "#dfe7ef": "#27313c", "#eccfb4": "#6a4a2d"}


def dark_svg(svg: str) -> str:
    """On-screen overview only: dark background, light lines and labels.  The 1:1 print templates are NOT passed through this."""
    svg = svg.replace('<rect width="100%" height="100%" fill="white"/>', '<rect width="100%" height="100%" fill="#14181d"/>')
    svg = svg.replace('fill="white"', 'fill="#1c2229"')
    for a, b in DARK_MAP.items():
        svg = svg.replace(f'"{a}"', f'"{b}"')
    return svg


def panel_template() -> str:
    ox, oy = 22.0, 16.0                                         # page offset; span along page x, chord (aft) down the page
    P = lambda y, x: (ox + y, oy + x)
    b = [text(ox, 8, "ARTICLE 1 - WING PANEL, 1:1 - right panel as drawn; turn the template over for the left panel", 3.6),
         poly([P(0, 0), P(HALF, le(HALF)), P(HALF, te(HALF)), P(0, CR)], 0.4)]
    b.append(line(*P(0, 0.25 * CR), *P(HALF, le(HALF) + 0.25 * CT), 0.2, "3 2", "#666"))
    b.append(text(*P(100, le(100) + 0.25 * chord(100) - 1.5), "quarter chord", 2.6, "middle", "#666"))
    for frac, lab in ((0.80, "hinge for 20 % / 10 deg lip (80 % chord)"), (0.85, "hinge for 15 % / 8 deg lip (85 % chord)")):
        b.append(line(*P(STRIP_Y0, le(STRIP_Y0) + frac * chord(STRIP_Y0)), *P(HALF, le(HALF) + frac * CT), 0.25, "1.5 1.5", "#c2621a"))
        b.append(text(*P(STRIP_Y0 - 2, le(STRIP_Y0) + frac * chord(STRIP_Y0) + (1 if frac > 0.8 else -1)), lab, 2.4, "end", "#c2621a"))
    b.append(line(*P(STRIP_Y0, le(STRIP_Y0) + 0.78 * chord(STRIP_Y0)), *P(STRIP_Y0, te(STRIP_Y0) + 3), 0.2, "1 1", "#c2621a"))
    b.append(text(*P(STRIP_Y0, te(STRIP_Y0) + 6.5), "lip starts 80 mm from centre", 2.4, "middle", "#c2621a"))
    b.append(line(*P(JOIN, le(JOIN) - 2), *P(JOIN, te(JOIN) + 2), 0.2, "2 1", "#1f5fa8"))
    b.append(text(*P(JOIN + 1.5, 30), f"insert to here ({JOIN:.0f} mm)", 2.4, "start", "#1f5fa8"))
    for x in range(0, int(CR) + 1, 10):                         # root-edge station ticks, mm aft of root LE
        b += [line(*P(-3, x), *P(0, x), 0.2), text(*P(-4, x + 1), str(x), 2.4, "end")]
    b.append(text(*P(-4, -3), "mm aft of root LE", 2.4, "end"))
    b.append(text(*P(HALF + 2, le(HALF) + CT / 2), f"tip {CT:.1f}", 2.8))
    b.append(text(*P(4, CR + 5), f"root {CR:.1f}", 2.8))
    b.append(text(*P(HALF, le(HALF) - 2), f"tip LE {le(HALF):.1f} aft of root LE", 2.6, "end"))
    b.append(text(*P(100, CR + 22), f"semi-span {HALF:.0f} mm; panel area {S_MM2 / 200:.0f} cm2", 2.8, "middle"))
    b += check_bar(ox + 60, 192)
    return page(250, 200, b)


def small_parts_template() -> str:
    b = [text(12, 8, "ARTICLE 1 - SMALL PARTS, 1:1", 3.6)]
    x0, y0 = 14.0, 20.0
    for i in range(2):                                           # tip fins
        xx = x0 + i * (CT + 14)
        b.append(poly([(xx, y0), (xx + CT, y0), (xx + CT, y0 + FIN_H), (xx, y0 + FIN_H)], 0.4))
        b.append(line(xx, y0 + FIN_H / 2, xx + CT, y0 + FIN_H / 2, 0.2, "2 1", "#1f5fa8"))
        b.append(text(xx + CT / 2, y0 + FIN_H + 4.5, f"tip fin {i + 1}: {CT:.1f} x {FIN_H:.1f}; blue line = wing plane", 2.5, "middle"))
    y1 = y0 + FIN_H + 16
    for j, (c, lab) in enumerate(((TAIL_CHORD, "tail, 10 % of wing area"), (20.0, "tail alternate, 8 %"))):
        yy = y1 + j * 40
        b.append(poly([(x0, yy), (x0 + TAIL_SPAN, yy), (x0 + TAIL_SPAN, yy + c), (x0, yy + c)], 0.4))
        b.append(line(x0 + TAIL_SPAN / 2, yy - 2, x0 + TAIL_SPAN / 2, yy + c + 2, 0.2, "2 1", "#666"))
        b.append(text(x0 + TAIL_SPAN / 2, yy + c + 5, f"{lab}: {TAIL_SPAN:.0f} x {c:.0f}; leading edge at top", 2.5, "middle"))
    yf = y1 + 82
    b.append(poly([(x0, yf), (x0 + 36, yf), (x0 + 36, yf + 59), (x0, yf + 59)], 0.4))
    b.append(text(x0 + 40, yf + 30, "spare centre fin 36 chord x 59 tall (6 %), backup if the tip fins are not enough", 2.5))
    b += check_bar(60, 272)
    return page(200, 282, b)


def overview(d: dict) -> str:
    W, H, ox, oy = 470.0, 430.0, 235.0, 62.0
    P = lambda y, x: (ox + y, oy + x)
    b = [text(12, 12, "ARTICLE 1 OVERVIEW (not to print scale) - P07 planform, dimensions in mm", 6)]
    b.append(poly([P(-HALF, le(HALF)), P(0, 0), P(HALF, le(HALF)), P(HALF, te(HALF)), P(0, CR), P(-HALF, te(HALF))], 0.7, "#dfe7ef"))
    for s in (1, -1):
        b.append(poly([P(s * STRIP_Y0, te(STRIP_Y0) - 0.2 * chord(STRIP_Y0)), P(s * HALF, te(HALF) - 0.2 * CT), P(s * HALF, te(HALF)), P(s * STRIP_Y0, te(STRIP_Y0))], 0.4, "#eccfb4"))
        b.append(line(*P(s * HALF, le(HALF)), *P(s * HALF, te(HALF)), 2.8, None, "#dde3ea"))     # light edge so the dark bar shows on the dark page
        b.append(line(*P(s * HALF, le(HALF)), *P(s * HALF, te(HALF)), 1.6, None, "#18222d"))
    b.append(f'<rect x="{ox - 8}" y="{oy - 30}" width="16" height="{CR + 30 - 20:.1f}" rx="7" fill="white" stroke="#000" stroke-width="0.7"/>')
    b.append(f'<rect x="{ox - 2.5}" y="{oy + 45}" width="5" height="33" fill="#1f5fa8"/>')
    b.append(line(*P(0, CR - 20), *P(0, TAIL_QC + 0.75 * TAIL_CHORD), 0.9, "4 3", "#2f7d4f"))
    b.append(poly([P(-TAIL_SPAN / 2, TAIL_QC - 0.25 * TAIL_CHORD), P(TAIL_SPAN / 2, TAIL_QC - 0.25 * TAIL_CHORD), P(TAIL_SPAN / 2, TAIL_QC + 0.75 * TAIL_CHORD), P(-TAIL_SPAN / 2, TAIL_QC + 0.75 * TAIL_CHORD)], 0.7, "none", "#2f7d4f", "4 3"))
    # dimensions
    b += [line(*P(-HALF, -42), *P(HALF, -42), 0.3), line(*P(-HALF, -46), *P(-HALF, -38), 0.3), line(*P(HALF, -46), *P(HALF, -38), 0.3), text(*P(0, -45), f"span {B:.0f}", 5, "middle")]
    b += [text(*P(12, CR * 0.36), f"root chord {CR:.1f}", 4.5), text(*P(12, 62), "CG scale 45-78", 4.5, "start", "#1f5fa8")]
    b += [text(*P(HALF + 4, le(HALF) + 18), f"tip {CT:.1f}", 4.5), text(*P(HALF + 4, le(HALF) + 26), f"fin {FIN_H:.0f} tall", 4.5)]
    b += [line(*P(HALF + 2, 0), *P(HALF + 2, le(HALF)), 0.3), text(*P(HALF + 4, le(HALF) / 2), f"{le(HALF):.1f}", 4.5)]
    b += [text(*P(140, te(140) + 9), "lip strip, aft 20 %", 4.5, "middle", "#c2621a"), text(*P(-140, te(140) + 9), "from 80 mm out", 4.5, "middle", "#c2621a")]
    b += [text(*P(14, -22), "nose pod, 30 ahead of LE", 4.5), text(*P(TAIL_SPAN / 2 + 5, TAIL_QC + 4), f"tail {TAIL_SPAN:.0f} x {TAIL_CHORD:.0f}, quarter chord {TAIL_QC:.0f} aft, -3 deg", 4.5, "start", "#2f7d4f")]
    b.append(text(*P(-HALF, le(HALF) - 5), f"sweep {SWEEP:.0f} deg at quarter chord", 4.5))
    # front view
    fy, rise = 365.0, d["tip_rise_mm"]
    b.append(text(12, fy - 28, "FRONT VIEW (dihedral exaggerated 3x for clarity; true tip rise below)", 5))
    for s in (1, -1):
        b.append(line(ox, fy, ox + s * HALF, fy - 3 * rise, 1.2))
        b.append(line(ox + s * HALF, fy - 3 * rise - FIN_H / 2, ox + s * HALF, fy - 3 * rise + FIN_H / 2, 1.2))
    b.append(line(ox - HALF, fy, ox + HALF, fy, 0.25, "3 3", "#666"))
    b += [text(ox, fy + 14, f"each panel raised {DIH:.0f} deg; tip rise {rise:.1f} mm; centre joint included angle {180 - 2 * DIH:.0f} deg", 4.5, "middle"),
          text(ox, fy + 22, f"tip fins {FIN_H:.0f} mm tall, {FIN_H / 2:.0f} above and {FIN_H / 2:.0f} below the wing", 4.5, "middle")]
    return dark_svg(page(W, H, b, physical=False))


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    d = derived()
    rows = ballast_rows(d)
    (OUT / "geometry.json").write_text(json.dumps(dict(geometry=d, assumptions=dict(sheets_kg_m2=SHEETS, centre_piece_g=CENTRE_G, centre_piece_cg_mm=CENTRE_X,
                                                                                  note="sheet densities and centre-piece mass are assumed, not measured"), ballast=rows), indent=1), encoding="utf-8")
    cols = list(rows[0])
    (OUT / "ballast_table.csv").write_text(",".join(cols) + "\n" + "\n".join(",".join(str(r[c]) for c in cols) for r in rows) + "\n", encoding="utf-8")
    (OUT / "panel_template_1to1.svg").write_text(panel_template(), encoding="utf-8")
    (OUT / "small_parts_template_1to1.svg").write_text(small_parts_template(), encoding="utf-8")
    (OUT / "overview.svg").write_text(overview(d), encoding="utf-8")
    for k, v in d.items():
        print(f"{k}: {v}")
    for r in rows:
        if r["target_cg_mm"] in (54, 64, 78) and r["ws_n_m2"] in (6.0, 8.0, 12.0):
            print(r["sheet"][:12], r["ws_n_m2"], r["all_up_g"], "wing", r["wing_g"], "ballast", r["ballast_g"], "cg", r["target_cg_mm"], "station", r["ballast_station_mm"], "V", r["speed_cl0p3_m_s"], "Re", r["re_mac"], r["re_tip"])


if __name__ == "__main__":
    main()
