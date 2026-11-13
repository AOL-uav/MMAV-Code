"""Swift A at 400 mm, printed: mass / balance comparison of constructions.  NO STL is written.  ARITHMETIC only (plus 4 AVL runs for the A outline, to get the same trim CG as the build files).

Reuses, read-only: bird_outline_v4_swift (outline A = GLIDE, evaluate4, trim_res), bird_outline_v2.size_numbers (sheet mass, rail, ballast arithmetic, wing loading),
flat_test (PLA density 1.24 g/cm3 ASSUMED, E_PLA 3.3 GPa ASSUMED, air density 1.225).  Writes ONLY into results/runs/rigid_glider/swiftA_printed400_options/.
    py -B rigid_glider/swiftA_printed400_options.py
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import html
import math
from pathlib import Path

import bird_outline_v2 as v2
import bird_outline_v4_swift as sw
import flat_test as ft

OUT = ft.ROOT / "results/runs/rigid_glider/swiftA_printed400_options"
SPAN = 400.0
BEND = 6.0                      # deg, the gauge angle nearest the 10 % margin (sw.nearest_gauge(5.99)) -- same as the existing foam 400 build
RHO_PLA = ft.PLA_DENSITY_G_CM3  # 1.24, ASSUMED in flat_test.py, solid
E_PLA = ft.E_PLA_GPA            # 3.3 GPa, ASSUMED in flat_test.py
RHO_AIR = ft.RHO_AIR            # 1.225, ASSUMED in flat_test.py
CL_TRIM = 0.3                   # the trim CL used for CG / ballast / pull-out throughout the project

# ---- constants that are NEW here (nothing equivalent exists in the project) -- all ASSUMED
SPAR_END_FRAC = 0.90            # ASSUMED: spar stops at 90 % of the half span ("near the tip")
STATION_RAIL_MM = None          # filled from the rail: its most forward ballast hole (the project's own default station)
STATION_NOSE_MM = -30.0         # ASSUMED alternative: a nose weight 6 mm behind the nose tip (-36), needs its own pocket, not on the rail
GLUE_G = 0.2                    # ASSUMED: CA / epoxy on the seam + rail bond
SPLICE_W_MM, SPLICE_T_MM = 8.0, 0.4   # ASSUMED: flat PLA splice strip over the seam, length = rail length (the rail's own saddle is only 4 mm wide, 2 mm per half)
SLEEVE_LEN_MM, SLEEVE_WALL_MM = 16.0, 0.8   # ASSUMED: nose spar joiner sleeve, 8 mm each side of the centreline, 0.8 mm wall
LW_RHO = (0.6, 0.7, 0.8)        # ASSUMED foamed LW-PLA density range g/cm3 (caller's range), 0.7 central
LW_E_EXPONENT = 1.0             # ASSUMED E_LW = E_PLA * (rho/1.24)^1.0 ; an exponent of 2 (foam scaling) would halve the D stiffness numbers
SPARS = [("h2.5", "half-round bead, 2.5 mm wide x 1.25 high", 2.5, "half"),
         ("h3.0", "half-round bead, 3.0 mm wide x 1.5 high", 3.0, "half"),
         ("r3.0", "round rod 3.0 mm dia (tangent to the underside)", 3.0, "round")]


def tag(s, t):
    return f"{s} [{t}]"


# =============================================================================================== geometry from the existing outline
def spar_path(g, k):
    """Polyline (mm, x aft of root LE, y from the centreline) of the leading edge of the right half, from the nose tip to SPAR_END_FRAC of the half span."""
    ylim = SPAR_END_FRAC * g["half_span"]
    pts = []
    for (x, y) in g["half_outline"]:
        if pts and y < pts[-1][1] - 1e-12:
            break
        if y > ylim:
            x0, y0 = pts[-1]
            f = (ylim - y0) / (y - y0)
            pts.append((x0 + f * (x - x0), ylim))
            break
        pts.append((x, y))
    return [(x * k, y * k) for x, y in pts]


def path_stats(pts):
    L, mx = 0.0, 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        L += d
        mx += d * (x0 + x1) / 2
    return L, mx / L


def bbox_rotated(poly):
    best = None
    for i in range(0, 180):
        a = math.radians(i * 0.5)
        c, s = math.cos(a), math.sin(a)
        u = [x * c + y * s for x, y in poly]
        v = [-x * s + y * c for x, y in poly]
        w, h = max(u) - min(u), max(v) - min(v)
        if best is None or w * h < best[0]:
            best = (w * h, w, h, i * 0.5)
    return best


# =============================================================================================== section stiffness (spanwise bending at the root)
def section_EI(chord, t_plate, E_plate, spar=None, E_spar=None):
    """Composite EI (N mm2) of plate chord x t_plate plus a spar below it, about the common neutral axis. z up, plate mid-plane z=0, spar below the underside.
    spar = (D, 'half'|'round'); E in GPa -> N/mm2 x 1000."""
    parts = [(chord * t_plate, 0.0, chord * t_plate ** 3 / 12.0, E_plate * 1e3)]
    if spar:
        D, kind = spar
        r = D / 2
        if kind == "half":
            A, zc, I = math.pi * r * r / 2, -(t_plate / 2 + 4 * r / (3 * math.pi)), (math.pi / 8 - 8 / (9 * math.pi)) * r ** 4
        else:
            A, zc, I = math.pi * r * r, -(t_plate / 2 + r), math.pi * r ** 4 / 4
        parts.append((A, zc, I, E_spar * 1e3))
    EA = sum(a * e for a, z, i, e in parts)
    zn = sum(a * e * z for a, z, i, e in parts) / EA
    return sum(e * (i + a * (z - zn) ** 2) for a, z, i, e in parts)


def spar_area(D, kind):
    r = D / 2
    return math.pi * r * r / 2 if kind == "half" else math.pi * r * r


def spar_perimeter(D, kind):
    r = D / 2
    return (math.pi * r + 2 * r) if kind == "half" else 2 * math.pi * r


# =============================================================================================== one option
def option(name, desc, base, membrane_t, rho_mem, spar=None, rho_spar=None, tags=""):
    g, k = base["g"], SPAN
    area = base["area_sheet"]                                        # mm2, full sheet outline incl. tail (v2.sheet_mass_g screening convention: area x t x rho)
    comps = []                                                       # (label, mass g, x mm)
    comps.append(("membrane/plate", area * membrane_t * 1e-3 * rho_mem, base["cx_sheet"]))
    m_spar = m_sleeve = 0.0
    if spar:
        D, kind = spar
        m_spar = 2 * base["spar_len"] * spar_area(D, kind) * 1e-3 * rho_spar
        comps.append(("spar (both halves)", m_spar, base["spar_cx"]))
        m_sleeve = SLEEVE_LEN_MM * spar_perimeter(D, kind) * SLEEVE_WALL_MM * 1e-3 * RHO_PLA
        comps.append(("nose joiner sleeve", m_sleeve, g["nose_x"] * k))
    comps.append(("centre rail (foam-400 rail)", base["rail_g"], base["rail_cx"]))
    comps.append(("seam glue + splice strip", GLUE_G + base["splice_g"], base["rail_cx"]))
    M0 = sum(m for _, m, _ in comps)
    cg0 = sum(m * x for _, m, x in comps) / M0
    tgt = base["target"]
    out = dict(option=name, desc=desc, membrane_t=membrane_t, rho_mem=rho_mem, spar=("none" if not spar else f"{spar[1]} {spar[0]:g}"),
               membrane_g=comps[0][1], spar_g=m_spar, sleeve_g=m_sleeve, rail_g=base["rail_g"], glue_g=GLUE_G + base["splice_g"], M0=M0, cg0=cg0, target=tgt, tags=tags)
    for lab, st in (("rail", base["station_rail"]), ("nose", STATION_NOSE_MM)):
        mb = M0 * (tgt - cg0) / (st - tgt)
        out[f"cg_ahead_mm_{lab}"] = max(0.0, tgt - cg0)         # bare CG already ahead of the trim CG: no nose ballast; a tiny aft correction would be needed
        mb = max(mb, 0.0)
        out[f"ballast_{lab}_g"] = mb
        out[f"allup_{lab}_g"] = M0 + mb
        out[f"station_{lab}"] = st
    out["allup_g"] = out["allup_rail_g"]
    M = out["allup_g"]
    out["ballast_frac"] = out["ballast_rail_g"] / M
    out["ws"] = M * 1e-3 * v2.G_ACCEL / (base["S_w"] * 1e-6)
    out["ws_ratio"] = M / base["foam_M"]
    out["V"] = math.sqrt(2 * out["ws"] / (RHO_AIR * CL_TRIM))
    out["V_ratio"] = math.sqrt(out["ws_ratio"])
    # ballast contribution of each component: m (x - target) / (target - station); negative = the part sits ahead of the trim CG and REMOVES ballast
    st = base["station_rail"]
    out["contrib"] = {lab: m * (x - tgt) / (tgt - st) for lab, m, x in comps}
    out["spar_x"] = base["spar_cx"] if spar else None
    return out, comps


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    work = OUT / "avl_work"
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    o = sw.evaluate4("A_as_measured", sw.GLIDE, work, 1.0)        # tip_loading reference unused here
    g = o["g"]
    res = sw.trim_res(o, BEND)
    foam = v2.size_numbers(g, res, "foam", BEND)
    printed250 = v2.size_numbers(g, res, "printed", BEND, exact=True)
    k = SPAN
    rail = foam["rail"]
    path = spar_path(g, k)
    L, cx = path_stats(path)
    half = [(x * k, y * k) for x, y in g["half_outline"]]
    full = [(x * k, y * k) for x, y in g["full_outline"]]
    base = dict(g=g, area_sheet=foam["area_sheet"], cx_sheet=foam["cx_sheet"], S_w=foam["S_w"], target=foam["cg_target"], foam_M=foam["M"], rail_g=rail["mass_g"], rail_cx=rail["cx"],
                station_rail=min(rail["nose_holes"]), spar_len=L, spar_cx=cx, splice_g=SPLICE_W_MM * SPLICE_T_MM * rail["length"] * 1e-3 * RHO_PLA)
    cr = g["c_root"] * k
    rows, detail = [], {}

    def add(name, desc, t, rho_mem, spar_def=None, rho_spar=None, tags=""):
        r, comps = option(name, desc, base, t, rho_mem, spar_def, rho_spar, tags)
        sp = (spar_def[0], spar_def[1]) if spar_def else None
        E_m = E_PLA * (rho_mem / RHO_PLA) ** LW_E_EXPONENT
        E_s = E_PLA * (rho_spar / RHO_PLA) ** LW_E_EXPONENT if spar_def else None
        r["EI"] = section_EI(cr, t, E_m, sp, E_s)
        r["EI_spar_only"] = section_EI(1e-6, 1e-6, 1e-9, sp, E_s) if sp else 0.0
        rows.append(r)
        detail[name] = comps
        return r

    add("A", "plain PLA sheet 0.6 mm (scale-up of the printed 250)", 0.6, RHO_PLA, tags="PLA 1.24 ASSUMED")
    add("B", "plain PLA sheet 0.4 mm", 0.4, RHO_PLA)
    for sid, sdesc, D, kind in SPARS:
        add(f"C-{sid}", f"PLA, 0.4 mm membrane + {sdesc}", 0.4, RHO_PLA, (D, kind), RHO_PLA)
    add("C-h3.0-m0.3", "PLA, 0.3 mm membrane + half-round 3.0 mm", 0.3, RHO_PLA, (3.0, "half"), RHO_PLA)
    for sid, sdesc, D, kind in SPARS:
        add(f"D-{sid}", f"LW-PLA {LW_RHO[1]:g} g/cm3, 0.4 mm membrane + {sdesc}", 0.4, LW_RHO[1], (D, kind), LW_RHO[1])
    add("D-h3.0-m0.3", f"LW-PLA {LW_RHO[1]:g} g/cm3, 0.3 mm membrane + half-round 3.0 mm", 0.3, LW_RHO[1], (3.0, "half"), LW_RHO[1])
    # LW-PLA density bounds for the D rows
    for r in rows:
        if r["option"].startswith("D-"):
            sid = r["option"][2:]
            for lab, rho in (("lo", LW_RHO[0]), ("hi", LW_RHO[2])):
                m = 0.3 if "m0.3" in sid else 0.4
                spec = [s for s in SPARS if sid.startswith(s[0])][0]
                rr, _ = option(r["option"], "", base, m, rho, (spec[2], spec[3]), rho)
                r[f"allup_{lab}"] = rr["allup_g"]
        else:
            r["allup_lo"] = r["allup_hi"] = r["allup_g"]
    EI_A = rows[0]["EI"]
    for r in rows:
        r["EI_ratio"] = r["EI"] / EI_A

    # footprint
    xs, ys = [p[0] for p in half], [p[1] for p in half]
    rb = bbox_rotated(half)
    fp = dict(x0=min(xs), x1=max(xs), y0=min(ys), y1=max(ys))
    fp["len"], fp["wid"] = fp["x1"] - fp["x0"], fp["y1"] - fp["y0"]
    tail_end = g["tail_end_x"] * k
    seam_len = tail_end - g["nose_x"] * k
    rail_x0, rail_x1 = rail["x_lo"], rail["saddle_end"]
    # printed-flat tail: the bend is made at the hinge afterwards
    info = dict(path_len=L, path_cx=cx, spar_end_y=path[-1][1], half_span=g["half_span"] * k, nose=g["nose_x"] * k, hinge=g["x_h"] * k, target=base["target"], cg0_foam=foam["cg0"],
                foam_M=foam["M"], foam_ws=foam["ws"], foam_sheet=foam["m_sheet"], foam_rail=rail["mass_g"], station_rail=base["station_rail"], c_root=cr, S_w=base["S_w"], area_sheet=base["area_sheet"],
                p250_M=printed250["M"], p250_sheet=printed250["m_sheet"], p250_ballast=printed250["ballast_g"], rail_len=rail["length"], rail_x0=rail_x0, rail_x1=rail_x1, rail_cx=rail["cx"],
                seam_len=seam_len, tail_end=tail_end, splice_g=base["splice_g"], fp=fp, rot=rb, tri_nose_x=g["nose_x"] * k, np_span=o["np_span"], mac=o["mac"] * k,
                wing_thickness_ref=ft.THICKNESS_MM, p250_cg=printed250["cg_target"] / 1.0)
    write_csv(rows, info)
    write_html(rows, info, half, path, rail)
    write_readme(rows, info, detail)
    print_summary(rows, info)
    return rows, info, detail


# =============================================================================================== outputs
def write_csv(rows, info):
    cols = ["option", "desc", "membrane_t", "rho_mem", "spar", "membrane_g", "spar_g", "sleeve_g", "rail_g", "glue_g", "M0", "cg0", "target", "station_rail", "ballast_rail_g", "allup_rail_g",
            "ballast_frac", "station_nose", "ballast_nose_g", "allup_nose_g", "allup_lo", "allup_hi", "ws", "ws_ratio", "V", "V_ratio", "EI", "EI_ratio"]
    with open(OUT / "options.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["option", "description", "membrane/plate mm [ASSUMED design]", "density g/cm3 [ASSUMED]", "spar", "membrane g [CALCULATED]", "spar g [CALCULATED]", "nose sleeve g [CALCULATED]",
                    "centre rail g [CALCULATED, foam-400 rail]", "glue+splice g [ASSUMED]", "bare mass g", "bare CG mm aft of root LE [CALCULATED]", "trim CG mm aft of root LE [MODEL]",
                    "ballast station mm (rail hole) [CALCULATED]", "ballast g at rail hole [CALCULATED]", "all-up g at rail hole [CALCULATED]", "ballast % of all-up", "alt ballast station mm [ASSUMED]",
                    "ballast g at alt station", "all-up g at alt station", "all-up g, LW density 0.6", "all-up g, LW density 0.8", "wing loading N/m2 [CALCULATED]", "wing loading / foam 400",
                    "trim speed m/s at CL 0.3 [CALCULATED]", "speed / foam 400", "root EI N mm2 [ROUGH]", "EI / option A"])
        for r in rows:
            w.writerow([f"{r[c]:.3f}" if isinstance(r[c], float) else r[c] for c in cols])
        w.writerow([])
        w.writerow(["reference: foam 400 (existing build)", "", "", "", "", f"{info['foam_sheet']:.3f}", "", "", f"{info['foam_rail']:.3f}", "", "", f"{info['cg0_foam']:.3f}", f"{info['target']:.3f}",
                    f"{info['station_rail']:.1f}", "", f"{info['foam_M']:.3f}", "", "", "", "", "", "", f"{info['foam_ws']:.3f}", "1.000",
                    f"{math.sqrt(2 * info['foam_ws'] / (RHO_AIR * CL_TRIM)):.3f}", "1.000", "", ""])


def write_readme(rows, info, detail):
    fp, rb = info["fp"], info["rot"]
    R = {r["option"]: r for r in rows}
    t = []
    t.append("# Swift A printed at 400 mm: construction options (mass and balance only, no STL)\n")
    t.append("Generated by `rigid_glider/swiftA_printed400_options.py` (`py -B`). Tags: MODEL = AVL-based, CALCULATED = arithmetic on project constants, ASSUMED = a number nobody measured, KAI = Kai's statement, MEASURED = weighed or measured (none used here).\n")
    t.append(f"Common basis (all rows): Swift A outline = `GLIDE` in bird_outline_v4_swift.py, span 400 mm, tail bend {BEND:g} deg, trim CG {info['target']:.1f} mm aft of root LE = {info['target'] - info['tri_nose_x']:.1f} mm aft of the nose tip [MODEL, 10 % margin at CL 0.3, same as the foam 400 build]. Sheet area {info['area_sheet']:,.0f} mm2 incl. tail, wing area {info['S_w']:,.0f} mm2 [CALCULATED]. Reference = foam 400: {info['foam_M']:.2f} g all-up, {info['foam_ws']:.2f} N/m2 [CALCULATED, reproduces the 9.77 g]. The 250 print reproduces as {info['p250_M']:.2f} g incl {info['p250_ballast']:.2f} g ballast [CALCULATED].\n")
    t.append("## Options table (ballast on the rail's most forward hole, root LE - %.1f mm)\n" % abs(info["station_rail"]))
    t.append("| option | membrane g | spar g | rail g | glue+splice g | bare CG mm | ballast g (% of all-up) | all-up g | W/S N/m2 | W/S vs foam | V trim m/s | V vs foam | EI root vs A |")
    t.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        lw = f" ({r['allup_lo']:.1f}-{r['allup_hi']:.1f} for 0.6-0.8)" if r["option"].startswith("D-") else ""
        t.append(f"| {r['option']} | {r['membrane_g']:.2f} | {r['spar_g'] + r['sleeve_g']:.2f} | {r['rail_g']:.2f} | {r['glue_g']:.2f} | {r['cg0']:.1f} | {r['ballast_rail_g']:.2f} ({100 * r['ballast_frac']:.0f}%) | {r['allup_g']:.2f}{lw} | {r['ws']:.2f} | {r['ws_ratio']:.2f}x | {r['V']:.2f} | {r['V_ratio']:.2f}x | {r['EI_ratio']:.2f} |")
    t.append(f"\nOptions: A = 0.6 mm PLA sheet; B = 0.4 mm PLA sheet; C-h2.5 / C-h3.0 / C-r3.0 = 0.4 mm PLA membrane + spar (half-round 2.5, half-round 3.0, round rod 3.0); C-h3.0-m0.3 = same with 0.3 mm membrane; D = the C rows in LW-PLA at {LW_RHO[1]:g} g/cm3 (range {LW_RHO[0]:g}-{LW_RHO[2]:g}, ASSUMED). 'spar g' includes the nose sleeve. Full numbers incl. the alternative nose ballast station: `options.csv`.\n")
    t.append("Bare CG = CG of the parts before ballast. Ballast = M0 (target - bare CG) / (station - target); same equation as `bird_outline_v2.size_numbers`. The ballast is a weight of lead or similar added at the station; ASSUMED point mass.\n")
    t.append("## What the table says\n")
    a, b, c3, c25, d3 = R["A"], R["B"], R["C-h3.0"], R["C-h2.5"], R["D-h3.0"]
    pla_c = [r for r in rows if r["option"].startswith("C-")]
    best = min(pla_c, key=lambda r: r["allup_g"])
    t.append(f"- Against the foam 400 ({info['foam_M']:.2f} g): plain 0.6 sheet A is {a['allup_g']:.1f} g ({a['ws_ratio']:.2f}x wing loading, {a['V_ratio']:.2f}x trim speed); 0.4 sheet B {b['allup_g']:.1f} g ({b['ws_ratio']:.2f}x, {b['V_ratio']:.2f}x); lightest PLA spar+membrane row {best['option']} {best['allup_g']:.1f} g ({best['ws_ratio']:.2f}x, {best['V_ratio']:.2f}x); heaviest PLA spar row {max(pla_c, key=lambda r: r['allup_g'])['allup_g']:.1f} g. No solid-PLA option gets below about 1.4x the foam wing loading.")
    t.append(f"- LW-PLA with the spar (D-h3.0, 0.4 membrane, {LW_RHO[1]:g} g/cm3): {d3['allup_g']:.1f} g ({d3['allup_lo']:.1f}-{d3['allup_hi']:.1f} g over {LW_RHO[0]:g}-{LW_RHO[2]:g}), {d3['ws_ratio']:.2f}x wing loading, {d3['V_ratio']:.2f}x speed: the only family that reaches the foam bird's wing loading. It rests on ASSUMED densities and on LW-PLA actually printing a 0.3-0.4 mm membrane (not verified; foaming normally needs thicker lines). At 0.3 mm (D-h3.0-m0.3) the bare CG is already at the trim CG, so no ballast (a -0.05 g arithmetic result is clipped to zero).")
    t.append(f"- Ballast: {a['ballast_rail_g']:.1f} g for A, {b['ballast_rail_g']:.1f} g B, {c3['ballast_rail_g']:.1f} g C-h3.0, {d3['ballast_rail_g']:.2f} g D-h3.0, on the rail's front hole ({abs(info['station_rail']):.1f} mm ahead of root LE; lever {info['target'] - info['station_rail']:.1f} mm). The bare CG is behind the trim CG ({info['target']:.1f} mm) in every row but the last, because the sheet centroid is at {info['cg0_foam']:.0f}-{a['cg0']:.0f} mm. A nose pocket at {STATION_NOSE_MM:g} mm (ASSUMED) lowers A to {a['ballast_nose_g']:.1f} g ballast, {a['allup_nose_g']:.1f} g all-up; see CSV.")
    t.append(f"- The spar pulls the CG forward: its centroid is {info['path_cx']:.1f} mm aft of root LE (trim CG {info['target']:.1f}). A spar centroid ahead of the trim CG removes ballast; one behind adds. Spar centroid minus trim CG = {info['path_cx'] - info['target']:+.1f} mm. Ballast contribution of the spar (g, negative = saves ballast): " + ", ".join(f"{r['option']} {r['contrib']['spar (both halves)']:+.2f}" for r in rows if r["spar_g"] > 0 and r["option"].startswith("C-")) + ". The saving is a few tenths of a gram; the dominant lever is membrane thickness and density, not the spar.")
    t.append(f"- Stiffness (spanwise bending at the root, chord {info['c_root']:.1f} mm, ROUGH, E = {E_PLA} GPa ASSUMED, solid, anisotropy and the real airfoil-less thin section ignored): EI of A = {a['EI']:.0f} N mm2. B (0.4 sheet) is {b['EI_ratio']:.2f} of A (the t^3 law). The 3 mm half-round spar on a 0.4 membrane gives {c3['EI_ratio']:.2f} of A.  LW-PLA rows scale E linearly with density (ASSUMED; with an exponent of 2 they would be about half).")
    t.append("\n## Kai's vase-mode data\n")
    t.append(f"- KAI: one-wall vase-mode PLA wing panel at 400 mm span = 16 g per panel (0.42 mm wall), 13 g at 0.28-0.3 mm. Not used as an input: its planform and cross-section are not recorded. Check against this model: a flat sheet of this outline at 0.42 mm is {info['area_sheet'] * 0.42 * 1e-3 * RHO_PLA:.1f} g for BOTH halves ({info['area_sheet'] * 0.42 * 1e-3 * RHO_PLA / 2:.1f} g per half) [CALCULATED]. His 16 g per panel is about {16 / (info['area_sheet'] * 0.42 * 1e-3 * RHO_PLA / 2):.1f} times that, which fits a closed two-skin airfoil shell (two surfaces plus ends) better than a single flat membrane. That is an inference; the factor is not a measurement of this outline. If a printed flat half comes out heavier than area x thickness x 1.24 (over-extrusion, seam, first layer), every PLA row here is low. First action: print one 0.4 mm half and weigh it.\n")
    t.append("## Half-panel footprint (one half, printed flat, tail unbent, no bed size assumed)\n")
    t.append(f"- Axis-aligned bounding box of one half: {fp['len']:.0f} mm (fore-aft, nose tip to the farthest trailing edge, which is the swept wingtip; the tail ends at %.0f mm)" % info['tail_end'] + f" x {fp['wid']:.0f} mm (spanwise, centreline to tip) [CALCULATED from the outline]. x from {fp['x0']:.0f} to {fp['x1']:.0f} mm aft of root LE.")
    t.append(f"- Smallest rotated rectangle (0.5 deg steps): {rb[1]:.0f} x {rb[2]:.0f} mm at {rb[3]:.1f} deg (area {rb[0]:,.0f} mm2). Fits a bed whose diagonal is long enough; compare to Kai's bed.")
    t.append(f"- Centre rail printed separately: {info['rail_len']:.1f} mm long x 7 mm deep x 4 mm thick, lying on its side (flat_test.STICK_*), spanning x = {info['rail_x0']:.1f} to {info['rail_x1']:.1f} mm. Together with a half it is inside the same box in x (the rail begins ahead of the root LE but behind the nose tip at {info['tri_nose_x']:.0f}); so the rail adds no bed length. The spar bead and the joiner sleeve stand 1.5 mm (half-round 3.0) or 3 mm (round 3.0) proud of the bed.")
    t.append("- Tail bend is made at the hinge afterwards, as with the existing printed 250.\n")
    t.append("## Centre joint (described and sized, no CAD)\n")
    t.append(f"- Seam: the centreline from the nose tip ({info['nose']:.0f} mm) to the tail end ({info['tail_end']:.0f} mm aft of root LE), {info['seam_len']:.0f} mm [CALCULATED]. The existing rail (foam-400 rail, {info['rail_len']:.1f} mm, {info['foam_rail']:.2f} g) covers x = {info['rail_x0']:.0f} to {info['rail_x1']:.0f} mm of it. Its saddle is only 4 mm wide, so each half has a 2 mm lap on a 0.4 mm membrane. That is thin for a glue joint. Recommend adding a 8 x 0.4 mm flat PLA splice strip on the top face over the same length ({info['splice_g']:.2f} g, ASSUMED size) so the lap is 4 mm per side; rail then bonds to the strip.")
    t.append(f"- The nose from {info['nose']:.0f} to {info['rail_x0']:.0f} mm is ahead of the rail: bond with a thin glue line plus the spar sleeve. The tail from the hinge ({info['hinge']:.0f} mm) onward bonds with a glue line only (the hinge strip is the weak place; the tail neck is the part the v2 beam check cares about).")
    t.append(f"- Spar joiner sleeve at the nose: at the centreline the nose curve is tangent to the span direction (the nose Bezier leaves the axis along y), so the two spars meet end to end in a straight line. A sleeve {SLEEVE_LEN_MM:g} mm long ({SLEEVE_LEN_MM / 2:g} mm each side), {SLEEVE_WALL_MM:g} mm wall, wrapping the spar section ({rows[3]['sleeve_g']:.2f} g for the 3.0 half-round, {rows[4]['sleeve_g']:.2f} g for the round, PLA) [ASSUMED size, CALCULATED mass]. Glue both halves into it before gluing the membrane seam.")
    t.append(f"- Spar path: along the leading edge from the nose tip to {100 * SPAR_END_FRAC:.0f} % of the half span (y = {info['spar_end_y']:.0f} mm of {info['half_span']:.0f} mm; ASSUMED stop), length {info['path_len']:.0f} mm per half [CALCULATED from the outline polyline]. It starts at the nose tip, follows the nose blend, then the swept leading edge; centroid {info['path_cx']:.1f} mm aft of root LE.\n")
    t.append("## What AVL sees\n")
    t.append("AVL treats the wing as a flat plate (zero thickness, zero camber; `NACA_0000` in `run_case2`) of the outline only. A 2.5-3 mm bead on one face is invisible to it: no thickness, no camber, no leading-edge shape, no drag. So the trim CG, bend angle and neutral point do not change in the model; the only things that change are the mass and its distribution, which this file handles by arithmetic. What the bead could change in reality and AVL cannot say: a rounder leading edge on the lower side (could delay LE separation; no data at this Reynolds number), added pressure drag from a ridge on the windward face, a small camber-like asymmetry, and more spanwise inertia (mass out along the leading edge raises roll and pitch inertia). All unquantified. If we want the effect, it is a flight or CFD question, not an AVL one.\n")
    t.append("## Sources of constants\n")
    t.append("| constant | value | tag | where |")
    t.append("|---|---|---|---|")
    S = [("PLA density", f"{RHO_PLA} g/cm3", "ASSUMED", "flat_test.PLA_DENSITY_G_CM3, used for every printed mass in the project"),
         ("PLA E", f"{E_PLA} GPa", "ASSUMED", "flat_test.E_PLA_GPA"),
         ("air density", f"{RHO_AIR}", "ASSUMED", "flat_test.RHO_AIR"),
         ("g", f"{v2.G_ACCEL}", "constant", "bird_outline.G_ACCEL via v2"),
         ("sheet mass rule", "area x t x rho", "CALCULATED", "bird_outline_v2.sheet_mass_g (hinge strip neglected, < 1 %)"),
         ("outline, trim CG, bend", f"A, {BEND:g} deg, {info['target']:.1f} mm", "MODEL", "bird_outline_v4_swift GLIDE, evaluate4 + trim_res (4 AVL runs rerun into avl_work/); 10 % margin at CL 0.3"),
         ("rail", f"{info['foam_rail']:.2f} g, {info['rail_len']:.1f} mm", "CALCULATED", "bird_outline_v2.make_rail via size_numbers('foam') = the A_foam_span400/rail.stl geometry"),
         ("ballast station", f"{info['station_rail']:.1f} mm", "CALCULATED", "the rail's most forward ballast hole (v2.size_numbers uses the rail default hole)"),
         ("ballast equation", "M0 (tgt - cg0)/(st - tgt)", "CALCULATED", "bird_outline_v2.size_numbers"),
         ("CL for speed/trim", f"{CL_TRIM}", "project convention", "TRIM_CLS / pull-out study"),
         ("spar stop", f"{SPAR_END_FRAC}", "ASSUMED", "this file"), ("glue", f"{GLUE_G} g", "ASSUMED", "this file"), ("splice strip", f"{SPLICE_W_MM} x {SPLICE_T_MM} mm", "ASSUMED", "this file"),
         ("sleeve", f"{SLEEVE_LEN_MM} mm long, {SLEEVE_WALL_MM} mm wall", "ASSUMED", "this file"), ("nose ballast alt station", f"{STATION_NOSE_MM} mm", "ASSUMED", "this file"),
         ("LW-PLA density", f"{LW_RHO[0]}-{LW_RHO[2]} g/cm3 ({LW_RHO[1]} central)", "ASSUMED", "from the brief; no LW-PLA weighed in the project"),
         ("LW-PLA E", "E_PLA x (rho/1.24)", "ASSUMED", "this file; optimistic, see above"),
         ("vase-mode panel masses", "16 g / 13 g", "KAI", "memory note 2026-10-12, not an input")]
    for s in S:
        t.append("| " + " | ".join(s) + " |")
    t.append("\n## Not found / not done\n")
    t.append("- No measured mass of any printed Swift in the project (the 9.34 g for the 250 is CALCULATED). No LW-PLA data. No bed size (not assumed). Hinge for thin sheets: the existing hinge strip is 0.4 mm (v2.HINGE_V2_MM); with a 0.4 mm or thinner sheet a 0.4 mm strip is no longer a thinned hinge, so B, C, D need a score or a 0.2 mm strip; the v2 neck check at 0.2 mm was not rerun here.")
    t.append("- Torsion, chordwise bending, flutter and the actual tip deflection under load are not computed. EI is the spanwise bending stiffness at the root section only, for relative comparison.")
    t.append("- Dark-mode view: `index.html` (top view of option C-h3.0, spar line, rail, CG).")
    (OUT / "README.md").write_text("\n".join(t) + "\n", encoding="utf-8")


def write_html(rows, info, half, path, rail):
    full = half + [(x, -y) for x, y in reversed(half[1:-1])]
    xs = [p[0] for p in full]
    ys = [p[1] for p in full]
    x0, x1, y0, y1 = min(xs) - 15, max(xs) + 15, min(ys) - 15, max(ys) + 15
    sc = 2.2
    W, H = (x1 - x0) * sc, (y1 - y0) * sc
    # drawing: span horizontal, nose up: svg X = y (span), svg Y = x (fore-aft)
    P = lambda x, y: ((y - y0) * sc, (x - x0) * sc)
    W, H = (y1 - y0) * sc, (x1 - x0) * sc
    poly = " ".join("%.1f,%.1f" % P(x, y) for x, y in full)
    spar_r = " ".join("%.1f,%.1f" % P(x, y) for x, y in path)
    spar_l = " ".join("%.1f,%.1f" % P(x, -y) for x, y in path)
    rx0, rx1 = info["rail_x0"], info["rail_x1"]
    rl = P(rx0, -2)
    rr = P(rx1, 2)
    cgy = P(info["target"], 0)[1]
    hy = P(info["hinge"], 0)[1]
    s = [f"<svg viewBox='0 0 {W:.0f} {H:.0f}' width='{W:.0f}' height='{H:.0f}' xmlns='http://www.w3.org/2000/svg' style='background:#14181d'>",
         f"<polygon points='{poly}' fill='#2a3440' stroke='#8fa3b8' stroke-width='1'/>",
         f"<polyline points='{spar_r}' fill='none' stroke='#ffb454' stroke-width='5' stroke-linecap='round'/>",
         f"<polyline points='{spar_l}' fill='none' stroke='#ffb454' stroke-width='5' stroke-linecap='round'/>",
         f"<rect x='{rl[0]:.1f}' y='{rl[1]:.1f}' width='{abs(rr[0] - rl[0]):.1f}' height='{abs(rr[1] - rl[1]):.1f}' fill='#59c2ff' opacity='0.8'/>",
         f"<line x1='0' x2='{W:.0f}' y1='{cgy:.1f}' y2='{cgy:.1f}' stroke='#7fd962' stroke-dasharray='6 4'/><text x='6' y='{cgy - 4:.1f}' fill='#7fd962' font-size='12'>trim CG {info['target']:.1f} mm aft of root LE</text>",
         f"<line x1='0' x2='{W:.0f}' y1='{hy:.1f}' y2='{hy:.1f}' stroke='#c792ea' stroke-dasharray='2 4'/><text x='6' y='{hy - 4:.1f}' fill='#c792ea' font-size='12'>tail hinge</text>",
         f"<text x='6' y='16' fill='#d0d7de' font-size='12'>nose up, 400 mm span, scale 1 mm = {sc} px</text></svg>"]
    css = "body{background:#0d1117;color:#d0d7de;font-family:Segoe UI,Arial,sans-serif;margin:20px}table{border-collapse:collapse}td,th{border:1px solid #30363d;padding:4px 8px;font-size:13px}th{background:#161b22}code{color:#ffb454}"
    h = [f"<!doctype html><html><head><meta charset='utf-8'><title>Swift A printed 400 options</title><style>{css}</style></head><body>",
         "<h2>Swift A printed at 400 mm: top view of option C (spar along the leading edge)</h2>",
         f"<p>Orange: spar line, nose tip to {100 * SPAR_END_FRAC:.0f} % of the half span ({info['path_len']:.0f} mm per half, ASSUMED stop). Blue: centre rail ({info['rail_len']:.0f} mm). Green dashed: trim CG [MODEL]. Overall mm numbers in README.md.</p>"]
    h += s
    h.append("<h3>Options</h3><table><tr><th>option</th><th>membrane g</th><th>spar+sleeve g</th><th>ballast g</th><th>all-up g</th><th>W/S vs foam</th><th>V vs foam</th><th>EI vs A</th></tr>")
    for r in rows:
        h.append(f"<tr><td>{html.escape(r['option'])}</td><td>{r['membrane_g']:.2f}</td><td>{r['spar_g'] + r['sleeve_g']:.2f}</td><td>{r['ballast_rail_g']:.2f}</td><td>{r['allup_g']:.2f}</td><td>{r['ws_ratio']:.2f}x</td><td>{r['V_ratio']:.2f}x</td><td>{r['EI_ratio']:.2f}</td></tr>")
    h.append("</table></body></html>")
    (OUT / "index.html").write_text("\n".join(h), encoding="utf-8")


def print_summary(rows, info):
    print(f"foam M {info['foam_M']:.3f} ws {info['foam_ws']:.3f} target {info['target']:.2f}  p250 M {info['p250_M']:.3f}  spar len {info['path_len']:.1f} cx {info['path_cx']:.1f}")
    for r in rows:
        print(f"{r['option']:14s} mem {r['membrane_g']:6.2f} spar {r['spar_g']:5.2f} sl {r['sleeve_g']:.2f} M0 {r['M0']:6.2f} cg0 {r['cg0']:5.1f} bal {r['ballast_rail_g']:6.2f} allup {r['allup_g']:6.2f} "
              f"[{r['allup_lo']:.1f}-{r['allup_hi']:.1f}] ws {r['ws']:.2f} x{r['ws_ratio']:.2f} V {r['V']:.2f} x{r['V_ratio']:.2f} EI x{r['EI_ratio']:.2f} sparcontrib {r['contrib'].get('spar (both halves)', 0):+.2f} nose-st allup {r['allup_nose_g']:.2f}")
    fp = info["fp"]
    print("footprint", fp, info["rot"], "rail", info["rail_x0"], info["rail_x1"], "seam", info["seam_len"], "splice g", info["splice_g"])


if __name__ == "__main__":
    main()
