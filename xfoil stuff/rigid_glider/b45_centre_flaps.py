"""B45 foamboard flat wing with two small centre trailing-edge flaps (AVL CONTROL surfaces).  Standard library only.
Run:  py -B rigid_glider/b45_centre_flaps.py     Writes only under results/runs/rigid_glider/b45_centre_flaps/ (results.json, tables.md, avl_work/).
Thickness ignored (flat plate).  Flap deflection argument is degrees trailing edge UP."""
import json, math, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import flat_test as ft            # read-only import
from avl import run_avl
from geometry import Wing

ROOT = ft.ROOT
OUT = ROOT / "results/runs/rigid_glider/b45_centre_flaps"
EXE = ft.AVL_EXE
G = ft.wing_geometry(ft.FOAM_SPAN_MM, ft.FOAM_AR)
FIN_H = ft.fin_dims(ft.FOAM_SPAN_MM, ft.FOAM_THICKNESS_MM)["height"]
FLAP_CHORD_MM = 10.0
GAIN = -1.0                       # AVL positive = TE down; input is deg UP.  Verified by Cm sign (test + run).
LOCATIONS = {"nominal": (30.0, 80.0), "alternate": (5.0, 55.0)}     # y inboard, outboard, mm (ASSUMED)
ALPHAS = [0.0, 4.0, 8.0]
FLAPS = [0.0, 5.0, 10.0, 15.0]
SETTINGS = ft.sweep_settings(70.23775, G["mac"])                    # same band as geometry.json (57..78)
CG_BAND = (SETTINGS[0], SETTINGS[-1])
CG_NOM = SETTINGS[len(SETTINGS) // 2]                               # middle setting (ASSUMED nominal)
MASS_G = 17.498153237191072       # ASSUMED all-up with ballast at middle setting, geometry.json B45 thickness_table
RHO = ft.RHO_AIR
EXISTING_XNP = 70.23775           # geometry.json avl_with_tip_fins (mean of 4 points)


def chord(y):
    return G["cr"] + (G["ct"] - G["cr"]) * y / (G["span"] / 2)


def le_x(y):
    return G["tipx"] * y / (G["span"] / 2)


def deck(xcg_mm, loc, name="b45flap", with_flap=True):
    m = lambda v: v / 1000.0
    y1, y2 = LOCATIONS[loc]
    half = G["span"] / 2
    s = (f"{name}\n0.0\n0 0 0.0\n{G['area'] * 1e-6:.9f} {m(G['mac']):.9f} {m(G['span']):.9f}\n{m(xcg_mm):.9f} 0.0 0.0\n"
         f"SURFACE\nWing\n{ft.AVL_NCHORD} 1.0\nYDUPLICATE\n0.0\n")
    def sec(y, nspan, ctrl):
        t = f"SECTION\n{m(le_x(y)):.9f} {m(y):.9f} 0.0 {m(chord(y)):.9f} 0.0 {nspan} 1.0\nNACA\n0000\n"
        if ctrl:
            t += f"CONTROL\nflap {GAIN:.1f} {1 - FLAP_CHORD_MM / chord(y):.9f} 0 0 0 1.0\n"
        return t
    n1 = max(2, round(y1 / 5)); n3 = round((half - y2) / 8)
    s += sec(0.0, n1, False) + sec(y1, 8, with_flap) + sec(y2, n3, with_flap) + sec(half, 0, False)
    s += (f"SURFACE\nTipFin\n5 1.0 8 -2.0\nYDUPLICATE\n0.0\n"
          f"SECTION\n{m(G['tipx']):.9f} {m(half):.9f} {m(FIN_H / 2):.9f} {m(G['ct']):.9f} 0.0\nNACA\n0000\n"
          f"SECTION\n{m(G['tipx']):.9f} {m(half):.9f} {m(-FIN_H / 2):.9f} {m(G['ct']):.9f} 0.0\nNACA\n0000\n")
    return s


def avl_points(xcg, loc, flap_up, tag):
    """Run AVL at ALPHAS for one CG / location / flap; returns list of dicts."""
    w = Wing(span_m=G["span"] / 1000, aspect_ratio=G["ar"], taper=G["taper"], quarter_chord_sweep_deg=G["sweep"], washout_deg=0.0,
             dihedral_deg=0.0, section="NACA_0000", cg_x_m=xcg / 1000)
    d = OUT / "avl_work" / tag
    pts = run_avl(w, "b45flap", ALPHAS, d, EXE, deck_text=deck(xcg, loc), oper_prefix=[f"D1 D1 {flap_up:.6f}"])
    return [dict(alpha=p.alpha_deg if hasattr(p, "alpha_deg") else a, cl=p.cl, cm=p.cm, xnp=p.xnp_m * 1000, cla=p.cla_per_rad, cma=p.cma_per_rad,
                 clb=p.clb_per_rad, cnb=p.cnb_per_rad) for p, a in zip(pts, ALPHAS)]


def linfit(xs, ys):
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    return my - b * mx, b


def summarise(pts, xcg):
    a = [p["alpha"] for p in pts]
    cm0, cma = linfit(a, [p["cm"] for p in pts])         # per deg
    cl0, cla = linfit(a, [p["cl"] for p in pts])
    xnp = sum(p["xnp"] for p in pts) / len(pts)
    a_trim = -cm0 / cma if cma else float("nan")
    cl_trim = cl0 + cla * a_trim
    v = math.sqrt(2 * MASS_G * 1e-3 * 9.80665 / (RHO * G["area"] * 1e-6 * cl_trim)) if cl_trim > 0.01 else None
    return dict(xcg=xcg, xnp=xnp, sm_pct=100 * (xnp - xcg) / G["mac"], cm0=cm0, cm_alpha_per_deg=cma, cl0=cl0, cl_alpha_per_deg=cla,
                alpha_trim=a_trim, cl_trim=cl_trim, v_trim=v, cn_beta=sum(p["cnb"] for p in pts) / len(pts), cl_beta=sum(p["clb"] for p in pts) / len(pts))


def main():
    res = dict(geometry=dict(span=G["span"], mac=G["mac"], cg_nominal=CG_NOM, cg_band=CG_BAND, mass_g=MASS_G, existing_xnp=EXISTING_XNP), cases=[], sweep=[], conv=None)
    for loc in LOCATIONS:
        for fl in FLAPS:
            s = summarise(avl_points(CG_NOM, loc, fl, f"{loc}_flap{fl:g}_cg{CG_NOM:g}"), CG_NOM)
            s.update(loc=loc, flap=fl); res["cases"].append(s)
        for xcg in [x + 0.0 for x in range(int(CG_BAND[0]) - 5, int(CG_BAND[1]) + 6)]:
            s = summarise(avl_points(xcg, loc, 10.0, f"{loc}_flap10_cg{xcg:g}"), xcg)
            s.update(loc=loc, flap=10.0); res["sweep"].append(s)
    # sensitivity: finer panelling at nominal, 10 deg (not part of the rule)
    (OUT / "results.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    write_tables(res)
    print("done", len(res["cases"]), len(res["sweep"]))


def f(x, n=3):
    return "n/a" if x is None else f"{x:.{n}f}"


def write_tables(res):
    L = ["## Results (MODEL unless tagged; run by rigid_glider/b45_centre_flaps.py)", "",
         f"Nominal CG {CG_NOM:g} mm aft of root LE (ASSUMED, middle of the {CG_BAND[0]:g}-{CG_BAND[1]:g} mm band, JUDGEMENT); MAC {G['mac']:.2f} mm (GEOMETRY); mass {MASS_G:.2f} g (ASSUMED).", "",
         "### Flap sweep at nominal CG (MODEL)", "", "| location | flap up deg | Xnp mm | SM % MAC | Cm0 | dCm0/dflap per deg | Cm_alpha /deg | trim alpha deg | trim CL | trim V m/s |", "|---|---|---|---|---|---|---|---|---|---|"]
    for loc in LOCATIONS:
        base = next(c for c in res["cases"] if c["loc"] == loc and c["flap"] == 0.0)
        for c in res["cases"]:
            if c["loc"] != loc: continue
            d = (c["cm0"] - base["cm0"]) / c["flap"] if c["flap"] else None
            L.append(f"| {loc} | {c['flap']:g} | {c['xnp']:.2f} | {c['sm_pct']:.2f} | {c['cm0']:.4f} | {f(d, 5)} | {c['cm_alpha_per_deg']:.5f} | {c['alpha_trim']:.2f} | {c['cl_trim']:.3f} | {f(c['v_trim'], 2)} |")
    nom0 = next(c for c in res["cases"] if c["loc"] == "nominal" and c["flap"] == 0.0)
    L += ["", f"Cn_beta {nom0['cn_beta']:.4f} /rad, Cl_beta {nom0['cl_beta']:.4f} /rad at nominal (flap 0, mean over alpha 0/4/8; MODEL).", "",
          "### CG sweep at 10 deg up (MODEL)", "", "| location | CG mm | SM % MAC | trim CL | trim V m/s |", "|---|---|---|---|---|"]
    for c in res["sweep"]:
        L.append(f"| {c['loc']} | {c['xcg']:g} | {c['sm_pct']:.2f} | {c['cl_trim']:.3f} | {f(c['v_trim'], 2)} |")
    (OUT / "tables.md").write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
