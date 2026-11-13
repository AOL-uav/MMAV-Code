# stallPlotter.py
#
# Reads the archived results/legacy/wingAnalysis/glider_metrics.csv,
# selects top foil by best L/D and top foil by max CL (runner-up if same),
# and plots stall-speed heatmaps Vs(b, cr).
#
# Implemented:
#  (1) Annotation moved to upper-left (axes-fraction coords) with a white bbox (no overlap).
#  (2) Optional wing-area contours overlaid (dashed, labeled).
#
# Required CSV columns:
#   foil,bestLD,maxCL,nConverged,nTotal,note

import csv
import os
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


GLIDER_METRICS_CSV = Path(__file__).resolve().parents[2] / "results" / "legacy" / "wingAnalysis" / "glider_metrics.csv"


MASS_KG = 0.060
RHO = 1.225
G0 = 9.80665

TAPER = 1.0

# Dense sweep bounds (meters)
B_MIN, B_MAX = 0.55, 1.60
CR_MIN, CR_MAX = 0.06, 0.20

# Resolution controls (increase for prettier, decrease for faster)
NB = 220     # number of span samples
NCR = 220    # number of root-chord samples

# ---------- PLOT OPTIONS ----------
DRAW_VS_CONTOURS = True
N_VS_LEVELS = 10

DRAW_AREA_CONTOURS = True
N_AREA_LEVELS = 8
AREA_CONTOUR_LINESTYLE = "dashed"
AREA_CONTOUR_LINEWIDTH = 0.8
AREA_CONTOUR_COLOR = "white"
AREA_LABEL_FMT = "S=%.3f m²"


def wing_area(b, cr, taper):
    ct = taper * cr
    return 0.5 * b * (cr + ct)


def stall_speed(mass_kg, rho, g, S, CLmax):
    # Vs = sqrt(2W / (rho S CLmax))
    W = mass_kg * g
    denom = rho * S * CLmax
    return np.sqrt(np.maximum(0.0, 2.0 * W / np.maximum(denom, 1e-12)))


def read_glider_metrics(path):
    rows = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                row["bestLD"] = float(row["bestLD"])
                row["maxCL"] = float(row["maxCL"])
                row["nConverged"] = int(row["nConverged"])
                row["nTotal"] = int(row["nTotal"])
            except Exception:
                continue
            rows.append(row)
    return rows


def pick_top2(rows):
    good = [r for r in rows if r["nConverged"] > 0 and r["maxCL"] > 0]
    if not good:
        raise RuntimeError("No converged glider rows found in glider_metrics.csv")

    top_by_ld = max(good, key=lambda r: r["bestLD"])
    top_by_cl = max(good, key=lambda r: r["maxCL"])

    # if same foil wins both, take runner-up for maxCL
    if top_by_cl["foil"] == top_by_ld["foil"]:
        sorted_by_cl = sorted(good, key=lambda r: r["maxCL"], reverse=True)
        if len(sorted_by_cl) > 1:
            top_by_cl = sorted_by_cl[1]

    return top_by_ld, top_by_cl


def plot_heatmap_for_foil(foil_row):
    foil = foil_row["foil"]
    CLmax = foil_row["maxCL"]
    bestLD = foil_row["bestLD"]
    conv = f'{foil_row["nConverged"]}/{foil_row["nTotal"]}'
    note = foil_row.get("note", "")

    b = np.linspace(B_MIN, B_MAX, NB)
    cr = np.linspace(CR_MIN, CR_MAX, NCR)
    BB, CRR = np.meshgrid(b, cr)  # shape (NCR, NB)

    S = wing_area(BB, CRR, TAPER)
    AR = (BB * BB) / np.maximum(S, 1e-12)
    Vs = stall_speed(MASS_KG, RHO, G0, S, CLmax)

    # best (min Vs)
    idx = np.unravel_index(np.argmin(Vs), Vs.shape)
    b_star = float(BB[idx])
    cr_star = float(CRR[idx])
    Vs_star = float(Vs[idx])
    S_star = float(S[idx])
    AR_star = float(AR[idx])

    plt.figure()
    ax = plt.gca()

    im = ax.imshow(
        Vs,
        origin="lower",
        aspect="auto",
        extent=[B_MIN, B_MAX, CR_MIN, CR_MAX],
    )
    plt.colorbar(im, ax=ax, label="Vs (m/s)")

    # --- Stall-speed target contours ---
    target_vs = [2.5,3,3.5, 4.0, 4.5]
    cs_vs = plt.contour(
        BB, CRR, Vs,
        levels=target_vs,
        linewidths=2.0,
        colors="white"
    )
    plt.clabel(cs_vs, inline=True, fontsize=9, fmt="Vs=%.1f m/s")


    # ---- Area contours (overlay on top of heatmap) ----
    if DRAW_AREA_CONTOURS:
        smin = float(np.nanmin(S))
        smax = float(np.nanmax(S))
        if np.isfinite(smin) and np.isfinite(smax) and smax > smin:
            area_levels = np.linspace(smin, smax, N_AREA_LEVELS)
            cs_area = ax.contour(
                BB, CRR, S,
                levels=area_levels,
                linestyles=AREA_CONTOUR_LINESTYLE,
                linewidths=AREA_CONTOUR_LINEWIDTH,
                colors=AREA_CONTOUR_COLOR
            )
            ax.clabel(cs_area, inline=True, fontsize=8, fmt=AREA_LABEL_FMT)

    # ---- Vs contours (optional) ----
    if DRAW_VS_CONTOURS:
        vmin = float(np.nanmin(Vs))
        vmax = float(np.nanmax(Vs))
        if np.isfinite(vmin) and np.isfinite(vmax) and vmax > vmin:
            levels = np.linspace(vmin, vmax, N_VS_LEVELS)
            ax.contour(BB, CRR, Vs, levels=levels, linewidths=0.8)

    # mark minimum point
    ax.scatter([b_star], [cr_star], s=60, zorder=5)

    # annotation in upper-left corner of axes (no overlap)
    annotation_text = (
        f"min Vs = {Vs_star:.2f} m/s\n"
        f"S = {S_star:.3f} m²\n"
        f"AR = {AR_star:.1f}\n"
        f"b* = {b_star:.2f} m  cr* = {cr_star:.3f} m"
    )

    ax.text(
        0.02, 0.98,                 # axes-fraction coordinates
        annotation_text,
        transform=ax.transAxes,
        ha="left",
        va="top",
        bbox=dict(
            boxstyle="round",
            facecolor="white",
            alpha=0.85,
            edgecolor="none"
        )
    )

    ax.set_xlabel("Span b (m)")
    ax.set_ylabel("Root chord cr (m)")
    ax.set_title(
        f"{foil} — stall speed heatmap\n"
        f"CLmax={CLmax:.3f}  bestLD={bestLD:.2f}  conv={conv}  {note}"
    )
    ax.grid(False)


def main():
    if not os.path.exists(GLIDER_METRICS_CSV):
        raise FileNotFoundError(
            f"Missing {GLIDER_METRICS_CSV}\n"
            "Write this CSV from your C++ run (glider metrics table)."
        )

    rows = read_glider_metrics(GLIDER_METRICS_CSV)
    topLD, topCL = pick_top2(rows)

    print("Top by best L/D:", topLD["foil"], "bestLD=", topLD["bestLD"], "maxCL=", topLD["maxCL"])
    print("Top by max CL  :", topCL["foil"], "bestLD=", topCL["bestLD"], "maxCL=", topCL["maxCL"])

    plot_heatmap_for_foil(topLD)
    plot_heatmap_for_foil(topCL)

    plt.show()


if __name__ == "__main__":
    main()
