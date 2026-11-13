# llt_plot.py
# Usage:
#   py scripts/plots/llt_plot.py results/legacy/wingAnalysis/glider_wingpolar_llt.dat results/legacy/wingAnalysis/glider_llt_bestalpha_stations.dat results/runs/glider_lltPlot.png

import sys
from pathlib import Path
import matplotlib.pyplot as plt

def _parse_kv_from_header_line(s):
    kv = {}
    for tok in s.replace("#", " ").split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            kv[k.strip()] = v.strip()
    return kv

def read_wingpolar(path):
    meta = {}
    a=[]; cl=[]; cdp=[]; cdi=[]; cd=[]; ld=[]; conv=[]; note=[]
    with open(path, "r") as f:
        for line in f:
            s=line.strip()
            if not s:
                continue

            # Parse meta from header lines
            if s.startswith("#"):
                kv = _parse_kv_from_header_line(s)
                if kv:
                    meta.update(kv)
                continue

            if s.lower().startswith("alpha"):
                continue

            parts = s.split()
            if len(parts) < 9:
                continue
            try:
                a.append(float(parts[0]))
                cl.append(float(parts[1]))
                cdp.append(float(parts[2]))
                cdi.append(float(parts[3]))
                cd.append(float(parts[4]))
                ld.append(float(parts[5]))
                conv.append(int(parts[7]) == 1)
                note.append(parts[8])
            except:
                continue
    return meta, a, cl, cdp, cdi, cd, ld, conv, note

def _build_column_index_from_header(header_line):
    cols = header_line.strip().split()
    idx = {}
    for i, name in enumerate(cols):
        idx[name] = i
    return idx

def read_station_dump(path):
    meta = {}
    rows = []
    col_idx = None

    with open(path, "r") as f:
        for line in f:
            s=line.strip()
            if not s:
                continue

            if s.startswith("#"):
                kv = _parse_kv_from_header_line(s)
                if kv:
                    meta.update(kv)
                continue

            # Detect the column header line (starts with "i")
            if s.lower().startswith("i "):
                col_idx = _build_column_index_from_header(s)
                continue

            parts = s.split()
            if len(parts) < 9:
                continue
            rows.append(parts)

    # Fallback: try a reasonable guess for your current dump layout.
    # Your C++ writes:
    # i theta yFrac yAbs chord wTip Re alpha_i alpha_eff_raw alpha_eff minWindow maxWindow clamp_delta identity_error
    # ReLoRoot ReHiRoot ReLoTip ReHiTip cl_blend cd_blend
    if col_idx is None:
        col_idx = {
            "yFracHalfSpan": 2,
            "alpha_i_deg": 7,
            "alpha_eff_deg": 9,
            "cl_blend": 18,
            "cd_blend": 19,
        }

    def getf(parts, key, default=None):
        i = col_idx.get(key, None)
        if i is None or i >= len(parts):
            return default
        try:
            return float(parts[i])
        except:
            return default

    y=[]; a_i=[]; a_eff=[]; cl=[]; cd=[]
    for parts in rows:
        yy   = getf(parts, "yFracHalfSpan", None)
        ai   = getf(parts, "alpha_i_deg", None)
        aeff = getf(parts, "alpha_eff_deg", None)
        cli  = getf(parts, "cl_blend", None)
        cdi  = getf(parts, "cd_blend", None)
        if yy is None or ai is None or aeff is None or cli is None or cdi is None:
            continue
        y.append(yy); a_i.append(ai); a_eff.append(aeff); cl.append(cli); cd.append(cdi)

    # sort by y
    idx_sorted = sorted(range(len(y)), key=lambda i: y[i])
    y     = [y[i] for i in idx_sorted]
    a_i   = [a_i[i] for i in idx_sorted]
    a_eff = [a_eff[i] for i in idx_sorted]
    cl    = [cl[i] for i in idx_sorted]
    cd    = [cd[i] for i in idx_sorted]

    return meta, y, a_i, a_eff, cl, cd

def find_best_ld(a, ld, conv):
    best = None
    for i in range(len(a)):
        if not conv[i]:
            continue
        if best is None or ld[i] > best[1]:
            best = (a[i], ld[i])
    return best

def _meta_get(meta, key, default=None):
    v = meta.get(key, default)
    return default if v is None else v

def _build_case_title(meta_polar, meta_dump):
    # Prefer wingpolar meta; fall back to dump meta if missing
    meta = dict(meta_dump)
    meta.update(meta_polar)

    root  = _meta_get(meta, "root", "UNKNOWN_ROOT")
    tip   = _meta_get(meta, "tip",  "UNKNOWN_TIP")
    mode  = _meta_get(meta, "mode", None)
    ReRef = _meta_get(meta, "Re_ref", None)
    Ncrit = _meta_get(meta, "Ncrit", None)

    parts = []
    if mode:
        parts.append(mode)
    parts.append(f"{root} → {tip}")

    extras = []
    if ReRef is not None:
        extras.append(f"Re_ref={ReRef}")
    if Ncrit is not None:
        extras.append(f"Ncrit={Ncrit}")
    if extras:
        parts.append("(" + ", ".join(extras) + ")")

    return "  |  ".join(parts)

def main():
    if len(sys.argv) != 4:
        print("Usage: python llt_plot.py <wingpolar_llt_case.dat> <llt_bestalpha_stations.dat> <out.png>")
        return 2

    polar_path = sys.argv[1]
    dump_path  = sys.argv[2]
    out_png    = sys.argv[3]

    meta_polar, a, CL, CDp, CDi, CD, LD, conv, note = read_wingpolar(polar_path)
    meta_dump,  y, a_i, a_eff, cl_s, cd_s = read_station_dump(dump_path)

    best = find_best_ld(a, LD, conv)
    best_a  = best[0] if best else None
    best_ld = best[1] if best else None

    # Trim extreme tip points for station plot
    y2=[]; a_i2=[]; a_eff2=[]; cl2=[]; cd2=[]
    for i in range(len(y)):
        if y[i] > 0.99:
            continue
        y2.append(y[i]); a_i2.append(a_i[i]); a_eff2.append(a_eff[i]); cl2.append(cl_s[i]); cd2.append(cd_s[i])

    fig = plt.figure(figsize=(12, 8))

    # Figure-level title: airfoil combo (applies to both glider and rotor runs)
    case_title = _build_case_title(meta_polar, meta_dump)
    fig.suptitle(case_title, fontsize=12)

    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(a, CL, marker="o", markersize=3, linewidth=1)
    ax1.set_title("CL vs alpha (wing)")
    ax1.set_xlabel("alpha (deg)")
    ax1.set_ylabel("CL")
    ax1.grid(True)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(a, CD,  marker="o", markersize=3, linewidth=1, label="CD total")
    ax2.plot(a, CDi, marker="o", markersize=3, linewidth=1, label="CDi")
    ax2.plot(a, CDp, marker="o", markersize=3, linewidth=1, label="CD profile")
    ax2.set_title("Drag breakdown")
    ax2.set_xlabel("alpha (deg)")
    ax2.set_ylabel("CD")
    ax2.grid(True)
    ax2.legend()

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(a, LD, marker="o", markersize=3, linewidth=1)
    ax3.set_title("L/D vs alpha")
    ax3.set_xlabel("alpha (deg)")
    ax3.set_ylabel("L/D")
    ax3.grid(True)

    if best_a is not None and best_ld is not None:
        ax3.axvline(best_a, linewidth=1)

        # Decide whether to put the label above or below the point,
        # based on how close the point is to the top of the axes.
        y0, y1 = ax3.get_ylim()
        yr = max(1e-9, (y1 - y0))

        # If the point is in the top ~12% of the plot, place label BELOW it.
        near_top = (best_ld > (y1 - 0.12 * yr))

        dx = 8
        dy = -18 if near_top else 12
        va = "top" if near_top else "bottom"

        ax3.annotate(
            f"best ~ {best_ld:.2f}\n@ alpha={best_a:.2f}",
            xy=(best_a, best_ld),
            xytext=(dx, dy),
            textcoords="offset points",
            ha="left",
            va=va,
            fontsize=9,
            bbox=dict(
                boxstyle="round,pad=0.25",
                facecolor="white",
                edgecolor="none",
                alpha=0.75
            )
        )


    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(y2, a_eff2, marker="o", markersize=3, linewidth=1, label="alpha_eff (deg)")
    ax4.plot(y2, a_i2,   marker="o", markersize=3, linewidth=1, label="alpha_i (deg)")
    ax4.set_title("Best-alpha station angles")
    ax4.set_xlabel("yFrac (half-span)")
    ax4.set_ylabel("deg")
    ax4.grid(True)

    blend_start = meta_dump.get("blendStart", None)
    try:
        if blend_start is not None:
            blend_start = float(blend_start)
    except:
        blend_start = None

    if blend_start is not None:
        ax4.axvline(blend_start, linestyle="--", linewidth=1.2, label="blendStart")

    ax4b = ax4.twinx()
    ax4b.plot(y2, cl2, marker="x", markersize=4, linewidth=1, label="cl")
    ax4b.plot(y2, cd2, marker="x", markersize=4, linewidth=1, label="cd")
    ax4b.set_ylabel("cl / cd")

    lines = ax4.get_lines() + ax4b.get_lines()
    labels = [l.get_label() for l in lines]
    ax4.legend(lines, labels, loc="best")

    # leave room for suptitle
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=140)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
