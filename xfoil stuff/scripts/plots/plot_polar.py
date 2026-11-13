import sys
import os
import math
import argparse
from pathlib import Path
import matplotlib.pyplot as plt

def is_headerish(line: str) -> bool:
    s = line.strip().lower()
    if not s:
        return True
    if s.startswith("#"):
        return True
    return ("alpha" in s and "cl" in s and "cd" in s)

def try_float(tok: str):
    try:
        x = float(tok)
        if math.isfinite(x):
            return x
    except Exception:
        pass
    return None

def parse_row(line: str):
    """
    Accepts:
      alpha CL CD CDp CM [Top_Xtr Bot_Xtr] [Note]
    We need alpha, CL, CD, and CM.
    """
    if is_headerish(line):
        return None

    parts = line.strip().split()
    if len(parts) < 5: # Changed to ensure we can reach CM at index 4
        return None

    a = try_float(parts[0])
    cl = try_float(parts[1])
    cd = try_float(parts[2])
    cm = try_float(parts[4]) # CDp is parts[3], CM is parts[4]
    
    if a is None or cl is None or cd is None or cm is None:
        return None

    # sanity (match your C++ filters)
    if cd <= 0.0 or cd > 5.0:
        return None
    if abs(cl) > 5.0:
        return None

    return a, cl, cd, cm

def read_data(path: str):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            row = parse_row(line)
            if row is None:
                continue
            rows.append(row)

    rows.sort(key=lambda t: t[0])

    alpha = [r[0] for r in rows]
    cl    = [r[1] for r in rows]
    cd    = [r[2] for r in rows]
    cm    = [r[3] for r in rows]
    return alpha, cl, cd, cm

def out_png_name(input_path: str, suffix: str) -> str:
    base = os.path.splitext(os.path.basename(input_path))[0]
    return f"{base}_{suffix}.png"

def safe_div(a, b, eps=1e-12):
    if abs(b) < eps:
        return float("nan")
    return a / b

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input_dat", nargs="?", default=str(Path(__file__).resolve().parents[2] / "results" / "legacy" / "test_polar_output.dat"))
    ap.add_argument("output_png", nargs="?", default=None,
                    help="If provided, save to this exact path and do NOT show GUI.")
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    path = args.input_dat
    if not os.path.exists(path):
        print(f"File not found: {path}")
        return 2

    alpha, cl, cd, cm = read_data(path)
    if not alpha:
        print(f"No numeric polar rows parsed from: {path}")
        return 1

    ld = [safe_div(cL, cD) for cL, cD in zip(cl, cd)]

    fig = plt.figure(figsize=(10, 6))

    # 1. Cl v Alpha
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(alpha, cl)
    ax1.set_title("Cl v Alpha")
    ax1.set_xlabel("Alpha [deg]")
    ax1.set_ylabel("Cl")
    ax1.grid(True)

    # 2. Cl/Cd v Alpha
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(alpha, ld)
    ax2.set_title("Cl/Cd v Alpha")
    ax2.set_xlabel("Alpha [deg]")
    ax2.set_ylabel("Cl/Cd")
    ax2.grid(True)

    # 3. Cd v Alpha
    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(alpha, cd)
    ax3.set_title("Cd v Alpha")
    ax3.set_xlabel("Alpha [deg]")
    ax3.set_ylabel("Cd")
    ax3.grid(True)

    # 4. Cm v Alpha
    ax4 = plt.subplot(2, 2, 4)
    ax4.plot(alpha, cm, color='purple', label='Cm')
    ax4.axhline(0, color='red', linestyle='--', label='Cm = 0')
    ax4.set_title("Cm v Alpha")
    ax4.set_xlabel("Alpha [deg]")
    ax4.set_ylabel("Cm")
    ax4.legend()
    ax4.grid(True)

    if args.title:
        fig.suptitle(args.title)

    fig.tight_layout()

    # Decide output path
    if args.output_png is None:
        dash_png = out_png_name(path, "polar_dashboard")
        fig.savefig(dash_png, dpi=200, bbox_inches="tight")
        print(f"Saved: {dash_png}")
        plt.show()
    else:
        outp = args.output_png
        os.makedirs(os.path.dirname(outp) or ".", exist_ok=True)
        fig.savefig(outp, dpi=200, bbox_inches="tight")
        print(f"Saved: {outp}")
        plt.close(fig)

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
