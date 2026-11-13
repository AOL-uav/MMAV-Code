"""Post-process plate/section_summary.csv: blank result fields for polars below the convergence floor (15 of 29 points) and add
below_floor.  Reads and rewrites only results/runs/rigid_glider/plate/section_summary.csv.  py -B rigid_glider/plate_summary_flag.py"""
import sys
sys.dont_write_bytecode = True
import csv
from pathlib import Path

P = Path(__file__).resolve().parents[1] / "results/runs/rigid_glider/plate/section_summary.csv"
FLOOR = 15
BLANK = ["cl_max", "alpha_at_cl_max", "cm_at_cl_0.1", "cm_at_cl_0.2", "cm_at_cl_0.3", "cm_at_cl_0.4", "peak_nose_up_cm", "cl_at_peak_cm",
         "alpha_at_peak_cm", "lip_dcm_cl0.25", "inviscid_lip_dcm_cl0.25", "retained_fraction_cl0.25"]

rows = list(csv.DictReader(P.open(encoding="utf-8")))
fields = list(rows[0].keys())
if "below_floor" not in fields:
    fields.append("below_floor")
for r in rows:
    low = int(r["n_converged"]) < FLOOR
    r["below_floor"] = int(low)
    if low:
        for k in BLANK:
            r[k] = ""
        r["note"] = f"below convergence floor ({FLOOR} of 29 points); results blanked"
with P.open("w", newline="", encoding="utf-8") as s:
    w = csv.DictWriter(s, fieldnames=fields)
    w.writeheader()
    w.writerows(rows)
print(sum(r["below_floor"] for r in rows), "of", len(rows), "rows below floor")
