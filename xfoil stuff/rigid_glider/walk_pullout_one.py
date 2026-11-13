"""Walkthrough step 4: one nose-down drop in the pull-out simulation (pullout_sim.py), for one kit wing, one CG and one tab angle.

   py -B rigid_glider/walk_pullout_one.py F45_span250 39 6        (wing, CG in mm behind the root leading edge, tab angle in degrees, trailing edge up)

wings: F45_span250, F60_span250, B45_span400.  The CG must be one of the kit's whole-millimetre settings (F45: 36..49, F60: 34..44, B45: 57..78).
It uses the same functions as pullout_sim.py (flat_specimen, PlateTable, run_case); the only new code is the printing.
Writes to results/runs/pullout_one/ next to the script's ROOT (so a copy in a practice folder writes inside the practice folder).  Standard library only.
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pullout_sim as ps
import plate_balance as pb


def main():
    if len(sys.argv) != 4 or sys.argv[1] not in ps.SPECS:
        sys.exit(__doc__)
    name, cg, tab = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    out = ps.ROOT / "results/runs/pullout_one"
    out.mkdir(parents=True, exist_ok=True)
    print(f"1. loading {name} and running AVL for its derivatives (3 short AVL runs) ...")
    spec = ps.flat_specimen(name, out, log=lambda s: None)
    if cg not in spec["cgs_all"]:
        sys.exit(f"CG {cg:g} is not a kit setting; choose one of {', '.join(f'{c:g}' for c in spec['cgs_all'])}")
    rows = pb.read_rows()
    plate = ps.PlateTable(spec["ar"], spec["taper"], rows, pb.group_runs(rows))
    print(f"2. dropping it nose-down from rest: CG {cg:g} mm, tab {tab:g} deg ...")
    m, rec = ps.run_case(spec, plate, cg, tab, "one drop")
    sm = m["static_margin_pct"]
    print()
    print(f"   glider mass            {m['mass_g']:.2f} g")
    print(f"   static margin          {sm:+.1f} % of the chord (CG ahead of the neutral point = positive)")
    print(f"   VERDICT                {m['verdict']}")
    sd = m["swoop_depth_m"]
    print(f"   first-swoop depth      {'-' if sd is None else format(sd, '.1f') + ' m'}   (lowest point reached before the first level-off)")
    f = m["depth_formula_m"]
    print(f"   formula depth          {'-' if f is None else format(f, '.1f') + ' m'}   (3 W/S / (rho g CL), a cross-check)")
    gam = m["gamma_final_deg"]
    print(f"   settled flight path    {'-' if gam is None else format(gam, '.1f') + ' deg'}   glide ratio {'-' if m['glide_ratio_final'] is None else format(m['glide_ratio_final'], '.1f')}")
    if abs(sm) < ps.CANNOT_SAY_SM_PCT:
        print(f"   NOTE: the margin is under {ps.CANNOT_SAY_SM_PCT:g} %: the model cannot say whether this verdict is right.")
    keys = ["spec", "cg_mm", "up_deg", "static_margin_pct", "mass_g", "verdict", "swoop_depth_m", "depth_formula_m", "h_first_level_m", "gamma_final_deg", "speed_final", "glide_ratio_final"]
    with (out / "one_drop.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(keys)
        w.writerow([ps.fv(m.get(k)) for k in keys])
    with (out / "trajectory.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "x_forward_m", "height_change_m"])
        w.writerows((f"{r[0]:.3f}", f"{r[1]:.3f}", f"{r[2]:.3f}") for r in rec[::5])
    (out / "trajectory.svg").write_text(ps.svg_traj(f"{name}, CG {cg:g} mm, tab {tab:g} deg: {m['verdict']}", [("this drop", "#7fb4ff", ps.traj_pts(rec))]), encoding="utf-8")
    print()
    print(f"3. files written to {out}:  one_drop.csv, trajectory.csv, trajectory.svg")


if __name__ == "__main__":
    main()
