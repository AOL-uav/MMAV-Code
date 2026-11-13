"""Walkthrough step 3: turn the AVL neutral point into the CG and the nose weight for Swift A, foam 400 mm, with the arithmetic printed line by line.

   py -B rigid_glider/walk_balance.py [XNP_MM] [MARGIN]        defaults: 41.713 mm (read from the AVL screen), 0.10

It calls the same functions the v4 build uses (bird_outline_v2.size_numbers); nothing is re-modelled here.  No AVL run, nothing written.
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import bird_outline_v2 as v2
import bird_outline_v4_swift as v4

K = 400.0


def main():
    xnp = float(sys.argv[1]) if len(sys.argv) > 1 else 41.713
    margin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
    g = v4.build_geometry4(v4.GLIDE)
    mac = g["mac"] * K
    print("STEP 1  static margin in millimetres")
    print(f"   MAC (the 'Cref' line on the AVL screen)      = {mac:.3f} mm")
    print(f"   margin {margin:.2f} x MAC                          = {margin * mac:.3f} mm")
    cg = xnp - margin * mac
    print("STEP 2  CG = neutral point - margin")
    print(f"   {xnp:.3f} - {margin * mac:.3f}                        = {cg:.3f} mm behind the root leading edge")
    res = v4.fake_res(cg / K, margin)
    n = v2.size_numbers(g, res, "foam", 6.0)
    print("STEP 3  the glider before any nose weight (foam sheet + printed rail)")
    print(f"   foam sheet     {n['m_sheet']:.3f} g   (sheet area x foamboard areal mass, ASSUMED 300 g/m2)")
    print(f"   printed rail   {n['rail']['mass_g']:.3f} g")
    print(f"   together M0    {n['M0']:.3f} g, balancing at cg0 = {n['cg0']:.2f} mm")
    print("STEP 4  how much weight at the nose hole to move the balance point back to the target")
    print(f"   target CG      {n['cg_target']:.2f} mm;  nose weight position (rail hole) {n['station']:.1f} mm")
    print(f"   weight = M0 x (target - cg0) / (position - target)")
    print(f"          = {n['M0']:.3f} x ({n['cg_target']:.2f} - {n['cg0']:.2f}) / ({n['station']:.1f} - {n['cg_target']:.2f})")
    print(f"          = {n['ballast_g']:.3f} g")
    print(f"   all-up mass    {n['M']:.3f} g   (nose weight is {100 * n['ballast_frac']:.1f} % of it)")
    print(f"   wing loading   {n['ws']:.2f} N/m2")
    print("STEP 5  first-swoop depth estimate (formula, MODEL)")
    cl = 0.3
    d0 = 3.0 * n["ws"] / (v2.ft.RHO_AIR * 9.80665 * cl)
    print(f"   depth = 3 x (W/S) / (rho x g x CL) = 3 x {n['ws']:.2f} / ({v2.ft.RHO_AIR} x 9.80665 x {cl}) = {d0:.2f} m")


if __name__ == "__main__":
    main()
