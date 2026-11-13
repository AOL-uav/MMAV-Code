"""Copy the pieces needed for the tools walkthrough into a PRACTICE folder, so practice runs never overwrite the project's results.

   py -B rigid_glider/make_practice_folder.py <HOME>\\Documents\\mmav_practice

The project scripts work out their paths from where the script file sits (ROOT = the folder above rigid_glider).  A script run from inside the practice copy therefore
reads and writes only inside the practice copy.  Nothing in the project folder is changed.  Standard library only.
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V4 = "results/runs/rigid_glider/bird_outline_v4_swift"


def copy_tree(src: Path, dst: Path, patterns=("*",)):
    n = 0
    for p in patterns:
        for f in src.glob(p):
            if f.is_file():
                dst.mkdir(parents=True, exist_ok=True)
                shutil.copy2(f, dst / f.name)
                n += 1
    return n


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    dest = Path(sys.argv[1]).resolve()
    if ROOT in dest.parents or dest == ROOT:
        sys.exit("The practice folder must be OUTSIDE the project folder.")
    dest.mkdir(parents=True, exist_ok=True)
    print("practice folder:", dest)
    n = copy_tree(ROOT / "rigid_glider", dest / "rigid_glider", ("*.py",))
    print(f"  rigid_glider: {n} scripts")
    n = copy_tree(ROOT / "tools/avl", dest / "tools/avl", ("avl352.exe", "README.md"))
    n += copy_tree(ROOT / "tools/drop_tracker", dest / "tools/drop_tracker", ("*.html", "README.md"))
    print(f"  tools: {n} files (AVL, drop tracker)")
    n = copy_tree(ROOT / "research/plate_data", dest / "research/plate_data", ("*",))
    print(f"  research/plate_data: {n} files (read by the pull-out simulation)")
    kit = ROOT / "results/runs/rigid_glider/flat_test_v2"
    n = 0
    for f in kit.rglob("*"):
        if f.is_file():
            t = dest / "results/runs/rigid_glider/flat_test_v2" / f.relative_to(kit)
            t.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, t)
            n += 1
    print(f"  flat_test_v2 kit tables: {n} files (read by the pull-out simulation)")
    pu = ROOT / "results/runs/rigid_glider/pullout/cases.csv"                     # the v4 build page reads the pull-out ratios from here
    if pu.is_file():
        (dest / "results/runs/rigid_glider/pullout").mkdir(parents=True, exist_ok=True)
        shutil.copy2(pu, dest / "results/runs/rigid_glider/pullout/cases.csv")
        print("  pullout/cases.csv (read by the v4 build page)")
    pr = dest / "print_these"
    pr.mkdir(exist_ok=True)
    for rel in (f"{V4}/A_templates.pdf", f"{V4}/B_templates.pdf", "results/runs/rigid_glider/fin_kit_v4/fin_templates.pdf", "results/runs/rigid_glider/test_plan/drop_test_sheet.pdf"):
        if (ROOT / rel).is_file():
            shutil.copy2(ROOT / rel, pr / Path(rel).name)
    print("  print_these: the template and drop-sheet PDFs")
    # the Swift A foam 400 AVL deck used in the walkthrough: written here from the v4 geometry (the project folder is only read)
    sys.path.insert(0, str(ROOT / "rigid_glider"))
    import bird_outline_v2 as v2
    import bird_outline_v4_swift as v4
    g = v4.build_geometry4(v4.GLIDE)
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    lines = v2.avl_deck2(g, 400.0, 6.0, name="SwiftA_foam400_bend6").split("\n")
    lines[4] = "0.03697 0.0 0.0"                                 # moment reference = the trim CG, 37.0 mm (v4 outlines.csv: 23.1 mm at 250 mm span x 1.6)
    inp = dest / "practice_inputs"
    inp.mkdir(exist_ok=True)
    (inp / "SwiftA_foam400_bend6.avl").write_text("\n".join(lines), encoding="ascii")
    print("  practice_inputs/SwiftA_foam400_bend6.avl")
    print("done. Run everything from this folder; it is a copy, so outputs stay here.")


if __name__ == "__main__":
    main()
