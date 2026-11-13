"""Two extra checks added after review (new file; lesson2_batch.py and its results are untouched).
  1. CG move at ONE operating point (CL 0.3): Xref 0.030021207 then 0.040, Cma and Xnp read at each.
  2. Does the NACA thickness digit matter in AVL?  Same F45 deck with NACA 0000 / 0006 / 0012 (new decks in variants/), CL 0.3.
     py -B review_checks.py        (about 10 s; writes review_checks_results.txt)"""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
import run_case as rc

HERE = Path(__file__).resolve().parent
bare = (HERE / "f45_bare.avl").read_text()
out = []
KEYS = ("alpha", "CL", "Xref", "Cm", "Cma", "CLa", "Xnp", "Clb", "Cnb", "e")


def fmt(label, o):
    return f"{label:34s}" + "  ".join(f"{k}={o[k]:.6f}" for k in KEYS)


out.append("CHECK A: CG move at one operating point, F45 bare, CL = 0.3")
for x in (None, 0.040):
    o = rc.run(HERE / "f45_bare.avl", cl=0.3, xcg=x)
    out.append(fmt("Xref header" if x is None else f"Xref moved to {x}", o))
out.append("")
out.append("CHECK B: NACA digits on the F45 deck (both sections changed), CL = 0.3")
for code in ("0000", "0006", "0012"):
    txt = bare.replace("NACA\n0000", f"NACA\n{code}")
    assert txt.count(f"NACA\n{code}") == 2
    p = HERE / "variants" / f"naca{code}.avl"
    p.write_text(txt)
    out.append(fmt(f"NACA {code}", rc.run(p, cl=0.3)))
(HERE / "review_checks_results.txt").write_text("\n".join(out) + "\n")
print("\n".join(out))
