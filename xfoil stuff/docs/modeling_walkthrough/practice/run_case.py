"""Run ONE AVL deck from Python and print the numbers the walkthrough talks about.  Standard library only.

    py -B run_case.py DECK.avl --alpha 4          # fixed angle of attack, degrees
    py -B run_case.py DECK.avl --cl 0.3           # fixed lift coefficient
    py -B run_case.py DECK.avl --cl 0.3 --xcg 0.040   # also move the CG / moment reference point (metres)

It types the same keystrokes you type in lesson 1 (see lesson1.in) and reads the screen text AVL prints.
The AVL program is tools/avl/avl352.exe in the project root.  Nothing here changes any project output."""
import argparse
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AVL = ROOT / "tools" / "avl" / "avl352.exe"
NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][-+]?\d+)?"


def keystrokes(alpha=None, cl=None, xcg=None):
    k = ["PLOP", "G", "", "OPER"]                      # PLOP / G: switch AVL's plot windows off, then back out; OPER: operating-point menu
    if xcg is not None:
        k += ["M", f"X {xcg}", ""]                     # M: modify parameters; X = X_cg (moment reference); blank = back to OPER
    k += [f"A C {cl}" if cl is not None else f"A A {alpha}"]   # A A <deg>: hold alpha; A C <CL>: hold lift coefficient (alpha is solved for)
    k += ["X", "ST", "", "", "QUIT"]                   # X: execute; ST: stability derivatives (blank file name = print to screen); blank: leave OPER; QUIT
    return k


def field(text, key, last=True):
    m = re.findall(rf"\b{re.escape(key)}\s*=\s*({NUM})", text)
    if not m:
        return None
    return float((m[-1] if last else m[0]).replace("D", "E"))


def run(deck, alpha=None, cl=None, xcg=None, timeout=300):
    deck = Path(deck)
    script = "\n".join(keystrokes(alpha if alpha is not None or cl is not None else 0.0, cl, xcg)) + "\n"
    r = subprocess.run([str(AVL), deck.name], input=script, text=True, cwd=deck.parent or HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
    t = r.stdout
    st = t[t.rfind("Stability-axis derivatives"):] if "Stability-axis derivatives" in t else ""
    out = dict(alpha=field(t, "Alpha"), CL=field(t, "CLtot"), CDind=field(t, "CDind"), e=field(t, "e"), Cm=field(t, "Cmtot"), Sref=field(t, "Sref"), Cref=field(t, "Cref"),
               Bref=field(t, "Bref"), Xref=field(t, "Xref"), CLa=field(st, "CLa", False), Cma=field(st, "Cma", False), Clb=field(st, "Clb", False), Cnb=field(st, "Cnb", False), Xnp=field(st, "Xnp", False))   # first match: a later line "Clb Cnr / Clr Cnb = ..." also contains "Cnb ="
    if out["Xnp"] is not None and out["Xref"] is not None and out["Cref"]:
        out["static_margin_pct_cref"] = 100 * (out["Xnp"] - out["Xref"]) / out["Cref"]
    out["_raw"] = t
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("deck")
    ap.add_argument("--alpha", type=float)
    ap.add_argument("--cl", type=float)
    ap.add_argument("--xcg", type=float)
    a = ap.parse_args()
    o = run(a.deck, a.alpha, a.cl, a.xcg)
    for k, v in o.items():
        if k != "_raw":
            print(f"{k:24s} {v}")
    if o["CL"] is None:
        print(o["_raw"][-1500:])
        sys.exit(1)


if __name__ == "__main__":
    main()
