"""Focused, reproducible XFOIL polars for actual bent symmetric sections.

Run: py rigid_glider/reflex_xfoil.py --chord 0.10 --angle 3
The generated foil is a rigid rotation of the aft chord about the hinge.
This is a sharp but zero-gap bend, not an ideal zero-thickness flat plate.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/reflex/xfoil"
EXE = ROOT / "tools/xfoil/xfoil.exe"
CONFIG = Path(__file__).with_name("reflex_config.json")


def coordinates(thickness: float, chord_fraction: float, angle_deg: float,
                n_half: int = 160) -> list[tuple[float, float]]:
    """Cosine-spaced closed-TE NACA 00xx, rotated aft of hinge; upper TE first."""
    xh = 1 - chord_fraction
    angle = math.radians(angle_deg)
    result = []
    for upper, indexes in ((True, range(n_half, -1, -1)),
                           (False, range(1, n_half + 1))):
        for i in indexes:
            x = (1 - math.cos(math.pi*i/n_half))/2
            y = 5*thickness*(0.2969*math.sqrt(x)-0.1260*x-0.3516*x*x+
                             0.2843*x**3-0.1036*x**4)
            if not upper:
                y = -y
            if x > xh:
                dx = x-xh
                x, y = xh+dx*math.cos(angle)-y*math.sin(angle), dx*math.sin(angle)+y*math.cos(angle)
            result.append((x, y))
    return result


def read_polar(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(errors="replace").splitlines():
        parts = line.split()
        if len(parts) >= 7:
            try:
                alpha, cl, cd, cdp, cm, top, bot = map(float, parts[:7])
            except ValueError:
                continue
            if -5 <= alpha <= 10 and cd >= 0:
                rows.append(dict(alpha_deg=alpha, cl=cl, cd=cd, cdp=cdp,
                                 cm=cm, top_xtr=top, bot_xtr=bot))
    # PACC can include a seeded alpha and a repeated start point.
    return sorted({round(row["alpha_deg"], 5): row for row in rows}.values(),
                  key=lambda row: row["alpha_deg"])


def run(section: str, reynolds: int, chord: float, angle: float,
        inviscid: bool = False, reverse: bool = False) -> dict:
    label = (f"{section}_{'inviscid' if inviscid else 'Re'+str(reynolds)}_"
             f"c{int(chord*100):02d}_a{angle:g}{'_reverse' if reverse else ''}").replace(".", "p")
    folder = OUT / label
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "polar.dat").unlink(missing_ok=True)
    foil = folder / "foil.dat"
    thickness = int(section[-2:])/100
    foil.write_text(label+"\n"+"\n".join(f"{x:.8f} {y:.8f}" for x,y in coordinates(thickness,chord,angle))+"\n",
                    encoding="ascii")
    # Match historical Ncrit=5; use XFOIL's native 160-panel re-paneling.
    # A 220-panel attempt is retained nowhere: this build's PPAR batch
    # prompt is not compatible with the historical generator's syntax.
    script = ("PLOP\nG\n\nLOAD foil.dat\nPANE\nOPER\n"+
              ("" if inviscid else f"VISC {reynolds}\nVPAR\nN 5\n\nITER 150\n")+
              "PACC\npolar.dat\n\nALFA 0\n"+
              ("" if inviscid else "INIT\n")+
              ("ALFA 8\nASEQ 8 -3 -0.5\n" if reverse else
               "ALFA -3\nASEQ -3 8 0.5\n")+
              "PACC\n\nQUIT\n")
    (folder / "commands.in").write_text(script, encoding="ascii")
    try:
        proc = subprocess.run([str(EXE)], input=script, text=True, cwd=folder,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90)
        log = proc.stdout
        code = proc.returncode
    except subprocess.TimeoutExpired as error:
        log = (error.stdout or b"").decode(errors="replace") if isinstance(error.stdout, bytes) else error.stdout or ""
        code = "timeout"
    (folder / "xfoil.log").write_text(log, encoding="utf-8", errors="replace")
    rows = read_polar(folder / "polar.dat")
    return dict(section=section, reynolds=None if inviscid else reynolds, chord_fraction=chord,
                angle_deg=angle, exit_code=code, rows=rows, expected_count=23,
                converged_count=len(rows), inviscid=inviscid, reverse=reverse,
                folder=str(folder))


def at_cl(rows: list[dict], target: float) -> dict | None:
    # Only interpolate adjacent alpha samples, with no extrapolation across gaps.
    for a, b in zip(rows, rows[1:]):
        if b["alpha_deg"]-a["alpha_deg"] > 0.51:
            continue
        if min(a["cl"],b["cl"]) <= target <= max(a["cl"],b["cl"]) and abs(b["cl"]-a["cl"]) > 1e-6:
            t = (target-a["cl"])/(b["cl"]-a["cl"])
            return {key:a[key]+t*(b[key]-a[key]) for key in ("alpha_deg","cd","cm")}
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--chord", type=float, default=0.10)
    parser.add_argument("--angle", type=float, default=3.0)
    parser.add_argument("--inviscid-check-only", action="store_true")
    parser.add_argument("--reverse-check-only", action="store_true")
    args = parser.parse_args()
    if args.inviscid_check_only:
        checks = [run("NACA_0008", 80000, args.chord, angle, inviscid=True)
                  for angle in (0.0, args.angle)]
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "inviscid_check.json").write_text(json.dumps(checks, indent=2))
        print("Wrote inviscid geometry check:", OUT / "inviscid_check.json")
        return
    if args.reverse_check_only:
        checks = [run("NACA_0008", reynolds, args.chord, args.angle, reverse=True)
                  for reynolds in (60000, 80000, 100000)]
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "reverse_check.json").write_text(json.dumps(checks, indent=2))
        print("Wrote reverse-sweep check:", OUT / "reverse_check.json")
        return
    cfg = json.loads(CONFIG.read_text())
    jobs = [(section, reynolds, args.chord, angle)
            for section in cfg["xfoil_sections"] for reynolds in cfg["xfoil_reynolds"]
            for angle in (0.0, args.angle)]
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = [future.result() for future in as_completed(pool.submit(run, *job) for job in jobs)]
    results.sort(key=lambda r:(r["section"], r["reynolds"], r["angle_deg"]))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "summary.json").write_text(json.dumps(results, indent=2))
    with (OUT / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["section","reynolds","angle_deg","converged_count",
                                                 "target_cl","alpha_deg","cd","cm"])
        writer.writeheader()
        for r in results:
            for target in (.2,.3,.4):
                sample = at_cl(r["rows"],target) or {}
                writer.writerow({"section":r["section"],"reynolds":r["reynolds"],
                                 "angle_deg":r["angle_deg"],"converged_count":r["converged_count"],
                                 "target_cl":target,**sample})
    print(f"{len(results)} XFOIL runs; {sum(r['converged_count'] for r in results)} of "
          f"{23*len(results)} requested alpha points. Output: {OUT}")


if __name__ == "__main__":
    main()
