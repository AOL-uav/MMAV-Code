"""XFOIL reliability checks for the reflex-lip section polars (numerical sensitivity, not validation).

Writes ONLY under results/runs/rigid_glider/reflex_ext/gaps/ (or --out). Reuses (never
edits) reflex_xfoil (coordinates, read_polar, at_cl, run) and reflex_ext_predict (matched).
Existing polars are only READ.

Stages (run from the repository root with py -B):
  py -B rigid_glider/reflex_ext_gaps.py gate step1 step2 step3
  gate   repro gate: baseline settings must reproduce the stored 15%/8 and clean 0008 Re 80k polars
  step1  paneling sensitivity (PANE 160 vs PPAR 240/320, hinge refinement, no-PANE as-is input)
  step2  boundary-layer DUMP cases and separation/reattachment summary
  step3  retries of the unusable NACA 0008 20%/10 deg Re 100k polar

Terms: "lip" = trailing-edge-up bend of chord fraction cf and angle deg (hinge at x/c = 1-cf).
Cm positive nose-up. Missing/unconverged points are reported as missing, never filled.
"""

from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import json
import math
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import reflex_xfoil as rx
import reflex_ext_predict as rp

ROOT = rx.ROOT
OUT = ROOT / "results/runs/rigid_glider/reflex_ext/gaps"
EXE = rx.EXE
STORED_BENT = ROOT / "results/runs/rigid_glider/reflex_ext/predict/xfoil/NACA_0008_Re80000_c15_a8"
STORED_CLEAN = ROOT / "results/runs/rigid_glider/reflex/xfoil/NACA_0008_Re80000_c10_a0"

SECTION = "NACA_0008"
THICK = 0.08
LIPS = [(0.10, 3.0), (0.15, 8.0), (0.20, 10.0)]
RES = [60000, 80000, 100000]
TARGET_CL = [0.20, 0.25, 0.30]
SLOPE_CL = (0.15, 0.35)
REFINE_HALF = 0.05          # refinement window = hinge x/c +- this
TIMEOUT_POLAR = 150
TIMEOUT_LONG = 300

# variant -> settings.  mode "pane": LOAD, optional PPAR, PANE.  mode "asis": LOAD only (no PANE).
VARIANTS = {
    "base160":     dict(mode="pane", n=160, refine=False, gen="cos"),
    "ppar240":     dict(mode="pane", n=240, refine=False, gen="cos"),
    "ppar320":     dict(mode="pane", n=320, refine=False, gen="cos"),
    "ref160":      dict(mode="pane", n=160, refine=True, gen="cos"),
    "ref240":      dict(mode="pane", n=240, refine=True, gen="cos"),
    "asis_hinge":  dict(mode="asis", n=None, refine=False, gen="hinge"),
    "asis_cos321": dict(mode="asis", n=None, refine=False, gen="cos"),
    "asis_cos161": dict(mode="asis", n=None, refine=False, gen="cos80"),
}
# the clean section is generated per hinge window when the variant is hinge-specific
WINDOWED = {"ref160", "ref240", "asis_hinge"}


# ======================================================================= geometry
def _yt(t: float, x: float) -> float:
    return 5 * t * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4)


def _hinge_x_list(xh: float, n_half: int, spacing: float, half_window: float) -> list[float]:
    """Cosine nodes (LE->TE) with a uniform-spacing window around the hinge that contains xh exactly."""
    base = [(1 - math.cos(math.pi * i / n_half)) / 2 for i in range(n_half + 1)]
    lo, hi = xh - half_window, min(xh + half_window, 1.0)
    keep = [x for x in base if x < lo - 0.4 * spacing or x > hi + 0.4 * spacing]
    k = max(int(round((hi - lo) / spacing)), 2)
    win = [lo + (hi - lo) * j / k for j in range(k + 1)]
    j0 = min(range(len(win)), key=lambda j: abs(win[j] - xh))
    win[j0] = xh                                   # exact hinge node
    if hi >= 1.0 - 1e-12:
        win[-1] = 1.0
    return sorted(set(keep + win))


def coordinates_hinge(thickness: float, chord_fraction: float, angle_deg: float,
                      n_half: int = 70, spacing: float = 0.0025, half_window: float = 0.06
                      ) -> list[tuple[float, float]]:
    """Closed-TE NACA 00xx with a rigid TE-up rotation about (xh, 0) and a REAL corner at the hinge.

    Same rotation as reflex_xfoil.coordinates, but (a) a node sits exactly at x = xh, (b) the node
    spacing near the hinge is fine, (c) on the upper (fold) side the unrotated forward curve and
    the rotated aft curve are cut at their true intersection (no overlapping/back-tracking nodes);
    on the lower (gap) side both the unrotated hinge node and the rotated hinge node are kept, so
    the wedge gap is bridged by one short straight panel.  Order: upper TE -> LE -> lower TE.
    """
    xh = 1 - chord_fraction
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    xs = _hinge_x_list(xh, n_half, spacing, half_window)
    if angle_deg == 0:
        upper = [(x, _yt(thickness, x)) for x in xs]
        lower = [(x, -_yt(thickness, x)) for x in xs]
        return list(reversed(upper)) + lower[1:]

    def rot(x: float, y: float) -> tuple[float, float]:
        dx = x - xh
        return xh + dx * ca - y * sa, dx * sa + y * ca

    fwd = [x for x in xs if x <= xh + 1e-12]
    aft = [x for x in xs if x >= xh - 1e-12]
    # lower (gap) side, LE -> TE: forward nodes incl. hinge node, then rotated aft nodes incl. rotated hinge node
    lower = [(x, -_yt(thickness, x)) for x in fwd] + [rot(x, -_yt(thickness, x)) for x in aft]
    # upper (fold) side: cut both curves at their true intersection near x = xh
    upper_fwd = [(x, _yt(thickness, x)) for x in fwd]

    def f(dx: float) -> float:
        px, py = rot(xh + dx, _yt(thickness, xh + dx))
        return py - _yt(thickness, px)

    # dx0: where the rotated upper curve crosses x = xh (px is monotonic in dx)
    lo, hi = 0.0, 4 * _yt(thickness, xh) * math.tan(a) + 1e-6
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if rot(xh + mid, _yt(thickness, xh + mid))[0] < xh:
            lo = mid
        else:
            hi = mid
    dx0 = 0.5 * (lo + hi)
    if f(0.0) < 0 <= f(dx0):
        # true intersection of the rotated aft curve with the forward curve, at px < xh
        lo, hi = 0.0, dx0
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if f(mid) < 0:
                lo = mid
            else:
                hi = mid
        dxs = 0.5 * (lo + hi)
        ix, iy = rot(xh + dxs, _yt(thickness, xh + dxs))
        fwd_keep = [p for p in upper_fwd if p[0] < ix - 1e-9]
        aft_keep = [rot(x, _yt(thickness, x)) for x in aft if x - xh > dxs + 1e-9]
        upper = fwd_keep + [(ix, iy)] + aft_keep
    else:
        # rotated curve passes below the forward end point at x = xh (step of order 1e-5..1e-4 c):
        # corner node = the unrotated hinge node itself; drop rotated nodes at or ahead of it.
        aft_keep = [pt for pt in (rot(x, _yt(thickness, x)) for x in aft) if pt[0] > xh + 0.3 * spacing]
        upper = upper_fwd + aft_keep
    return list(reversed(upper)) + lower[1:]


def make_foil(variant: str, chord_fraction: float, angle_deg: float) -> list[tuple[float, float]]:
    gen = VARIANTS[variant]["gen"]
    if gen == "hinge":
        return coordinates_hinge(THICK, chord_fraction, angle_deg)
    if gen == "cos80":
        return rx.coordinates(THICK, chord_fraction, angle_deg, n_half=80)
    return rx.coordinates(THICK, chord_fraction, angle_deg)


# ======================================================================= bend sharpness
def read_xy(path: Path) -> list[tuple[float, float]]:
    pts = []
    for line in Path(path).read_text(errors="replace").splitlines():
        parts = line.split()
        if len(parts) >= 2:
            try:
                pts.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    return pts


def split_surfaces(pts: list[tuple[float, float]]) -> tuple[list, list]:
    """Return (upper, lower), each LE -> TE, from a TE(upper) -> LE -> TE(lower) loop."""
    i_le = min(range(len(pts)), key=lambda i: pts[i][0])
    upper = list(reversed(pts[:i_le + 1]))
    lower = pts[i_le:]
    return upper, lower


def seg_angles(surface: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """(x_mid, tangent angle deg) per panel of a LE->TE surface."""
    out = []
    for (x0, y0), (x1, y1) in zip(surface, surface[1:]):
        if math.hypot(x1 - x0, y1 - y0) < 1e-12:
            continue
        out.append(((x0 + x1) / 2, math.degrees(math.atan2(y1 - y0, x1 - x0))))
    return out


def _interp(xs: list[float], ys: list[float], x: float) -> float:
    order = sorted(range(len(xs)), key=lambda i: xs[i])
    xs = [xs[i] for i in order]
    ys = [ys[i] for i in order]
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1] and xs[i + 1] > xs[i]:
            t = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + t * (ys[i + 1] - ys[i])
    return ys[-1]


def bend_sharpness(bent: list[tuple[float, float]], clean: list[tuple[float, float]],
                   angle_deg: float, xh: float, window: float = 0.12, plateau_from: float = 0.04) -> dict:
    """Excess tangent-angle profile of one surface (LE->TE) relative to the clean paneled surface.

    dphi(panel) = panel angle(bent) - panel angle(clean, interpolated at the same panel-mid x).
    Reported: plateau (median dphi aft of the hinge; should approach the commanded angle), the
    largest single-node jump of dphi (turning concentrated at one node) as degrees and as a fraction
    of the commanded angle, x/c where dphi first reaches 10% and 90% of the commanded angle, the
    x/c width between them, the number of nodes between them, and the local panel length at the hinge.
    """
    bs = seg_angles(bent)
    cs = seg_angles(clean)
    cx = [p[0] for p in cs]
    cphi = [p[1] for p in cs]
    prof = [(x, phi - _interp(cx, cphi, x)) for x, phi in bs]
    win = [(i, x, d) for i, (x, d) in enumerate(prof) if abs(x - xh) <= window]
    res = {"commanded_deg": angle_deg, "hinge_x": xh, "n_nodes_window": len(win)}
    if len(win) < 3:
        res["note"] = "too few panels in window"
        return res
    aft = sorted(d for _, x, d in win if xh + plateau_from <= x <= xh + window)
    res["plateau_deg"] = aft[len(aft) // 2] if aft else None
    jumps = [(abs(win[k + 1][2] - win[k][2]), win[k + 1][1]) for k in range(len(win) - 1)]
    mj, mx = max(jumps)
    res["max_node_turn_deg"] = mj
    res["max_node_turn_x"] = mx
    res["max_node_turn_frac"] = mj / abs(angle_deg) if angle_deg else None
    if angle_deg:
        s = 1.0 if angle_deg > 0 else -1.0
        i10 = next((k for k, (_, x, d) in enumerate(win) if s * d >= 0.1 * abs(angle_deg)), None)
        i90 = next((k for k, (_, x, d) in enumerate(win) if s * d >= 0.9 * abs(angle_deg)), None)
        if i10 is not None and i90 is not None:
            res["x10"], res["x90"] = win[i10][1], win[i90][1]
            res["width_xc"] = abs(win[i90][1] - win[i10][1])
            res["nodes_10_90"] = i90 - i10
        else:
            res["note"] = "10%/90% levels not reached in window"
    near = [(x, abs(x - xh)) for x, _ in bs if abs(x - xh) <= 0.02]
    pan = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(bent, bent[1:])
           if abs((a[0] + b[0]) / 2 - xh) <= 0.02]
    res["panel_len_near_hinge_mean"] = sum(pan) / len(pan) if pan else None
    res["panel_len_near_hinge_max"] = max(pan) if pan else None
    res["nodes_within_0p05"] = sum(1 for x, _ in bent if abs(x - xh) <= 0.05)
    return res


# ======================================================================= dump parsing / separation
def parse_dump(text: str) -> list[dict]:
    rows = []
    for line in text.splitlines():
        p = line.split()
        if len(p) == 8 and not line.lstrip().startswith("#"):
            try:
                v = list(map(float, p))
            except ValueError:
                continue
            rows.append(dict(s=v[0], x=v[1], y=v[2], ue=v[3], dstar=v[4], theta=v[5], cf=v[6], h=v[7]))
    return rows


def split_dump(rows: list[dict], n_surface: int) -> tuple[list[dict], list[dict]]:
    """Surface rows only (wake dropped); split at the Ue sign change.  Each returned in flow order
    (stagnation -> trailing edge): (upper, lower)."""
    surf = rows[:n_surface]
    k = None
    for i in range(len(surf) - 1):
        if surf[i]["ue"] >= 0 > surf[i + 1]["ue"]:
            k = i
            break
    if k is None:
        raise ValueError("no stagnation point (Ue sign change) found in dump")
    upper = list(reversed(surf[:k + 1]))
    lower = surf[k + 1:]
    return upper, lower


def surface_summary(rows: list[dict], xtr: float | None, hinge_x: float | None) -> dict:
    """Separation analysis of one surface in flow order (rows[0] near the stagnation point).

    Separation = Cf crossing from >=0 to <0 (linear interpolation of x between nodes); reattachment
    = next crossing back to >=0.  Node 0 is skipped as a start (stagnation).  `laminar_sep` is True when
    the first separation lies ahead of the reported transition location.
    """
    out = {"n_nodes": len(rows), "xtr": xtr}
    if len(rows) < 3:
        out["note"] = "too few nodes"
        return out
    cf = [r["cf"] for r in rows]
    xs = [r["x"] for r in rows]
    crossings = []   # (kind, x)
    state = cf[0] < 0
    if state:
        crossings.append(("sep", xs[0]))
    for i in range(1, len(rows)):
        now = cf[i] < 0
        if now != state:
            f = cf[i - 1] / (cf[i - 1] - cf[i]) if cf[i - 1] != cf[i] else 0.0
            xc = xs[i - 1] + f * (xs[i] - xs[i - 1])
            crossings.append(("sep" if now else "re", xc))
            state = now
    seps = [x for k, x in crossings if k == "sep"]
    res = [x for k, x in crossings if k == "re"]
    out["n_separation_regions"] = len(seps)
    out["sep_x"] = seps[0] if seps else None
    out["reattach_x"] = None
    if seps:
        first_re = next((x for k, x in crossings[crossings.index(("sep", seps[0])) + 1:] if k == "re"), None)
        out["reattach_x"] = first_re
    out["sep_at_te"] = bool(cf[-1] < 0)
    out["min_cf"] = min(cf)
    hmax = max(rows, key=lambda r: r["h"])
    out["max_h"], out["max_h_x"] = hmax["h"], hmax["x"]
    out["laminar_sep"] = (None if not seps or xtr is None else bool(seps[0] < xtr))
    if hinge_x is not None:
        out["sep_minus_hinge"] = None if not seps else seps[0] - hinge_x
        out["reattach_minus_hinge"] = None if out["reattach_x"] is None else out["reattach_x"] - hinge_x
        out["max_h_minus_hinge"] = hmax["x"] - hinge_x
        out["xtr_minus_hinge"] = None if xtr is None else xtr - hinge_x
    return out


# ======================================================================= xfoil plumbing
HEAD_KEEP, TAIL_KEEP, RUNAWAY_BYTES = 3_000_000, 1_000_000, 30_000_000


def run_xfoil(folder: Path, script: str, timeout: int, tag: str = "") -> dict:
    """Run XFOIL in `folder`; saves commands{tag}.in and xfoil{tag}.log; skips if the log already exists.

    XFOIL 6.99 can fall into an endless NaN loop that prints hundreds of MB. Output is therefore streamed:
    the log keeps the first 3 MB and last 1 MB (marked if truncated) and the process is killed once
    30 MB have been produced ("runaway") or `timeout` seconds elapse.
    """
    import threading
    folder.mkdir(parents=True, exist_ok=True)
    cmd, logp = folder / f"commands{tag}.in", folder / f"xfoil{tag}.log"
    if logp.exists() and cmd.exists():
        return {"code": "cached", "elapsed": 0.0, "log": logp.read_text(errors="replace"), "runaway": False}
    cmd.write_text(script, encoding="ascii")
    t0 = time.time()
    proc = subprocess.Popen([str(EXE)], cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
    head, tail, total = bytearray(), bytearray(), [0]

    def feed():
        try:
            proc.stdin.write(script.encode("ascii"))
            proc.stdin.close()
        except OSError:
            pass

    def drain():
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                return
            total[0] += len(chunk)
            if len(head) < HEAD_KEEP:
                take = chunk[:HEAD_KEEP - len(head)]
                head.extend(take)
                chunk = chunk[len(take):]
            if chunk:
                tail.extend(chunk)
                if len(tail) > TAIL_KEEP:
                    del tail[:len(tail) - TAIL_KEEP]

    threads = [threading.Thread(target=feed, daemon=True), threading.Thread(target=drain, daemon=True)]
    for t in threads:
        t.start()
    code, runaway = None, False
    while proc.poll() is None:
        if time.time() - t0 > timeout:
            code = "timeout"
        elif total[0] > RUNAWAY_BYTES:
            code, runaway = "runaway_output", True
        if code:
            proc.kill()
            break
        time.sleep(0.05)
    proc.wait()
    for t in threads:
        t.join(timeout=5)
    if code is None:
        code = proc.returncode
    data = bytes(head)
    if total[0] > len(head) + len(tail):
        data += (f"\n\n[... LOG TRUNCATED by reflex_ext_gaps.py: {total[0]} bytes produced, "
                 f"first {len(head)} and last {len(tail)} kept ...]\n\n").encode()
    data += bytes(tail)
    log = data.decode(errors="replace")
    logp.write_text(log, encoding="utf-8", errors="replace")
    return {"code": code, "elapsed": time.time() - t0, "log": log, "runaway": runaway,
            "bytes_produced": total[0]}


def ppar_block(variant: str, hinge_x: float | None) -> str:
    v = VARIANTS[variant]
    lines = []
    if v["n"] not in (None, 160):
        lines.append(f"N {v['n']}")
    if v["refine"]:
        lo, hi = hinge_x - REFINE_HALF, hinge_x + REFINE_HALF
        lines += [f"XT {lo:.4f} {hi:.4f}", f"XB {lo:.4f} {hi:.4f}"]
    if not lines:
        return ""
    # PPAR needs a blank to leave the "change what" prompt and a second blank to leave the menu
    # (the first re-paneling after a change re-enters the menu once).
    return "PPAR\n" + "\n".join(lines) + "\n\n\n"


def header(variant: str, hinge_x: float | None, psav: str = "paneled.dat") -> str:
    s = "PLOP\nG\n\nLOAD foil.dat\n"
    if VARIANTS[variant]["mode"] == "pane":
        s += ppar_block(variant, hinge_x) + "PANE\n"
    return s + f"PSAV {psav}\nOPER\n"


def baseline_body(reynolds: int | None, iters: int = 150, polar: str = "polar.dat") -> str:
    """Identical in content to reflex_xfoil.run()'s command body."""
    return ((f"VISC {reynolds}\nVPAR\nN 5\n\nITER {iters}\n" if reynolds else "") +
            f"PACC\n{polar}\n\nALFA 0\n" + ("INIT\n" if reynolds else "") +
            "ALFA -3\nASEQ -3 8 0.5\nPACC\n\nQUIT\n")


def hinge_of(cf: float | None) -> float | None:
    return None if cf is None else 1 - cf


def count_log(log: str) -> dict:
    return rp.log_counts(log)


def polar_job(root: Path, variant: str, kind: str, cf: float, ang: float, reynolds: int | None) -> dict:
    """One polar (viscous if reynolds else inviscid) for `kind` in {bent, clean}; cf is the hinge window
    for clean sections of windowed variants (ignored for non-windowed ones)."""
    if kind == "clean":
        cf_eff = cf if variant in WINDOWED else 0.10
        ang = 0.0
    else:
        cf_eff = cf
    label = f"{kind}_c{int(round(cf_eff * 100)):02d}_a{ang:g}_{'inv' if not reynolds else 'Re' + str(reynolds)}".replace(".", "p")
    folder = root / variant / label
    folder.mkdir(parents=True, exist_ok=True)
    foil = folder / "foil.dat"
    if not foil.exists():
        pts = make_foil(variant, cf_eff, ang)
        foil.write_text(label + "\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in pts) + "\n", encoding="ascii")
    hx = hinge_of(cf_eff) if (VARIANTS[variant]["refine"]) else None
    script = header(variant, hx) + baseline_body(reynolds)
    r = run_xfoil(folder, script, TIMEOUT_POLAR)
    rows = rx.read_polar(folder / "polar.dat")
    return dict(variant=variant, kind=kind, cf=cf_eff, ang=ang, reynolds=reynolds, rows=rows,
                folder=str(folder), code=r["code"], elapsed=r["elapsed"], runaway=r.get("runaway", False),
                nan_count=r["log"].count("NaN"), iwx_warnings=r["log"].count("Array size"),
                **count_log(r["log"]))


def surface_nodes(folder) -> int | str:
    p = Path(folder) / "paneled.dat"
    return len(read_xy(p)) if p.exists() else ""


def wc_for(variant: str, cf: float) -> float:
    return cf if variant in WINDOWED else 0.10


# ======================================================================= gate
def stage_gate(out: Path) -> bool:
    gdir = out / "gate"
    gdir.mkdir(parents=True, exist_ok=True)
    rx.OUT = gdir / "reflex_xfoil_run"
    res = {}
    cases = {"bent_15_8_Re80k": (("NACA_0008", 80000, 0.15, 8.0), STORED_BENT),
             "clean_Re80k": (("NACA_0008", 80000, 0.10, 0.0), STORED_CLEAN)}
    ok = True
    for name, (args, stored_dir) in cases.items():
        new = rx.run(*args)
        stored = rx.read_polar(stored_dir / "polar.dat")
        res[name] = compare_polars(new["rows"], stored)
        res[name]["stored_dir"] = str(stored_dir)
        res[name]["regenerated_dir"] = new["folder"]
        # also confirm the script's own command builder (base160) gives the same polar
        mine = polar_job(gdir / "own_builder", "base160", "bent" if args[3] else "clean", args[2], args[3], 80000)
        res[name]["own_builder_vs_stored"] = compare_polars(mine["rows"], stored)
        res[name]["identical"] = bool(res[name]["max_abs_diff"] == 0 and res[name]["only_new"] == []
                                      and res[name]["only_stored"] == []
                                      and res[name]["own_builder_vs_stored"]["max_abs_diff"] == 0
                                      and res[name]["own_builder_vs_stored"]["only_new"] == []
                                      and res[name]["own_builder_vs_stored"]["only_stored"] == [])
        ok = ok and res[name]["identical"]
    res["gate_passed"] = ok
    (out / "gate_result.json").write_text(json.dumps(res, indent=1))
    print("GATE:", json.dumps({k: (v if k == "gate_passed" else {kk: v[kk] for kk in ("n_new", "n_stored", "max_abs_diff", "identical")})
                               for k, v in res.items()}, indent=1))
    return ok


def compare_polars(new: list[dict], stored: list[dict]) -> dict:
    n = {round(r["alpha_deg"], 3): r for r in new}
    s = {round(r["alpha_deg"], 3): r for r in stored}
    common = sorted(set(n) & set(s))
    per = {k: max((abs(n[a][k] - s[a][k]) for a in common), default=None)
           for k in ("cl", "cd", "cdp", "cm", "top_xtr", "bot_xtr")}
    vals = [v for v in per.values() if v is not None]
    return {"n_new": len(n), "n_stored": len(s), "n_common": len(common),
            "only_new": sorted(set(n) - set(s)), "only_stored": sorted(set(s) - set(n)),
            "max_abs_diff_by_quantity": per, "max_abs_diff": max(vals) if vals else None}


# ======================================================================= step 1
def step1_jobs(out: Path) -> dict:
    root = out / "step1"
    jobs = []
    for v in VARIANTS:
        for cf, ang in LIPS:
            for re_ in RES + [None]:
                jobs.append((root, v, "bent", cf, ang, re_))
        wins = [cf for cf, _ in LIPS] if v in WINDOWED else [0.10]
        for wc in wins:
            for re_ in RES + [None]:
                jobs.append((root, v, "clean", wc, 0.0, re_))
    jobs = list(dict.fromkeys(jobs))
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda j: polar_job(*j), jobs))
    return {(r["variant"], r["kind"], round(r["cf"], 2), r["reynolds"]): r for r in results}


def write_csv(path: Path, rows: list[dict]):
    rp.write_csv(path, rows)


def step1_tables(out: Path, res: dict):
    long_rows, count_rows, slope_rows = [], [], []
    for (v, kind, cf, re_), r in sorted(res.items(), key=lambda kv: str(kv[0])):
        got = {round(x["alpha_deg"], 1) for x in r["rows"]}
        count_rows.append(dict(variant=v, kind=kind, hinge_window_or_lip_cf=cf, lip_deg=r["ang"] if kind == "bent" else "",
                               viscous=bool(re_), Re=re_ or "", requested=23, returned=len(r["rows"]),
                               missing_alphas=" ".join(f"{a:g}" for a in rp.REQUESTED_ALPHAS if a not in got),
                               exit=r["code"], elapsed_s=round(r["elapsed"], 1),
                               viscal_fail=r["viscal_convergence_failures"], seq_halted=r["sequence_halted"],
                               runaway_killed=r["runaway"], nan_lines_in_kept_log=r["nan_count"],
                               iwx_wake_warnings=r["iwx_warnings"], n_nodes=surface_nodes(r["folder"]),
                               folder=r["folder"]))
    for v in VARIANTS:
        for cf, ang in LIPS:
            for re_ in RES:
                bent = res[(v, "bent", round(cf, 2), re_)]["rows"]
                clean = res[(v, "clean", round(wc_for(v, cf), 2), re_)]["rows"]
                ibent = res[(v, "bent", round(cf, 2), None)]["rows"]
                iclean = res[(v, "clean", round(wc_for(v, cf), 2), None)]["rows"]
                srow = dict(variant=v, lip=f"{int(round(cf * 100))}/{ang:g}", Re=re_)
                for nm, pol in (("clean", clean), ("bent", bent), ("inv_clean", iclean), ("inv_bent", ibent)):
                    p0, p1 = rx.at_cl(pol, SLOPE_CL[0]), rx.at_cl(pol, SLOPE_CL[1])
                    srow[f"{nm}_dCm_dCl"] = (p1["cm"] - p0["cm"]) / (SLOPE_CL[1] - SLOPE_CL[0]) if p0 and p1 else None
                slope_rows.append(srow)
                for t in TARGET_CL:
                    m = rp.matched(clean, bent, t)
                    ic, ib = rx.at_cl(iclean, t), rx.at_cl(ibent, t)
                    row = dict(variant=v, lip=f"{int(round(cf * 100))}/{ang:g}", Re=re_, Cl=t,
                               returned_bent=len(bent), returned_clean=len(clean),
                               inviscid_lip_dCm=(ib["cm"] - ic["cm"]) if ic and ib else None,
                               inviscid_clean_cm=ic["cm"] if ic else None)
                    if m:
                        row.update(clean_cm=m["clean_cm"], bent_cm=m["bent_cm"], lip_dCm=m["dcm"], dCd=m["dcd"],
                                   n_clean_brk=m["n_clean_brk"], n_bent_brk=m["n_bent_brk"],
                                   dCm_min=m["dcm_min"], dCm_max=m["dcm_max"],
                                   note="MULTI-BRACKET (first used)" if m["n_clean_brk"] > 1 or m["n_bent_brk"] > 1 else "")
                    else:
                        row["note"] = "MISSING: Cl not bracketed by adjacent returned points; not filled"
                    long_rows.append(row)
    # clean section rows
    clean_rows = []
    for v in VARIANTS:
        for wc in ([cf for cf, _ in LIPS] if v in WINDOWED else [0.10]):
            for re_ in RES:
                pol = res[(v, "clean", round(wc, 2), re_)]["rows"]
                p0, p1 = rx.at_cl(pol, SLOPE_CL[0]), rx.at_cl(pol, SLOPE_CL[1])
                for t in TARGET_CL:
                    p = rx.at_cl(pol, t)
                    clean_rows.append(dict(variant=v, hinge_window_cf=wc if v in WINDOWED else "none", Re=re_, Cl=t,
                                           returned=len(pol), clean_cm=p["cm"] if p else None, clean_cd=p["cd"] if p else None,
                                           clean_dCm_dCl_0p15_0p35=(p1["cm"] - p0["cm"]) / 0.2 if p0 and p1 else None))
    write_csv(out / "step1_counts.csv", count_rows)
    write_csv(out / "step1_values.csv", long_rows)
    write_csv(out / "step1_slopes.csv", slope_rows)
    write_csv(out / "step1_clean_section.csv", clean_rows)
    # comparison vs baseline
    base = {(r["lip"], r["Re"], r["Cl"]): r for r in long_rows if r["variant"] == "base160"}
    bslope = {(r["lip"], r["Re"]): r for r in slope_rows if r["variant"] == "base160"}
    cmp_rows = []
    def chg(new, old):
        if new is None or old is None:
            return None, None
        d = new - old
        return d, (100 * d / old if old != 0 else None)
    for r in long_rows:
        if r["variant"] == "base160":
            continue
        b = base[(r["lip"], r["Re"], r["Cl"])]
        row = dict(variant=r["variant"], lip=r["lip"], Re=r["Re"], Cl=r["Cl"],
                   base_lip_dCm=b.get("lip_dCm"), var_lip_dCm=r.get("lip_dCm"))
        row["d_lip_dCm"], row["pct_lip_dCm"] = chg(r.get("lip_dCm"), b.get("lip_dCm"))
        row["base_clean_cm"], row["var_clean_cm"] = b.get("clean_cm"), r.get("clean_cm")
        row["d_clean_cm"], _ = chg(r.get("clean_cm"), b.get("clean_cm"))
        row["d_dCd"], _ = chg(r.get("dCd"), b.get("dCd"))
        row["base_inv_lip_dCm"], row["var_inv_lip_dCm"] = b.get("inviscid_lip_dCm"), r.get("inviscid_lip_dCm")
        row["d_inv_lip_dCm"], row["pct_inv_lip_dCm"] = chg(r.get("inviscid_lip_dCm"), b.get("inviscid_lip_dCm"))
        sv, sb = next(s for s in slope_rows if s["variant"] == r["variant"] and s["lip"] == r["lip"] and s["Re"] == r["Re"]), bslope[(r["lip"], r["Re"])]
        row["d_bent_dCm_dCl"], _ = chg(sv["bent_dCm_dCl"], sb["bent_dCm_dCl"])
        row["d_clean_dCm_dCl"], _ = chg(sv["clean_dCm_dCl"], sb["clean_dCm_dCl"])
        row["returned_bent_var"], row["returned_bent_base"] = r["returned_bent"], b["returned_bent"]
        row["returned_clean_var"], row["returned_clean_base"] = r["returned_clean"], b["returned_clean"]
        row["note"] = "; ".join(x for x in (r.get("note"), b.get("note")) if x)
        cmp_rows.append(row)
    write_csv(out / "step1_comparison_vs_baseline.csv", cmp_rows)
    return long_rows, slope_rows, cmp_rows


def step1_sharpness(out: Path, res: dict):
    rows = []
    for v in VARIANTS:
        for cf, ang in LIPS:
            xh = 1 - cf
            bent_res = res[(v, "bent", round(cf, 2), None)]
            clean_res = res[(v, "clean", round(wc_for(v, cf), 2), None)]
            for src, bfile, cfile in (("input", "foil.dat", "foil.dat"), ("paneled", "paneled.dat", "paneled.dat")):
                if src == "input" and v not in ("base160", "asis_hinge", "asis_cos161"):
                    continue    # inputs are identical across 'cos' variants; list once per generator
                bpts = read_xy(Path(bent_res["folder"]) / bfile if src == "paneled" else Path(bent_res["folder"]) / "foil.dat")
                cpts = read_xy(Path(clean_res["folder"]) / cfile if src == "paneled" else Path(clean_res["folder"]) / "foil.dat")
                if src == "input":   # drop the label line effects: read_xy skips non-numeric lines
                    pass
                bu, bl = split_surfaces(bpts)
                cu, cl_ = split_surfaces(cpts)
                for surf, b, c in (("upper", bu, cu), ("lower", bl, cl_)):
                    m = bend_sharpness(b, c, ang, xh)
                    rows.append(dict(variant=v if src == "paneled" else v + "(input)", source=src,
                                     lip=f"{int(round(cf * 100))}/{ang:g}", surface=surf, n_nodes_total=len(bpts), **m))
    write_csv(out / "step1_bend_sharpness.csv", rows)
    return rows


def pct(v):
    return "  n/a " if v is None else f"{v:+6.1f}%"


def step1_report(out: Path, long_rows, slope_rows, cmp_rows, sharp_rows, counts_res):
    lines = []
    tot = {}
    for v in VARIANTS:
        n = sum(len(r["rows"]) for k, r in counts_res.items() if k[0] == v and k[3])
        req = 23 * sum(1 for k in counts_res if k[0] == v and k[3])
        tot[v] = (n, req)
        lines.append(f"{v:12s} viscous points returned {n} of {req}")
    lines.append("")
    lines.append("Lip moment dCm = bent Cm - clean Cm at matched Cl (clean from the SAME paneling variant)")
    for lip in ("10/3", "15/8", "20/10"):
        for re_ in RES:
            for t in TARGET_CL:
                items = []
                for v in VARIANTS:
                    r = next(x for x in long_rows if x["variant"] == v and x["lip"] == lip and x["Re"] == re_ and x["Cl"] == t)
                    items.append(f"{v}={r['lip_dCm']:+.5f}" if r.get("lip_dCm") is not None else f"{v}=MISSING")
                lines.append(f"{lip:>5} Re{re_ // 1000}k Cl{t:.2f}  " + "  ".join(items))
    lines.append("")
    lines.append("% change in lip dCm vs base160 (Re80k Cl0.25) and range over all other Cl/Re cells")
    for v in VARIANTS:
        if v == "base160":
            continue
        for lip in ("10/3", "15/8", "20/10"):
            rows = [r for r in cmp_rows if r["variant"] == v and r["lip"] == lip and r["pct_lip_dCm"] is not None]
            core = next((r for r in rows if r["Re"] == 80000 and r["Cl"] == 0.25), None)
            others = [r["pct_lip_dCm"] for r in rows if not (r["Re"] == 80000 and r["Cl"] == 0.25)]
            absd = [r["d_lip_dCm"] for r in rows if not (r["Re"] == 80000 and r["Cl"] == 0.25)]
            lines.append(f"{v:12s} {lip:>5} core {pct(core['pct_lip_dCm']) if core else 'MISSING'} "
                         f"(abs {core['d_lip_dCm']:+.5f})" if core else f"{v:12s} {lip:>5} core MISSING")
            if others:
                lines.append(f"{'':19s} others pct [{min(others):+.1f}, {max(others):+.1f}]  abs [{min(absd):+.5f}, {max(absd):+.5f}]  n={len(others)}")
    (out / "step1_summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return tot


def choose_best_variant(res: dict, sharp_rows: list[dict]) -> tuple[str | None, dict]:
    """Best non-baseline variant for the boundary-layer dumps: the narrowest measured bend transition
    (smallest mean 10%-90% x/c width of the excess tangent angle over lips and surfaces), among variants
    whose fraction of returned viscous points is at least the baseline's."""
    frac, width = {}, {}
    for v in VARIANTS:
        runs = [r for k, r in res.items() if k[0] == v and k[3]]
        frac[v] = sum(len(r["rows"]) for r in runs) / (23 * len(runs))
        w = [float(r["width_xc"]) for r in sharp_rows
             if r["variant"] == v and r["source"] == "paneled" and r.get("width_xc") not in (None, "")]
        width[v] = sum(w) / len(w) if w else None
    cands = [v for v in VARIANTS if v != "base160" and width[v] is not None and frac[v] >= frac["base160"] - 1e-9]
    best = min(cands, key=lambda v: width[v]) if cands else None
    return best, {"mean_width_10_90_xc": width, "viscous_fraction_returned": frac, "candidates": cands}


def stage_step1(out: Path):
    res = step1_jobs(out)
    long_rows, slope_rows, cmp_rows = step1_tables(out, res)
    sharp = step1_sharpness(out, res)
    step1_report(out, long_rows, slope_rows, cmp_rows, sharp, res)
    best, info = choose_best_variant(res, sharp)
    (out / "step1_best_variant.json").write_text(json.dumps({"best": best, **info}, indent=1))
    print("best refined variant:", best, json.dumps(info))
    # PPAR settings record
    rec = {}
    for v, s in VARIANTS.items():
        rec[v] = {"mode": s["mode"], "N": s["n"] if s["n"] else "input nodes unchanged (no PANE)",
                  "other_PPAR": "defaults (P 1.0, T 0.15, R 0.2)" if s["mode"] == "pane" else "none",
                  "refinement": ({str(cf): f"XT/XB {1 - cf - REFINE_HALF:.4f} {1 - cf + REFINE_HALF:.4f}" for cf, _ in LIPS}
                                 if s["refine"] else "none"),
                  "generator": ("reflex_xfoil.coordinates (n_half=160, 321 nodes)" if s["gen"] == "cos" else "reflex_xfoil.coordinates (n_half=80, 161 nodes)")
                  if s["gen"] in ("cos", "cos80")
                  else "coordinates_hinge (n_half=70 cosine + corner node at hinge, 0.0025 spacing within +-0.06 of hinge)"}
    (out / "step1_variant_settings.json").write_text(json.dumps(rec, indent=1))
    return res, best


# ======================================================================= step 2
STEP2_CASES = [("clean", None, None), ("10/3", 0.10, 3.0), ("15/8", 0.15, 8.0), ("20/10", 0.20, 10.0)]
DUMP_ALPHAS = [0, 2, 4, 6]


def dump_main_script(variant: str, hx, reynolds: int) -> str:
    s = header(variant, hx, "paneled_main.dat") + f"VISC {reynolds}\nVPAR\nN 5\n\nITER 150\nPACC\npolar_main.dat\n\nALFA 0\nINIT\nALFA -3\n"
    edges = [-3] + DUMP_ALPHAS
    for lo, hi in zip(edges, edges[1:]):
        s += f"ASEQ {lo:g} {hi:g} 0.5\nDUMP dump_a{hi:g}.dat\n"
    s += "ASEQ 6 8 0.5\nPACC\n\nQUIT\n"
    return s


def dump_astar_script(variant: str, hx, reynolds: int, astar: float) -> str:
    g = math.floor(astar / 0.5) * 0.5
    s = header(variant, hx, "paneled_astar.dat") + f"VISC {reynolds}\nVPAR\nN 5\n\nITER 150\nPACC\npolar_astar.dat\n\nALFA 0\nINIT\nALFA -3\n"
    if g > -3:
        s += f"ASEQ -3 {g:g} 0.5\n"
    s += f"ALFA {astar:.3f}\nDUMP dump_astar.dat\nPACC\n\nQUIT\n"
    return s


def surface_count(folder: Path, name: str) -> int:
    return len(read_xy(folder / name))


def step2_case(out: Path, step1_res: dict, variant: str, sec: str, cf, ang, re_: int) -> list[dict]:
    root = out / "step2" / variant / f"{sec.replace('/', '_')}_Re{re_}"
    root.mkdir(parents=True, exist_ok=True)
    cf_eff = cf if cf is not None else 0.15       # clean section: windowed variants use the 15% window
    kind = "bent" if cf is not None else "clean"
    foil = root / "foil.dat"
    pts = make_foil(variant, cf_eff, ang or 0.0)
    foil.write_text(f"{variant}_{sec}\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in pts) + "\n", encoding="ascii")
    hx = hinge_of(cf_eff) if VARIANTS[variant]["refine"] else None
    hinge_x = hinge_of(cf)    # None for clean
    # alpha for Cl 0.25 from the same-variant step-1 polar
    s1 = step1_res.get((variant, kind, round(cf_eff if kind == "bent" else wc_for(variant, cf_eff), 2), re_))
    a_star = None
    cl_target = 0.25
    if s1:
        p = rx.at_cl(s1["rows"], cl_target)
        a_star = None if p is None else round(p["alpha_deg"], 3)
    main = run_xfoil(root, dump_main_script(variant, hx, re_), TIMEOUT_POLAR, "_main")
    polar_main = rx.read_polar(root / "polar_main.dat")
    cases = [(f"a{a:g}", float(a), "dump_a%g.dat" % a, polar_main, "main") for a in DUMP_ALPHAS]
    if a_star is not None:
        run_xfoil(root, dump_astar_script(variant, hx, re_, a_star), TIMEOUT_POLAR, "_astar")
        polar_a = rx.read_polar(root / "polar_astar.dat")
        cases.append(("astar_Cl0.25", a_star, "dump_astar.dat", polar_a, "astar"))
    n_surf = surface_count(root, "paneled_main.dat")
    out_rows = []
    for tag, alpha, dfile, polar, run_tag in cases:
        byA = {round(r["alpha_deg"], 3): r for r in polar}
        prow = byA.get(round(alpha, 3))
        row = dict(variant=variant, section=sec, Re=re_, case=tag, alpha_deg=alpha, hinge_x=hinge_x,
                   converged=prow is not None, polar_run=run_tag, dump_file=str(root / dfile),
                   CL=prow and prow["cl"], CM=prow and prow["cm"], CD=prow and prow["cd"])
        dpath = root / dfile
        if prow is None or not dpath.exists():
            row["note"] = "MISSING: point did not converge (or no dump); not summarised"
            out_rows.append(row)
            continue
        try:
            rows = parse_dump(dpath.read_text(errors="replace"))
            upper, lower = split_dump(rows, n_surf)
        except Exception as err:  # noqa: BLE001 - recorded, not hidden
            row["note"] = f"dump parse error: {err}"
            out_rows.append(row)
            continue
        for nm, surf, xtr in (("top", upper, prow["top_xtr"]), ("bot", lower, prow["bot_xtr"])):
            sm = surface_summary(surf, xtr, hinge_x)
            row.update({f"{nm}_{k}": v for k, v in sm.items()})
        out_rows.append(row)
    # consistency of the dump run with the step-1 polar
    if s1:
        cmpd = compare_polars(polar_main, s1["rows"])
        for r in out_rows:
            r["dumprun_vs_step1_polar_max_abs_diff"] = cmpd["max_abs_diff"]
            r["dumprun_n_points"] = len(polar_main)
            r["step1_n_points"] = len(s1["rows"])
    return out_rows


def stage_step2(out: Path, step1_res: dict, best: str | None):
    variants = ["base160"] + ([best] if best else [])
    jobs = []
    for v in variants:
        for sec, cf, ang in STEP2_CASES:
            jobs.append((v, sec, cf, ang, 80000))
        jobs += [(v, "15/8", 0.15, 8.0, 60000), (v, "15/8", 0.15, 8.0, 100000)]
    with ThreadPoolExecutor(max_workers=4) as pool:
        lists = list(pool.map(lambda j: step2_case(out, step1_res, *j), jobs))
    rows = [r for lst in lists for r in lst]
    write_csv(out / "step2_separation_summary.csv", rows)
    print(f"step2: {len(jobs)} sessions, {len(rows)} dump cases, converged {sum(1 for r in rows if r['converged'])}")
    return rows


# ======================================================================= step 3
RETRY_CF, RETRY_ANG, RETRY_RE = 0.20, 10.0, 100000


def retry_header(iters: int, reynolds: int, polar: str) -> str:
    return (header("base160", None, "paneled.dat") + f"VISC {reynolds}\nVPAR\nN 5\n\nITER {iters}\nPACC\n{polar}\n\n")


def alpha_list(lo: float, hi: float, step: float) -> list[float]:
    step = abs(step)
    n = int(round(abs(hi - lo) / step))
    sgn = 1 if hi >= lo else -1
    return [round(lo + sgn * step * i, 4) for i in range(n + 1)]


def retry_strategies() -> dict:
    S = {}
    base = "ALFA 0\nINIT\nALFA -3\nASEQ -3 8 0.5\nPACC\n\nQUIT\n"
    S["S0_baseline_rerun_iter150"] = retry_header(150, RETRY_RE, "polar.dat") + base
    S["S1a_iter200_step0p5_from_m3"] = retry_header(200, RETRY_RE, "polar.dat") + base
    S["S1b_iter400_step0p5_from_m3"] = retry_header(400, RETRY_RE, "polar.dat") + base
    S["S1c_iter400_step0p25_from_m3"] = retry_header(400, RETRY_RE, "polar.dat") + \
        "ALFA 0\nINIT\nALFA -3\nASEQ -3 8 0.25\nPACC\n\nQUIT\n"
    S["S2_up_from0_aseq0p25_iter300"] = retry_header(300, RETRY_RE, "polar.dat") + \
        "ALFA 0\nINIT\nASEQ 0 8 0.25\nPACC\n\nQUIT\n"
    S["S3_up_from0_alfa0p25_iter300"] = retry_header(300, RETRY_RE, "polar.dat") + "ALFA 0\nINIT\nALFA 0\n" + "".join(
        f"ALFA {a:g}\n" for a in alpha_list(0.25, 8, 0.25)) + "PACC\n\nQUIT\n"
    S["S4_down_from8_aseq0p25_iter300"] = retry_header(300, RETRY_RE, "polar.dat") + \
        "ALFA 8\nINIT\nASEQ 8 -3 -0.25\nPACC\n\nQUIT\n"
    S["S5_down_from8_alfa0p25_iter300"] = retry_header(300, RETRY_RE, "polar.dat") + "ALFA 8\nINIT\nALFA 8\n" + "".join(
        f"ALFA {a:g}\n" for a in alpha_list(7.75, -3, -0.25)) + "PACC\n\nQUIT\n"
    S["S6_up_from0_aseq0p25_iter400"] = retry_header(400, RETRY_RE, "polar.dat") + \
        "ALFA 0\nINIT\nASEQ 0 8 0.25\nPACC\n\nQUIT\n"
    return S


def re_ladder_script(start_re: int, levels: list[int], iters: int = 300) -> str:
    """Converge a 0..8 sweep at start_re, then step Re through `levels`, alternating sweep direction so
    each Re level starts from the previous level's converged neighbouring solution (no INIT between)."""
    s = header("base160", None, "paneled.dat") + f"VISC {start_re}\nVPAR\nN 5\n\nITER {iters}\n"
    # bare "PACC" closes accumulation; a blank line would leave the OPER menu
    s += f"PACC\npolar_Re{start_re}.dat\n\nALFA 0\nINIT\nASEQ 0 8 0.5\nPACC\n"
    up = False
    for re_ in levels:
        s += f"Re {re_}\nPACC\npolar_Re{re_}.dat\n\n"
        s += ("ASEQ 0 8 0.5\n" if up else "ASEQ 8 0 -0.5\n")
        s += "PACC\n"
        up = not up
    return s + "\nQUIT\n"


def stage_step3(out: Path):
    root = out / "step3"
    root.mkdir(parents=True, exist_ok=True)
    foil_pts = rx.coordinates(THICK, RETRY_CF, RETRY_ANG)
    results = {}

    def prep(name: str) -> Path:
        d = root / name
        d.mkdir(parents=True, exist_ok=True)
        f = d / "foil.dat"
        if not f.exists():
            f.write_text("NACA_0008_c20_a10_retry\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in foil_pts) + "\n", encoding="ascii")
        return d

    strategies = retry_strategies()

    def run_s(item):
        name, script = item
        d = prep(name)
        r = run_xfoil(d, script, TIMEOUT_LONG)
        rows = rx.read_polar(d / "polar.dat")
        return name, dict(rows=rows, code=r["code"], elapsed=r["elapsed"], **count_log(r["log"]))

    ladders = {"S7_Re_ladder_up_from_80k": (80000, [85000, 90000, 95000, 100000]),
               "S8_Re_ladder_down_from_150k": (150000, [140000, 130000, 120000, 110000, 100000])}

    def run_l(item):
        name, (start, levels) = item
        d = prep(name)
        r = run_xfoil(d, re_ladder_script(start, levels), TIMEOUT_LONG)
        polars = {lv: rx.read_polar(d / f"polar_Re{lv}.dat") for lv in [start] + levels}
        return name, dict(polars=polars, code=r["code"], elapsed=r["elapsed"], **count_log(r["log"]))

    with ThreadPoolExecutor(max_workers=4) as pool:
        for name, res in pool.map(run_s, list(strategies.items())):
            results[name] = res
        for name, res in pool.map(run_l, list(ladders.items())):
            results[name] = res
    # per-strategy counts
    count_rows, point_rows = [], []
    final = {}
    for name, res in results.items():
        if "rows" in res:
            rows = res["rows"]
            final[name] = rows
            count_rows.append(dict(strategy=name, Re=RETRY_RE, returned=len(rows),
                                   alpha_min=min((r["alpha_deg"] for r in rows), default=None),
                                   alpha_max=max((r["alpha_deg"] for r in rows), default=None),
                                   exit=res["code"], elapsed_s=round(res["elapsed"], 1),
                                   viscal_fail=res["viscal_convergence_failures"], seq_halted=res["sequence_halted"]))
        else:
            for lv, rows in res["polars"].items():
                count_rows.append(dict(strategy=name, Re=lv, returned=len(rows),
                                       alpha_min=min((r["alpha_deg"] for r in rows), default=None),
                                       alpha_max=max((r["alpha_deg"] for r in rows), default=None),
                                       exit=res["code"], elapsed_s=round(res["elapsed"], 1),
                                       viscal_fail=res["viscal_convergence_failures"], seq_halted=res["sequence_halted"]))
                if lv == RETRY_RE:
                    final[name] = rows
    for name, rows in final.items():
        for r in rows:
            point_rows.append(dict(strategy=name, alpha_deg=r["alpha_deg"], CL=r["cl"], CD=r["cd"], CM=r["cm"],
                                   top_xtr=r["top_xtr"], bot_xtr=r["bot_xtr"]))
    write_csv(out / "step3_counts.csv", count_rows)
    write_csv(out / "step3_points.csv", point_rows)
    # agreement across strategies at common alphas (grid keys to 3 decimals)
    by_alpha: dict[float, dict] = {}
    for name, rows in final.items():
        for r in rows:
            by_alpha.setdefault(round(r["alpha_deg"], 3), {})[name] = r
    agree = []
    for a in sorted(by_alpha):
        d = by_alpha[a]
        if len(d) < 2:
            agree.append(dict(alpha_deg=a, n_strategies=len(d), strategies=" ".join(d), note="single strategy"))
            continue
        cls = {k: v["cl"] for k, v in d.items()}
        cms = {k: v["cm"] for k, v in d.items()}
        cds = {k: v["cd"] for k, v in d.items()}
        agree.append(dict(alpha_deg=a, n_strategies=len(d), strategies=" ".join(d),
                          CL_min=min(cls.values()), CL_max=max(cls.values()), CL_spread=max(cls.values()) - min(cls.values()),
                          CM_spread=max(cms.values()) - min(cms.values()), CD_spread=max(cds.values()) - min(cds.values()),
                          disagree=bool(max(cls.values()) - min(cls.values()) > 0.01 or max(cms.values()) - min(cms.values()) > 0.003),
                          note=""))
    write_csv(out / "step3_agreement_by_alpha.csv", agree)
    union = sorted(by_alpha)
    summary = {"strategies": {r["strategy"] + (f"@Re{r['Re']}" if r["Re"] != RETRY_RE else ""): r["returned"] for r in count_rows},
               "alphas_with_any_value": len(union),
               "alphas_with_2plus": sum(1 for r in agree if r["n_strategies"] >= 2),
               "alphas_disagree": [r["alpha_deg"] for r in agree if r.get("disagree")],
               "max_CL_spread": max((r.get("CL_spread") or 0 for r in agree), default=0)}
    (out / "step3_summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))
    return summary


# ======================================================================= main
def main():
    global OUT
    ap = argparse.ArgumentParser()
    ap.add_argument("stages", nargs="*", default=["gate", "step1", "step2", "step3"])
    ap.add_argument("--out", default=None, help="override output folder (development only)")
    args = ap.parse_args()
    out = Path(args.out) if args.out else OUT
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    res = best = None
    if "gate" in args.stages:
        if not stage_gate(out):
            print("REPRO GATE FAILED: stopping.")
            sys.exit(2)
    if "step1" in args.stages or "step2" in args.stages:
        res, best = stage_step1(out)
    if "step2" in args.stages:
        stage_step2(out, res, best)
    if "step3" in args.stages:
        stage_step3(out)
    print(f"elapsed {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
