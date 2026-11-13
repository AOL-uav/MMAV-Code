"""XFOIL polars for the flat-plate sections (4.3 %, 3.0 %, and the 4.3 % plate with 15/8 and 20/10 lips).

Writes ONLY under results/runs/rigid_glider/plate/polars/ (and plate/section_summary.csv).
The grid is the reflex_small_polars grid imported unchanged: Re 15k..100k (7 values), Ncrit 5 and 9, 160 panels,
alpha -4..10 step 0.5 (sweep 0 -> 10 then 0 -> -4, INIT at 0), ITER 400, timeout 300 s, the same XFOIL script
(build_script), the same sweep merge, runner (reflex_ext_gaps.run_xfoil) and lip-moment definition
(lip_moment_cells).  Only the foil coordinates differ (plate_section).  240 panels are NOT run (brief: 160).
Missing / unconverged points stay missing.  Positive cm nose-up, about the quarter chord.

Run from the repo root:  py -B rigid_glider/plate_polars.py [run] [tables]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import reflex_small_polars as sp
import reflex_xfoil as rx
import reflex_ext_gaps as g
import pure_wing_polars as pwp
import pure_wing_model as m
import arch_model as A
import plate_section as ps

ROOT = g.ROOT
PLATE_OUT = ROOT / "results/runs/rigid_glider/plate"
OUT = PLATE_OUT / "polars"
RES, NCRITS, GRID = sp.RES, sp.NCRITS, sp.GRID
PANELS = 160
BASE = "PLATE_43"
CLEAN_SECTIONS = ["PLATE_43", "PLATE_30"]
LIPS = [(0.15, 8.0), (0.20, 10.0)]            # on the 4.3 % plate only
FIELDS = sp.FIELDS
DB = OUT / "plate_polar_database.csv"
CL_TARGETS = (0.1, 0.2, 0.3, 0.4)


# ------------------------------------------------------------------ jobs and running
def all_jobs() -> list[tuple]:
    """(section, cf, ang, re, ncrit, panels); ang 0 = clean; re None = inviscid."""
    jobs = []
    cases = [(s, 0.0, 0.0) for s in CLEAN_SECTIONS] + [(BASE, cf, ang) for cf, ang in LIPS]
    for s, cf, ang in cases:
        jobs.append((s, cf, ang, None, 0, PANELS))
        for nc in NCRITS:
            for re in RES:
                jobs.append((s, cf, ang, re, nc, PANELS))
    return jobs


def write_foil(folder: Path, section: str, cf: float, ang: float, label: str):
    pts = ps.section_coordinates(section, cf if ang else 0.10, ang)
    (folder / "foil.dat").write_text(label + "\n" + "\n".join(f"{x:.8f} {y:.8f}" for x, y in pts) + "\n",
                                     encoding="ascii")


def run_case(root: Path, section: str, cf: float, ang: float, re, ncrit: int, panels: int) -> dict:
    tag = "clean" if ang == 0 else f"c{int(round(cf * 100)):02d}_a{ang:g}"
    cond = "inv" if re is None else f"Re{re}_N{ncrit}"
    folder = root / f"{section}_{tag}_{cond}_p{panels}".replace(".", "p")
    folder.mkdir(parents=True, exist_ok=True)
    if not (folder / "foil.dat").exists():
        write_foil(folder, section, cf, ang, folder.name)
    r = g.run_xfoil(folder, sp.build_script(re, ncrit, panels), sp.TIMEOUT)
    if re is None:
        rows = pwp.parse_polar(folder / "polar_inv.dat", sp.AMIN, sp.AMAX)
    else:
        rows = sp.merge_sweeps([pwp.parse_polar(folder / f"polar_p1_{k}.dat", sp.AMIN, sp.AMAX) for k in range(2)])
    return dict(section=section, cf=cf if ang else 0.0, ang=ang, re=re, ncrit=ncrit, panels=panels, rows=rows,
                code=r["code"], runaway=r.get("runaway", False), elapsed=r["elapsed"], folder=str(folder),
                iwx=r["log"].count("Array size"), nan=r["log"].count("NaN"))


def stage_run(out: Path = OUT) -> list[dict]:
    jobs = all_jobs()
    with ThreadPoolExecutor(max_workers=4) as pool:
        return list(pool.map(lambda j: run_case(out / "raw", *j), jobs))


# ------------------------------------------------------------------ database, coverage
def write_database(res: list[dict], out: Path = OUT):
    with (out / "plate_polar_database.csv").open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=FIELDS)
        w.writeheader()
        for r in sorted(res, key=lambda r: (r["section"], r["re"] or 0, r["ncrit"], r["cf"], r["ang"])):
            for x in r["rows"]:
                w.writerow(dict(section=r["section"], ncrit=r["ncrit"] if r["re"] else 0, re=r["re"] or 0,
                                alpha=x["alpha"], cl=x["cl"], cd=x["cd"], cdp=x["cdp"], cm=x["cm"],
                                xtr_top=x["xtr_top"], xtr_bot=x["xtr_bot"], **{"pass": 1},
                                lip_chord_frac=r["cf"], lip_angle_deg=r["ang"], panels=r["panels"]))
    cov = []
    for r in res:
        cov.append(dict(section=r["section"], lip=sp.lip_label(r["cf"], r["ang"]) if r["ang"] else "clean",
                        re=r["re"] or "inviscid", ncrit=r["ncrit"] if r["re"] else "", panels=r["panels"],
                        **sp.coverage_row(r["rows"]), exit=r["code"], runaway_killed=r["runaway"],
                        iwx_warnings=r["iwx"], nan_lines=r["nan"], elapsed_s=round(r["elapsed"], 1), folder=r["folder"]))
    g.write_csv(out / "coverage.csv", cov)
    return cov


# ------------------------------------------------------------------ table access in the arch_model interface
class PlateTables:
    """Same interface ArchTables gives the coupled model (clean / bent / inv_curve), built from plate_polar_database.csv."""

    def __init__(self, db: Path = DB):
        rows = A.read_lip_db(db)
        grp = {}
        for r in rows:
            grp.setdefault((r["section"], r["cf"], r["ang"], r["panels"], r["ncrit"]), {}).setdefault(r["re"], []).append(r)
        self.grp = grp
        self.inv = {}
        for (sec, cf, ang, pan, nc), byre in grp.items():
            if nc == 0:
                self.inv[(sec, cf, ang, pan)] = sorted(byre[0], key=lambda x: x["alpha"])
        self._clean, self._bent = {}, {}

    def _byre(self, key):
        # a polar with no converged point has no rows in the CSV -> empty table (every lookup is then UNCOVERED)
        return {re: sorted(rs, key=lambda x: x["alpha"]) for re, rs in self.grp.get(key, {}).items()}

    def clean(self, section, ncrit, panels):
        k = (section, ncrit, panels)
        if k not in self._clean:
            self._clean[k] = m.SectionTable(self._byre((section, 0.0, 0.0, panels, ncrit)), self.inv[(section, 0.0, 0.0, panels)])
        return self._clean[k]

    def bent(self, section, cf, ang, ncrit, panels):
        k = (section, cf, ang, ncrit, panels)
        if k not in self._bent:
            self._bent[k] = A.BentSectionTable(self._byre((section, cf, ang, panels, ncrit)), self.inv[(section, cf, ang, panels)])
        return self._bent[k]

    def inv_curve(self, section, cf, ang, panels):
        return A.InvCurve(self.inv[(section, cf, ang, panels)])


def load_polars(db: Path = DB) -> dict:
    """-> {(section, cf, ang, re, ncrit): rows sorted by alpha}; re 0 / ncrit 0 = inviscid."""
    res = {}
    for r in A.read_lip_db(db):
        res.setdefault((r["section"], r["cf"], r["ang"], r["re"], r["ncrit"]), []).append(r)
    return {k: sorted(v, key=lambda x: x["alpha"]) for k, v in res.items()}


# ------------------------------------------------------------------ section summary
def cm_at(rows: list[dict], target: float):
    p = rx.at_cl(sp.to_cl_rows(rows), target)
    return None if p is None else p["cm"]


def section_summary_rows(pol: dict) -> list[dict]:
    out = []
    cases = [(s, 0.0, 0.0) for s in CLEAN_SECTIONS] + [(BASE, cf, ang) for cf, ang in LIPS]
    for s, cf, ang in cases:
        inv_clean = pol[(BASE if ang else s, 0.0, 0.0, 0, 0)]
        inv_bent = pol[(s, cf, ang, 0, 0)] if ang else None
        for nc in NCRITS:
            for re in RES:
                rows = pol.get((s, cf, ang, re, nc), [])
                got = {round(x["alpha"], 4) for x in rows}
                d = dict(section=s, lip=sp.lip_label(cf, ang) if ang else "clean", re=re, ncrit=nc, n_converged=len(got & set(GRID)),
                         n_requested=len(GRID), alpha_min=min(got) if got else "", alpha_max=max(got) if got else "",
                         missing_alphas=" ".join(f"{a:g}" for a in GRID if a not in got))
                pos = [x for x in rows if x["cl"] >= 0]
                d["cl_max"] = max((x["cl"] for x in rows), default="")
                d["alpha_at_cl_max"] = max(rows, key=lambda x: x["cl"])["alpha"] if rows else ""
                for t in CL_TARGETS:
                    v = cm_at(rows, t)
                    d[f"cm_at_cl_{t:g}"] = "" if v is None else v
                if pos:
                    pk = max(pos, key=lambda x: x["cm"])
                    d["peak_nose_up_cm"], d["cl_at_peak_cm"], d["alpha_at_peak_cm"] = pk["cm"], pk["cl"], pk["alpha"]
                else:
                    d["peak_nose_up_cm"] = d["cl_at_peak_cm"] = d["alpha_at_peak_cm"] = ""
                if ang:
                    clean = pol.get((BASE, 0.0, 0.0, re, nc), [])
                    cell = next(c for c in sp.lip_moment_cells(clean, rows, inv_clean, inv_bent) if abs(c["cl"] - 0.25) < 1e-9)
                    d["lip_dcm_cl0.25"] = cell.get("lip_dcm", "")
                    d["inviscid_lip_dcm_cl0.25"] = cell.get("inviscid_lip_dcm", "")
                    f = cell.get("fraction_of_inviscid")
                    d["retained_fraction_cl0.25"] = "" if f is None else f
                    d["note"] = cell.get("note", "")
                else:
                    d["lip_dcm_cl0.25"] = d["inviscid_lip_dcm_cl0.25"] = d["retained_fraction_cl0.25"] = ""
                    d["note"] = ""
                out.append(d)
    return out


def stage_tables(res: list[dict] | None = None, out: Path = OUT):
    out.mkdir(parents=True, exist_ok=True)
    if res is not None:
        write_database(res, out)
    g.write_csv(PLATE_OUT / "section_summary.csv", section_summary_rows(load_polars(out / "plate_polar_database.csv")))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    st = sys.argv[1:] or ["run", "tables"]
    t0 = time.time()
    res = None
    if "run" in st:
        res = stage_run()
        write_database(res)
    if "tables" in st:
        stage_tables()
    print(f"elapsed {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
