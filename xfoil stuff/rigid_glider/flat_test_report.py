"""Results page for the flat-wing flight test.  Reads a filled log CSV (columns as in flat_test/log_template.csv) and writes
results/runs/rigid_glider/flat_test/results/index.html: per wing / thickness / config / launch method a table (hand launches and drone drops are never pooled) and an inline SVG strip chart of outcomes
against CG setting, the CG settings that meet the PROPOSED pass rule, and the raw rows.  No model comparison.

    py -B rigid_glider/flat_test_report.py my_log.csv [--out results/runs/rigid_glider/flat_test/results/index.html]

Rows whose wing starts with EXAMPLE (and empty rows) are ignored.  A row whose outcome is not in the fixed vocabulary is NOT counted and is listed as an error.
CG settings are 1 mm bins: measured_cg_mm is rounded to the nearest whole mm (so 40.4 and 39.6 are both setting 40).
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import statistics
from pathlib import Path

OUTCOMES = ["steady glide", "dive", "pitch-up or stall", "pitch oscillation", "tumble", "roll-off left", "roll-off right", "yaw wander"]
# pitch oscillation = repeated swooping or bounding, nose up and down; tumble = flips end over end.  The old bare word "oscillation" is NOT accepted.
COLORS = ["#4cc38a", "#5aa5ff", "#ff8a5c", "#e0b83a", "#b88cff", "#aab4bf", "#ff6fa8", "#4dd0e1"]      # one per outcome, readable on the dark page
COLUMNS = ["date", "wing", "thickness_mm", "config", "measured_mass_g", "measured_cg_mm", "launch_method", "release_no", "outcome",
           "release_height_m", "release_speed_mps", "distance_m", "time_s", "notes"]
PASS_STEADY = 8              # PROPOSED: >= 8 steady ...
PASS_RELEASES = 10           # ... of 10 releases at one setting
NEIGHBOUR_GLIDES_FRAC = 0.5  # PROPOSED definition of "a neighbour also glides": at least half of its releases are steady glides
OUT_DEFAULT = Path(__file__).resolve().parents[1] / "results/runs/rigid_glider/flat_test/results/index.html"


def _num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def read_log(path: Path):
    """-> (valid_rows, errors, n_ignored).  Each valid row is a dict with cg (float) and setting (int)."""
    valid, errors, ignored = [], [], 0
    with Path(path).open(newline="", encoding="utf-8-sig") as f:
        rd = csv.DictReader(f)
        missing = [c for c in COLUMNS if c not in (rd.fieldnames or [])]
        if missing:
            raise ValueError(f"log is missing columns: {missing}")
        for n, r in enumerate(rd, start=2):
            wing = (r.get("wing") or "").strip()
            if not wing and not any((v or "").strip() for v in r.values()):
                ignored += 1
                continue
            if wing.upper().startswith("EXAMPLE"):
                ignored += 1
                continue
            out = (r.get("outcome") or "").strip()
            cg = _num(r.get("measured_cg_mm"))
            problems = []
            if out.lower() == "oscillation":
                problems.append("outcome 'oscillation' is no longer used: write 'pitch oscillation' (repeated swooping or bounding, nose up and down) or 'yaw wander' (nose swings side to side); not counted until you choose")
            elif out not in OUTCOMES:
                problems.append(f"outcome '{out}' is not one of: {', '.join(OUTCOMES)}")
            if cg is None:
                problems.append("measured_cg_mm is missing or not a number")
            if not wing:
                problems.append("wing is empty")
            if problems:
                errors.append(dict(line=n, problems="; ".join(problems), row=r))
                continue
            row = dict(r)
            row.update(wing=wing, outcome=out, cg=cg, setting=int(round(cg)), config=(r.get("config") or "").strip(), launch=(r.get("launch_method") or "").strip().lower() or "(not given)",
                       thickness=(r.get("thickness_mm") or "").strip(), line=n)
            valid.append(row)
    return valid, errors, ignored


def analyse(valid):
    """Group by (wing, thickness, config); per CG setting counts, steady fraction, medians; settings meeting the PROPOSED rule."""
    groups = {}
    for r in valid:
        groups.setdefault((r["wing"], r["thickness"], r["config"], r["launch"]), []).append(r)
    out = []
    for key, rows in sorted(groups.items()):
        by = {}
        for r in rows:
            by.setdefault(r["setting"], []).append(r)
        settings = {}
        for s, rs in sorted(by.items()):
            counts = {o: sum(1 for r in rs if r["outcome"] == o) for o in OUTCOMES}
            n = len(rs)
            steady = [r for r in rs if r["outcome"] == "steady glide"]
            med = lambda k: (statistics.median([_num(r[k]) for r in steady if _num(r[k]) is not None])
                             if any(_num(r[k]) is not None for r in steady) else None)
            settings[s] = dict(n=n, counts=counts, steady=counts["steady glide"], frac=counts["steady glide"] / n,
                               dist=med("distance_m"), time=med("time_s"), height=med("release_height_m"))
        passing = []
        for s, d in settings.items():
            if d["n"] >= PASS_RELEASES and d["steady"] >= PASS_STEADY:
                nb = [settings.get(s - 1), settings.get(s + 1)]
                if all(x is not None and x["frac"] >= NEIGHBOUR_GLIDES_FRAC for x in nb):
                    passing.append(s)
        out.append(dict(wing=key[0], thickness=key[1], config=key[2], launch=key[3], n=len(rows), settings=settings, passing=sorted(passing)))
    return out


def strip_chart(settings):
    """Inline SVG: one stacked bar per CG setting (counts of each outcome); a label with steady/total above each bar."""
    lo, hi = min(settings), max(settings)
    xs = list(range(lo, hi + 1))
    nmax = max(d["n"] for d in settings.values())
    bw, gap, left, top, hgt = 34, 8, 54, 30, 150
    W = left + len(xs) * (bw + gap) + 20
    H = top + hgt + 120
    sc = hgt / max(nmax, 1)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{max(W, 700)}" height="{H}" viewBox="0 0 {max(W, 700)} {H}" font-family="Segoe UI,Arial" font-size="11" fill="#dde3ea">',
         '<rect width="100%" height="100%" fill="#1c2229"/>',
         f'<line x1="{left - 4}" y1="{top + hgt}" x2="{W - 10}" y2="{top + hgt}" stroke="#9aa7b4"/>',
         f'<line x1="{left - 4}" y1="{top}" x2="{left - 4}" y2="{top + hgt}" stroke="#9aa7b4"/>',
         f'<text x="14" y="{top + hgt / 2}" transform="rotate(-90 14 {top + hgt / 2})" text-anchor="middle">releases</text>']
    for k in range(0, nmax + 1, max(1, nmax // 5)):
        y = top + hgt - k * sc
        o.append(f'<text x="{left - 8}" y="{y + 4:.1f}" text-anchor="end">{k}</text><line x1="{left - 4}" y1="{y:.1f}" x2="{W - 10}" y2="{y:.1f}" stroke="#333c46"/>')
    for i, s in enumerate(xs):
        x = left + i * (bw + gap)
        o.append(f'<text x="{x + bw / 2}" y="{top + hgt + 16}" text-anchor="middle">{s}</text>')
        d = settings.get(s)
        if not d:
            continue
        y = top + hgt
        for oc, col in zip(OUTCOMES, COLORS):
            c = d["counts"][oc]
            if c:
                o.append(f'<rect x="{x}" y="{y - c * sc:.1f}" width="{bw}" height="{c * sc:.1f}" fill="{col}"><title>{oc}: {c}</title></rect>')
                y -= c * sc
        o.append(f'<text x="{x + bw / 2}" y="{y - 4:.1f}" text-anchor="middle">{d["steady"]}/{d["n"]}</text>')
    o.append(f'<text x="{left + (W - left) / 2}" y="{top + hgt + 34}" text-anchor="middle">CG setting, mm aft of root leading edge (rounded to 1 mm); label = steady glides / releases</text>')
    for i, (oc, col) in enumerate(zip(OUTCOMES, COLORS)):
        lx, ly = left + (i % 4) * 150, top + hgt + 52 + (i // 4) * 18
        o.append(f'<rect x="{lx}" y="{ly - 10}" width="12" height="12" fill="{col}"/><text x="{lx + 18}" y="{ly}">{oc}</text>')
    o.append("</svg>")
    return "\n".join(o)


CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37;--bad:#ff8a80}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 "Segoe UI",system-ui,sans-serif}
main{max-width:1100px;margin:0 auto;padding:32px 20px 64px;display:flex;flex-direction:column;gap:26px}
h1{font-size:28px;margin:0}h2{font-size:21px;margin:0}h3{font-size:17px;margin:0}p{margin:0;max-width:74ch}
section{display:flex;flex-direction:column;gap:10px}.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:15px}
table{border-collapse:collapse;font-size:14px;background:var(--panel)}th,td{border:1px solid var(--line);padding:5px 9px;text-align:left}td.n{text-align:right}
.scroll{overflow-x:auto}.err{color:var(--bad)}figure{margin:0;background:var(--panel);border:1px solid var(--line);padding:10px;overflow-x:auto}th{background:var(--thead)}
"""


def render(analysis, valid, errors, ignored, source="") -> str:
    h = html.escape
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Flat-wing test results</title><style>{CSS}</style></head><body><main>",
         "<h1>Flat-wing test: results</h1>",
         "<p class='note'>Counts of what happened in the logged flights. No model comparison is made here and nothing is predicted. "
         f"Pass rule (PROPOSED, to be agreed): at least {PASS_STEADY} of {PASS_RELEASES} steady glides at one setting, and both settings 1 mm either side also glide "
         f"(at least {int(NEIGHBOUR_GLIDES_FRAC * 100)} % of their releases steady glides). CG settings are rounded to 1 mm.</p>",
         f"<p>Log file: {h(str(source))}. Flights counted: {len(valid)}. Example / empty rows ignored: {ignored}. Rows rejected: {len(errors)}.</p>"]
    if errors:
        P.append("<section><h2 class='err'>Rows not counted</h2><div class='scroll'><table><tr><th>Line</th><th>Problem</th></tr>"
                 + "".join(f"<tr><td class='n'>{e['line']}</td><td>{h(e['problems'])}</td></tr>" for e in errors) + "</table></div></section>")
    if not valid:
        P.append("<section><h2>No flights logged yet</h2><p>No real flight rows were found in the log. Fill in <code>log_template.csv</code> (one row per throw) and run this script again.</p></section>")
    for a in analysis:
        P.append(f"<section><h2>{h(a['wing'])}, {h(a['thickness'])} mm, {h(a['config'] or '(no config given)')}, launch: {h(a['launch'])}</h2><p>{a['n']} flights.</p>")
        P.append("<figure>" + strip_chart(a["settings"]) + "</figure>")
        P.append("<div class='scroll'><table><tr><th>CG setting, mm</th><th>Releases</th>" + "".join(f"<th>{o}</th>" for o in OUTCOMES)
                 + "<th>Steady fraction</th><th>Median distance, m</th><th>Median time, s</th><th>Median release height, m</th></tr>")
        f = lambda v: "-" if v is None else f"{v:g}"
        for s, d in a["settings"].items():
            P.append(f"<tr><td class='n'>{s}</td><td class='n'>{d['n']}</td>" + "".join(f"<td class='n'>{d['counts'][o]}</td>" for o in OUTCOMES)
                     + f"<td class='n'>{d['frac']:.2f}</td><td class='n'>{f(d['dist'])}</td><td class='n'>{f(d['time'])}</td><td class='n'>{f(d['height'])}</td></tr>")
        P.append("</table></div>")
        if a["passing"]:
            P.append(f"<p><b>Settings meeting the PROPOSED rule:</b> {', '.join(str(s) for s in a['passing'])} mm aft of root leading edge (measured width of the passing set: {max(a['passing']) - min(a['passing']) + 1} mm, from {len(a['passing'])} passing settings).</p>")
        else:
            P.append("<p><b>Settings meeting the PROPOSED rule:</b> none so far (a setting needs 10 releases, at least 8 steady, and both 1 mm neighbours gliding).</p>")
        P.append("</section>")
    if valid:
        P.append("<section><h2>Raw rows</h2><div class='scroll'><table><tr><th>Line</th>" + "".join(f"<th>{c}</th>" for c in COLUMNS) + "</tr>")
        for r in valid:
            P.append(f"<tr><td class='n'>{r['line']}</td>" + "".join(f"<td>{h(str(r.get(c, '')))}</td>" for c in COLUMNS) + "</tr>")
        P.append("</table></div></section>")
    P.append("</main></body></html>")
    return "\n".join(P)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    a = ap.parse_args()
    valid, errors, ignored = read_log(a.log)
    page = render(analyse(valid), valid, errors, ignored, a.log)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(page, encoding="utf-8")
    print(f"{len(valid)} flights, {len(errors)} rejected, {ignored} ignored -> {a.out}")


if __name__ == "__main__":
    main()
