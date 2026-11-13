"""Drop-test sheet: ONE document, 2 pages, JIS B5 182 x 257 mm portrait (page 1 plan, page 2 drop log).

   py -B test_plan.py

Reads existing result files only (nothing is re-run) and writes results/runs/rigid_glider/test_plan/drop_test_sheet.html and .pdf.
The HTML is printed with headless Edge or Chrome.  Standard library only."""
from __future__ import annotations
import csv
import html
import re
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RG = ROOT / "results/runs/rigid_glider"
OUT = RG / "test_plan"
E = html.escape

COLORS = {"MODEL": "#3d5a80", "MEASURED": "#3f7d5a", "ASSUMED": "#9a6a2f", "JUDGEMENT": "#7a5a8a", "CALCULATED": "#4a7a7e"}
BUILD_DATE = __import__("datetime").date.today().isoformat()

def pill(tag):
    c = COLORS[tag]
    return f"<span class='pill' style='color:{c};border-color:{c}'>{tag}</span>"


# =============================================================================================== data (read from result files)
def read_csv(p):
    with Path(p).open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def load():
    D = {}
    v4 = RG / "bird_outline_v4_swift"
    outl = {r["outline"][0]: r for r in read_csv(v4 / "outlines.csv")}
    D["mac"] = {k: float(outl[k]["MAC mm"]) for k in "AB"}                                    # mm at 250 mm span
    D["np_pct"] = {k: float(outl[k]["NP % MAC"]) for k in "AB"}
    D["bend10"] = {k: float(outl[k]["bend for 10% margin deg"]) for k in "AB"}
    page = (v4 / "index.html").read_text(encoding="utf-8")
    D["v4_page"] = page
    builds = {}
    for m in re.finditer(r"<tr><td>([AB]) (printed|foam) (\d+) mm</td>(.*?)</tr>", page):
        cells = re.findall(r"<td class='n'>(.*?)</td>", m.group(4))
        bend, margin, cg, sheet, rail, nose, allup, ws, depth = cells
        lo, hi = re.match(r"(\d+) to (\d+)", depth).groups()
        builds[(m.group(1), m.group(2))] = dict(span=int(m.group(3)), bend=float(bend), margin=margin, cg=float(cg), nose=nose, mass=float(allup), ws=float(ws), lo=int(lo), hi=int(hi))
    assert len(builds) == 4, builds
    D["builds"] = builds
    m = re.search(r"\(([\d.]+) printed, ([\d.]+) for the light foam bird\) and by the factor for a tail that is only 50% as effective as AVL says \(([\d.]+); ([\d.]+) at 75%\)", page)
    D["ratio_printed"], D["ratio_foam"], D["eff50"], D["eff75"] = (float(x) for x in m.groups())
    D["re_swift"] = re.search(r"Re (15,000-40,000)", page).group(1)
    D["re_tail"] = re.search(r"Reynolds number near ([\d,]+)", page).group(1)
    src = (ROOT / "rigid_glider/bird_outline_v4_swift.py").read_text(encoding="utf-8")
    D["gauge"] = tuple(float(x) for x in re.search(r"GAUGE_ANGLES = \(([^)]*)\)", src).group(1).split(","))
    # flat kit
    ballast = read_csv(RG / "flat_test_v2/ballast_table.csv")
    kcg = list(ballast[0])[2]
    D["kit_cg"] = {}
    for w in ("F45", "F60", "B45"):
        v = [float(r[kcg]) for r in ballast if r["wing"] == w]
        D["kit_cg"][w] = (min(v), max(v))
    band = read_csv(RG / "pullout/kit_band.csv")
    D["kit_np"] = {r["specimen"].split("_")[0]: float(r["AVL neutral point mm"]) for r in band if r["variant"] == "base"}
    spec = read_csv(RG / "pullout/specimens.csv")
    D["plate_re"] = sorted({round(float(r["table Re"]), -3) for r in spec})
    cases = read_csv(RG / "pullout/cases.csv")
    D["n_cases"] = len(cases)
    D["n_cannot"] = sum(1 for r in cases if r["cannot_say"] == "True")
    bs = [r for r in cases if r["spec"].startswith("BS")]
    D["n_bs"], D["n_bs_cannot"] = len(bs), sum(1 for r in bs if r["cannot_say"] == "True")
    near = [r for r in cases if "within 3% MAC of the neutral point" in r["cannot_say_why"]]
    D["n_near_np"] = len(near)
    kit = {}
    for sp in ("F45_span250", "F60_span250", "B45_span400"):
        base = [r for r in cases if r["spec"] == sp and r["tag"] == "base"]
        zero = [r for r in base if float(r["up_deg"]) == 0 and r["cannot_say"] == "False"]
        ok = [r for r in base if r["verdict"].startswith("SETTLES") and r["cannot_say"] == "False"]
        at6 = [float(r["swoop_depth_m"]) for r in ok if float(r["up_deg"]) == 6]
        allv = [float(r["swoop_depth_m"]) for r in ok]
        kit[sp.split("_")[0]] = dict(n_zero=len(zero), n_zero_dive=sum(1 for r in zero if r["verdict"] == "DIVES"), n_ok=len(ok), at6=(min(at6), max(at6)) if at6 else None,
                                     allv=(min(allv), max(allv)), tabs=sorted({float(r["up_deg"]) for r in ok}), ratio=statistics.median(float(r["depth_over_formula"]) for r in ok))
    D["kit"] = kit
    return D


def neighbours(angle, gauge):
    g = sorted(gauge)
    i = g.index(angle)
    return (g[i - 1] if i > 0 else None, g[i + 1] if i + 1 < len(g) else None)


def swift_cg(D, k, size, margin):
    """CG (mm aft of root LE) for a static margin: the 10 % margin CG from the build table, moved by (10 % - margin) of the MAC (ARITHMETIC on MODEL values)."""
    b = D["builds"][(k, size)]
    scale = b["span"] / 250.0
    return b["cg"] + (0.10 - margin) * D["mac"][k] * scale


# =============================================================================================== html helpers
SECOND = "#b9772a"
CHIPS = {"G": "#3f7d5a", "PS": "#b9772a", "D": "#b4524a", "T": "#5b6b7a", "SP": "#7a5a8a", "RF": "#8a9199", "H": "#4a8fa0"}
OUTCOMES = [("G", "pulls out and glides"), ("PS", "pulls out, then porpoises or stalls"), ("D", "dives, no recovery"), ("T", "tumbles"),
            ("SP", "spirals or turns away"), ("RF", "release fault"), ("H", "hand glide (trim check)")]
CSS = """
@page{size:182mm 257mm;margin:5mm 5mm 5mm 10mm}
*{box-sizing:border-box}
html,body{margin:0;padding:0}
body{background:#fff;color:#14181d;font:8.6pt/1.28 "Segoe UI",system-ui,sans-serif;font-variant-numeric:tabular-nums}
.sheet{height:246mm;display:flex;flex-direction:column;break-after:page;page-break-after:always;border-left:1.8mm solid #1d4e5f;padding-left:2.2mm}
.sheet:last-child{break-after:auto;page-break-after:auto}
.hd{background:#1d4e5f;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:2.2mm 3mm;border-radius:1mm;flex:none;margin-bottom:2mm}
.hd b{font-size:14pt;letter-spacing:.01em}.hd span{font-size:8.5pt}
.fld{display:inline-block;border-bottom:1px solid #fff;min-width:26mm;height:4.4mm;vertical-align:bottom;margin-left:1mm}
.bd{flex:1;display:flex;flex-direction:column;gap:0}
.ft{flex:none;margin-top:1mm;font-size:6.6pt;color:#4a5560;display:flex;justify-content:space-between;gap:4mm}
.blk{background:#eaf1f4;border-radius:1mm;margin-bottom:2mm;overflow:hidden}
.blk>h2{background:#1d4e5f;color:#fff;margin:0;padding:.8mm 2.4mm;font-size:8.8pt;font-weight:700;letter-spacing:.02em}
.blk.sec>h2{background:#b9772a}
.in{padding:1.6mm 2.4mm}
p{margin:0 0 1mm}
ul,ol{margin:0;padding-left:4.6mm}li{margin:0 0 .5mm}
table{border-collapse:collapse;width:100%;font-size:8pt;line-height:1.2;background:#fff}
th,td{border:.6px solid #9fb3bb;padding:.6mm 1.3mm;text-align:left;vertical-align:top}
th{background:#1d4e5f;color:#fff;font-weight:700;border-color:#1d4e5f}
td.n{text-align:right;white-space:nowrap}
.pill{display:inline-block;font-size:5.8pt;font-weight:700;border:.8px solid;border-radius:6px;padding:0 1mm;margin:0 .3mm;line-height:1.2;vertical-align:1px;white-space:nowrap;background:#fff}
.note{color:#3b4650;font-size:7.2pt;margin-top:1mm}
.chk{display:grid;grid-template-columns:1fr 1fr 1fr;gap:1.2mm 3mm}
.chk div{background:#fff;border:.6px solid #9fb3bb;padding:1mm 1.6mm;display:flex;gap:1.6mm;align-items:center}
.box{display:inline-block;width:3.6mm;height:3.6mm;border:1.1px solid #1d4e5f;background:#fff;flex:none}
.two{display:grid;grid-template-columns:1fr 56mm;gap:0 3mm;align-items:start}
.chip{display:inline-block;color:#fff;font-weight:700;font-size:7.6pt;border-radius:3mm;padding:0 2.2mm;min-width:7mm;text-align:center;line-height:1.5}
.keyrow{display:flex;flex-wrap:wrap;gap:1mm 4mm;align-items:center;font-size:7.8pt}
.log td{height:9.6mm}.log tr:nth-child(even) td{background:#e2edf1}
.best td{height:6.6mm}.best tr:nth-child(even) td{background:#e2edf1}
.lbl{background:#fff;font-weight:700;color:#1d4e5f;width:20mm}
.sess td{height:7.6mm}
svg text{font-family:"Segoe UI",system-ui,sans-serif}
@media screen{body{background:#cfd6db}.sheet{background:#fff;margin:6mm auto;width:172mm}}
"""


def table(head, rows, widths=None, num_cols=(), cls=""):
    h = "".join(f"<th{(' style=width:' + widths[i]) if widths else ''}>{c}</th>" for i, c in enumerate(head))
    b = "".join("<tr>" + "".join(f"<td{' class=n' if i in num_cols else ''}>{c}</td>" for i, c in enumerate(r)) + "</tr>" for r in rows)
    return f"<table class='{cls}'><tr>{h}</tr>{b}</table>"


def blk(title, body, sec=False):
    return f"<div class='blk{' sec' if sec else ''}'><h2>{title}</h2><div class='in'>{body}</div></div>"


def chip(code):
    return f"<span class='chip' style='background:{CHIPS[code]}'>{code}</span>"


def sheet(title, right, body, src):
    return (f"<div class='sheet'><div class='hd'><b>{title}</b><span>{right}</span></div><div class='bd'>{body}</div>"
            f"<div class='ft'><span>{src}</span><span>built {BUILD_DATE}</span></div></div>")


def head_html(title):
    return f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>{title}</title><style>{CSS}</style></head><body>"


DROP_SVG = """<svg viewBox='0 0 150 100' xmlns='http://www.w3.org/2000/svg' width='100%'>
<g fill='none' stroke='#1d4e5f' stroke-width='1' stroke-linecap='round'>
<line x1='4' y1='92' x2='148' y2='92' stroke-width='.7'/>
<line x1='10' y1='8' x2='10' y2='92' stroke-dasharray='2 2' stroke-width='.7'/>
<path d='M10 10 C12 46 22 68 48 68 C64 68 72 60 80 56' stroke-width='1.8'/>
<path d='M80 56 L140 80' stroke='#b9772a' stroke-dasharray='4 2.5' stroke-width='1.3'/>
<line x1='28' y1='10' x2='28' y2='68' stroke-width='.7'/><line x1='25' y1='10' x2='31' y2='10' stroke-width='.7'/><line x1='25' y1='68' x2='31' y2='68' stroke-width='.7'/>
<line x1='96' y1='62.4' x2='124' y2='62.4' stroke-width='.7'/><line x1='124' y1='62.4' x2='124' y2='73.6' stroke-width='.7'/>
</g>
<g fill='#1d4e5f'><circle cx='10' cy='10' r='2.4'/><circle cx='48' cy='68' r='1.8'/><circle cx='80' cy='56' r='1.8'/></g>
<g font-size='5.6' fill='#14181d'>
<text x='15' y='8'>release point</text>
<text x='31' y='36'>swoop depth</text><text x='31' y='42'>(release to</text><text x='31' y='48'>lowest point)</text>
<text x='38' y='78'>lowest point</text>
<text x='64' y='50'>level-off</text>
<text x='98' y='58'>horizontal distance</text><text x='126' y='68'>height</text><text x='126' y='74'>lost</text>
<text x='66' y='87' font-size='5.2'>glide angle = distance / height lost</text>
<text x='13' y='99' font-size='5.2'>height reference (pole)</text>
</g></svg>"""


# =============================================================================================== the sheet
def settings_rows(D):
    B, g, kit = D["builds"], D["gauge"], D["kit"]
    rows = []
    for k in "AB":
        lo, hi = neighbours(B[(k, "foam")]["bend"], g)
        for size, lab in (("foam", "foam 400"), ("printed", "printed 250")):
            bb = B[(k, size)]
            cgf, cgn, cga = (swift_cg(D, k, size, s) for s in (0.15, 0.10, 0.08))
            rows.append([f"swift {k} {lab}", f"{cgf:.1f} to {cga:.1f} ({cgn:.1f})", f"{bb['bend']:g}", f"{'-' if lo is None else f'{lo:g}'} / {'-' if hi is None else f'{hi:g}'}", f"{bb['mass']:.1f}", f"{bb['lo']} to {bb['hi']}"])
    for w, sp in (("F45", "250"), ("F60", "250"), ("B45", "400")):
        c, k_ = D["kit_cg"][w], kit[w]
        lo, hi = neighbours(6.0, k_["tabs"])
        rows.append([f"kit {w} ({sp})", f"{c[0]:g} to {c[1]:g} (NP {D['kit_np'][w]:.1f})", "6 (tab)", f"{lo:g} / {hi:g}", "kit table", f"{k_['at6'][0]:.0f} to {k_['at6'][1]:.0f}" if k_["at6"] else "n/a"])
    return rows


def plan_page(D):
    kit = D["kit"]
    b = []
    b.append(blk("Goal", "<p style='margin:0'>Release nose-down at rest from the quad: pulls out unaided, how much height it takes, then a straight, steady glide. Nothing has been flown; all predictions are model values.</p>"))
    b.append(blk("Settings", table(["Article", "CG range, mm aft of root LE (nominal)", "Bend, deg", "Steps either side", "Mass g", "Swoop depth m"], settings_rows(D), widths=["19%", "31%", "11%", "14%", "10%", "15%"], num_cols=(1, 2, 3, 4, 5)) +
                 f"<p class='note'>Swift: 15% to 8% static-margin CG window, nominal 10% {pill('MODEL')} (NP {D['np_pct']['A']:.1f}% MAC A, {D['np_pct']['B']:.1f}% B); bend is the gauge angle nearest the 10% value ({D['bend10']['A']:.1f} / {D['bend10']['B']:.1f} deg) {pill('MODEL')}; mass {pill('CALCULATED')} (PLA density {pill('ASSUMED')}); depth = formula up to formula x {D['ratio_printed']:.2f} (printed) or {D['ratio_foam']:.2f} (foam) x {D['eff50']:.2f} (50% tail) {pill('MODEL')}. "
                 f"Kit: CG range {pill('JUDGEMENT')}, NP {pill('MODEL')}; 6 deg tab {pill('JUDGEMENT')}; depth over settled, unflagged cases {pill('MODEL')}. Order: forward CG first, step aft 2 to 3 mm {pill('JUDGEMENT')}, then bend one step either side; 2 drops per setting minimum {pill('JUDGEMENT')}.</p>"))
    items = ["Mass: within 5% of table", "CG: within 1 mm of target", "Bend: read on gauge", "Warp / twist: tips level", "Symmetry: matches template", "Hook / string: unchanged"]
    b.append(blk("Bench check", "<div class='chk'>" + "".join(f"<div><span class='box'></span><span>{t}</span></div>" for t in items) + "</div>"))
    b.append(blk("Trim guide", table(["Symptom (hand glide or drop)", "Adjustment"], [
        ["Dives / no recovery", "CG aft one step, or bend up one step"],
        ["Porpoise / stall / growing oscillation", "CG forward one step, or bend down one step"],
        ["Turns one way", "Fix warp first, then a small trailing-edge trim"],
        ["Wanders or spirals", "Tip fins or a small fin"],
        ["Swoop deeper than predicted", "Check bend on the gauge and CG on the balance"],
    ], widths=["42%", "58%"])))
    b.append(blk("Procedure", "<div class='two'><ol>"
                 "<li>Servo test with propellers off; drop zone clear of people; keep clear of the quad.</li>"
                 "<li>Steady hover; release only when the article hangs still; abort if it swings.</li>"
                 "<li>Record the drop height against a fixed reference; film side-on with it in view.</li>"
                 "<li>One change between drops.</li>"
                 f"<li>Hand-glide low over grass first as a trim check; log it as {chip('H')}.</li></ol><div>{DROP_SVG}</div></div>"))
    b.append(blk("Model limits", "<ul>"
                 f"<li>Force data: plates at Re {', '.join(f'{int(r):,}' for r in D['plate_re'])} {pill('MEASURED')}; swift outlines are swept and tapered at Re {D['re_swift']} {pill('MODEL')}.</li>"
                 "<li>Pitch-only simulation from rest in still air: no yaw, roll, downwash or string.</li>"
                 f"<li>Near-neutral CG unresolved: {D['n_cannot']} of {D['n_cases']} cases flagged; {D['n_bs_cannot']} of {D['n_bs']} swift cases {pill('MODEL')}.</li>"
                 f"<li>Tail-bend effect rests on an inviscid tail in the wake, Re near {D['re_tail']}; unvalidated {pill('MODEL')}.</li></ul>", sec=True))
    return "".join(b)


def log_page():
    b = []
    fields = [("Date", ""), ("Site", ""), ("Wind", ""), ("Height reference", "")]
    b.append("<div class='blk'><div class='in' style='padding:1.2mm 2.4mm'><table class='sess'><tr>" + "".join(f"<td class='lbl'>{a}</td><td></td>" for a, _ in fields) + "</tr></table></div></div>")
    head = ["#", "Article", "CG mm", "Bend deg", "Height m", "Outcome", "Swoop depth m", "Glide dist m", "Clip", "Note"]
    widths = ["4%", "17%", "9%", "8%", "9%", "11%", "9%", "10%", "9%", "14%"]
    rows = "".join("<tr>" + f"<td class='n' style='color:#1d4e5f;font-weight:700'>{i}</td>" + "<td></td>" * (len(head) - 1) + "</tr>" for i in range(1, 13))
    key = "".join(f"<span>{chip(c)} {d}</span>" for c, d in OUTCOMES)
    b.append(blk("Drop log", "<table class='log'><tr>" + "".join(f"<th style='width:{w}'>{h}</th>" for h, w in zip(head, widths)) + f"</tr>{rows}</table>"
                 f"<div class='keyrow' style='margin-top:1.8mm'>{key}</div>"
                 "<p class='note'>Swoop depth: lowest point below the release point (video). Glide dist: horizontal distance over the steady glide.</p>"))
    arts = ["swift A foam 400", "swift B foam 400", "swift A printed 250", "swift B printed 250", "kit F45", "kit F60", "kit B45"]
    best = "<table class='best'><tr><th style='width:26%'>Article</th><th style='width:12%'>CG mm</th><th style='width:12%'>Bend deg</th><th>Comment</th></tr>" + "".join(f"<tr><td>{a}</td><td></td><td></td><td></td></tr>" for a in arts) + "</table>"
    b.append(blk("Best setting found", best, sec=True))
    return "".join(b)


def sheet_html(D):
    src = "Sources: results/runs/rigid_glider/bird_outline_v4_swift, flat_test_v2, pullout; angle gauge: field_pack.pdf p. 4"
    P = [head_html("Drop-test sheet")]
    P.append(sheet("Drop-test sheet", "swift A and B, flat kit F45 F60 B45 &nbsp;|&nbsp; date <span class='fld'></span>", plan_page(D), src))
    P.append(sheet("Drop log", "swift A and B, flat kit F45 F60 B45 &nbsp;|&nbsp; page 2 of 2", log_page(), "Supersedes the flight log in field_pack.pdf for drop tests."))
    P.append("</body></html>")
    return "\n".join(P)


# =============================================================================================== printing
def find_browser():
    for p in (r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe", r"C:\Program Files\Microsoft\Edge\Application\msedge.exe", r"C:\Program Files\Google\Chrome\Application\chrome.exe",
              r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe"):
        if Path(p).is_file():
            return p
    return None


def page_count(pdf: Path):
    d = pdf.read_bytes()
    return len(re.findall(rb"/Type\s*/Page(?![s\w])", d))


def media_boxes(pdf: Path):
    return [tuple(float(x) for x in m) for m in re.findall(rb"/MediaBox\s*\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", pdf.read_bytes())]


def to_pdf(html_path: Path, pdf_path: Path):
    br = find_browser()
    if not br:
        raise SystemExit("no Chromium browser found")
    if pdf_path.exists():
        pdf_path.unlink()
    subprocess.run([br, "--headless", "--disable-gpu", "--no-pdf-header-footer", f"--print-to-pdf={pdf_path}", html_path.as_uri()], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)
    return page_count(pdf_path)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    D = load()
    res = {}
    for name, fn in (("drop_test_sheet", sheet_html),):
        h = OUT / f"{name}.html"
        h.write_text(fn(D), encoding="utf-8")
        res[name] = to_pdf(h, OUT / f"{name}.pdf")
        print(name, "pages:", res[name], "MediaBox:", sorted(set(media_boxes(OUT / f"{name}.pdf"))))
    return res


if __name__ == "__main__":
    main()
