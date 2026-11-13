"""Flight-day page for 2026-10-20: what to print or cut, links to model files, settings, balance points, blank record fields.

Writes ONLY results/runs/flight_day_2026-10-20/index.html.  Reads (never writes) existing study outputs; imports bird_outline_v4_swift and
bird_outline_v2 read-only for size_numbers (moment balance, no AVL run).  No new aerodynamics: the B45 trim table is the linear relation
CL_trim = Cm_np / SM taken from results/runs/rigid_glider/b45_centre_flaps/results.json (checked against its own CG sweep).

Run:  py -B rigid_glider/flight_day_page.py          (writes the page and checks every link)
"""
import csv
import html
import json
import math
import os
import sys
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "results/runs/flight_day_2026-10-20"
sys.path.insert(0, str(ROOT / "rigid_glider"))

RUNS = ROOT / "results/runs"
B45 = RUNS / "rigid_glider/b45_centre_flaps"
SW = RUNS / "rigid_glider/bird_outline_v4_swift"
ZAN = RUNS / "known_good/zanonia"

LINKS = []   # (label, absolute path) for the checker


def rel(p):
    return os.path.relpath(str(p), str(OUT_DIR)).replace("\\", "/")


def L(p, label=None):
    p = Path(p)
    LINKS.append((label or p.name, p))
    href = rel(p) + ("/" if p.is_dir() else "")
    return f'<a href="{html.escape(href)}">{html.escape(label or p.name)}</a>'


def T(tag):
    return f' <span class="tag t-{tag.split()[0].split("-")[0].lower()}">{tag}</span>'


# ------------------------------------------------------------------------------------------------ numbers
def b45_numbers():
    d = json.load(open(B45 / "results.json"))
    mac = d["geometry"]["mac"]
    mass = d["geometry"]["mass_g"]
    span, ar = 400.0, 4.5
    area = span * span / ar                                    # mm2, GEOMETRY (results.json header: AR 4.5)
    nom = [c for c in d["cases"] if c["loc"] == "nominal" and c["flap"] > 0]
    k = sum(c["cl_trim"] * c["sm_pct"] / 100 / c["flap"] for c in nom) / len(nom)       # Cm_np per deg of flap
    xnp = {c["flap"]: c["xnp"] for c in nom}
    xnp_per_deg = (xnp[15.0] - xnp[5.0]) / 10.0

    def xnp_at(f):
        return xnp[5.0] + (f - 5.0) * xnp_per_deg

    rho = 1.225

    def trim(f, cg):
        sm = (xnp_at(f) - cg) / mac
        if sm <= 0.002:
            return sm, None, None
        cl = k * f / sm
        v = math.sqrt(2 * mass * 1e-3 * 9.80665 / (rho * area * 1e-6 * cl))
        return sm, cl, v

    # check against the file's own 10 deg CG sweep
    chk = []
    for r in d["sweep"]:
        if r["loc"] == "nominal" and 63 <= r["xcg"] <= 68 and r["cl_trim"] > 0:
            sm, cl, v = trim(10.0, r["xcg"])
            chk.append((r["xcg"], r["cl_trim"], cl, r["v_trim"], v))
    return dict(mac=mac, mass=mass, k=k, trim=trim, xnp_at=xnp_at, chk=chk, area=area, d=d)


def swift_numbers():
    import bird_outline_v4_swift as s
    import bird_outline_v2 as v2
    rows = list(csv.DictReader(open(SW / "outlines.csv", encoding="utf8")))
    A = [r for r in rows if r["outline"] == "A_as_measured"][0]
    g = s.build_geometry4(s.GLIDE)
    out = {"bend8": float(A["bend for 8% margin deg"]), "bend10": float(A["bend for 10% margin deg"]),
           "cm0pd": float(A["CM0 per deg of bend"]), "csv": A}
    # trim CG at 10% from outlines.csv (250 mm); 8% = 10% + 0.02 MAC (margin definition cg = np - margin*MAC, bird_outline_v4_swift.evaluate4)
    cg10_250 = float(A["trim CG mm at 10% margin (250)"])
    mac250 = float(A["MAC mm"])
    for key, span, sk in (("foam", 400.0, "foam"), ("printed", 250.0, "printed")):
        f = span / 250.0
        cg10 = cg10_250 * f
        cg8 = cg10 + 0.02 * mac250 * f
        res = {}
        for lab, cg in (("10", cg10), ("8", cg8)):
            r = s.fake_res(cg / span, 0.1)
            r["trim"] = {c: dict(x_cg_span=cg / span, static_margin=0.1) for c in v2.TRIM_CLS}
            n = v2.size_numbers(g, r, sk, 6.0)
            res[lab] = dict(cg=cg, ballast=n["ballast_g"], M=n["M"], station=n["station"], frac=n["ballast_frac"],
                            sheet=n["m_sheet"], rail=n["rail"]["mass_g"], cg0=n["cg0"], ws=n["ws"])
        out[key] = res
        out[key]["hinge"] = g["x_h"] * span
        out[key]["mac"] = mac250 * f
    out["margin_at_5deg"] = 5.0 * out["cm0pd"] / 0.3
    return out


# ------------------------------------------------------------------------------------------------ page parts
CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37;--bad:#ff8a80;--good:#6cc08b}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 "Segoe UI",system-ui,sans-serif}
main{max-width:1100px;margin:0 auto;padding:28px 20px 64px;display:flex;flex-direction:column;gap:26px}
h1{font-size:28px;margin:0}h2{font-size:22px;margin:0;border-bottom:1px solid var(--line);padding-bottom:4px}h3{font-size:16px;margin:8px 0 0}p{margin:0;max-width:80ch}
section{display:flex;flex-direction:column;gap:12px}
.box{background:var(--panel);border:2px solid var(--accent);padding:12px 16px;display:flex;flex-direction:column;gap:8px}
.card{background:var(--panel);border:1px solid var(--line);padding:14px 16px;display:flex;flex-direction:column;gap:10px}
.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:15px}
.warn{border-left:3px solid var(--bad);padding:2px 0 2px 12px;color:var(--ink);font-size:15px}
table{border-collapse:collapse;font-size:14.5px;background:var(--panel)}
th,td{border:1px solid var(--line);padding:5px 9px;text-align:left;vertical-align:top}th{background:var(--thead)}td.n{text-align:right;font-variant-numeric:tabular-nums}
a{color:var(--link)}code{font-size:13.5px}
ul,ol{margin:0;padding-left:22px;max-width:84ch;display:flex;flex-direction:column;gap:5px}
.scroll{overflow-x:auto}
.tag{font-size:11px;color:var(--muted);white-space:nowrap;border:1px solid var(--line);border-radius:8px;padding:0 5px}
.t-model{color:#7fb4ff}.t-assumed{color:#e89a55}.t-measured{color:#6cc08b}.t-geometry{color:#c39bff}.t-unknown{color:#ff8a80;border-color:#ff8a80}
.t-calculated{color:#9fd6e8}.t-observed{color:#6cc08b}.t-judgement{color:#e8d77f}
.src{font-size:12.5px;color:var(--muted)}
.cols{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px}
input[type=text]{background:#0f1317;color:var(--ink);border:1px solid var(--line);padding:4px 6px;font:inherit;width:100%;box-sizing:border-box;min-width:70px}
table.rec td{padding:3px 4px}
.hi{background:#2c2418}.ok{background:#18242c}.stall{color:var(--bad)}
@media print{:root{color-scheme:light;--bg:#fff;--panel:#fff;--line:#999;--ink:#000;--muted:#333;--link:#000;--thead:#eee;--accent:#000}.tag{color:#000!important;border-color:#666!important}input[type=text]{background:#fff;border:1px solid #000}}
"""


def fields(extra=()):
    labs = ["Measured CG, mm behind centre LE", "Measured mass, g"] + list(extra)
    return ('<table class="rec"><tr>' + "".join(f"<th>{html.escape(x)}</th>" for x in labs) + "</tr><tr>" +
            "".join('<td><input type="text"></td>' for _ in labs) + "</tr></table>")


def flight_table(n=6):
    h = ["#", "setting used", "launch (height / how)", "result (glide, swoops, stall, tumble, turn L/R)", "distance, m", "video file"]
    r = "".join("<tr><td>%d</td>" % (i + 1) + "".join('<td><input type="text"></td>' for _ in h[1:]) + "</tr>" for i in range(n))
    return '<table class="rec"><tr>' + "".join(f"<th>{x}</th>" for x in h) + "</tr>" + r + "</table>"


def build():
    b = b45_numbers()
    sw = swift_numbers()
    P = []
    a = P.append

    f10 = sw["foam"]["10"]; f8 = sw["foam"]["8"]; p10 = sw["printed"]["10"]; p8 = sw["printed"]["8"]
    sw_out = SW / "index.html"

    # ---------------- header
    a("<h1>Flight day 2026-10-20: what to print or cut, settings, balance</h1>")
    a(f"<p class='note'><b>To mark the balance point (CG) on each real glider:</b> {L(OUT_DIR/'cg_marks.html', 'CG marks page (top view of each glider, printable 1:1)')}.</p>")
    a(f"<p>Four gliders, in the order to fly them. Every number is tagged{T('MODEL')}{T('ASSUMED')}{T('MEASURED')}{T('GEOMETRY')}{T('CALCULATED')} or{T('UNKNOWN')}. "
      "Nothing from the model is a promise that a glider flies; the knife-edge balance on the real part is what counts. "
      "All balance points are <b>mm behind the centre leading edge</b> (the leading edge at the centreline, the 0 mark on the drawings). "
      "The text fields below do not save: photograph the page or write it down.</p>")
    a('<p class="note">Links are relative, so they work from file://. STL, DXF and PDF open in your slicer or viewer, and Firefox may offer to download '
      '<code>.md</code> files instead of showing them.</p>')

    # ---------------- print tonight
    a("<section><h2>Print tonight</h2><div class='box'><ul>")
    a(f"<li><b>Swift A, printed 250 mm:</b> {L(SW/'A_printed_span250/sheet.stl')} (PLA plate 0.6 mm, print flat on the bed) and {L(SW/'A_printed_span250/rail.stl')} (CG rail, {p10['rail']:.2f} g{T('CALCULATED')}). "
      f"Drawing: {L(SW/'A_printed_span250/drawing.svg')}. Folder: {L(SW/'A_printed_span250', 'A_printed_span250/')}.</li>")
    a(f"<li><b>Zanonia 155 mm:</b> {L(ZAN/'print/zanonia_plate_155mm.stl')} (LW-PLA preferred, solid PLA is the backup; flight-upper face on the bed, no supports, 100% infill or all perimeters), "
      f"bending jigs {L(ZAN/'print/zanonia_jig_155mm_2deg.stl','2 deg')}, {L(ZAN/'print/zanonia_jig_155mm_4deg.stl','4 deg')}, {L(ZAN/'print/zanonia_jig_155mm_6deg.stl','6 deg')} "
      f"(optional extras {L(ZAN/'print/zanonia_jig_155mm_8deg.stl','8')}, {L(ZAN/'print/zanonia_jig_155mm_10deg.stl','10')}, {L(ZAN/'print/zanonia_jig_155mm_12deg.stl','12')}). "
      f"Folder: {L(ZAN/'print', 'print/')}.</li>")
    a(f"<li><b>Swift tail-bend angle gauge (paper, print at 100%):</b> {L(RUNS/'rigid_glider/field_pack/field_pack.pdf')}, page 4 has wedges 2, 2.5, 4, 5, 6, 7.5, 8 deg with a 100 mm check bar "
      f"(page SVG: {L(RUNS/'rigid_glider/field_pack/pages/page4_angle_gauge.svg','page4_angle_gauge.svg')}). The Swift template note pages also carry a 6 deg gauge: {L(SW/'A_templates.pdf')}. No separate printed (STL) bend gauge exists.</li>")
    a(f"<li><b>Swift A foam 400 (cut from board, if not already cut):</b> templates {L(SW/'A_foam_span400/template_page1.svg')}, {L(SW/'A_foam_span400/template_page2.svg')} or the PDF {L(SW/'A_templates.pdf')} (Letter, 1:1, check the 100 mm bar). "
      f"CG rail for it: {L(SW/'A_foam_span400/rail.stl')} ({f10['rail']:.2f} g{T('CALCULATED')}); skip if the rail from 2026-10-17 is still on the glider.</li>")
    a(f"<li><b>Nothing to print for B45</b> (foamboard, already built). Spare templates if needed: {L(RUNS/'rigid_glider/flat_test_v2/B45_span400/B45_templates_letter.pdf')}.</li>")
    a("<li>Also bring: knife-edge or ruler edge, 0.01 g scale, clay or blu-tack, thin tape for ballast, phone for side-view video, a pen.</li></ul></div></section>")

    # ---------------- every glider
    a("<section><h2>For every glider</h2><div class='box'><ul>"
      "<li>Balance on a knife edge. Write the point as <b>mm behind the centre leading edge</b>, and the mass in grams, before the first throw. Re-check after any change.</li>"
      "<li><b>One change at a time.</b> Fly the same setting several times (5 throws) before touching anything.</li>"
      "<li>Side-view video if you can: phone at the side of the flight line, at glider height, still. Panning from underneath cannot show pitch.</li>"
      "<li>Write down launch height and how it was released (nose-down drop is the fixed start for the Swifts).</li></ul></div></section>")

    # ---------------- overview
    a("<section><h2>Order of the day</h2><div class='scroll'><table>"
      "<tr><th>#</th><th>Glider</th><th>Why now</th><th>Setting</th><th>Balance point, mm</th><th>Expected mass, g</th></tr>"
      f"<tr><td>1</td><td>B45 foamboard 400 mm (control)</td><td>Flew 2026-10-19 with centre flaps; glides with swoops</td><td>Measure first, then ONE change: flaps 6-7 deg OR CG 1-2 mm forward</td><td>63-68 works in the model{T('MODEL')}; as flown UNKNOWN</td><td>17.5{T('ASSUMED')}</td></tr>"
      f"<tr><td>2</td><td>Swift A foam 400 mm</td><td>Flew 2026-10-17 with no tail bend and tumbled</td><td>Tail bend 6 deg (10% margin){T('MODEL')}; alt 5 deg (8%)</td><td>{f10['cg']:.1f} (alt {f8['cg']:.1f}){T('MODEL')}</td><td>{f10['M']:.2f} (alt {f8['M']:.2f}){T('CALCULATED')}</td></tr>"
      f"<tr><td>3</td><td>Swift A printed 250 mm</td><td>Not flown yet</td><td>Tail bend 6 deg (10%); alt 5 deg (8%)</td><td>{p10['cg']:.1f} (alt {p8['cg']:.1f}){T('MODEL')}</td><td>{p10['M']:.2f} (alt {p8['M']:.2f}){T('CALCULATED')}</td></tr>"
      f"<tr><td>4</td><td>Zanonia replica 155 mm, printed</td><td>Known-good outline, first flight</td><td>Jig curl 4 deg first, then 2 or 6 per the rule</td><td>17.2 (CG cross){T('GEOMETRY')}</td><td>2.99 LW-PLA 0.6 / 3.99 LW-PLA 0.8 / 6.18 solid{T('CALCULATED')}</td></tr>"
      "</table></div></section>")

    # ---------------- 1 B45
    tr = b["trim"]
    a("<section><h2>1. B45 foamboard 400 mm (control)</h2><div class='card'>")
    a(f"<p><b>Flown 2026-10-19 with centre flaps.</b> It glides, with phugoid or possibly stall-porpoise swoops (Kai's description{T('OBSERVED-from-video')}). "
      f"Flaps: about 10 mm chord x 50 mm span, about 3 cm from the centreline, about 10 deg trailing edge UP (Kai's reading, not measured{T('ASSUMED')}).</p>")
    a(f"<p>Files: spec {L(ROOT/'rigid_glider/FLAT_TEST.md')}; model {L(B45/'README.md')}, {L(B45/'tables.md')}, {L(B45/'results.json')}; "
      f"build files {L(RUNS/'rigid_glider/flat_test_v2/B45_span400', 'B45_span400/')}: {L(RUNS/'rigid_glider/flat_test_v2/B45_span400/template_half_wing_letter.svg','half-wing template')}, "
      f"{L(RUNS/'rigid_glider/flat_test_v2/B45_span400/template_fins_letter.svg','fin template')}, {L(RUNS/'rigid_glider/flat_test_v2/B45_span400/B45_templates_letter.pdf','templates PDF')}, "
      f"{L(RUNS/'rigid_glider/flat_test_v2/B45_span400/stick.stl','stick.stl')}, {L(RUNS/'rigid_glider/flat_test_v2/B45_span400/drawing.svg','drawing.svg')}, {L(RUNS/'rigid_glider/flat_test_v2/B45_span400/geometry.json','geometry.json')}; "
      f"test page {L(RUNS/'rigid_glider/flat_test_v2/index.html','flat_test_v2/index.html')}.</p>")
    a(f"<p>Model facts: neutral point 70.24 mm behind the root LE (flap 0), 70.30 at 10 deg{T('MODEL')}; MAC {b['mac']:.1f} mm{T('GEOMETRY')}. "
      f"The model says the flight works only for CG about 63-68 mm{T('MODEL')} (trim CL 0.2-0.9 rule, <code>tables.md</code> verdict). Behind about 70.3 mm it is unstable. The model has no stall in it and sees no swoop dynamics.</p>")
    a("<h3>Plan</h3><ol>"
      "<li><b>First, as flown, no changes:</b> balance on the knife edge, record the CG and the mass (fields below). Throw 5 times and note what the swoops look like.</li>"
      "<li><b>Then ONE change aimed at the swoops</b> (not both):<ul>"
      "<li><b>Option F:</b> bend the flaps down from about 10 to about 6-7 deg up. Less flap means a lower trim CL and a faster glide.</li>"
      "<li><b>Option W:</b> add nose weight to move the CG 1-2 mm forward. Same effect on trim CL, flaps left alone. "
      "Moment balance (ARITHMETIC): m = M d / (CG - x_hole - d). With M 17.5 g (ASSUMED), CG 66 mm, a hole at -37.5 mm (the stick's furthest ballast hole, GEOMETRY from "
      f"{L(RUNS/'rigid_glider/flat_test_v2/B45_span400/geometry.json','geometry.json')}): d = 1 mm needs about {17.5*1/(66+37.5-1):.2f} g, d = 2 mm about {17.5*2/(66+37.5-2):.2f} g. "
      "Use the real CG and mass you measure; this is only a sizing hint.</li></ul></li></ol>")
    a(f"<h3>Trim CL and trim speed against CG and flap (nominal flap location){T('MODEL')}</h3>")
    a("<p class='src'>Source: <code>b45_centre_flaps/results.json</code> (cases and CG sweep) and <code>tables.md</code>. The model is linear in flap angle (dCm0/dflap 0.00206 per deg at the nominal location), "
      f"so CL_trim = Cm_np / SM with Cm_np = {b['k']:.5f} per deg of flap, SM = (Xnp - CG) / MAC. Flap 10 deg reproduces the file's own CG sweep (check below); 5 and 7 deg are the same relation, "
      f"not separate runs. Speed at {b['mass']:.2f} g and wing area 35,556 mm<sup>2</sup> ({'ASSUMED mass, GEOMETRY area'}), air density 1.225 kg/m<sup>3</sup>: use your measured mass to rescale (speed goes as the square root of mass). "
      "The alternate flap location (y 5-55 mm) gives CL about 15% lower. The flap deflection here is the real fold angle, which on foamboard is a crude reflex.</p>")
    a("<div class='scroll'><table><tr><th rowspan=2>CG, mm</th><th rowspan=2>SM, % MAC</th><th colspan=2>flap 5 deg up</th><th colspan=2>flap 7 deg up</th><th colspan=2>flap 10 deg up</th></tr>"
      "<tr><th>trim CL</th><th>V, m/s</th><th>trim CL</th><th>V, m/s</th><th>trim CL</th><th>V, m/s</th></tr>")
    for cg in range(63, 70):
        sm10 = (b["xnp_at"](10.0) - cg) / b["mac"] * 100
        a(f"<tr><td class='n'>{cg}</td><td class='n'>{sm10:.1f}</td>")
        for f in (5.0, 7.0, 10.0):
            sm, cl, v = tr(f, cg)
            if cl is None:
                a("<td class='n' colspan=2>unstable</td>")
            else:
                cls = "stall" if cl > 0.9 else ""
                a(f"<td class='n {cls}'>{cl:.2f}</td><td class='n {cls}'>{v:.1f}</td>")
        a("</tr>")
    a("</table></div>")
    worst = max(abs(c[2] - c[1]) for c in b["chk"])
    a(f"<p class='src'>Check: flap 10 deg by this relation against the file's own CG sweep at CG 63-68 mm differs by at most {worst:.3f} in trim CL "
      f"(results.json sweep). Red = trim CL above 0.9, the upper limit of the model's own 0.2-0.9 'works' rule{T('JUDGEMENT')}; there a flat plate is near stall and the model cannot see it. SM in the second column is for flap 10 (Xnp 70.30).</p>")
    a("<p class='note'>Reading it: forward CG or less flap gives a faster glide at lower CL. Aft CG or more flap gives a slower glide, nearer stall, and the trim CL runs away as CG approaches 70.3 mm "
      "(unstable past it). The model's neutral point may be up to about 4.6 mm too far aft for a real flat plate (README caveat), so the real margin could be thinner than the table. "
      "This favours trying forward first.</p>")
    a("<h3>Record</h3>" + fields(["Flap angle, deg (as set)", "Nose weight added, g (position)"]) + flight_table(6))
    a("</div></section>")

    # ---------------- Swift common
    def swift_card(title, sk, key, files_html, print_html):
        n10 = sw[key]["10"]; n8 = sw[key]["8"]
        mac = sw[key]["mac"]
        hinge = sw[key]["hinge"]
        hole10 = n10["station"]
        a(f"<section><h2>{title}</h2><div class='card'>")
        a(files_html)
        a(f"<div class='scroll'><table><tr><th></th><th>Primary: 10% margin</th><th>Alternate: 8% margin</th><th>Source</th></tr>"
          f"<tr><td>Tail bend needed, deg (trailing edge UP)</td><td class='n'><b>{sw['bend10']:.1f} -> use the 6 deg gauge</b>{T('MODEL')}</td>"
          f"<td class='n'>{sw['bend8']:.1f} -> nearest gauge is 5 deg (gives {sw['margin_at_5deg']:.1f}% margin by arithmetic){T('MODEL')}</td>"
          f"<td class='src'><code>bird_outline_v4_swift/outlines.csv</code> row A_as_measured (bend for 8% / 10% margin); <code>criteria.csv</code> (4.8 deg); page section 8 (6 deg build)</td></tr>"
          f"<tr><td>Trim CG, mm behind centre LE</td><td class='n'><b>{n10['cg']:.1f}</b>{T('MODEL')}</td><td class='n'>{n8['cg']:.1f}{T('MODEL')}</td>"
          f"<td class='src'>10%: <code>outlines.csv</code> (23.1 at 250 mm, scaled x{ {'foam':1.6,'printed':1.0}[key]:g}) and the drawing label; 8%: 10% value + 0.02 x MAC ({mac:.1f} mm), the margin definition in <code>bird_outline_v4_swift.evaluate4</code> (cg = NP - margin x MAC); 250 mm 8% value 23.7 agrees with <code>criteria.csv</code></td></tr>"
          f"<tr><td>Nose ballast, g (at the {hole10:g} mm nose hole of the rail)</td><td class='n'><b>{n10['ballast']:.2f}</b>{T('CALCULATED')}</td><td class='n'>{n8['ballast']:.2f}{T('CALCULATED')}</td>"
          f"<td class='src'><code>bird_outline_v2.size_numbers</code> imported read-only, moment balance with ASSUMED masses (bare CG {n10['cg0']:.1f} mm); 10% value matches page section 8 table; 8% value matches <code>criteria.csv</code> ballast fraction</td></tr>"
          f"<tr><td>All-up mass, g</td><td class='n'><b>{n10['M']:.2f}</b>{T('CALCULATED')}</td><td class='n'>{n8['M']:.2f}{T('CALCULATED')}</td>"
          f"<td class='src'>sheet {n10['sheet']:.2f} g + rail {n10['rail']:.2f} g + ballast; materials ASSUMED; weigh yours</td></tr>"
          f"<tr><td>Ballast / all-up mass</td><td class='n'>{100*n10['frac']:.1f}%</td><td class='n'>{100*n8['frac']:.1f}%</td><td class='src'>same</td></tr>"
          f"<tr><td>Hinge line (bend line), mm behind centre LE</td><td class='n' colspan=2>{hinge:.1f}{T('GEOMETRY')}</td><td class='src'>drawing.svg label 'hinge line (bend UP, trailing edge raised)'; DXF layer HINGE_BEND_UP in parts.dxf</td></tr>"
          f"</table></div>")
        a(print_html)
        a("<h3>Record</h3>" + fields(["Tail bend used, deg", "Ballast, g (hole)"]) + flight_table(6))
        a("</div></section>")

    bend_how = ("<h3>How to make the bend</h3><ul>"
                "<li>The tail is the fan behind the hinge line (orange line on the drawing). Bend it so the <b>trailing edge goes UP</b> (tail set nose-down relative to the wing). One sheet, one bend.</li>"
                f"<li>Template note page text: 'Score the top face, bend up to the gauge, glue the crease.' Lay the wing along the long edge of the gauge wedge; the tail follows the slanted edge. "
                f"The 6 deg wedge is on template page 2 ({L(SW/'A_foam_span400/template_page2.svg','foam page 2')}, {L(SW/'A_printed_span250/template_page2.svg','printed page 2')}). Other angles (5 deg for the alternate): {L(RUNS/'rigid_glider/field_pack/field_pack.pdf','field_pack.pdf page 4')}.</li>"
                "<li>Check the 100 mm scale bar on any printed gauge before using it. The wrist line and body edge on the drawing are markers only; do not cut or crease there.</li></ul>")

    # ---------------- 2 Swift foam
    foam_files = (f"<p><b>Flew 2026-10-17 with NO tail bend and tumbled; the design always assumed a bend.</b> The 10-17 build: CG 37.0 mm, ballast 1.20 g at the -12.5 mm nose hole, all-up 9.76 g "
                  f"({L(ROOT/'research/tumble_model/README.md')}{T('CALCULATED')}). That CG is already the 10% trim CG for a 6 deg bend, so the primary test is <b>that same glider plus a 6 deg bend, no ballast change</b> (compare the 37.0 mm above). "
                  f"Weigh and balance as it is first.</p>"
                  f"<p>Files: {L(SW/'A_foam_span400', 'A_foam_span400/')}: {L(SW/'A_foam_span400/drawing.svg')}, {L(SW/'A_foam_span400/parts.dxf')}, {L(SW/'A_foam_span400/rail.stl')}, "
                  f"{L(SW/'A_foam_span400/template_page1.svg')}, {L(SW/'A_foam_span400/template_page2.svg')}; PDF {L(SW/'A_templates.pdf')}. "
                  f"Source page {L(sw_out,'bird_outline_v4_swift/index.html')} (section 8), tables {L(SW/'criteria.csv')}, {L(SW/'outlines.csv')}. "
                  f"Span 400 mm, foamboard 5 mm (areal mass ASSUMED), wing area 17,548 mm<sup>2</sup>, MAC 47.4 mm{T('GEOMETRY')}.</p>")
    swift_card("2. Swift A foamboard 400 mm", "foam", "foam", foam_files, bend_how +
               f"<p class='note'>Primary 6 deg is the build setting on the page (10% margin; the 8% floor is JUDGEMENT and the real lift point may sit 0-5% MAC forward of the model, so 8% could be 3-5% in practice). "
               f"The model's tail effect is unvalidated (inviscid AVL, small tail in the wake). If 6 deg still tumbles, do not add a second change blindly: move the CG, not the bend, next, and record the CG.</p>")

    # ---------------- 3 Swift printed
    pr_files = (f"<p><b>3D print; nothing printed has been flown yet.</b> Files: {L(SW/'A_printed_span250', 'A_printed_span250/')}: {L(SW/'A_printed_span250/sheet.stl')}, {L(SW/'A_printed_span250/rail.stl')}, "
                f"{L(SW/'A_printed_span250/drawing.svg')}, {L(SW/'A_printed_span250/parts.dxf')}, {L(SW/'A_printed_span250/template_page1.svg')}, {L(SW/'A_printed_span250/template_page2.svg')}; PDF {L(SW/'A_templates.pdf')}. "
                f"Source page {L(sw_out,'bird_outline_v4_swift/index.html')} (section 8), {L(SW/'criteria.csv')}. Span 250 mm, wing area 6,855 mm<sup>2</sup>, MAC 29.6 mm{T('GEOMETRY')}.</p>")
    pr_print = ("<h3>Print settings (only what the project states)</h3><ul>"
                "<li>Print <code>sheet.stl</code> flat on the bed. Plate 0.6 mm = 3 layers of 0.2 mm (<code>rigid_glider/FLAT_TEST.md</code>, <code>bird_outline.py</code> docstring).</li>"
                "<li>The sheet is three overlapping closed shells: main sheet, a scored hinge strip 0.4 mm thick (two layers) and 1.0 mm wide, and the fan tail (<code>bird_outline_v2.py</code> HINGE_V2_MM, <code>bird_outline.py</code> HINGE_WIDTH_MM). Slicers should merge them; look at the preview to confirm the hinge strip is there.</li>"
                "<li>Material: PLA density 1.24 g/cm<sup>3</sup> (ASSUMED) is behind the mass numbers. A 0.6 mm plate is one to two 0.4 mm perimeters, so use 100% infill or all perimeters (the project's advice for the Zanonia plate, same thickness). Nozzle, speed, temperature: UNKNOWN, not stated by the project.</li>"
                "<li>Hinge strength: the printed hinge strip is screening arithmetic only (page section 8); a crease bent more than once may fail. Bend once to the gauge.</li></ul>" + bend_how)
    swift_card("3. Swift A printed 250 mm", "printed", "printed", pr_files, pr_print)

    # ---------------- 4 Zanonia
    card = ROOT / "research/known_good/zanonia/TEST_CARD.md"
    a("<section><h2>4. Zanonia replica 155 mm (3D print, LW-PLA preferred)</h2><div class='card'>")
    a(f"<p>Test card (written before any flight, rule fixed): {L(card, 'TEST_CARD.md')}. Also {L(ROOT/'research/known_good/zanonia/RESULTS.md','RESULTS.md')}, {L(ZAN/'index.html','results page')}, {L(ZAN/'numbers.json')}, {L(ZAN/'templates.pdf')} (cardstock templates). "
      f"Files: {L(ZAN/'print/zanonia_plate_155mm.stl')}, jigs {L(ZAN/'print/zanonia_jig_155mm_2deg.stl','2')}, {L(ZAN/'print/zanonia_jig_155mm_4deg.stl','4')}, {L(ZAN/'print/zanonia_jig_155mm_6deg.stl','6')} deg "
      "(more: 8, 10, 12 are in the same folder).</p>")
    a(f"<p><b>Balance point: the raised CG cross, 17.2 mm behind the centre leading edge</b>{T('GEOMETRY')} ({L(ZAN/'numbers.json','numbers.json')}: cg_mm 17.22 at 155 mm). Nose weight goes in the 4 mm nose hole (blu-tack or clay), until the plate balances on the knife edge at the cross.</p>")
    a(f"<div class='scroll'><table><tr><th>Material</th><th>Plate, g</th><th>Nose weight, g</th><th>Total, g</th><th>Wing loading, N/m<sup>2</sup></th></tr>"
      f"<tr><td>LW-PLA 0.6 g/cm<sup>3</sup></td><td class='n'>1.76</td><td class='n'>1.23</td><td class='n'>2.99</td><td class='n'>6.0</td></tr>"
      f"<tr><td>LW-PLA 0.8 g/cm<sup>3</sup></td><td class='n'>2.34</td><td class='n'>1.64</td><td class='n'>3.99</td><td class='n'>8.0</td></tr>"
      f"<tr><td>PLA solid</td><td class='n'>3.63</td><td class='n'>2.55</td><td class='n'>6.18</td><td class='n'>12.4</td></tr></table></div>")
    a(f"<p class='src'>{T('CALCULATED')} densities {T('ASSUMED')}: <code>known_good/zanonia/numbers.json</code> (mass table, 155 mm) and TEST_CARD.md section 2. Weigh the real plate and set the nose weight on the balance, not from the table. Higher loading than the foam original (1.7 N/m<sup>2</sup>) means faster flight and a deeper first swoop: launch a bit higher.</p>")
    a("<h3>Build, in short (TEST_CARD section 3)</h3><ol>"
      "<li>Weigh the plate.</li><li>Warm the outer tips (hot water about 60-70 degC, or a heat gun on low) and press the plate <b>upside down</b> on the <b>4 deg</b> jig until cool; both tips must match.</li>"
      "<li>Add nose weight until it balances on the knife edge at the CG cross; weigh the finished glider; note any left-right twist and do not trim it out until the first flights.</li></ol>")
    a("<h3>'It works' rule (fixed 2026-10-19, TEST_CARD section 4)</h3><ul>"
      "<li>Indoors or dead-calm air. Gentle, level, straight hand launch at shoulder height (about 1.5 m); 5 launches per setting.</li>"
      "<li><b>Works</b> = at the marked CG and some single jig curl between 2 and 12 deg, at least <b>4 of 5</b> launches give a steady glide: at most one swoop, then no repeated stall-dive cycle, no tumble, no nose-in, and a ground distance at least <b>4 times the launch height</b> (about 6 m from 1.5 m). A steady turn counts; note its direction.</li>"
      "<li>Curl search: start at 4 deg. Dives or no pull-up: re-bend at 6, then 8, 10, 12. Stalls or porpoises: re-bend at 2. Stop at the first curl that meets the rule, then fly 5 more there to confirm.</li>"
      "<li><b>Does not work</b> = no curl from 2 to 12 deg meets the rule at the marked CG. The model already said the marked CG is near neutral (margin 1% MAC, AVL DISAGREE verdict).</li>"
      "<li>Side tests only after the verdict: CG 1 and 2 mm forward with more curl; two nose-down drops from about 2 m released at rest.</li></ul>")
    a("<h3>Record</h3>" + fields(["Material", "Plate mass, g", "Nose weight, g", "Curl, deg"]) + flight_table(6))
    a("</div></section>")

    # ---------------- video note
    a("<section><h2>2026-10-19 B45 video note</h2><div class='card'>"
      f"<p>{T('OBSERVED-from-video')} Release about 20 s into the clip. About 3 s of disorderly rotation and rolling right after release, then settled into a glide by about 24 s. Landed about 34.5 s, so about 14-15 s from release to landing.</p>"
      "<p>The phone was underneath and panning, so the pitch period cannot be measured from it. <b>Ask for a side view next time</b> (still phone, side-on, glider height) so the swoop period and amplitude can be read off.</p>"
      "<p class='src'>Clip times are Kai's reading of the video, not re-measured here. The 3 s of rolling after release is not in the AVL model (it has no release transient).</p></div></section>")

    # ---------------- sources
    a("<section><h2>Where the numbers come from</h2><ul>"
      f"<li>B45: {L(B45/'results.json')}, {L(B45/'tables.md')}, {L(B45/'README.md')}; mass 17.50 g ASSUMED; geometry {L(RUNS/'rigid_glider/flat_test_v2/B45_span400/geometry.json','geometry.json')}.</li>"
      f"<li>Swift: {L(SW/'outlines.csv')}, {L(SW/'criteria.csv')}, {L(sw_out,'index.html')}; masses and ballast by <code>rigid_glider/bird_outline_v2.size_numbers</code> (read-only import, no AVL run); 10-17 build record {L(ROOT/'research/tumble_model/README.md')}.</li>"
      f"<li>Zanonia: {L(ZAN/'numbers.json')}, {L(card,'TEST_CARD.md')}.</li>"
      "<li>Generator: <code>rigid_glider/flight_day_page.py</code>. Palette from <code>rigid_glider/flat_test.py</code> CSS.</li></ul>"
      "<p class='note'>UNKNOWN on this page: the as-flown B45 CG and mass (measure them); the real flap angle and extent on B45; printer settings beyond layer height and plate thickness; any 3D-printed bend gauge (none exists).</p></section>")

    body = "\n".join(P)
    doc = ("<!DOCTYPE html>\n<html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<meta name='color-scheme' content='dark'><title>Flight day 2026-10-20</title><style>" + CSS + "</style></head><body><main>\n" + body + "\n</main></body></html>\n")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "index.html").write_text(doc, encoding="utf-8")
    return b, sw


class _P(HTMLParser):
    VOID = {"meta", "br", "input", "link", "img", "hr"}

    def __init__(self):
        super().__init__()
        self.stack, self.err, self.hrefs = [], [], []

    def handle_starttag(self, tag, attrs):
        if tag not in self.VOID:
            self.stack.append(tag)
        for k, v in attrs:
            if k == "href":
                self.hrefs.append(v)

    def handle_endtag(self, tag):
        if tag in self.VOID:
            return
        if not self.stack or self.stack[-1] != tag:
            self.err.append(f"mismatch </{tag}> vs stack top {self.stack[-1] if self.stack else None}")
            if tag in self.stack:
                while self.stack and self.stack.pop() != tag:
                    pass
        else:
            self.stack.pop()


def check():
    text = (OUT_DIR / "index.html").read_text(encoding="utf-8")
    p = _P()
    p.feed(text)
    bad = []
    for h in p.hrefs:
        tgt = (OUT_DIR / h).resolve()
        if not tgt.exists():
            bad.append(h)
    print(f"parse errors: {len(p.err)} {p.err[:5]}; unclosed: {p.stack}")
    print(f"links: {len(p.hrefs)} (unique {len(set(p.hrefs))}); broken: {len(bad)} {bad}")
    return not p.err and not p.stack and not bad


if __name__ == "__main__":
    b, sw = build()
    print("wrote", OUT_DIR / "index.html")
    print("B45 k", round(b["k"], 6), "check (cg, file CL, rel CL, file V, rel V):")
    for c in b["chk"]:
        print("  ", [round(x, 3) for x in c])
    for k in ("foam", "printed"):
        for m in ("10", "8"):
            n = sw[k][m]
            print(k, m, {x: round(n[x], 3) for x in ("cg", "ballast", "M", "frac")}, n["station"])
    sys.exit(0 if check() else 1)
