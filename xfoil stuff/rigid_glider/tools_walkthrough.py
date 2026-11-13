"""Tools walkthrough PDF: a click-by-click mock design-and-test pass for Swift A, foam 400 mm.

   py -B rigid_glider/tools_walkthrough.py --run C:\\scratch\\mmav_practice   re-run every step in a practice copy, save the screen text, then build the PDF
   py -B rigid_glider/tools_walkthrough.py                                    build the PDF from the saved screen text (no runs)

Writes results/runs/tools_walkthrough/ : tools_walkthrough.pdf, tools_walkthrough.html, transcripts/ (the real screen text and screenshots of every step run).
--run makes the practice copy with make_practice_folder.py, runs AVL by piping the keystrokes, runs walk_balance.py, walk_pullout_one.py (twice) and pullout_sim.py --quick,
regenerates the v4 templates and the fin kit INSIDE the practice copy (they write only there), runs the drop tracker self-test and a demo screenshot with headless Edge.
The project's own results are never written by --run.  The PDF is printed with headless Edge (own temporary profile).  Standard library only.
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import argparse
import base64
import html
import json
import re
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/tools_walkthrough"
TR = OUT / "transcripts"
DOC_PRACTICE = r"<HOME>\Documents\mmav_practice"                       # the folder the PDF tells Kai to make
CODES = r"<MAV-2026>\Codes"
EDGE = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe", r"C:\Program Files\Microsoft\Edge\Application\msedge.exe", r"C:\Program Files\Google\Chrome\Application\chrome.exe"]
AVL_KEYS = ["plop", "g", "", r"load practice_inputs\SwiftA_foam400_bend6.avl", "oper", "a c 0.3", "x", "st", "", "", "quit"]


def edge():
    for p in EDGE:
        if Path(p).is_file():
            return p
    sys.exit("no Edge / Chrome found")


# =============================================================================================== running the steps (verification)
def sh(cmd, cwd, timeout=900, stdin=None):
    t = time.time()
    r = subprocess.run(cmd, cwd=str(cwd), input=stdin, text=True, capture_output=True, timeout=timeout)
    return (r.stdout or "") + (r.stderr or ""), time.time() - t, r.returncode


def save(name, text, practice):
    TR.mkdir(parents=True, exist_ok=True)
    (TR / name).write_text(text.replace(str(practice), DOC_PRACTICE).replace(str(practice).replace("\\", "/"), DOC_PRACTICE), encoding="utf-8")


def run_steps(practice: Path):
    facts = {}
    TR.mkdir(parents=True, exist_ok=True)
    py = sys.executable
    # 0 practice folder
    txt, dt, rc = sh([py, "-B", r"rigid_glider\make_practice_folder.py", str(practice)], ROOT)
    assert rc == 0, txt
    save("practice_make.txt", txt, practice)
    # 2 AVL by keystrokes
    avl = practice / "tools/avl/avl352.exe"
    txt, dt, rc = sh([str(avl)], practice, 120, "\n".join(AVL_KEYS) + "\n")
    assert rc == 0, rc
    save("avl_session.txt", txt, practice)
    # AVL eigen / MODE: what happens
    txt2, dt2, rc2 = sh([str(avl)], practice, 120, "\n".join(["plop", "g", "", r"load practice_inputs\SwiftA_foam400_bend6.avl", "oper", "a c 0.3", "x", "", "mode", "n", "quit"]) + "\n")
    save("avl_mode_crash.txt", txt2, practice)
    facts["avl_mode_rc"] = rc2
    # 3 balance
    m = re.search(r"Neutral point\s+Xnp\s*=\s*([0-9.]+)", txt)
    xnp_mm = float(re.search(r"Neutral point\s+Xnp\s*=\s*([0-9.]+)", (TR / "avl_session.txt").read_text()).group(1)) * 1000
    facts["xnp_mm"] = xnp_mm
    for margin, nm in ((0.10, "balance.txt"), (0.08, "balance_m08.txt")):
        t, dt, rc = sh([py, "-B", r"rigid_glider\walk_balance.py", f"{xnp_mm:.3f}", f"{margin:g}"], practice)
        assert rc == 0, t
        save(nm, t, practice)
    # 4 pull-out, one drop twice, and the quick study
    for tab, nm in ((6, "pullout_tab6.txt"), (0, "pullout_tab0.txt")):
        t, dt, rc = sh([py, "-B", r"rigid_glider\walk_pullout_one.py", "F45_span250", "39", str(tab)], practice)
        assert rc == 0, t
        save(nm, t, practice)
        facts[f"pullout_tab{tab}_s"] = dt
    t, dt, rc = sh([py, "-B", r"rigid_glider\pullout_sim.py", "--out", r"results\runs\pullout_practice", "--quick"], practice)
    assert rc == 0, t
    save("pullout_quick.txt", t[-600:], practice)
    facts["pullout_quick_s"] = dt
    # 5 regenerate the templates and the fin kit (inside the practice copy)
    t, dt, rc = sh([py, "-B", r"rigid_glider\bird_outline_v4_swift.py", "--out", r"results\runs\v4_practice"], practice)
    assert rc == 0, t[-800:]
    save("v4_regen.txt", t[-900:], practice)
    facts["v4_regen_s"] = dt
    facts["v4_regen_files"] = sorted(p.name for p in (practice / r"results\runs\v4_practice").iterdir() if p.suffix == ".pdf")
    t, dt, rc = sh([py, "-B", r"rigid_glider\fin_kit_v4.py"], practice)
    assert rc == 0, t[-800:]
    save("fin_regen.txt", t[-500:], practice)
    facts["fin_regen_s"] = dt
    # screenshots / self-test with headless Edge
    with tempfile.TemporaryDirectory() as prof:
        base = ["--headless", "--disable-gpu", "--no-first-run", f"--user-data-dir={prof}", "--hide-scrollbars"]
        trk = (practice / r"tools\drop_tracker\drop_tracker.html").as_uri()
        t, dt, rc = sh([edge()] + base + ["--virtual-time-budget=8000", "--dump-dom", trk + "?selftest"], practice, 120)
        mm = re.search(r"SELFTEST (PASS|FAIL)[^<]*", t)
        (TR / "tracker_selftest.txt").write_text(mm.group(0) if mm else "no selftest line found", encoding="utf-8")
        sh([edge()] + base + ["--virtual-time-budget=8000", f"--screenshot={TR / 'tracker_demo.png'}", "--window-size=1250,2250", trk + "?demo"], practice, 120)
        for nm, rel in (("template_a_p1.png", r"results\runs\v4_practice\A_foam_span400\template_page1.svg"), ("fin_p1.png", r"results\runs\rigid_glider\fin_kit_v4\fin_template_page1.svg")):
            sh([edge()] + base + [f"--screenshot={TR / nm}", "--window-size=830,1070", (practice / rel).as_uri()], practice, 120)
    # numbers read back from the practice copy's own outputs
    page = (practice / r"results\runs\v4_practice\index.html").read_text(encoding="utf-8")
    mm = re.search(r"\(([\d.]+) printed, ([\d.]+) for the light foam bird\) and by the factor for a tail that is only 50% as effective as AVL says \(([\d.]+)", page)
    facts["ratio_printed"], facts["ratio_foam"], facts["eff50"] = (float(x) for x in mm.groups())
    row = re.search(r"<tr><td>A foam 400 mm</td>(.*?)</tr>", page).group(1)
    facts["v4_row_A_foam"] = re.findall(r"<td class='n'>(.*?)</td>", row)
    (TR / "facts.json").write_text(json.dumps(facts, indent=1), encoding="utf-8")
    return facts


def run_extra(practice: Path):
    """The 'change one thing' check of step 2: the same AVL keystrokes with  a c 0.5  instead of  a c 0.3."""
    keys = [k if k != "a c 0.3" else "a c 0.5" for k in AVL_KEYS]
    txt, dt, rc = sh([str(practice / "tools/avl/avl352.exe")], practice, 120, "\n".join(keys) + "\n")
    assert rc == 0
    save("avl_c05.txt", txt, practice)
    f = json.loads((TR / "facts.json").read_text(encoding="utf-8"))
    g = lambda pat: float(re.search(pat, txt).group(1))
    f["c05"] = dict(alpha=g(r"Alpha\s*=\s*([0-9.]+)"), cl=g(r"CLtot\s*=\s*([0-9.]+)"), xnp_mm=1000 * g(r"Neutral point\s+Xnp\s*=\s*([0-9.]+)"), cm=g(r"Cmtot\s*=\s*([-0-9.]+)"))
    (TR / "facts.json").write_text(json.dumps(f, indent=1), encoding="utf-8")
    return f["c05"]


# =============================================================================================== the document
CSS = """
@page{size:Letter;margin:13mm 14mm 12mm 14mm}
*{box-sizing:border-box}
body{margin:0;color:#111;font:9.6pt/1.35 "Segoe UI",Arial,sans-serif}
.pg{page-break-after:always;break-after:page;height:250mm;position:relative}
.pg:last-child{page-break-after:auto;break-after:auto}
h1{font-size:19pt;margin:0 0 2mm;color:#1d4e5f}
h2{font-size:13.5pt;margin:0 0 2mm;color:#fff;background:#1d4e5f;padding:1.6mm 3mm;border-radius:1mm}
h3{font-size:10.6pt;margin:3mm 0 1mm;color:#1d4e5f;border-bottom:1px solid #9fb3bb}
p{margin:0 0 1.6mm}
table{border-collapse:collapse;width:100%;margin:0 0 2mm;font-size:9pt}
th,td{border:.6px solid #9fb3bb;padding:1mm 1.6mm;text-align:left;vertical-align:top}
th{background:#e7eff2;color:#1d4e5f}
td.n{width:6mm;text-align:center;font-weight:700;color:#1d4e5f}
pre{font:8pt/1.25 Consolas,"Courier New",monospace;background:#f3f5f7;border:.6px solid #cfd8dd;padding:1.4mm 2mm;margin:0 0 2mm;white-space:pre-wrap;overflow:hidden}
code,.k{font:8.7pt Consolas,"Courier New",monospace;background:#eef2f4;padding:0 1mm;border-radius:.6mm}
.k{border:.6px solid #9fb3bb;background:#fff}
mark{background:#ffe97a;color:#000;font-weight:700;padding:0 .4mm}
.note{border-left:3px solid #b9772a;padding:.6mm 0 .6mm 3mm;margin:0 0 2mm;background:#fbf5ec}
.bad{border-left:3px solid #b4524a;padding:.6mm 0 .6mm 3mm;margin:0 0 2mm;background:#fbefee}
.ex{background:#fff3b0;font-weight:700;padding:0 1.2mm;border:.6px solid #b9772a}
.small{font-size:8.2pt;color:#444}
.two{display:grid;grid-template-columns:1fr 1fr;gap:4mm}.two.wide{grid-template-columns:1.05fr 1fr}
img{max-width:100%}
.foot{position:absolute;bottom:0;left:0;right:0;font-size:7.6pt;color:#555;border-top:.6px solid #9fb3bb;padding-top:1mm}
"""


def rd(name):
    p = TR / name
    return p.read_text(encoding="utf-8") if p.is_file() else f"[missing transcript: {name}]"


def esc(s):
    return html.escape(s)


def pre(text, marks=()):
    t = esc(text.rstrip("\n"))
    for m in marks:
        t = t.replace(esc(m), f"<mark>{esc(m)}</mark>")
    return f"<pre>{t}</pre>"


def lines(text, start, stop=None, n=None):
    L = text.split("\n")
    i = next(i for i, l in enumerate(L) if start in l)
    j = len(L) if stop is None else next(k for k in range(i + 1, len(L)) if stop in L[k])
    out = L[i:j] if n is None else L[i:i + n]
    return "\n".join(l.rstrip() for l in out)


def img(name, width="100%", crop=None):
    p = TR / name
    if not p.is_file():
        return f"<p>[missing image {name}]</p>"
    b = base64.b64encode(p.read_bytes()).decode()
    if crop:
        top, h = crop
        return f"<div style='width:{width};height:{h};overflow:hidden'><img style='width:100%;margin-top:-{top}' src='data:image/png;base64,{b}'></div>"
    return f"<img style='width:{width}' src='data:image/png;base64,{b}'>"


def step_table(rows):
    t = "<table><tr><th>#</th><th>Do this</th><th>You should see</th></tr>"
    for i, (do, see) in enumerate(rows, 1):
        t += f"<tr><td class='n'>{i}</td><td>{do}</td><td>{see}</td></tr>"
    return t + "</table>"


def build_html():
    F = json.loads((TR / "facts.json").read_text(encoding="utf-8"))
    avl = rd("avl_session.txt")
    grab = lambda pat: re.search(pat, avl).group(1)
    xnp, clt, cmt, alpha = grab(r"Neutral point\s+Xnp\s*=\s*([0-9.]+)"), grab(r"CLtot\s*=\s*([0-9.]+)"), grab(r"Cmtot\s*=\s*([-0-9.]+)"), grab(r"Alpha\s*=\s*([0-9.]+)")
    cref, sref, xref = grab(r"Cref\s*=\s*([0-9.E+-]+)"), grab(r"Sref\s*=\s*([0-9.E+-]+)"), grab(r"Xref\s*=\s*([0-9.E+-]+)")
    cnb, clb = grab(r"Cnb\s*=\s*([-0-9.]+)\s*\n\n"), grab(r"Clb\s*=\s*([-0-9.]+)")
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Tools walkthrough</title><style>{CSS}</style></head><body>"]
    foot = lambda n: f"<div class='foot'>Tools walkthrough, page {n} of 10. Screen text is real output from a practice copy; folder paths are shown as if the practice folder were {DOC_PRACTICE}.</div>"
    # ------------------------------------------------------------------ page 1
    P.append("<div class='pg'><h1>Tools walkthrough: one mock pass on Swift A, foam 400 mm</h1>"
             "<p>This sheet walks through the tools in the order of a design-and-test pass: look up a number in AVL, turn it into a CG and a nose weight, simulate the drop, make the parts, fill in a test-day row, read the video, feed the result back. "
             "Each step says what to <b>type</b> or <b>click</b> and what you should <b>see</b>. Every command here was run before it was written down; the screen text is the real output.</p>"
             "<p class='note'><b>Safety rule: work in a practice copy.</b> The project scripts find their folders from where the script file sits. Run from a copy, they read and write only inside the copy, so practice never overwrites the project's results. "
             "Do the steps below in <b>PowerShell</b>.</p>")
    P.append("<h2>Step 0: make the practice folder (once)</h2>")
    P.append(step_table([
        ("Press the Windows key, type <code>PowerShell</code>, press <span class='k'>Enter</span>.", "A blue or black window with a prompt like <code>PS <HOME>&gt;</code>."),
        (f"Type and press Enter:<br><code>cd \"{CODES}\"</code>", "The prompt now ends in <code>...\\Codes&gt;</code>."),
        (f"Type and press Enter:<br><code>py -B rigid_glider\\make_practice_folder.py {DOC_PRACTICE}</code>", "A list of what was copied (below). Takes a few seconds."),
        (f"Type and press Enter:<br><code>cd {DOC_PRACTICE}</code>", "The prompt ends in <code>...\\mmav_practice&gt;</code>. <b>Stay in this folder for every step.</b>"),
    ]))
    P.append(pre(rd("practice_make.txt")))
    P.append("<p class='small'>The copy holds the scripts (<code>rigid_glider</code>), AVL and the drop tracker (<code>tools</code>), the plate data and kit tables the pull-out simulation reads, the PDFs to print (<code>print_these</code>) and the AVL file for Swift A (<code>practice_inputs</code>). "
             "Delete the folder any time; nothing else depends on it.</p>")
    P.append("<h2>The eight steps</h2><p>1 Map: where is everything. 2 AVL: neutral point and trim. 3 Balance: CG and nose weight. 4 Pull-out: does a nose-down drop recover, how deep. "
             "5 Parts: print and cut. 6 Test day: fill in the sheet, read the video, feed the result back. 7 CFD: what it is for. 8 Cheat sheet: every command.</p>")
    P.append(foot(1) + "</div>")
    # ------------------------------------------------------------------ page 2
    P.append("<div class='pg'><h2>Step 1: where things are</h2>"
             "<p>Only the folders and tools used in this walkthrough. Paths are inside <code>Codes\\</code>.</p><table><tr><th>Path</th><th>What it is</th><th>Question it answers</th></tr>"
             "<tr><td><code>tools\\avl\\avl352.exe</code></td><td>AVL 3.52, the vortex-lattice program (inviscid)</td><td>Neutral point, lift, pitching moment, yaw and roll stiffness of an outline</td></tr>"
             "<tr><td><code>rigid_glider\\bird_outline_v4_swift.py</code></td><td>Builds the Swift outlines A, B, C: AVL runs, CG, nose weight, templates, cut files</td><td>What does Swift A look like, where does it balance, what do I cut?</td></tr>"
             "<tr><td><code>rigid_glider\\walk_balance.py</code></td><td>New small script: the balance arithmetic printed line by line</td><td>How does the neutral point become the CG and the nose weight?</td></tr>"
             "<tr><td><code>rigid_glider\\pullout_sim.py</code> and <code>walk_pullout_one.py</code></td><td>Nose-down drop simulation (flat kit wings); the second runs one drop</td><td>Does it pull out of the dive, and how deep is the first swoop?</td></tr>"
             "<tr><td><code>rigid_glider\\fin_kit_v4.py</code></td><td>Sizes tip fins and a tail fin with AVL, makes fin templates</td><td>What fins if it wanders in yaw?</td></tr>"
             "<tr><td><code>rigid_glider\\test_plan.py</code></td><td>Makes <code>drop_test_sheet.pdf</code></td><td>What do I check and write down on test day?</td></tr>"
             "<tr><td><code>tools\\drop_tracker\\drop_tracker.html</code></td><td>Offline page: click the glider in a phone video</td><td>How deep was the swoop, how far did it glide?</td></tr>"
             "<tr><td><code>cfd\\</code> (runs in WSL)</td><td>OpenFOAM checks of AVL against a tunnel plate</td><td>Can AVL's moment be trusted? Not used on our wings.</td></tr></table>")
    P.append("<h3>Folders</h3>" + pre(
        "Codes\\\n  tools\\avl\\, tools\\drop_tracker\\        the programs (AVL, the video tracker)\n  rigid_glider\\                      all the Python scripts for the glider work\n"
        "  results\\runs\\rigid_glider\\         what each script wrote, one folder per study:\n"
        "      bird_outline_v4_swift\\   index.html, A_templates.pdf, outlines.csv  (Swift A, B, C)\n"
        "      pullout\\                 index.html, cases.csv                      (drop simulation)\n"
        "      fin_kit_v4\\              index.html, fin_templates.pdf               (fins)\n"
        "      test_plan\\               drop_test_sheet.pdf                          (test day)\n"
        "  research\\                          notes and data (plate data is read-only input)\n"
        "  cfd\\                               CFD notes and scripts (cases live in WSL)"))
    P.append("<div class='bad'><b>Never touch:</b> <code>data\\polars</code>, <code>results\\legacy</code>, <code>archive</code>. Never run a <code>rigid_glider</code> script from the project folder to practice: it overwrites that study's results. Practice in the copy.</div>")
    P.append("<p><b>To look at a finished study:</b> in File Explorer open <code>Codes\\results\\runs\\rigid_glider\\bird_outline_v4_swift\\</code> and double-click <code>index.html</code>. It opens in your browser.</p>")
    P.append(foot(2) + "</div>")
    # ------------------------------------------------------------------ page 3
    P.append("<div class='pg'><h2>Step 2: AVL by hand on Swift A</h2>"
             f"<p>AVL is a program you talk to with short commands. Swift A at 400 mm, tail bent 6 degrees, is already written as an AVL file: <code>practice_inputs\\SwiftA_foam400_bend6.avl</code> (made from the v4 geometry; its moment reference is the 37.0 mm CG). "
             "Type each line, then press <span class='k'>Enter</span>. A blank row means press Enter on its own. AVL does not care about capital letters.</p>")
    P.append(step_table([
        ("<code>.\\tools\\avl\\avl352.exe</code>", "The AVL banner, a menu, and the prompt <code>AVL   c&gt;</code>."),
        ("<code>plop</code> then <code>g</code> then a blank line", "Two small menus; the line <code>Graphics-enable flag F</code> means the plot window is now off (so AVL cannot hang on it)."),
        ("<code>load practice_inputs\\SwiftA_foam400_bend6.avl</code>", f"<code>Configuration: SwiftA_foam400_bend6</code> and the surfaces being built (Wing, Tail)."),
        ("<code>oper</code>", "The operating-point menu, prompt <code>.OPER (case 1/1)   c&gt;</code>."),
        ("<code>a c 0.3</code>", "Sets 'alpha is whatever gives CL = 0.3'. The table shows <code>A lpha -&gt; CL = 0.3000</code>."),
        ("<code>x</code>", "A few iteration lines, then <b>Total Forces</b> (the block below)."),
        ("<code>st</code> then a blank line", "<code>Enter filename, or &lt;return&gt; for screen output</code>: the blank line chooses the screen. Then <b>Stability-axis derivatives</b> (below)."),
        ("a blank line, then <code>quit</code>", "Back out of the menu, then AVL closes."),
    ]))
    P.append("<h3>The screen after <code>x</code> (real output, trimmed)</h3>")
    forces = lines(avl, "Vortex Lattice Output -- Total Forces", "Operation of run case")
    forces = "\n".join(l for l in forces.split("\n") if l.strip() and "----" not in l and not l.strip().startswith(("# ", "Standard axis", "Run case", "Mach", "CDvis", "CLff", "CYff", "Beta", "CZtot", "CDtot", "Configuration")))
    P.append(pre(forces, marks=[f"Alpha =   {alpha}", f"Cmtot =   {cmt}", f"CLtot =   {clt}"]))
    P.append("<h3>The screen after <code>st</code> (the neutral point is the last line)</h3>")
    der = lines(avl, "Stability-axis derivatives", "Operation of run case")
    dl = [l for l in der.split("\n") if l.strip()]
    i_na = next(i for i, l in enumerate(dl) if "Cna =" in l)
    der = "\n".join(dl[:i_na + 1] + [l for l in dl if "Neutral point" in l])
    P.append(pre(der, marks=[f"Xnp =   {xnp}", f"Cnb =   {cnb}", f"Clb =  {clb}"]))
    P.append(foot(3) + "</div>")
    # ------------------------------------------------------------------ page 4
    xnp_mm = float(xnp) * 1000
    P.append("<div class='pg'><h2>Step 2 (continued): read three numbers</h2>"
             "<table><tr><th>Number on screen</th><th>Value</th><th>Plain meaning</th></tr>"
             f"<tr><td><code>Xnp</code>, neutral point</td><td><b>{xnp} m = {xnp_mm:.1f} mm</b></td><td>Distance behind the root leading edge. The CG must sit <i>ahead</i> of this. (MODEL: AVL is inviscid; its neutral point can be off by a few percent of the chord.)</td></tr>"
             f"<tr><td><code>CLtot</code></td><td><b>{clt}</b></td><td>The lift coefficient you asked for with <code>a c 0.3</code>. Alpha came out as {alpha} degrees.</td></tr>"
             f"<tr><td><code>Cmtot</code></td><td><b>{cmt}</b></td><td>Pitching moment about the CG ({float(xref) * 1000:.1f} mm, the file's reference point). Near zero means the 6 degree tail bend balances this CG at this lift. Positive: nose-up left over; negative: nose-down.</td></tr>"
             f"<tr><td><code>Cref</code>, <code>Sref</code></td><td>{float(cref) * 1000:.1f} mm, {float(sref) * 1e6:,.0f} mm&sup2;</td><td>Chord and wing area AVL uses (MAC and wing area of Swift A at 400 mm).</td></tr>"
             f"<tr><td><code>Clb</code>, <code>Cnb</code></td><td>{clb}, {cnb}</td><td>Roll and yaw stiffness per radian. <code>Cnb</code> positive means it weathercocks; the project's rough floor is 0.004, and Swift A is below it (see the fin kit).</td></tr></table>")
    P.append("<div class='bad'><b>Do not use AVL's <code>MODE</code> menu.</b> Typing <code>mode</code> then <code>n</code> (calculate eigenmodes) makes AVL 3.52 crash on this machine. The same crash happens with the root-locus commands. The crash seen here:</div>")
    crash = rd("avl_mode_crash.txt").split("\n")
    k = next((i for i, l in enumerate(crash) if "SIGSEGV" in l or "Zero Velocity" in l), None)
    P.append(pre("\n".join(l.rstrip() for l in crash[max(0, k - 3):k + 6]) if k is not None else "(no crash text captured)"))
    P.append(f"<p class='small'>Exit code of that AVL run: {F.get('avl_mode_rc')} (non-zero = crashed). The project gets spiral and Dutch-roll numbers by writing AVL's system matrix with the <code>S</code> command and solving it in Python (see <code>fin_kit_v4.py</code>).</p>")
    P.append("<h3>Check that you can repeat it</h3>" + f"<p>Run <code>.\\tools\\avl\\avl352.exe</code> again, retype the same lines, and change one thing: <code>a c 0.5</code> instead of <code>a c 0.3</code>. Real result here: alpha {F['c05']['alpha']:.2f} degrees, CLtot {F['c05']['cl']:.5f}, <code>Xnp</code> {F['c05']['xnp_mm']:.2f} mm (against {xnp_mm:.2f} mm at CL 0.3: the neutral point belongs to the shape and hardly moves), <code>Cmtot</code> {F['c05']['cm']:+.5f} (no longer zero: this CG and bend balance only near CL 0.3).</p>")
    P.append("<p class='small'>Not run by me: AVL's other commands (geometry plot, Trefftz plot) open a graphics window; they were switched off in step 2 on purpose.</p>")
    P.append(foot(4) + "</div>")
    # ------------------------------------------------------------------ page 5
    bal = rd("balance.txt")
    P.append("<div class='pg'><h2>Step 3: from the neutral point to the CG and the nose weight</h2>"
             f"<p>This is arithmetic, done by <code>walk_balance.py</code> with the same functions the build uses. You type the neutral point you just read (<b>{xnp_mm:.3f}</b> mm) and the margin you want (0.10 = 10% of the chord).</p>")
    P.append(step_table([(f"<code>py -B rigid_glider\\walk_balance.py {xnp_mm:.3f} 0.10</code>", "Five labelled steps (below). Nothing is written to disk.")]))
    P.append(pre(bal, marks=["= 36.972 mm behind the root leading edge", "= 1.207 g", "9.770 g"]))
    P.append("<table><tr><th>What happened</th><th>In words</th></tr>"
             "<tr><td>Steps 1-2</td><td>The CG sits one tenth of a chord ahead of the neutral point: 41.713 &minus; 4.741 = <b>36.97 mm</b>, which the build files round to <b>37.0 mm</b> (v4's <code>outlines.csv</code> lists 23.1 mm at 250 mm span; &times;1.6 = 37.0).</td></tr>"
             "<tr><td>Step 3</td><td>The foam sheet and the printed rail alone balance at 43.95 mm, <i>behind</i> the target. So weight must go forward.</td></tr>"
             "<tr><td>Step 4</td><td>Weight needed at the nose hole = M0 &times; (target &minus; cg0) / (hole &minus; target). Put <b>1.21 g</b> at the nose hole, 12% of the 9.77 g all-up mass.</td></tr>"
             "<tr><td>Step 5</td><td>A first guess at the drop: 3 &times; wing loading / (air density &times; g &times; CL). <b>4.55 m</b> if the lift coefficient stays at 0.3.</td></tr></table>")
    P.append("<p class='note'>Where the sheet's depth range <b>5 to 18 m</b> for Swift A foam 400 comes from: the formula gives the low end, 4.55 m. The high end multiplies it by the pull-out study's simulation-to-formula ratio for the light foam bird "
             f"({F['ratio_foam']:.2f}) and by {F['eff50']:.2f} for a tail only half as effective as AVL says: 4.55 &times; {F['ratio_foam']:.2f} &times; {F['eff50']:.2f} = {4.55 * F['ratio_foam'] * F['eff50']:.1f} m. "
             "Swift A is <b>not</b> simulated by the pull-out program (it handles the flat kit wings and an older bird outline). So 5 to 18 m is a formula range, a MODEL estimate, not a prediction of a drop.</p>")
    P.append("<p class='small'>Foam sheet mass assumes 300 g/m&sup2; foamboard (ASSUMED, to be weighed). The printed rail and the nose hole position come from the v4 build. Your real glider will differ: weigh it and measure the balance point on test day (step 6).</p>")
    P.append(foot(5) + "</div>")
    # ------------------------------------------------------------------ page 6
    t6, t0 = rd("pullout_tab6.txt"), rd("pullout_tab0.txt")
    P.append("<div class='pg'><h2>Step 4: the pull-out simulation, one drop</h2>"
             "<p>The simulation drops a wing nose-down from rest and follows it for 30 seconds in still air (pitch only). It knows the three flat kit wings: <code>F45_span250</code>, <code>F60_span250</code>, <code>B45_span400</code>. "
             "The command takes three inputs: the wing, the CG (mm behind the root leading edge; a whole-mm kit setting) and the tab angle (degrees, trailing edge up).</p>")
    P.append(step_table([
        ("<code>py -B rigid_glider\\walk_pullout_one.py F45_span250 39 6</code>", f"Three lines of progress, then the result below. About {F['pullout_tab6_s']:.0f} seconds."),
        ("<code>py -B rigid_glider\\walk_pullout_one.py F45_span250 39 0</code> (same wing and CG, tab 0)", "The same drop with no nose-up tab: it dives."),
        (f"Open <code>results\\runs\\pullout_one\\trajectory.svg</code> (double-click)", "A side-view plot of the drop. <code>one_drop.csv</code> holds the headline numbers; <code>trajectory.csv</code> the path."),
    ]))
    P.append("<h3>Tab 6 degrees</h3>" + pre(t6.split("\n3. files")[0], marks=["SETTLES TO A GLIDE", "14.1 m"]))
    P.append("<h3>Tab 0 degrees</h3>" + pre(t0.split("\n3. files")[0], marks=["DIVES"]))
    P.append("<table><tr><th>Line</th><th>How to read it</th></tr>"
             "<tr><td>VERDICT</td><td><code>SETTLES TO A GLIDE</code> = it recovered. <code>DIVES</code> = it never pulled out. Other words: TUMBLES, DEEP STALL, NOT SETTLED.</td></tr>"
             "<tr><td>first-swoop depth</td><td>Height lost before the first level-off: <b>14.1 m</b> for this kit wing. Compare with the formula line (a cross-check).</td></tr>"
             "<tr><td>static margin</td><td>CG ahead of the neutral point, in % of the chord. Under 3% the model says it cannot tell.</td></tr></table>")
    P.append("<p><b>What you change:</b> only the command line. Second number = CG (try 41, then 36). Third = tab angle (try 2, 4, 8). First word = which wing. "
             "The same result as the project's full study: its <code>cases.csv</code> row for this wing, CG 39 and tab 6 reads 14.1 m, margin 8.59%, SETTLES TO A GLIDE.</p>")
    P.append(f"<p class='small'><b>Optional, the whole study in about {F['pullout_quick_s']:.0f} seconds:</b> <code>py -B rigid_glider\\pullout_sim.py --out results\\runs\\pullout_practice --quick</code>, then open <code>results\\runs\\pullout_practice\\index.html</code>. Leave off <code>--quick</code> for the full study; use <code>--out</code> every time so it writes to the practice folder.</p>")
    P.append(foot(6) + "</div>")
    # ------------------------------------------------------------------ page 7
    P.append("<div class='pg'><h2>Step 5: making the parts</h2>"
             "<p>The PDFs to print are ready in <code>print_these\\</code> (copied from the project). Printing is the normal route; regenerating is only needed if the outline changes.</p>")
    P.append("<h3>Print and cut</h3>")
    P.append(step_table([
        ("In File Explorer open <code>print_these</code>. Double-click <code>A_templates.pdf</code>.", "A 4-page PDF opens in your PDF viewer: foam 400 mm (page 1 outline, page 2 notes), then printed 250 mm (pages 3 and 4)."),
        ("Press <span class='k'>Ctrl</span>+<span class='k'>P</span>. Choose your printer and paper <b>Letter</b>. Set the scale to <b>100% / Actual size</b>, <b>not</b> 'Fit to page'. Print pages 1 and 2. Then lay a ruler on the <b>100 mm scale bar</b> (top right of each sheet): it must measure exactly 100 mm; if not, print again.", "The print dialog; the exact words vary by PDF viewer (not checked on your viewer)."),
        ("Cut out the solid outline on page 1. It is <b>one half</b> with its fan tail. Flip it on the centreline to draw the other half on the foam.", "A swept half wing with a fan tail on the right (picture)."),
        ("Mark the hinge (orange dashed), the wrist line and the body edge (markers only: do not cut them). Bend the tail up with the 6 degree wedge on page 2.", "The wedge on the notes page; its note reads 'Score the top face, bend up to the gauge, glue the crease.'"),
    ]))
    P.append("<div class='two'><div>" + img("template_a_p1.png", "54mm") + "<p class='small'>A_templates.pdf, page 1: one half of Swift A, foam 400 mm, with the 100 mm bar (real render).</p></div>"
             "<div>" + img("fin_p1.png", "54mm") + "<p class='small'>fin_templates.pdf, page 1: fin shapes for foam 400 mm. Orange dashed line = attachment line; hatched strip = glue or tape tab.</p></div></div>")
    P.append("<h3>Fins (only if it wanders in yaw)</h3><p>Print <code>print_these\\fin_templates.pdf</code> the same way (100%, check the bar). Page 1 is foam 400 mm, page 2 printed 250 mm. Each box names the fin: for example <i>swift A tip fin, cut 2, S3</i>. The sheet <code>drop_test_sheet.pdf</code> says to try <b>tip fins, S3, foam 400</b> first.</p>")
    P.append(f"<h3>Optional: regenerate the files (inside the practice folder)</h3>"
             f"<p><code>py -B rigid_glider\\bird_outline_v4_swift.py --out results\\runs\\v4_practice</code> took {F['v4_regen_s'] / 60:.0f} minutes here (52 serial AVL runs) and wrote {', '.join(F['v4_regen_files'])} plus the cut files and tables. "
             f"<code>py -B rigid_glider\\fin_kit_v4.py</code> took {F['fin_regen_s'] / 60:.0f} minutes and writes <code>results\\runs\\rigid_glider\\fin_kit_v4\\fin_templates.pdf</code> inside the practice copy.</p>"
             "<p class='bad'><b>Do not add <code>--quick</code> to the v4 script:</b> it writes the template PDFs, then stops with an error (<code>TypeError: 'NoneType' object is not subscriptable</code>). The full run finishes cleanly.</p>")
    P.append(foot(7) + "</div>")
    # ------------------------------------------------------------------ page 8
    P.append("<div class='pg'><h2>Step 6: mock test day, the sheet</h2>"
             "<p>Print <code>print_these\\drop_test_sheet.pdf</code> (2 pages, B5, one-sided). Page 1 is the plan; page 2 is the log. Before the first drop do the bench check on page 1 (tick the boxes): mass, CG, bend, warp, symmetry, hook.</p>"
             "<p class='note'><span class='ex'>EXAMPLE</span> Everything in the row below is <b>made up</b> to show how to fill it in. It is not a result.</p>")
    P.append("<table><tr><th>#</th><th>Article</th><th>CG mm</th><th>Bend deg</th><th>Height m</th><th>Outcome</th><th>Swoop depth m</th><th>Glide dist m</th><th>Clip</th><th>Note</th></tr>"
             "<tr><td>1</td><td>swift A foam 400</td><td>37.0</td><td>6</td><td>10</td><td>D</td><td>-</td><td>-</td><td>IMG_0001</td><td>dived, no recovery</td></tr>"
             "<tr><td>2</td><td>swift A foam 400</td><td>39.4</td><td>6</td><td>10</td><td>G</td><td>6.5</td><td>14</td><td>IMG_0002</td><td>clean glide, slight turn right</td></tr></table>"
             "<p class='small'><span class='ex'>EXAMPLE</span> rows. Outcome codes: <b>G</b> pulls out and glides, <b>PS</b> pulls out then porpoises or stalls, <b>D</b> dives, <b>T</b> tumbles, <b>SP</b> spirals or turns away, <b>RF</b> release fault, <b>H</b> hand glide. "
             "The starting CG 37.0 mm, bend 6 degrees and depth range 5 to 18 m for Swift A foam 400 are the real settings from page 1 of the sheet (step 3).</p>")
    P.append("<h3>What to do between drops (the trim guide on page 1 of the sheet)</h3><table><tr><th>What happened</th><th>Adjustment</th></tr>"
             "<tr><td>Dives / no recovery</td><td>CG aft one step, or bend up one gauge step</td></tr>"
             "<tr><td>Porpoise / stall / growing oscillation</td><td>CG forward one step, or bend down one gauge step</td></tr>"
             "<tr><td>Turns one way</td><td>Fix warp first, then a small trailing-edge trim</td></tr>"
             "<tr><td>Wanders or spirals</td><td>Tip fins or a small fin</td></tr>"
             "<tr><td>Swoop deeper than predicted</td><td>Check bend on the gauge and CG on the balance</td></tr></table>"
             "<p><b>A step</b> is 2 to 3 mm of CG (JUDGEMENT, from the sheet) or the next angle on the field-pack gauge (2, 2.5, 4, 5, 6, 7.5, 8 degrees). <b>Change one thing between drops.</b> "
             "Example from the dive in row 1: move the CG one step aft (about 2.4 mm: 37.0 to 39.4 mm) and drop again, as in row 2.</p>")
    bm = rd("balance_m08.txt")
    P.append("<p><b>Re-running the arithmetic for a new CG.</b> A smaller margin puts the CG further back. <code>py -B rigid_glider\\walk_balance.py 41.713 0.08</code> (8% margin, the back end of the sheet's model range) gives:</p>")
    P.append(pre("\n".join(l for l in bm.split("\n") if l.strip().startswith(("STEP 2", "41.713", "target", "weight", "= ", "all-up")))))
    P.append("<p class='small'>The model range is 8% to 15% margin (CG 34.6 to 37.9 mm). Stepping beyond the back end (past 37.9 mm) goes outside the range the model is used for: do it only while it still pulls out, as the sheet says. "
             "Then <b>measure the real CG on the balance</b> and write it in the CG column; do not copy the plan.</p>")
    P.append(foot(8) + "</div>")
    # ------------------------------------------------------------------ page 9
    sst = rd("tracker_selftest.txt")
    P.append("<div class='pg'><h2>Step 6 (continued): the drop tracker</h2>"
             "<div class='two wide'><div>" + img("tracker_demo.png", "100%") + "<p class='small'>The tool in its own demo mode (a made-up path): the numbers shown are demo values, not a drop. Real screenshot.</p></div><div>"
             "<p>It is one web page that works offline. Your video never leaves the browser.</p>"
             "<p><b>Open it:</b> in File Explorer double-click <code>tools\\drop_tracker\\drop_tracker.html</code> in the practice folder.</p>"
             f"<p class='small'>Self-test (add <code>?selftest</code> to the address): <b>{esc(sst)}</b>.</p>"
             "<table><tr><th>#</th><th>Click this</th></tr>"
             "<tr><td class='n'>1</td><td><b>Choose File</b> (Video box), pick the phone video. Set <b>Frame rate</b> to 30 or 60 (buttons).</td></tr>"
             "<tr><td class='n'>2</td><td><b>Set scale</b>. Click both ends of the known reference (a marked pole), then type its length in metres in <b>Reference length (m)</b>.</td></tr>"
             "<tr><td class='n'>3</td><td>Optional: <b>Set level line</b>, click two points on the horizon or ground.</td></tr>"
             "<tr><td class='n'>4</td><td><b>Track</b>. Step to the release frame (<span class='k'>,</span> <span class='k'>.</span> or <b>&lt; Frame</b> / <b>Frame &gt;</b>). Click the glider. The video jumps N frames; click it again, every step, to the end.</td></tr>"
             "<tr><td class='n'>5</td><td>Click a point in the list, then tag it: <b>Release</b>, <b>Lowest</b>, <b>Level-off</b>, <b>Glide start</b>, <b>Glide end</b>. (<b>Lowest: auto</b> picks the deepest.)</td></tr>"
             "<tr><td class='n'>6</td><td>Read <b>Results</b>: swoop depth, glide distance, glide ratio and angle.</td></tr>"
             "<tr><td class='n'>7</td><td>Fill <b>Drop #</b>, <b>Article</b>, <b>CG (mm)</b>, <b>Bend (deg)</b>, <b>Drop height (m)</b>, <b>Outcome</b> (G, PS, D, T, SP, RF, H), <b>Note</b>. Click <b>Download summary CSV</b> (and <b>Download points CSV</b>).</td></tr></table>"
             "<p><b>What you get:</b> two files named after the video, <code>&lt;video&gt;_summary.csv</code> and <code>&lt;video&gt;_points.csv</code>. The summary is one row whose columns match the log on the sheet:</p>"
             "<pre>drop #,article,CG,bend,height,outcome,swoop depth,glide dist,clip,note</pre>"
             "<p><span class='ex'>EXAMPLE</span> row: <code>2,swift A foam 400,39.4,6,10,G,6.5,14,IMG_0002,clean glide</code>. <b>Copy summary row</b> puts it on the clipboard to paste into a spreadsheet.</p>"
             "<p class='small'>Limits: the scale holds only in the plane of the reference; keep the camera side-on and the reference at the flight path's distance. Heading cannot be read from one camera.</p>"
             "</div></div>")
    P.append("<p class='small'>Not run by me: the clicks above (a person has to click a video). They are copied from the tool's own <b>Steps</b> panel and its button names; the self-test and demo page were run with headless Edge.</p>")
    P.append(foot(9) + "</div>")
    # ------------------------------------------------------------------ page 10
    P.append("<div class='pg'><h2>Step 7: CFD (half a page)</h2>"
             "<p><b>What it is for.</b> Checking AVL against the wind-tunnel plate (a flat rectangular plate, Re 60,000) to see whether AVL's pitching moment and centre of pressure can be trusted. "
             "It is <b>not</b> run on our wings, and it is not used for balance or pull-out. The runs live in the WSL distro <code>MMAV-CFD</code>; do not start or stop one while another is running.</p>"
             "<p><b>Status, stop and start.</b> Copied exactly from <code>cfd\\validation_sweep\\refined_8deg\\README.md</code> (PowerShell; <b>I did not run them</b>):</p>")
    P.append(pre("Status: wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/ref/ref_status_a08.sh\n"
                 "Stop:   wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/ref/ref_stop_a08.sh\n"
                 "Start:  Start-Process -WindowStyle Hidden wsl.exe -ArgumentList '-d','MMAV-CFD','-u','cfd','--','bash','/home/cfd/run/ref/ref_launch_a08.sh'"))
    P.append("<p class='small'>The README adds: stop affects only that run; relaunching with the start command resumes from the latest complete time; a hidden Windows <code>wsl.exe</code> keeps the distro alive, so killing it, sleeping, restarting or logging off stops the run.</p>")
    P.append("<p><b>Open a case in ParaView</b> (installed at <code>C:\\Program Files\\ParaView 5.12.0</code>; the path exists, I did not start it):</p>")
    P.append(step_table([
        ("Start menu, type <code>ParaView</code>, Enter.", "The ParaView window."),
        ("<b>File</b> &gt; <b>Open</b>. In the file-name box type <code>\\\\wsl.localhost\\MMAV-CFD\\home\\cfd\\run\\vp_ref_a06\\case.foam</code> (swap in another case name) and press Enter.", "The case appears in the Pipeline Browser. (Case name from the README; the <code>case.foam</code> file is not checked here.)"),
        ("Click the green <b>Apply</b> button.", "The plate's mesh appears; pick the field (U, p) in the toolbar."),
    ]))
    P.append("<p class='small'>The ParaView clicks are standard ParaView menus, not run by me. The 6 degree case is finished; the 8 degree case is the one being run or stopped.</p>")
    P.append("<h2 style='margin-top:4mm'>Step 8: cheat sheet, every command</h2>")
    P.append(pre(
        "PowerShell, once:\n"
        f"  cd \"{CODES}\"\n  py -B rigid_glider\\make_practice_folder.py {DOC_PRACTICE}\n  cd {DOC_PRACTICE}          (stay here)\n\n"
        "AVL (type each line + Enter):\n  .\\tools\\avl\\avl352.exe\n  plop / g / (blank)\n  load practice_inputs\\SwiftA_foam400_bend6.avl\n  oper / a c 0.3 / x / st / (blank) / (blank) / quit      (never: mode, n)\n\n"
        f"Balance:    py -B rigid_glider\\walk_balance.py {xnp_mm:.3f} 0.10\n"
        "Pull-out:   py -B rigid_glider\\walk_pullout_one.py F45_span250 39 6      (wing, CG mm, tab deg)\n"
        "            py -B rigid_glider\\pullout_sim.py --out results\\runs\\pullout_practice --quick\n"
        "Regenerate: py -B rigid_glider\\bird_outline_v4_swift.py --out results\\runs\\v4_practice      (no --quick)\n"
        "            py -B rigid_glider\\fin_kit_v4.py\n"
        "Print:      print_these\\A_templates.pdf, fin_templates.pdf, drop_test_sheet.pdf   (100%, check the bar)\n"
        "Tracker:    double-click tools\\drop_tracker\\drop_tracker.html\n"
        "CFD:        status / stop / start commands on this page (do not run unless you mean to)"))
    P.append(foot(10) + "</div></body></html>")
    return "\n".join(P)


def to_pdf(h: Path, pdf: Path):
    if pdf.exists():
        pdf.unlink()
    with tempfile.TemporaryDirectory() as prof:
        subprocess.run([edge(), "--headless", "--disable-gpu", "--no-first-run", f"--user-data-dir={prof}", "--no-pdf-header-footer", f"--print-to-pdf={pdf}", h.as_uri()],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=180)
    d = pdf.read_bytes()
    return len(re.findall(rb"/Type\s*/Page(?![s\w])", d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, default=None, help="practice folder to build and run every step in (must be outside the project)")
    ap.add_argument("--extra", type=Path, default=None, help="practice folder (already made): run only the extra a c 0.5 check, then rebuild the PDF")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if a.run:
        if ROOT in a.run.resolve().parents:
            sys.exit("the practice folder must be outside the project folder")
        facts = run_steps(a.run.resolve())
        print("steps run:", json.dumps(facts, indent=1)[:600])
    if a.extra:
        print("a c 0.5 check:", run_extra(a.extra.resolve()))
    h = OUT / "tools_walkthrough.html"
    h.write_text(build_html(), encoding="utf-8")
    n = to_pdf(h, OUT / "tools_walkthrough.pdf")
    print("pages:", n, OUT / "tools_walkthrough.pdf")


if __name__ == "__main__":
    main()
