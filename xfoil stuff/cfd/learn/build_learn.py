#!/usr/bin/env python3
"""Builds cfd/learn/index.html. Quotes real files from learn_src/ (copied read-only from the practice cases and
from /home/cfd/run/vp_a4_coarse) and real numbers from cfd/validation_sweep/sweep_results.csv at build time."""
import base64, csv, html, os, re, datetime
NL = chr(10)

SP = os.path.dirname(os.path.abspath(__file__))          # cfd/learn
SRC = os.path.join(SP, "learn_src")                      # real file excerpts copied from the distro
CODES = os.path.normpath(os.path.join(SP, "..", ".."))   # .../MAV-2026/Codes
OUT = os.path.join(SP, "index.html")
os.makedirs(os.path.dirname(OUT), exist_ok=True)


def rd(*p):
    with open(os.path.join(SRC, *p), encoding="utf-8", errors="replace", newline="") as f:
        return f.read().replace("\r\n", "\n")


def body(*p):
    """dictionary file without the banner/FoamFile header and trailing rule"""
    t = rd(*p).split("\n")
    i = next((k for k, l in enumerate(t) if l.startswith("// * * *")), 0)
    t = t[i + 1:]
    t = [l for l in t if not l.startswith("// ****")]
    while t and not t[0].strip():
        t.pop(0)
    while t and not t[-1].strip():
        t.pop()
    return "\n".join(t)


def pre(text, cap=None):
    return "<pre>" + html.escape(text.rstrip("\n")) + "</pre>"


def fig(svg, cap):
    return f"<figure>{svg}<figcaption>{cap}</figcaption></figure>"


def png(path):
    with open(path, "rb") as f:
        return "data:image/png;base64," + base64.b64encode(f.read()).decode()


def lines(text, a, b):
    return "\n".join(text.split("\n")[a - 1:b])


# ---------------------------------------------------------------- real material
cav = {k: body("cavity", *k.split("/")) for k in ("system/controlDict", "system/fvSchemes", "system/fvSolution",
                                                    "0/U", "0/p", "constant/transportProperties")}
cav_bm = body("cavity", "system", "blockMeshDict")
cav_ico = rd("cavity", "01_icoFoam.log")
cav_chk = rd("cavity", "01_checkMesh.log")
cav_block = rd("cavity", "01_blockMesh.log")
ico_blocks = re.findall(r"(Time = [0-9.e-]+\n\nCourant.*?ExecutionTime[^\n]*\n)", cav_ico, re.S)
ico_first = ico_blocks[0]
ico_second = ico_blocks[1]
ico_last = ico_blocks[-1]

pl = {k: body("plate", *k.split("/")) for k in ("system/controlDict", "system/fvSchemes", "system/fvSolution",
                                                  "0/U", "0/p", "constant/transportProperties",
                                                  "constant/turbulenceProperties")}
pl_bm = rd("plate", "system", "blockMeshDict")
pl_chk = rd("plate", "log.checkMesh")
pl_log = rd("plate", "log.pimpleFoam")
coef = rd("plate", "coefficient.dat").split("\n")
coefm = rd("plate", "coefficientMid.dat").split("\n")
crow = [l for l in coef if l and not l.startswith("#")]
mrow = [l for l in coefm if l and not l.startswith("#")]
last_c = crow[-1].split()
last_m = mrow[-1].split()
cl_end, cd_end, cm_end = float(last_c[4]), float(last_c[1]), float(last_c[7])
cm_mid = float(last_m[7])
chk_geo = "\n".join(pl_chk.split("\n")[pl_chk.split("\n").index("Checking geometry...") :][:15]) if "Checking geometry..." in pl_chk else ""
chk_stats = "\n".join(l for l in pl_chk.split("\n") if re.search(r"^\s+(points|faces|cells|boundary patches):", l))
pl_sample = []
for i in (0, 20, 100, 400, 1000, len(crow) - 1):
    c = crow[i].split()
    pl_sample.append((float(c[0]), float(c[4]), float(c[1]), float(c[7])))
pl_co = [l for l in pl_log.split("\n") if l.startswith("Courant")]
pl_dt = [l for l in pl_log.split("\n") if l.startswith("deltaT")]
pl_steps = len([l for l in pl_log.split("\n") if l.startswith("Time =")])

sw = {
    "fvSolution": body("sweep", "system", "fvSolution"),
    "decompose": body("sweep", "system", "decomposeParDict"),
    "U": body("sweep", "0.orig", "U"),
    "p": body("sweep", "0.orig", "p"),
    "fvSchemes": body("sweep", "system", "fvSchemes"),
    "controlDict": body("sweep", "system", "controlDict"),
    "turb": body("sweep", "turbulenceProperties"),
    "trans": body("sweep", "transportProperties"),
}
sw_tail = rd("sweep", "log.pimpleFoam.tail")
sw_tail = "\n".join(sw_tail.split("\n")[-42:])
sw_chk = "\n".join(l for l in rd("sweep", "log.checkMeshDefault").split("\n")
                   if re.search(r"cells:|Mesh OK|non-orthogonality|skewness|aspect", l))
sw_ls = " ".join(rd("sweep", "ls_case.txt").split())
sw_proc = " ".join(rd("sweep", "ls_proc0.txt").split())

# sweep results (read at build time)
rows = list(csv.DictReader(open(os.path.join(CODES, "cfd", "validation_sweep", "sweep_results.csv"), encoding="utf-8")))


def f(x, n=4):
    try:
        return f"{float(x):.{n}f}"
    except Exception:
        return "-"


def pct(a, b):
    try:
        return f"{100 * float(a) / float(b):+.1f}%"
    except Exception:
        return "-"


sweep_tbl = ["<table><tr><th>alpha (deg)</th><th>status</th><th>CFD window (t)</th><th>CL CFD</th><th>CL measured</th><th>diff</th>"
             "<th>CD CFD</th><th>CD measured</th><th>diff</th><th>CM CFD</th><th>CM measured</th><th>diff</th></tr>"]
for r in rows:
    if r["CL_mean"] == "":
        continue
    st = r["status"] if r["status"] != "partial" else "PARTIAL (run in progress)"
    sweep_tbl.append(
        f"<tr><td class='n'>{float(r['alpha_deg']):g}</td><td>{html.escape(st)}</td><td>{f(r['window_start'],1)}-{f(r['window_end'],1)}</td>"
        f"<td class='n'>{f(r['CL_mean'])}</td><td class='n'>{f(r['meas_CL_mean'])}</td><td class='n'>{pct(r['dCL_cfd_minus_meas'], r['meas_CL_mean'])}</td>"
        f"<td class='n'>{f(r['CD_mean'])}</td><td class='n'>{f(r['meas_CD_mean'])}</td><td class='n'>{pct(r['dCD_cfd_minus_meas'], r['meas_CD_mean'])}</td>"
        f"<td class='n'>{f(r['CM_mean'],5)}</td><td class='n'>{f(r['meas_CM_mean'],5)}</td><td class='n'>{pct(r['dCM_cfd_minus_meas'], r['meas_CM_mean'])}</td></tr>")
sweep_tbl.append("</table>")
sweep_tbl = "".join(sweep_tbl)
built = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

img_forces = png(os.path.join(SRC, "plate", "forces.png"))
img_speed = png(os.path.join(SRC, "plate", "speed.png"))
slice_path = os.path.join(CODES, "cfd", "validation_scoping", "slice_yMid_coarse.png")
img_slice = png(slice_path) if os.path.exists(slice_path) else None

# ---------------------------------------------------------------- SVG helpers (always-dark palette)
C = dict(bg="#14181d", panel="#1c2229", line="#333c46", ink="#dde3ea", muted="#9aa7b4", link="#7fb4ff", accent="#e89a55",
         thead="#262e37", bad="#ff8a80", green="#6cc08b", purple="#c39bff")


def svg(w, h, inner):
    return (f'<svg viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg" role="img" style="max-width:100%;height:auto" '
            f'font-family="Segoe UI,system-ui,sans-serif" font-size="13">{inner}</svg>')


def box(x, y, w, h, text, fill=C["panel"], stroke=C["line"], color=C["ink"], size=13, sub=None):
    s = f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="4" fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>'
    s += f'<text x="{x + w / 2}" y="{y + h / 2 + (0 if not sub else -5)}" fill="{color}" font-size="{size}" text-anchor="middle" dominant-baseline="middle">{html.escape(text)}</text>'
    if sub:
        s += f'<text x="{x + w / 2}" y="{y + h / 2 + 11}" fill="{C["muted"]}" font-size="11" text-anchor="middle" dominant-baseline="middle">{html.escape(sub)}</text>'
    return s


def arrow(x1, y1, x2, y2, color=C["muted"], w=1.6):
    return (f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{w}" marker-end="url(#ah)"/>')


DEFS = (f'<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M0,0 L10,5 L0,10 z" fill="{C["muted"]}"/></marker></defs>')


def text(x, y, s, color=C["ink"], size=13, anchor="start", weight="normal"):
    return f'<text x="{x}" y="{y}" fill="{color}" font-size="{size}" text-anchor="{anchor}" font-weight="{weight}">{html.escape(s)}</text>'


# 1. pipeline diagram
svg_pipe = svg(980, 150, DEFS +
               box(10, 40, 210, 70, "folder of text files", sub="the case: settings + starting values") +
               arrow(220, 75, 290, 75) +
               box(290, 40, 210, 70, "command-line programs", sub="blockMesh, icoFoam, pimpleFoam ...") +
               arrow(500, 75, 570, 75) +
               box(570, 40, 190, 70, "folders of numbers", sub="time folders, postProcessing/") +
               arrow(760, 75, 800, 75) +
               box(800, 40, 170, 70, "a viewer (ParaView)", sub="separate program", stroke=C["accent"], color=C["accent"]) +
               text(10, 22, "You edit text  \u2192  you run commands  \u2192  you read numbers or plot them. There is no window to click.", C["muted"], 13))

# 3. folder anatomy
svg_tree = svg(980, 330, DEFS +
               box(10, 10, 140, 34, "my_case/", stroke=C["accent"], color=C["accent"]) +
               "".join(
                   box(190, 10 + i * 46, 150, 34, n) + text(355, 32 + i * 46, d, C["muted"], 13)
                   for i, (n, d) in enumerate([
                       ("0/", "starting values and boundary conditions: one file per field (U = velocity, p = pressure)"),
                       ("constant/", "things that never change: the mesh (constant/polyMesh/), fluid properties, turbulence choice"),
                       ("system/", "how to run: controlDict, fvSchemes, fvSolution, blockMeshDict, decomposeParDict ..."),
                       ("5/  10/  20/ (time folders)", "saved results at those times, same field files as 0/ (names are simulation time)"),
                       ("postProcessing/", "numbers written while running, e.g. forceCoeffs1/0/coefficient.dat"),
                       ("processor0/ ... processor11/", "only in parallel runs: the case split into 12 pieces, one per CPU core"),
                       ("log.* , case.foam, VTK/", "log files you save; empty marker for ParaView; exported results"),
                   ])) +
               "".join(f'<line x1="80" y1="44" x2="80" y2="{27 + i * 46}" stroke="{C["line"]}"/><line x1="80" y1="{27 + i * 46}" x2="190" y2="{27 + i * 46}" stroke="{C["line"]}"/>' for i in range(1, 7)) +
               f'<line x1="80" y1="44" x2="80" y2="27" stroke="{C["line"]}"/><line x1="80" y1="27" x2="190" y2="27" stroke="{C["line"]}"/>')

# 4b. finite-volume cell
svg_cell = svg(980, 300, DEFS +
               "".join(f'<rect x="{300 + i * 130}" y="{50 + j * 100}" width="130" height="100" fill="{C["panel"]}" stroke="{C["line"]}" stroke-width="1.5"/>' for i in range(3) for j in range(2) if not (i == 1 and j == 0)) +
               f'<rect x="430" y="50" width="130" height="100" fill="#2c2418" stroke="{C["accent"]}" stroke-width="2.5"/>' +
               text(495, 92, "cell P", C["accent"], 16, "middle", "bold") + text(495, 112, "one value of u, p stored here", C["muted"], 11, "middle") +
               text(365, 105, "cell W", C["muted"], 13, "middle") + text(625, 105, "cell E", C["muted"], 13, "middle") +
               text(495, 215, "cell S (below)", C["muted"], 13, "middle") +
               arrow(512, 100, 600, 100, C["accent"], 2.6) + arrow(478, 100, 390, 100, C["accent"], 2.6) + arrow(495, 150, 495, 205, C["accent"], 2.6) +
               text(10, 25, "Space is chopped into small cells. Each cell stores ONE value of each field (at its centre).", C["ink"], 14) +
               text(10, 52, "Orange arrows: the flux", C["accent"], 13) + text(10, 70, "through each face of cell P.", C["accent"], 13) +
               text(10, 100, "Flux out of P through its east", C["muted"], 13) + text(10, 118, "face = flux INTO cell E through", C["muted"], 13) +
               text(10, 136, "the same face. Nothing is lost", C["muted"], 13) + text(10, 154, "or invented at a face.", C["muted"], 13) +
               text(740, 70, "Cell P balance:", C["ink"], 13, "start", "bold") + text(740, 90, "rate of change in P", C["muted"], 13) +
               text(740, 108, "= sum over faces of", C["muted"], 13) + text(740, 126, "(flux in \u2212 flux out)", C["muted"], 13) +
               text(740, 144, "+ sources", C["muted"], 13))

# 4c. linear vs upwind sketch (schematic, not data)
xs = [60 + i * 70 for i in range(9)]
vals = [1, 1, 1, 1, 0, 0, 0, 0, 0]
pts_lin = "60,60 130,60 200,60 270,52 340,100 410,152 480,148 550,150 620,150"  # sketched overshoot/wiggle
pts_up = "60,60 130,60 200,60 270,64 340,86 410,120 480,142 550,150 620,150"
svg_scheme = svg(980, 230, DEFS +
                 f'<polyline points="60,60 340,60 340,150 620,150" fill="none" stroke="{C["muted"]}" stroke-width="2" stroke-dasharray="6 4"/>' +
                 f'<polyline points="{pts_lin}" fill="none" stroke="{C["bad"]}" stroke-width="2.4"/>' +
                 f'<polyline points="{pts_up}" fill="none" stroke="{C["green"]}" stroke-width="2.4"/>' +
                 "".join(f'<circle cx="{x}" cy="{y}" r="3.5" fill="{C["ink"]}"/>' for x, y in [(60 + i * 70, 60 if i < 5 else 150) for i in range(9)]) +
                 text(640, 66, "dashed: the true profile (a sharp step carried by the flow)", C["muted"], 13) +
                 text(640, 100, "red: a second-order (linear) scheme tends to", C["bad"], 13) + text(640, 118, "overshoot and wiggle at the step", C["bad"], 13) +
                 text(640, 152, "green: first-order upwind never wiggles", C["green"], 13) + text(640, 170, "but smears the step out", C["green"], 13) +
                 text(60, 205, "SCHEMATIC SKETCH of the trade-off, not output from any run. Dots = cell values; the flow goes left to right.", C["muted"], 12))

# 4d. PISO loop
svg_piso = svg(980, 190, DEFS +
               box(10, 40, 190, 70, "1. predict velocity", sub="momentum eq. with OLD pressure") +
               arrow(200, 75, 245, 75) +
               box(245, 40, 210, 70, "2. solve pressure eq.", sub="p that makes new flux divergence-free") +
               arrow(455, 75, 500, 75) +
               box(500, 40, 210, 70, "3. correct velocity + flux", sub="U = HbyA \u2212 rAU \u00b7 grad p") +
               arrow(710, 75, 755, 75) +
               box(755, 40, 215, 70, "4. repeat 2-3", sub="nCorrectors times, then next time step") +
               f'<path d="M860,110 C860,170 350,170 350,112" fill="none" stroke="{C["accent"]}" stroke-width="1.8" marker-end="url(#ah)"/>' +
               text(10, 22, "One time step of icoFoam (PISO). pimpleFoam adds an optional outer loop around the same block.", C["muted"], 13) +
               text(420, 182, "loop back", C["accent"], 12))

# 4f. Courant
svg_co = svg(980, 160, DEFS +
             "".join(f'<rect x="{80 + i * 120}" y="50" width="120" height="60" fill="{C["panel"]}" stroke="{C["line"]}" stroke-width="1.5"/>' for i in range(6)) +
             f'<circle cx="140" cy="80" r="9" fill="{C["accent"]}"/>' + arrow(152, 80, 240, 80, C["accent"], 2.6) +
             text(80, 135, "cell size \u0394x", C["muted"], 13) + text(200, 135, "small Courant number: the fluid moves a fraction of one cell per step", C["accent"], 13) +
             f'<circle cx="440" cy="80" r="9" fill="{C["bad"]}"/>' + arrow(452, 80, 700, 80, C["bad"], 2.6) +
             text(10, 22, "Courant number = (speed \u00d7 time step) / cell size. Below 1: a particle crosses less than one cell in a step.", C["ink"], 14) +
             text(460, 150, "Co > 1: crosses several cells in one step (red)", C["bad"], 13, "start"))

# ---------------------------------------------------------------- HTML
CSS = (":root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37;--bad:#ff8a80}"
       "body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 'Segoe UI',system-ui,sans-serif}"
       "main{max-width:1100px;margin:0 auto;padding:32px 20px 64px;display:flex;flex-direction:column;gap:30px}"
       "h1{font-size:28px;margin:0}h2{font-size:21px;margin:0;border-bottom:1px solid var(--line);padding-bottom:6px}h3{font-size:16px;margin:8px 0 0;color:var(--accent)}p{margin:0;max-width:78ch}"
       "section{display:flex;flex-direction:column;gap:12px}"
       "figure{margin:0;background:var(--panel);border:1px solid var(--line);padding:12px}figure img{display:block;max-width:100%;height:auto}"
       "figcaption{font-size:14px;color:var(--muted);margin-top:8px}"
       ".note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:15px;max-width:80ch}"
       ".box{background:var(--panel);border:2px solid var(--accent);padding:12px 16px;max-width:80ch}"
       ".warn{background:var(--panel);border:2px solid var(--bad);padding:12px 16px;max-width:80ch}"
       "table{border-collapse:collapse;font-size:14.5px;background:var(--panel)}th,td{border:1px solid var(--line);padding:6px 10px;text-align:left;vertical-align:top}"
       "td.n{text-align:right;font-variant-numeric:tabular-nums}th{background:var(--thead)}.scroll{overflow-x:auto}a{color:var(--link)}"
       "code{font-family:Consolas,monospace;font-size:14px;background:#0f1317;padding:0 4px;border-radius:3px}"
       "pre{background:#0f1317;border:1px solid var(--line);padding:10px 12px;overflow-x:auto;font:13px/1.4 Consolas,monospace;margin:0;max-width:100%}"
       "ol,ul{margin:0;padding-left:22px;max-width:80ch;display:flex;flex-direction:column;gap:6px}"
       ".eq{font-family:'Cambria Math','Segoe UI Symbol',serif;font-size:19px;background:var(--panel);border:1px solid var(--line);padding:10px 16px;max-width:80ch}"
       ".tag{font-size:12px;color:var(--muted)}nav a{margin-right:12px;font-size:14px}nav{max-width:100%}"
       ".cmd{border-left:3px solid var(--link)}.file{font-size:13px;color:var(--muted);margin:6px 0 -4px}"
       "dt{color:var(--accent);margin-top:6px}dd{margin:0 0 0 18px;max-width:78ch}")


def cmd(s):
    return "<pre class='cmd'>" + html.escape(s.strip("\n")) + "</pre>"


def file_q(path, content):
    return f"<div class='file'>{html.escape(path)}</div>" + pre(content)


H = []
a = H.append
a(f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>OpenFOAM first lessons</title><meta name='viewport' content='width=device-width,initial-scale=1'><style>{CSS}</style></head><body><main>")
a("<header><h1>OpenFOAM: a first walk-through</h1>"
  f"<p class='tag'>Built {built} from real files in the MMAV-CFD distro. Every file excerpt and log below was copied from a run on this machine. Not a substitute for the OpenFOAM user guide.</p>"
  "<nav><a href='#s1'>1 What it is</a><a href='#s2'>2 Get in</a><a href='#s3'>3 Case anatomy</a><a href='#s4'>4 The maths</a><a href='#s5'>5 Lesson 1</a>"
  "<a href='#s6'>6 Lesson 2</a><a href='#s7'>7 Real case</a><a href='#s8'>8 Viewing</a><a href='#s9'>9 Mistakes, glossary</a></nav></header>")

# ---- 1
a("<section id='s1'><h2>1. What OpenFOAM is, and is not</h2>"
  "<p>OpenFOAM has <b>no user interface</b>. It is a collection of command-line programs. Each program reads a folder of plain text files (the <b>case</b>) and writes new folders of numbers. You edit the text with any editor and run the programs from a terminal.</p>")
a(fig(svg_pipe, "The whole workflow. Programs used here: blockMesh (makes the mesh), checkMesh (checks it), icoFoam and pimpleFoam (solvers), foamToVTK (exports for a viewer)."))
a("<p><b>CFD</b> (computational fluid dynamics) means computing the flow of air or water on a computer instead of in a wind tunnel. <b>ParaView</b> is a separate program that draws the results. It is not part of OpenFOAM.</p>"
  "<div class='box'><b>What is installed on this machine (checked 2026-10-13)</b><ul>"
  "<li><b>OpenFOAM v2606</b> (the openfoam.com / ESI version) is installed in the WSL distro <code>MMAV-CFD</code>, with its tutorials and source code, plus gmsh, Python 3, numpy and matplotlib.</li>"
  "<li><b>ParaView 5.12.0 is installed on Windows</b> (installed with winget: <code>winget install --id Kitware.ParaView -e</code>) in <code>C:\\Program Files\\ParaView 5.12.0</code>. Start it from the Start menu. Its command-line tools <code>pvpython.exe</code> and <code>pvbatch.exe</code> are in the <code>bin</code> folder there. It is NOT installed inside the Linux distro; it does not need to be.</li>"
  "<li>Tested on this machine: ParaView on Windows opens the case files directly through the <code>\\\\wsl.localhost\\MMAV-CFD\\...</code> path and renders off-screen to PNG files (all pictures in section 8 were made that way). The ParaView window was started once and answered normally; the menu clicks below were not clicked through by me, they follow the same filters I ran from a script.</li>"
  "<li>The old wrapper <code>paraFoam</code> in the distro is not used and not needed.</li></ul></div></section>")

# ---- 2
a("<section id='s2'><h2>2. How to get in</h2>"
  "<p><b>WSL</b> (Windows Subsystem for Linux) runs a real Linux system inside Windows. A <b>distro</b> is one installed Linux system. OpenFOAM lives in the distro called <code>MMAV-CFD</code>.</p>"
  "<p>Open a Linux shell inside it, from PowerShell:</p>" + cmd("wsl -d MMAV-CFD") +
  "<p>You land as the user <code>cfd</code> in <code>/home/cfd</code>. The OpenFOAM commands are already available because <code>~/.bashrc</code> loads them. All practice cases are in <code>~/run/learn/</code>. The sweep cases are in <code>~/run/vp_*</code>.</p>"
  "<p>To see the files from Windows Explorer, paste this in the address bar:</p>" + cmd(r"\\wsl.localhost\MMAV-CFD\home\cfd\run") +
  "<div class='warn'><b>Never use the QuadFlightStack distro for this.</b> It belongs to another project. Always write <code>wsl -d MMAV-CFD</code> with the name; do not type plain <code>wsl</code>, because that opens the default distro, and the default is QuadFlightStack.</div>"
  "<p>Do not keep case data on OneDrive. Cases live in the distro's Linux home.</p></section>")

# ---- 3
a("<section id='s3'><h2>3. Anatomy of a case</h2>")
a(fig(svg_tree, "The folders of a case. Only 0/, constant/ and system/ are written by you; the rest appear when you run."))
a("<p>Here is a real case, the stock cavity tutorial, before and after running it (from the run on this machine):</p>"
  "<pre>before:  0  constant  system\nafter :  0  0.1  0.2  0.3  0.4  0.5  constant  system     (plus VTK/ if you export)</pre>"
  "<p>The case files, copied from <code>~/run/learn/01_cavity</code>:</p>")
a(file_q("system/controlDict  (run control)", cav["system/controlDict"]))
a(file_q("0/U  (velocity: starting value and boundary conditions)", cav["0/U"]))
a(file_q("constant/transportProperties  (the fluid)", cav["constant/transportProperties"]))
a("<p>Dictionaries are lists of <code>keyword value;</code> pairs; every entry ends in a semicolon. Braces group entries. The first block of every file, <code>FoamFile</code>, only says what kind of file it is.</p>"
  "<p>Parallel runs add <code>processor0 ... processor11</code> folders. The real validation case <code>vp_a4_coarse</code> contains (copied from <code>ls</code>):</p>"
  f"<pre>{html.escape(sw_ls)}</pre><p>and inside <code>processor0</code>: <code>{html.escape(sw_proc)}</code>.</p></section>")

# ---- 4
a("<section id='s4'><h2>4. The maths, built up gently</h2>")
a("<h3>(a) The equations: mass and momentum</h3>"
  "<p>The fluid is <b>incompressible</b>: its density does not change. Air at these speeds behaves that way. The velocity <b>u</b> is a vector and <i>p</i> is pressure. Two laws govern the flow.</p>"
  "<p><b>Conservation of mass.</b> What flows into any small volume flows out; nothing piles up. For incompressible flow the divergence of velocity is zero:</p>"
  "<div class='eq'>\u2207\u00b7<b>u</b> = 0</div>"
  "<p><b>Conservation of momentum</b> (Newton's second law for fluid; with the Navier-Stokes equations):</p>"
  "<div class='eq'>\u2202<b>u</b>/\u2202t &nbsp;+&nbsp; \u2207\u00b7(<b>u</b> <b>u</b>) &nbsp;=&nbsp; \u2212\u2207p &nbsp;+&nbsp; \u03bd \u2207\u00b2<b>u</b></div>"
  "<ul><li><b>\u2202<b>u</b>/\u2202t</b>: the <b>unsteady</b> term. How fast the velocity at a point changes with time.</li>"
  "<li><b>\u2207\u00b7(<b>u</b> <b>u</b>)</b>: the <b>convection</b> term. Momentum is carried along by the flow itself. This term makes the equations nonlinear.</li>"
  "<li><b>\u2212\u2207p</b>: the <b>pressure-gradient</b> term. Fluid is pushed from high pressure toward low pressure.</li>"
  "<li><b>\u03bd \u2207\u00b2<b>u</b></b>: <b>viscous diffusion</b>. Friction smooths velocity differences. \u03bd (nu) is the kinematic viscosity.</li></ul>"
  "<p>In these equations <i>p</i> means pressure divided by density (the solvers use this <b>kinematic pressure</b>, units m\u00b2/s\u00b2, as seen in <code>0/p</code>: <code>dimensions [0 2 -2 0 0 0 0]</code>).</p>"
  "<p>The <b>Reynolds number</b> compares the two big terms, convection and viscous diffusion:</p>"
  "<div class='eq'>Re = U L / \u03bd &nbsp; (speed \u00d7 length \u00f7 kinematic viscosity)</div>"
  "<p>Small Re: viscosity wins, the flow is smooth and orderly. Large Re: convection wins, the flow can become unsteady and turbulent. The cavity has Re = 1 \u00d7 0.1 / 0.01 = 10. The toy plate has Re = 1000. The real validation plate has Re = 60,000 (chord 88.9 mm).</p>")
a("<h3>(b) The finite-volume idea</h3>"
  "<p>Nobody can solve the equations for a whole wing at once. So the space is cut into small <b>cells</b> (the <b>mesh</b>). Each cell stores one value of each field. Then for each cell you <b>integrate the equation over the cell volume</b>. The <b>Gauss theorem</b> turns an integral of a divergence over a volume into a sum over the cell's faces:</p>"
  "<div class='eq'>\u222b<sub>V</sub> \u2207\u00b7<b>F</b> dV &nbsp;=&nbsp; \u222e<sub>S</sub> <b>F</b>\u00b7<b>n</b> dS &nbsp;\u2248&nbsp; \u03a3<sub>faces</sub> <b>F</b><sub>f</sub>\u00b7<b>S</b><sub>f</sub></div>"
  "<p><b>S</b><sub>f</sub> is the face area vector (area times the outward normal). So the equation for a cell becomes: the change inside = what comes through the faces. The flux leaving one cell is exactly the flux entering its neighbour, so mass and momentum are conserved by construction. That is why the method is called <b>finite volume</b>.</p>")
a(fig(svg_cell, "One cell and its neighbours. Each face is shared by exactly two cells."))
a("<h3>(c) What fvSchemes chooses</h3>"
  "<p>The balance needs the field value <b>at a face</b>, but only cell-centre values are stored. <code>system/fvSchemes</code> says how to estimate it, term by term. The real cavity file:</p>")
a(file_q("01_cavity/system/fvSchemes", cav["system/fvSchemes"]))
a("<ul><li><code>ddtSchemes</code>: how <i>\u2202/\u2202t</i> is approximated. <code>Euler</code> uses the current and previous time step (first-order). The validation case uses <code>backward</code>, which uses two previous levels (second-order).</li>"
  "<li><code>gradSchemes</code>: how a gradient is computed from cell values. <code>Gauss linear</code> is the Gauss theorem above with face values averaged linearly.</li>"
  "<li><code>divSchemes</code>: the convection term. This is the important choice. <b>Linear</b> interpolation is second-order accurate but can overshoot and wiggle near sharp changes. <b>Upwind</b> takes the value from the cell the flow comes from; it never wiggles but smears sharp features (first-order). <b>Limited</b> schemes mix the two: linear where the solution is smooth, upwind-like where it would wiggle.</li>"
  "<li><code>laplacianSchemes</code>, <code>snGradSchemes</code>: the viscous term and face-normal gradients. <code>corrected</code> adds a fix for cells that are not squarely aligned (non-orthogonal).</li></ul>")
a(fig(svg_scheme, "The accuracy-versus-wiggles trade-off, sketched."))
a("<p>The validation case uses a limited scheme, taken from its own <code>fvSchemes</code>:</p>")
a(file_q("vp_a4_coarse/system/fvSchemes", sw["fvSchemes"]))
a("<h3>(d) The pressure-velocity problem</h3>"
  "<p>In incompressible flow there is <b>no equation for pressure</b>. Pressure is not a thermodynamic variable here. It is whatever field makes the velocity satisfy \u2207\u00b7<b>u</b> = 0. So solvers use a loop (<b>PISO</b>: pressure-implicit with splitting of operators; <b>PIMPLE</b> is PISO with an optional outer loop). Here is the real loop, copied verbatim from the installed <code>icoFoam.C</code> (the <code>{ ... }</code> braces are part of the file):</p>")
a(pre(rd("src","icoFoam_loop.txt")))
a(fig(svg_piso, "Predict, solve pressure, correct. Repeat a few times each time step."))
a("<p>In words: (1) guess the new velocity using the old pressure; that guess does not conserve mass. (2) Solve one pressure equation whose answer is the pressure that would fix that. (3) Use the pressure to correct velocity and the face fluxes <code>phi</code> so that mass is conserved. Repeat a couple of times (<code>nCorrectors</code>) because the correction changes the momentum balance slightly. The log lines \"time step continuity errors\" report how well step (3) worked. <code>phi</code> is the volume flux through each face, one number per face.</p>")
a("<h3>(e) What fvSolution chooses</h3>"
  "<p>Each equation above, written for every cell at once, is a very large <b>sparse linear system</b> (millions of unknowns, each linked only to its few neighbours). It is solved <b>iteratively</b>: guess, improve, repeat until good enough. <code>system/fvSolution</code> picks the method and when to stop. The cavity file:</p>")
a(file_q("01_cavity/system/fvSolution", cav["system/fvSolution"]))
a("<ul><li>The <b>residual</b> is a normalised measure of how far the current guess is from satisfying the equation. Zero = perfect. The solver prints it twice: <i>Initial residual</i> (before this solve) and <i>Final residual</i> (after).</li>"
  "<li><code>tolerance 1e-06</code>: stop when the residual falls below this. <code>relTol 0.05</code>: or stop when it has fallen to 5 % of its starting value. <code>relTol 0</code> means demand the full tolerance.</li>"
  "<li><code>pFinal</code> is the pressure solve on the last corrector pass of a step, usually tighter. <code>$p;</code> copies the settings of <code>p</code>.</li>"
  "<li><code>PISO { nCorrectors 2; }</code>: two pressure passes per step, which is why each time step in the log has two <code>Solving for p</code> lines.</li></ul>")
a("<h3>(f) Time step and Courant number</h3>"
  "<p>The flow is advanced in small time steps \u0394t. If \u0394t is too large for the cell size, the numbers go wrong and the run <b>blows up</b> (values grow without limit and the program stops). The warning number is the <b>Courant number</b>. In one dimension it is</p>"
  "<div class='eq'>Co = u \u0394t / \u0394x</div>"
  "<p>speed times time step divided by cell size: the fraction of a cell the fluid crosses in one step. OpenFOAM computes it per cell from the face fluxes. The installed <code>CourantNo.H</code> says:</p>"
  + pre(rd("src","courant.txt")) +
  "<p>so \"max\" in the log is the worst cell and \"mean\" the average.</p>")
a(fig(svg_co, "Courant number: how far the fluid moves per step, in cells."))
a("<p>With <code>adjustTimeStep yes</code> the solver chooses \u0394t so the largest Courant number stays at <code>maxCo</code>. The toy case uses 1. The sweep cases use 1.5, which is inside the 1 to 2 range asked for the validation runs; they ran stable at that value. A larger <code>maxCo</code> means fewer, bigger steps and so a cheaper run, but with less margin before blow-up. The log of the real run confirms the limit is held: <code>Courant Number mean: 0.0149 max: 1.5001</code>. Small cells and fast flow near the plate's edges are what limit the step (an inference from the cell sizes, not a measurement).</p>")
a("<h3>(g) Boundary conditions state the physical problem</h3>"
  "<p>The equations alone have many solutions. <b>Boundary conditions</b> say what happens at the edges of the domain and pick the one you mean. They are set per <b>patch</b> (a named group of boundary faces) in the <code>0/</code> files. The cavity's real files:</p>")
a(file_q("01_cavity/0/U", cav["0/U"]))
a(file_q("01_cavity/0/p", cav["0/p"]))
a("<p>Reading them: the lid <code>movingWall</code> is a <b>fixed-value</b> velocity (1 0 0): the fluid touching it moves with it. <code>fixedWalls</code> are <b>no-slip</b>: velocity zero at a wall. For pressure, <code>zeroGradient</code> means pressure does not change across the wall. <code>empty</code> marks the two faces that make a 3D mesh act 2D.</p>"
  "<p>The real validation case needs more kinds. These are its actual files:</p>")
a(file_q("vp_a4_coarse/0.orig/U", sw["U"]))
a(file_q("vp_a4_coarse/0.orig/p", sw["p"]))
a("<ul><li><b>inlet, fixedValue</b>: air arrives at speed 1 along x.</li>"
  "<li><b>outlet</b>: pressure fixed at 0; velocity <code>inletOutlet</code> lets air leave freely, and if air were to come back in it is given the value 1 0 0.</li>"
  "<li><b>top, bottom, side, slip</b>: far walls that air may slide along but not cross. <b>sym, symmetryPlane</b>: the wing's mid-plane (only half the wing is modelled). <b>plate, noSlip</b>: the wing surface.</li></ul>")
a("<h3>(h) Laminar or turbulence model</h3>"
  "<p>At high Reynolds number flow becomes turbulent: many small swirls. Resolving them all needs enormous meshes, so engineers often add a <b>turbulence model</b> that stands in for the swirls. A <b>laminar</b> run adds no model and solves the equations as written.</p>"
  "<p>The validation case is run laminar. What <code>cfd/VALIDATION_SCOPING.md</code> records about that choice, quoted:</p>"
  "<div class='note'>\"pimpleFoam, laminar (no turbulence model), nu = 1.6667e-5.\"<br>"
  "\"The coarse solution is steady: the leading-edge bubble and transition are not resolved (LE radius 0.004 c, cell 0.008 c). The medium mesh develops unsteadiness the coarse mesh damped, so the coarse 'steady state' is partly numerical.\"</div>"
  "<p>That file states the choice but gives no longer reason, and none is added here. The real settings:</p>")
a(file_q("vp_a4_coarse/constant/turbulenceProperties", sw["turb"]))
a(file_q("vp_a4_coarse/constant/transportProperties", sw["trans"]))
a("</section>")

# ---- 5
cav_ls = "0  0.1  0.2  0.3  0.4  0.5  constant  system"
a("<section id='s5'><h2>5. Lesson 1: the lid-driven cavity</h2>"
  "<p>A square box of fluid, lid sliding at 1 m/s, everything else wall. 20 \u00d7 20 cells. This is OpenFOAM's \"hello world\". Everything below was run on this machine on one core in 0.25 s total (blockMesh + checkMesh + icoFoam).</p>"
  "<ol><li>Open the distro and go to the case:" + cmd("wsl -d MMAV-CFD\ncd ~/run/learn/01_cavity\nls") +
  "You should see <code>0  README.txt  case.foam  constant  system</code>.</li>"
  "<li><b>blockMesh</b> reads <code>system/blockMeshDict</code> and writes the mesh into <code>constant/polyMesh/</code>. The real dictionary:</li></ol>")
a(file_q("01_cavity/system/blockMeshDict", cav_bm))
a("<p><code>scale 0.1</code>: coordinates are multiplied by 0.1, so the box is 0.1 m wide. <code>vertices</code>: 8 corner points. <code>blocks</code>: one hexahedron (<code>hex</code>) of 20 \u00d7 20 \u00d7 1 cells (one cell thick = 2D). <code>boundary</code>: the patch names used in <code>0/U</code>.</p>"
  + cmd("blockMesh") + "<p>The end of the real output:</p>")
a(pre("\n".join(l for l in cav_block.split("\n") if l.strip())[-420:]))
a("<ol start='3'><li><b>checkMesh</b> inspects the mesh. Real output (trimmed):" + cmd("checkMesh") + "</li></ol>")
a(pre(NL.join(l for l in cav_chk.split(NL) if re.search(r"cells:|hexahedra|Boundary definition|aspect|non-orthogonality|skewness|Mesh OK", l))))
a("<ol start='4'><li>Run the solver:" + cmd("icoFoam > log.icoFoam\ntail log.icoFoam") + "Output goes to the file <code>log.icoFoam</code> so it does not flood the screen. After the run:" + cmd("ls") +
  "</li></ol><pre>" + html.escape(cav_ls) + "</pre>")
a("<h3>Reading the solver log, line by line</h3><p>The start of the real log:</p>")
a(pre("\n".join(cav_ico.split("\n")[cav_ico.split("\n").index("Create mesh for time = 0"):][:12])))
a("<p>Then, for every time step, a block like this (real, step 2):</p>")
a(pre(ico_second))
a("<dl><dt>Time = 0.01</dt><dd>Simulation time at the end of this step. The step is <code>deltaT 0.005</code>.</dd>"
  "<dt>Courant Number mean: 0.0977 max: 0.586</dt><dd>See section 4f. Here the time step is fixed, so the number tells you whether your fixed step was safe. Below 1 is comfortable.</dd>"
  "<dt>Solving for Ux, Initial residual = 0.160686, Final residual = 6.8e-06, No Iterations 19</dt><dd>The momentum equation for the x velocity. The solver started 0.16 away from satisfying it and iterated 19 times to reach 7e-06 (below the <code>tolerance 1e-05</code> in fvSolution). Uy likewise. There is no Uz line because the case is 2D.</dd>"
  "<dt>DICPCG: Solving for p ... (twice)</dt><dd>The pressure equation, once per corrector pass (<code>nCorrectors 2</code>). The first pass stops early (<code>relTol 0.05</code>: 0.43 \u2192 0.010). The second goes to the full tolerance (<code>pFinal</code>: 0.30 \u2192 5e-07). DIC and PCG are the names of the preconditioner and the solver.</dd>"
  "<dt>time step continuity errors : sum local = 6.6e-09, global = ...</dt><dd>How well mass is conserved after the pressure correction: local = the leftover imbalance summed cell by cell. It should be tiny (it is, 1e-09) and get smaller or stay small. Growing values mean trouble.</dd>"
  "<dt>ExecutionTime = 0 s  ClockTime = 0 s</dt><dd>CPU time and wall-clock time so far.</dd></dl>"
  "<p>The last step of the run:</p>")
a(pre(ico_last))
a("<p>By time 0.5 the residuals are around 1e-07 and the solver does 0 iterations: the flow has become steady. That is what \"converged\" looks like for a steady problem.</p>"
  "<div class='note'>Looking at it: <code>foamToVTK</code> writes <code>VTK/</code>; Windows ParaView can open <code>\\\\wsl.localhost\\MMAV-CFD\\home\\cfd\\run\\learn\\01_cavity\\case.foam</code> once installed. See section 8. After the run the case was cleaned back to the un-run state; commands are in <code>01_cavity/README.txt</code>.</div></section>")

# ---- 6
a("<section id='s6'><h2>6. Lesson 2: a 2D flat plate</h2>"
  "<div class='box'><b>This is a teaching toy.</b> The mesh is coarse, the Reynolds number (1000) is far from the real case (60,000), the domain is small, and no claim is made about accuracy. Use it to learn the workflow and to see how the settings change the result. Do not quote its numbers.</div>"
  "<p>A thin plate, chord 1, 4 % thick, flat along x. The free stream is tilted up by 4\u00b0, so the plate sees 4\u00b0 angle of attack. Laminar, Re = 1000, about 11,500 cells, one CPU core, about 85 s. Files are in <code>~/run/learn/02_plate2d</code>, and every entry in them has a one-line comment.</p>"
  "<h3>Run it</h3>" + cmd("cd ~/run/learn/02_plate2d\n./Allrun\npython3 plot_forces.py\npython3 plot_slice.py\n./Allclean      # back to the un-run state, when you want to start over") +
  "<p><code>Allrun</code> is a short shell script that runs blockMesh, checkMesh, pimpleFoam and foamToVTK and times them. Real result: <b>done in 85 s</b> (87 s on a second run), 2238 time steps, to time 20.</p>"
  "<h3>Meshing</h3><p>The plate is a hole in the mesh, made from eight blocks around it (<code>system/blockMeshDict</code>). The head of the real file:</p>")
a(pre("\n".join(pl_bm.split("\n")[:44])))
a("<p>The real checkMesh result for this mesh, and how to read it:</p>")
a(pre(chk_stats + "\n" + chk_geo))
a("<ul><li><b>Max aspect ratio = 63</b> (OK): the longest cell side divided by the shortest. The big, thin cells are far from the plate. High values hurt accuracy but are allowed up to a large threshold.</li>"
  "<li><b>non-orthogonality 0</b>: the angle between the line joining two cell centres and the face between them. 0 for a mesh of rectangles. In the real snappyHexMesh case it is up to 65 degrees (section 7), and that case sets <code>nNonOrthogonalCorrectors 1</code>.</li>"
  "<li><b>skewness</b>: how far the face centre sits from the line between cell centres. About 0 here.</li>"
  "<li>Always look for <code>Mesh OK.</code> or a list of failed checks. A failed check means fix the mesh before running.</li></ul>"
  "<h3>The files you edit</h3><p>Boundary conditions (each line has the real comment):</p>")
a(file_q("02_plate2d/0/U", body("plate", "0", "U")))
a(file_q("02_plate2d/system/fvSchemes (convection line)", "\n".join(l for l in rd("plate", "system", "fvSchemes").split("\n") if "div(phi,U)" in l)))
a("<p>The other files (<code>controlDict</code>, <code>fvSchemes</code>, <code>fvSolution</code>, <code>0/p</code>, <code>constant/</code>) are commented the same way; open them and read.</p>"
  "<h3>Running and watching the forces converge</h3>")
a(pre("\n".join(pl_co[:1] + [pl_dt[0]] + pl_co[29:30] + pl_dt[29:30] + pl_co[-1:] + pl_dt[-1:])))
a(f"<p>The solver starts with a tiny time step (<code>deltaT 0.001</code>) and grows it until the Courant number reaches <code>maxCo 1</code>. Final step: {pl_dt[-1].replace('deltaT = ', '')[:8]} after {pl_steps} steps.</p>"
  "<p>Forces are written <b>every step</b> to <code>postProcessing/forceCoeffs1/0/coefficient.dat</code>. The start of the real file:</p>")
a(pre("\n".join(l[:150] for l in coef[:3] + coef[11:14])))
a("<p>Columns: Time, Cd (total, front, rear), Cl (total, front, rear), CmPitch, ... Use <code>Cd</code> (column 2), <code>Cl</code> (column 5) and <code>CmPitch</code> (column 8). <b>CL</b> is lift, <b>CD</b> drag, <b>CM</b> pitching moment, each divided by dynamic pressure \u00d7 reference area (\u00d7 length for the moment), so they have no units. Real values at selected times:</p>")
a("<table><tr><th>time</th><th>CL</th><th>CD</th><th>CM about c/4</th></tr>" + "".join(f"<tr><td class='n'>{t:.3f}</td><td class='n'>{cl:.4g}</td><td class='n'>{cd:.4g}</td><td class='n'>{cm:.4g}</td></tr>" for t, cl, cd, cm in pl_sample) + "</table>")
a("<p>The first row is the impulsive start: the fluid is made to move everywhere at time 0, so the very first steps show enormous, meaningless forces. They settle within a few chord passages. The final values were CL = %.4f, CD = %.4f, CM = %.5f (nose-up positive).</p>" % (cl_end, cd_end, cm_end))
a(fig(f"<img alt='CL CD CM history' src='{img_forces}'>", "forces.png from plot_forces.py (real run). Settled means the curves go flat."))
a(f"<p><b>Sign check for CM.</b> The case writes a second file with the moment about mid-chord (<code>forceCoeffsMid</code>). Moving the reference point aft of where lift acts must make the moment more nose-up: CM(mid) \u2212 CM(c/4) should be about +0.25 \u00d7 CL. Real run: {cm_mid:.4f} \u2212 ({cm_end:.4f}) = {cm_mid - cm_end:.4f}, and 0.25 \u00d7 {cl_end:.4f} = {0.25 * cl_end:.4f}. The sign is right.</p>")
a(fig(f"<img alt='speed field' src='{img_speed}'>", "speed.png from plot_slice.py (real run): fluid speed around the plate. Dark = slow (the wake behind the plate and the plate surface); yellow = fast."))
a("<h3>Three experiments</h3><p>First copy the case so the original stays intact: <code>cp -r ~/run/learn/02_plate2d ~/run/learn/03_mine; cd ~/run/learn/03_mine; ./Allclean</code>. Run each change separately, compare <code>coefficient.dat</code> with the baseline, and write down what you see.</p>"
  "<ol><li><b>Change the angle.</b> In <code>0/U</code> change the free-stream vector to (cos \u03b1, sin \u03b1, 0) with a new \u03b1, and in <code>system/controlDict</code> change <code>liftDir</code> and <code>dragDir</code> to match (lift is perpendicular to the free stream, drag along it). Look at: does CL rise as you expect, and does the wake (speed.png) change shape? What happens if you forget to update liftDir?</li>"
  "<li><b>Halve the mesh spacing.</b> In <code>system/blockMeshDict</code> double every cell count (30 60 50 \u2192 60 120 100, and so on for the other blocks; keep shared edges equal). That is about four times the cells, and the time step also shrinks, so expect a much longer run. Look at: how much do CL, CD and CM move? If they barely move, the first mesh was already adequate for this question; if they move a lot, it was not.</li>"
  "<li><b>Switch to upwind.</b> In <code>system/fvSchemes</code> change <code>div(phi,U) Gauss limitedLinearV 1;</code> to <code>div(phi,U) Gauss upwind;</code>. Look at: do the forces change? Is the wake in speed.png smoother or more smeared? This is the accuracy-versus-wiggles trade-off from section 4c.</li></ol></section>")

# ---- 7
a("<section id='s7'><h2>7. How the real validation case differs from the toy</h2>"
  "<ul><li><b>3D half model.</b> The wing is 3D: a plate of aspect ratio 3 with a symmetry plane at its root, so only half is meshed (the <code>sym</code> patch above).</li>"
  "<li><b>snappyHexMesh.</b> Instead of hand-made blocks, a coarse background box mesh is made with blockMesh, then <code>snappyHexMesh</code> refines it around a surface file of the plate (an STL), cuts the plate out, and adds 5 thin layers of cells on the surface. Refinement boxes put small cells around the plate, the wake and the tip. Real checkMesh of the coarse mesh:</li></ul>")
a(pre(sw_chk))
a("<ul><li><b>Parallel.</b> The mesh is split into 12 pieces (<code>decomposePar</code>, using this <code>decomposeParDict</code>), one per CPU core, and run with <code>mpirun -np 12 pimpleFoam -parallel</code>. MPI is the library that lets the 12 processes exchange data across piece boundaries. Each piece lives in a <code>processorN</code> folder.</li></ul>")
a(file_q("vp_a4_coarse/system/decomposeParDict", sw["decompose"]))
a(file_q("vp_a4_coarse/system/fvSolution", sw["fvSolution"]))
a("<ul><li><b>Cost.</b> About 1 million cells; adaptive step with max Courant 1.5; 20 chord passages took 5233 s (1.45 h) on 12 cores and peaked at 2.5 GB. The toy: 11,500 cells, 85 s, one core.</li>"
  "<li><b>Run length and averaging.</b> The forces are not single numbers; they vary during the run. For angles 0 to 8 degrees the run is 20 chord passages and the mean and standard deviation are taken over t = 8 to 20 (the first part is the start-up transient and is thrown away). For 10 and 12 degrees the flow separates and is unsteady, so the run is 40 chord passages and the last half is averaged.</li></ul>")
a(pre(sw_tail[-1800:]))
a("<p>That is the real end of the alpha-4 run: GAMG pressure solves converged in 1 to 3 iterations, continuity errors around 1e-13, and the force coefficients with their pressure and viscous parts (<code>Cl 0.219</code>, <code>CmPitch 0.0073</code>), plus the y+ summary line.</p>")
a("<h3>Results so far against the measured wind-tunnel values</h3>"
  f"<p>Read from <code>cfd/validation_sweep/sweep_results.csv</code> when this page was built ({built}). Measured values: Ananda (2012) thesis tables, AR 3 rectangular flat plate, Re about 60,000, interpolated to the same angle (mean of the up and down legs; the measured angles are tunnel-corrected). If a row is marked PARTIAL it was still running when this page was built and is averaged over what existed; do not trust it. When this page was last built every angle was marked done (0, 2, 4, 6, 8 degrees for 20 chord passages; 10 and 12 for 40, averaged over the last half).</p>")
a("<div class='scroll'>" + sweep_tbl + "</div>")
a("<div class='box'><b>Stated plainly:</b> at 2 and 4 degrees the CFD lift is about 11 to 12 % <b>low</b> and the pitching moment about 40 % <b>low</b>, while drag is within a few percent. <b>The comparison is not finished.</b> The mesh is coarse (a finer mesh changed the drag by 15 % in the scoping run), the pitching moment differs in sign from 6 degrees upward (CFD negative where the measurement is still positive at 6 and 8 degrees; read it off the table), and the experiment and the model differ (tunnel corrections, a sting-mounted full-span wing, an assumed square wing tip). Treat these as work in progress, not as validation.</div>")
if img_slice:
    a(fig(f"<img alt='real slice' src='{img_slice}'>", "Real result: speed on a cut through the middle of the half wing (alpha 4, coarse mesh, from cfd/validation_scoping)."))
a("</section>")

# ---- 8
def imgfig(name, cap):
    return f"<figure><img alt='{html.escape(name)}' src='img/{name}'><figcaption>{cap}</figcaption></figure>"


UNC = r"\\wsl.localhost\MMAV-CFD\home\cfd\run\learn\02_plate2d\case.foam"
a("<section id='s8'><h2>8. Looking at results</h2>"
  "<p><b>ParaView</b> is the program that draws the numbers OpenFOAM writes. Version 5.12.0 is installed on Windows. It reads your case folder directly; the case does not need to be on a Windows drive.</p>"
  "<div class='box'><b>Order matters.</b> ParaView can only show what the solver has written. Run the case first (<code>blockMesh</code> and the solver, or <code>./Allrun</code>), then open it. The practice cases are left in the un-run state on purpose, so you do this yourself. <code>case.foam</code> is an empty marker file; ParaView opens it and then reads the time folders next to it.</div>"
  "<h3>Click by click</h3>"
  "<ol>"
  "<li><b>Run a case</b> in the distro, e.g. <code>cd ~/run/learn/02_plate2d; ./Allrun</code> (about 85 s).</li>"
  "<li><b>Start ParaView</b> from the Start menu: ParaView 5.12.0.</li>"
  "<li><b>File \u2192 Open.</b> In the \"File name\" box paste <code>" + html.escape(UNC) + "</code> and press Open. (Tested in a script: the OpenFOAM reader reads this path fine.) If ParaView asks which reader, choose <b>OpenFOAM</b>.</li>"
  "<li>In the <b>Properties</b> panel (left): under <b>Mesh Regions</b> keep <code>internalMesh</code> ticked (tick <code>patch/plate</code> too if you want the plate surface on its own). Under <b>Cell Arrays</b> tick <code>U</code> and <code>p</code>. Click the green <b>Apply</b> button. Nothing is drawn until you press Apply.</li>"
  "<li><b>Choose what to colour by.</b> In the toolbar, the dropdown that says <i>Solid Color</i> lists the fields: choose <code>U</code> and then <code>Magnitude</code> (speed), or <code>p</code> (pressure). Cell data is one flat colour per cell, so the picture looks blocky; that is the mesh (the cavity picture below is like that).</li>"
  "<li><b>Smooth colours (optional).</b> Filters \u2192 Alphabetical \u2192 <b>Cell Data to Point Data</b>, Apply, and colour that instead. The plate pictures below use it.</li>"
  "<li><b>Time.</b> The toolbar has VCR-style buttons and a time box. The reader does not list time 0 by default (a setting called <i>Skip Zero Time</i>), so the first time you see is the first saved folder: 0.1 for the cavity, 5 for the plate (checked: those were the times the reader reported). Click <b>Last Frame</b> to jump to the final saved time (0.5 and 20).</li>"
  "<li><b>Rescale colours.</b> Click <b>Rescale to Data Range</b> (next to the colour dropdown) to fit the colours to the current numbers, or <b>Rescale to Custom Range</b> and type a fixed range, e.g. \u22120.4 to 0.4 for pressure. A fixed range lets you compare two pictures fairly.</li>"
  "<li><b>2D view.</b> For these flat cases click the <b>2D</b> button in the view toolbar, and set the view direction to look along z, so you face the plane.</li>"
  "<li><b>Slice</b> (for a 3D case such as the real wing). Filters \u2192 Alphabetical \u2192 <b>Slice</b>. Set Origin and Normal (e.g. normal along y for a cut along the chord at a given span position) and Apply. Colour the slice by <code>U</code>.</li>"
  "<li><b>Streamlines.</b> Filters \u2192 Alphabetical \u2192 <b>Stream Tracer</b>. Vectors = <code>U</code>. Seed Type = <b>Line</b>: give two end points (for the plate: x = \u22121.5, from y = \u22120.6 to 0.8) and a Resolution (40 lines). Apply. <b>Glyph</b> (same Filters menu) draws arrows instead.</li>"
  "<li><b>Save a picture.</b> File \u2192 <b>Save Screenshot</b>, pick a resolution, OK.</li></ol>"
  "<p>What I ran to check this: a script (<code>make_images.py</code> in this folder, run with <code>pvbatch.exe --force-offscreen-rendering</code>) that does the same steps with the same filters and settings, with no window. It produced the pictures below. I did not click through the menus; names are as in ParaView 5.12, and buttons may sit slightly differently on screen.</p>"
  "<h3>What you should see</h3>")
a(imgfig("cavity_speed.png", "Lesson 1, the cavity: speed |U| at t = 0.5 (cell data). The lid at the top drags fluid along (red); one big slow circulation sits below (blue). You can see the 20 x 20 cells."))
a(imgfig("plate_speed.png", "Lesson 2, the toy plate: speed at t = 20 (smoothed). The plate is the dark bar; the dark band behind it is the wake (slow fluid); the free stream is about 1."))
a(imgfig("plate_pressure.png", "Pressure p/rho around the plate at t = 20, colour range fixed at \u00b10.4. Red = high pressure where the flow is stopped at the front; blue = low pressure (suction) on top near the leading edge. Pressure is relative (reference 0); only differences matter."))
a(imgfig("plate_streamlines.png", "Streamlines (white) from a seed line upstream, over a faint speed field. The lines bend up and over the plate and spread in the wake. There is no colour bar: the colour is the speed field underneath."))
a("<p><b>What to plot first</b> on a new case: (1) speed magnitude on a slice through the middle; (2) pressure on the body surface; (3) streamlines. Then compare the same picture at two angles with the colour range fixed.</p>")
if os.path.exists(os.path.join(SP, "img", "a08_slice_speed.png")):
    a("<h3>A real sweep case: alpha = 8 degrees</h3>"
      "<p>These two pictures come from the finished validation run <code>vp_sweep_a08</code> (about 1 million cells, t = 20), read-only: nothing was written into that case. Windows could not follow Linux symbolic links through the <code>\\\\wsl.localhost</code> path (tested: the reader found no times), so the fields and mesh of the last time were copied into a separate folder, and the reader was set to <b>Decomposed Case</b> (the real run is split across 12 <code>processor</code> folders). The copy was deleted afterwards; <code>make_images_sweep.py</code> lists the steps.</p>")
    a(imgfig("a08_slice_speed.png", "Speed on a slice at mid-semi-span (y = 0.75 c), alpha = 8 degrees, a SINGLE SNAPSHOT at t = 20 (not a time average), flow from the left, plate nose up. The blue patches on the upper surface and behind the trailing edge are slow, separated fluid; the red hump above the leading edge is fast flow accelerating over the nose."))
    a(imgfig("a08_plate_pressure_top.png", "Pressure on the plate's upper surface at t = 20 (single snapshot, not an average), looking down. Leading edge on the left, trailing edge on the right, root at the bottom edge, tip at the top edge. Strong suction (blue) near the leading edge, patchy and less negative toward the trailing edge."))
a("<h3>Without any viewer</h3>"
  "<p>The two scripts in the toy case need only Python (matplotlib and numpy are installed in the distro) and make PNG files you can open from Explorer at <code>\\\\wsl.localhost\\MMAV-CFD\\home\\cfd\\run\\learn\\02_plate2d</code>:</p>"
  + cmd("python3 plot_forces.py     # CL, CD, CM history -> forces.png\npython3 plot_slice.py      # speed around the plate -> speed.png") +
  "<p>For a quick look at numbers, <code>tail postProcessing/forceCoeffs1/0/coefficient.dat</code> is enough. <code>foamToVTK -ascii -latestTime</code> writes VTK files to <code>VTK/</code> (the toy case's <code>Allrun</code> already does this). I did not test opening those in ParaView; the <code>.foam</code> route above is the tested one.</p>"
  "<div class='note'>Avoid heavy post-processing on cases that are still running; it competes for the same CPU cores.</div></section>")

# ---- 9
a("<section id='s9'><h2>9. Common mistakes and how to recognise them</h2><dl>"
  "<dt>Blow-up from too large a time step</dt><dd>Typical signs: the Courant number climbing and the residuals growing, then a crash. Seen for real while setting up the validation case: starting the plate from uniform flow at <code>deltaT 2e-3</code> crashed the first step; the fix was starting at <code>deltaT 1e-5</code> with <code>adjustTimeStep yes</code>, so the step grows by itself. Fix: smaller start step or smaller <code>maxCo</code>.</dd>"
  "<dt>Forgetting to load the environment</dt><dd>Symptom: <code>blockMesh: command not found</code>. In an interactive <code>wsl -d MMAV-CFD</code> shell it is loaded for you. Inside a script run by other tools, add <code>source /usr/lib/openfoam/openfoam2606/etc/bashrc</code> first. During setup, a command started non-interactively did not see OpenFOAM until this line was used.</dd>"
  "<dt>Wrong or missing file or patch names</dt><dd>Two real examples from this machine. (1) A solver crashed at once with <code>cannot find file \".../constant/turbulenceProperties\"</code>: in v2606 the file is called <code>turbulenceProperties</code>; the error message names the exact file wanted. (2) When the plate patch was missing from the mesh, forceCoeffs ran but every coefficient was exactly 0 and the log had a warning from <code>polyBoundaryMesh::patchSet</code>. Patch names in <code>0/U</code>, <code>0/p</code>, <code>controlDict</code> must equal the names in the mesh (<code>constant/polyMesh/boundary</code>).</dd>"
  "<dt>Wrong units or missing semicolon</dt><dd>Gives a parse error naming the file and line. Read the first lines after <code>FOAM FATAL</code>.</dd>"
  "<dt>Running a second parallel job on top of the sweep</dt><dd>While the sweep ran (<code>/home/cfd/run/sweep/</code>, now finished) it used all 12 physical cores; any batch run does the same. A second heavy job shares those cores and slows both, so check what is running first (<code>pgrep -a pimpleFoam</code>). Small serial practice runs like the toy cases are fine: they use one core, and were run with <code>nice -n 19</code> (lowest priority). Do not edit or delete anything under <code>vp_*</code> or <code>sweep/</code>; they are the validation results.</dd>"
  "<dt>Continuing from stale results</dt><dd>Old time folders and <code>processor*</code> folders from an earlier run make a rerun start from them or fail to decompose. Clean first (<code>./Allclean</code> in the toy cases).</dd></dl>"
  "<h3>Glossary</h3><dl>"
  "<dt>boundary condition</dt><dd>Rule that fixes a field's value or gradient on a boundary patch.</dd>"
  "<dt>cell</dt><dd>One small volume of the mesh; stores one value per field.</dd>"
  "<dt>Courant number</dt><dd>Fraction of a cell the fluid crosses in one time step.</dd>"
  "<dt>CL, CD, CM</dt><dd>Lift, drag and pitching-moment coefficients: force or moment divided by dynamic pressure times reference area (times length for moment).</dd>"
  "<dt>convection</dt><dd>Transport of a quantity by the flow itself.</dd>"
  "<dt>diffusion (viscous)</dt><dd>Smoothing of velocity differences by friction.</dd>"
  "<dt>face</dt><dd>The flat side shared by two cells (or a cell and the boundary).</dd>"
  "<dt>flux (phi)</dt><dd>Volume of fluid crossing a face per second.</dd>"
  "<dt>incompressible</dt><dd>Density treated as constant.</dd>"
  "<dt>laminar</dt><dd>Smooth, orderly flow; also the choice to run without a turbulence model.</dd>"
  "<dt>mesh</dt><dd>All cells together.</dd>"
  "<dt>MPI / rank</dt><dd>MPI is the library for running one job on several cores; a rank is one of those processes.</dd>"
  "<dt>non-orthogonality</dt><dd>How far a face is from being square-on to the line between its two cell centres.</dd>"
  "<dt>patch</dt><dd>A named group of boundary faces.</dd>"
  "<dt>PISO / PIMPLE</dt><dd>The predict-solve pressure-correct loops used for incompressible flow.</dd>"
  "<dt>residual</dt><dd>Normalised measure of how badly an equation is still unsatisfied.</dd>"
  "<dt>Reynolds number</dt><dd>U L / \u03bd: ratio of convection to viscous effects.</dd>"
  "<dt>time step</dt><dd>The jump in simulated time between successive solutions.</dd>"
  "<dt>turbulence model</dt><dd>Extra equations that stand in for small unresolved swirls.</dd>"
  "<dt>y+</dt><dd>Distance of the first cell centre from a wall, in wall units; printed at the end of the real log (average 2.77 at the plate).</dd></dl></section>")
a("</main></body></html>")

with open(OUT, "w", encoding="utf-8", newline="\n") as fh:
    fh.write("".join(H))
print("wrote", OUT, os.path.getsize(OUT), "bytes")
