"""Printable field pack for the next flying sessions (hand throws and nose-down drops).  Four white US Letter pages -> one vector PDF:
  1. test card (specimens, CG table ranges, order of tests)
  2. what-you-see -> what-it-means -> what-to-change table (own page, larger type)
  3. flight log sheet (ruled, to fill by hand)
  4. angle gauge at 1:1 (cut-out wedges 2, 2.5, 4, 5, 6, 7.5, 8 deg) with a 100 mm scale bar

Nothing new is decided here: the CG tables, stop rule, releases and outcome words are those of FLAT_TEST.md and the flat_test_v2 kit outputs (read at run time);
the swift-like settings are read from bird_outline_v2/builds.csv.  The pull-out statements are tagged MODEL (results/runs/rigid_glider/pullout/).
Whether to bend the trailing edge up and what drop height to use are Kai's decisions; the pack says so and recommends neither.
Light: stdlib only, no AVL.     py -B rigid_glider/field_pack.py [--out DIR]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import json
import math
import re
import subprocess
import zlib
from pathlib import Path

import flat_test as ft

ROOT = ft.ROOT
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/field_pack"
KIT = ROOT / "results/runs/rigid_glider/flat_test_v2"
BIRD = ROOT / "results/runs/rigid_glider/bird_outline_v2"
W, H = 215.9, 279.4
X0, X1 = 10.0, W - 10.0
CHAR_EM = 0.53                      # conservative average character width in em for Helvetica (wrapping only)
WEDGE_ANGLES = (2.0, 2.5, 4.0, 5.0, 6.0, 7.5, 8.0)
WEDGE_LEG_MM = 70.0                 # both legs of each wedge: long enough to hold against a bent tab or the bird tail (60 to 80 mm asked)
SCALE_BAR_MM = 100.0
LOG_ROWS = 20
OUTCOMES = (("S", "steady glide"), ("D", "dive"), ("P", "pitch-up or stall"), ("O", "pitch oscillation"), ("T", "tumble"), ("L", "roll-off left"), ("R", "roll-off right"), ("Y", "yaw wander"))   # FLAT_TEST.md vocabulary, exact


# ---------------------------------------------------------------------------------------------- svg helpers (rect / line / polygon / text only)
class Page:
    def __init__(self):
        self.o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">', f'<rect x="0" y="0" width="{W}" height="{H}" fill="#ffffff"/>']

    def text(self, x, y, s, size=3.8, bold=False, color="#000000"):
        s = html.escape(str(s), quote=False)
        self.o.append(f'<text x="{x:.2f}" y="{y:.2f}" font-family="Helvetica" font-size="{size}" fill="{color}"' + (' font-weight="bold"' if bold else "") + f'>{s}</text>')

    def line(self, x1, y1, x2, y2, w=0.3, color="#000000", dash=None, ident=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        i = f' id="{ident}"' if ident else ""
        self.o.append(f'<line{i} x1="{x1:.4f}" y1="{y1:.4f}" x2="{x2:.4f}" y2="{y2:.4f}" stroke="{color}" stroke-width="{w}"{d}/>')

    def rect(self, x, y, w, h, fill="none", stroke="#000000", sw=0.3):
        self.o.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{w:.3f}" height="{h:.3f}" fill="{fill}"' + (f' stroke="{stroke}" stroke-width="{sw}"' if stroke else "") + "/>")

    def poly(self, pts, fill="none", stroke="#000000", sw=0.35):
        p = " ".join(f"{x:.4f},{y:.4f}" for x, y in pts)
        self.o.append(f'<polygon points="{p}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def para(self, x, y, s, width, size=3.8, bold=False, lead=1.28):
        """Wrapped paragraph; returns the y below it."""
        for ln in wrap(s, width, size):
            self.text(x, y, ln, size, bold)
            y += size * lead
        return y

    def scale_bar(self, x, y):
        self.line(x, y, x + SCALE_BAR_MM, y, 0.5, ident="scalebar")
        for k in range(0, int(SCALE_BAR_MM) + 1, 10):
            self.line(x + k, y - 2.2, x + k, y + 2.2, 0.35)
        self.text(x, y + 7.0, "Scale bar: this line must measure exactly 100 mm. If it does not, print at 100 % (actual size).", 3.4)

    def svg(self):
        return "\n".join(self.o + ["</svg>"])


def wrap(s, width, size):
    per = max(8, int(width / (size * CHAR_EM)))
    words, lines, cur = s.split(), [], ""
    for w_ in words:
        if len(cur) + len(w_) + (1 if cur else 0) <= per:
            cur = (cur + " " + w_).strip()
        else:
            lines.append(cur)
            cur = w_
    if cur:
        lines.append(cur)
    return lines


# ---------------------------------------------------------------------------------------------- facts read from the kit outputs
def kit_facts():
    out = {}
    with (KIT / "ballast_table.csv").open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    k = list(rows[0].keys())
    for wing, thick, spec in (("F45", "0.6", "F45_span250"), ("F60", "0.6", "F60_span250"), ("B45", "5.0", "B45_span400")):
        cgs = sorted({float(r[k[2]]) for r in rows if r[k[0]] == wing and r[k[1]] == thick})
        g = json.loads((KIT / spec / "geometry.json").read_text(encoding="utf-8"))
        out[wing] = dict(cg_lo=cgs[0], cg_hi=cgs[-1], n=len(cgs), np_mm=g["avl_with_tip_fins"]["neutral_point_mm"]["value"], span=g["span_mm"]["value"])
    with (BIRD / "builds.csv").open(encoding="utf-8") as f:
        br = [r for r in csv.DictReader(f) if r["family"] == "BS" and float(r["bend_deg"]) == 5.0]
    out["BS_printed"] = float(next(r for r in br if r["size"] == "printed")["target_trim_CG_CL0.3_mm"])
    out["BS_foam"] = float(next(r for r in br if r["size"] == "foam")["target_trim_CG_CL0.3_mm"])
    return out


# ---------------------------------------------------------------------------------------------- page 1: test card
def page_card(F):
    p = Page()
    y = 17.0
    p.text(X0, y, "Test card: flat wings and the swift-like outline", 6.2, True)
    y += 6.0
    y = p.para(X0, y, "MODEL = from our computer model, not a fact. FIRM = follows from symmetry or from the project's own tests. Whether to bend the trailing edge up, and what drop height to use, are Kai's decisions; this card recommends neither.", X1 - X0, 3.6)
    y += 1.5
    p.text(X0, y, "1. Specimens: CG table, mm aft of the root leading edge", 4.6, True)
    y += 5.0
    for wing, desc in (("F45", "printed, span 250 mm"), ("F60", "printed, span 250 mm"), ("B45", "foamboard, span 400 mm")):
        f = F[wing]
        p.text(X0 + 2, y, f"{wing}  ({desc})", 4.2, True)
        p.text(X0 + 78, y, f"CG {f['cg_lo']:g} to {f['cg_hi']:g} mm, 1 mm steps ({f['n']} settings)", 4.2)
        y += 4.8
        p.text(X0 + 4, y, f"AVL neutral point with the tip fins: {f['np_mm']:.1f} mm (outline only, a place to start, not a result)", 3.5)
        y += 5.0
    y = p.para(X0 + 2, y, "Stop rule (unchanged): after the table, keep going aft in 1 mm steps while any setting still glides or still dives. Stop when, after at least one gliding setting, two settings in a row give no steady glide, or at the bare-aircraft balance point. Releases: 5 per setting, 10 at any setting that glides. Weigh the glider at each setting.", X1 - X0 - 4, 3.6)
    y += 2.0
    p.text(X0, y, "2. Swift-like outline (bird, version 2)", 4.6, True)
    y += 5.0
    y = p.para(X0 + 2, y, f"Tail bend 5 deg, trailing edge UP. Trim CG at lift coefficient 0.3: {F['BS_printed']:.1f} mm (printed, span 250 mm) and {F['BS_foam']:.1f} mm (foamboard, span 400 mm) aft of the root leading edge. MODEL (AVL outline model): a reference point, not a CG window, not a prediction.", X1 - X0 - 4, 3.8)
    y += 2.0
    p.text(X0, y, "3. Order of tests", 4.6, True)
    y += 5.0
    for t in ("a.  Weigh every part. Find the CG on a knife edge to 0.5 mm. Check the wing is flat and not twisted.",
              "b.  LEVEL HAND GLIDE first: gross trim only. One or two seconds sort a dive from a pitch-up; they cannot show a repeatable glide.",
              "c.  NOSE-DOWN DROP from height: hold it nose straight down, let go at rest. Same height every time. Height is Kai's decision.",
              "d.  Later, quad release (not built yet): hang level, release in forward flight, not from a hover (FLAT_TEST.md procedure).",
              "Outcome words, exactly: steady glide, dive, pitch-up or stall, pitch oscillation, tumble, roll-off left, roll-off right, yaw wander."):
        y = p.para(X0 + 2, y, t, X1 - X0 - 4, 3.6)
    p.text(X0, 272.0, "Sources: FLAT_TEST.md, flat_test_v2 kit tables, bird_outline_v2. Field pack, page 1 of 4.", 3.0, color="#444444")
    return p.svg()


def page_table():
    p = Page()
    y = 17.0
    p.text(X0, y, "What you see -> what it means -> what to change next", 5.6, True)
    y += 5.5
    y = p.para(X0, y, "MODEL = from our computer model, not a fact (results/runs/rigid_glider/pullout/). FIRM = follows from symmetry or from the project's own tests. Bending the trailing edge up and the drop height are Kai's decisions.", X1 - X0, 3.8)
    y += 2.0
    cols = (X0, X0 + 46, X0 + 122)
    cw = (44.0, 74.0, X1 - (X0 + 122))
    p.rect(X0, y, X1 - X0, 6.4, fill="#e6e6e6", sw=0.3)
    for c, t in zip(cols, ("What you see", "What it probably means", "What to change next")):
        p.text(c + 1.2, y + 4.4, t, 3.8, True)
    y += 6.4
    rows = [
        ("Darts straight down (log: dive)", "FIRM: released exactly nose-down with the CG ahead of the lift point, a flat symmetric wing has no moment to start pitching, so it stays a dart. Says nothing against the wing.",
         "Do the level hand glide first. Then move the CG aft 1 mm at a time (kit table). Bending the trailing edge up is Kai's decision."),
        ("Swoops, then glides (log: steady glide)", "MODEL: a first swoop of about 6 m or more is expected even for a wing that will glide, so a short drop cannot tell 'never pulls out' from 'needs more height'.",
         "Log distance and time; repeat 5 times (10 if it glides). Drop height is Kai's decision."),
        ("Mushes: flat, slow, steep descent (log: steady glide, note 'mush')", "Flying near stall. The model cannot say at which CG an untabbed flat wing settles. Cause not established.",
         "Check flatness and twist. Try the neighbouring settings, 1 mm forward and aft, and compare."),
        ("Nose up, stalls, repeats (pitch-up or stall; pitch oscillation)", "Seen behind the neutral point. On a swept, tapered wing the tips may let go first (unknown). Pitch oscillation is about the balance point and the wing, not the fins.",
         "Check twist. Move the CG forward 1 mm. Do not blame the fins."),
        ("Tumbles (tumble)", "In the published plate study, tumbling came with the centre of mass furthest aft (unswept plates; not our wings).",
         "Move the CG forward. The stop rule decides when the sweep ends."),
        ("Turns or spirals (roll-off left / right; yaw wander)", "Yaw wander (nose swings side to side) points at the fins. Roll-off points at a twisted or uneven wing.",
         "Check the fins are square, the wing flat and not twisted. Re-fly."),
    ]
    for a, b, c in rows:
        ls = [wrap(t, w_ - 2.4, 3.9) for t, w_ in zip((a, b, c), cw)]
        n = max(len(x) for x in ls)
        hgt = n * 3.9 * 1.25 + 2.8
        p.rect(X0, y, X1 - X0, hgt, sw=0.25)
        for col, xx in zip(ls, cols):
            yy = y + 4.0
            for ln in col:
                p.text(xx + 1.2, yy, ln, 3.9, bold=(col is ls[0]))
                yy += 3.9 * 1.25
        for xx in cols[1:]:
            p.line(xx, y, xx, y + hgt, 0.25)
        y += hgt
    assert y < 268, y
    p.text(X0, 273.5, "Sources: FLAT_TEST.md; pull-out study (MODEL). Field pack, page 2 of 4.", 3.0, color="#444444")
    return p.svg()




# ---------------------------------------------------------------------------------------------- page 2: flight log
LOG_COLS = (("No", 8), ("Wing", 14), ("Mass g", 15), ("CG mm", 16), ("Tab / bend deg", 17), ("Launch H/N", 15), ("Height m", 14), ("Result", 14), ("Dist m", 14), ("Time s", 13))


def page_log():
    p = Page()
    y = 16.0
    p.text(X0, y, "Flight log", 6.2, True)
    y += 7.5
    p.text(X0, y, "Date ____________________", 4.2)
    p.text(X0 + 68, y, "Place ______________________", 4.2)
    p.text(X0 + 140, y, "Wind ________________", 4.2)
    y += 8.0
    p.text(X0, y, "First measured masses (g):  F45 ______  F60 ______  B45 ______  bird ______", 4.0)
    y += 5.0
    p.text(X0, y, "Which wing was the 'about 15 g' is still unconfirmed: write the weighed masses here.", 3.4)
    y += 6.0
    y = p.para(X0, y, "Launch: H = level hand throw, N = nose-down drop.  CG = mm aft of the root leading edge (or rail tick).  Tab / bend in degrees, trailing edge up.", X1 - X0, 3.5)
    y = p.para(X0, y + 0.8, "Result (write the letter): " + ",  ".join(f"{c} = {n}" for c, n in OUTCOMES) + ".  A mush is logged as S with a note in Notes.", X1 - X0, 3.5)
    y += 1.5
    widths = [w for _, w in LOG_COLS]
    notes_w = (X1 - X0) - sum(widths)
    xs = [X0]
    for w_ in widths + [notes_w]:
        xs.append(xs[-1] + w_)
    hh = 9.0
    p.rect(X0, y, X1 - X0, hh, fill="#e6e6e6", sw=0.4)
    heads = [h_ for h_, _ in LOG_COLS] + ["Notes"]
    for i, hd in enumerate(heads):
        for k, ln in enumerate(wrap(hd, (xs[i + 1] - xs[i]) - 1.4, 3.1)[:2]):
            p.text(xs[i] + 0.8, y + 3.6 + 3.3 * k, ln, 3.1, True)
    y0 = y
    y += hh
    rh = (270.0 - y) / LOG_ROWS
    for r in range(LOG_ROWS):
        p.line(X0, y + r * rh, X1, y + r * rh, 0.25)
        p.text(xs[0] + 1.5, y + r * rh + rh * 0.68, str(r + 1), 3.4)
    p.line(X0, y + LOG_ROWS * rh, X1, y + LOG_ROWS * rh, 0.4)
    for xx in xs:
        p.line(xx, y0, xx, y + LOG_ROWS * rh, 0.3)
    p.text(X0, 273.5, "Field pack, page 3 of 4.", 3.0, color="#444444")
    return p.svg()


# ---------------------------------------------------------------------------------------------- page 3: angle gauge
def wedge_points(theta_deg, leg, x_apex, y_mid):
    """Isosceles wedge: apex at (x_apex, y_mid), both legs of length `leg`, angle between the legs exactly theta (bisector along +x)."""
    h = math.radians(theta_deg) / 2.0
    return [(x_apex, y_mid), (x_apex + leg * math.cos(h), y_mid - leg * math.sin(h)), (x_apex + leg * math.cos(h), y_mid + leg * math.sin(h))]


def page_gauge():
    p = Page()
    y = 16.0
    p.text(X0, y, "Angle gauge, 1:1", 6.2, True)
    y += 6.0
    y = p.para(X0, y, "Print at 100 % and check the 100 mm bar below. Cut out each wedge along its three sides. Lay one long edge along the wing and the other along the bent trailing-edge tab or the bird tail: the gap should close at the tip. The angle is the angle between the two long edges (computed, not drawn by eye).", X1 - X0, 3.8)
    y += 2.0
    p.scale_bar(X0 + 10, y + 3.0)
    y += 15.0
    ys = []
    gap = 24.5
    for i, a in enumerate(WEDGE_ANGLES):
        ymid = y + 8.0 + i * gap
        ys.append(ymid)
        pts = wedge_points(a, WEDGE_LEG_MM, X0 + 12, ymid)
        p.poly(pts, fill="#f2f2f2", sw=0.4)
        p.text(X0 + 12 + WEDGE_LEG_MM + 6, ymid - 1.0, f"{a:g} deg", 6.0, True)
        p.text(X0 + 12 + WEDGE_LEG_MM + 6, ymid + 5.0, f"long edges {WEDGE_LEG_MM:g} mm, end gap {2 * WEDGE_LEG_MM * math.sin(math.radians(a) / 2):.1f} mm", 3.4)
    assert ys[-1] + 12 < 270, ys[-1]
    p.text(X0, 273.5, "Field pack, page 4 of 4.", 3.0, color="#444444")
    return p.svg()


# ---------------------------------------------------------------------------------------------- checks on the finished PDF
def read_pdf_streams(path: Path):
    data = path.read_bytes()
    streams = []
    for m in re.finditer(rb"stream\n(.*?)\nendstream", data, re.S):
        raw = m.group(1)
        try:
            raw = zlib.decompress(raw)
        except Exception:
            pass
        streams.append(raw.decode("latin-1"))
    return data, streams


def check_pdf(path: Path):
    data, streams = read_pdf_streams(path)
    n_pages = len(re.findall(rb"/Type /Page\b(?!s)", data))
    mm = 72.0 / 25.4
    res = dict(pages=n_pages)
    bars = []
    for si, st in enumerate(streams):
        for m in re.finditer(r"([-\d.]+) ([-\d.]+) m ([-\d.]+) ([-\d.]+) l S", st):
            x1, y1, x2, y2 = (float(v) for v in m.groups())
            if abs(y1 - y2) < 1e-6 and abs(abs(x2 - x1) - SCALE_BAR_MM) < 0.05:
                bars.append((si, abs(x2 - x1) * mm))
    res["scale_bars_pt"] = bars
    wedges = []
    for si, st in enumerate(streams):
        for m in re.finditer(r"([-\d.]+) ([-\d.]+) m ([-\d.]+) ([-\d.]+) l ([-\d.]+) ([-\d.]+) l h", st):
            v = [float(x) for x in m.groups()]
            a, b, c = (v[0], v[1]), (v[2], v[3]), (v[4], v[5])
            u1, u2 = (b[0] - a[0], b[1] - a[1]), (c[0] - a[0], c[1] - a[1])
            ang = math.degrees(math.acos((u1[0] * u2[0] + u1[1] * u2[1]) / (math.hypot(*u1) * math.hypot(*u2))))
            wedges.append((si, ang, math.hypot(*u1), math.hypot(*u2)))
    res["wedges"] = wedges
    return res


def run(out: Path = OUT_DEFAULT, make_pdf=True):
    out.mkdir(parents=True, exist_ok=True)
    pages_dir = out / "pages"
    pages_dir.mkdir(exist_ok=True)
    F = kit_facts()
    svgs = [("page1_test_card.svg", page_card(F)), ("page2_what_you_see.svg", page_table()), ("page3_flight_log.svg", page_log()), ("page4_angle_gauge.svg", page_gauge())]
    paths = []
    for nm, txt in svgs:
        (pages_dir / nm).write_text(txt, encoding="utf-8")
        paths.append(pages_dir / nm)
    pdf = out / "field_pack.pdf"
    subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(pdf)] + [str(x) for x in paths], check=True, stdout=subprocess.DEVNULL)
    chk = check_pdf(pdf)
    (out / "index.html").write_text(
        f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Field pack</title><style>{ft.CSS}</style></head><body><main><h1>Field pack</h1>"
        f"<p>One printable PDF, four white US Letter pages: test card, what-you-see table, flight log, angle gauge at 1:1. Print at 100 % and check the 100 mm bar on the gauge page.</p>"
        f"<p><a href='field_pack.pdf'>field_pack.pdf</a>. Settings are those of FLAT_TEST.md and the kit tables; the pull-out statements are MODEL. Bending the trailing edge up and the drop height are Kai's decisions.</p></main></body></html>", encoding="utf-8")
    return dict(F=F, check=chk, pdf=pdf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    a = ap.parse_args()
    R = run(a.out)
    c = R["check"]
    print("pages:", c["pages"], "| scale bars (pt):", [(s, round(v, 2)) for s, v in c["scale_bars_pt"]], "| wedges:", [round(w[1], 3) for w in c["wedges"]])
    print("wrote", R["pdf"])


if __name__ == "__main__":
    main()
