"""CG-marking page for the four gliders Kai flies on 2026-10-20.

Writes ONLY results/runs/flight_day_2026-10-20/cg_marks.html.  Reads (never writes) existing study outputs; imports
bird_outline_v4_swift and flight_day_page read-only (geometry and ballast arithmetic; no AVL run, no file written).
Every number drawn is read from its source file here and checked against the figure Kai was given; mismatches are printed.

Run:  py -B rigid_glider/flight_day_cg_page.py
"""
import csv
import html
import json
import math
import re
import sys
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "rigid_glider"))
OUT = ROOT / "results/runs/flight_day_2026-10-20/cg_marks.html"
RUNS = ROOT / "results/runs"
B45D = RUNS / "rigid_glider/flat_test_v2/B45_span400"
FLAPS = RUNS / "rigid_glider/b45_centre_flaps"
SW = RUNS / "rigid_glider/bird_outline_v4_swift"
ZAN = ROOT / "research/known_good/zanonia"

MISMATCH = []
CHECKS = []


def rel(p):
    return Path(p).relative_to(ROOT).as_posix()


def want(label, got, expect, tol):
    ok = abs(got - expect) <= tol
    if not ok:
        MISMATCH.append(f"{label}: source {got:.3f} vs stated {expect:.3f}")
    return got


# ------------------------------------------------------------------------------------------------ geometry
def interp_x(poly, yv):
    """x values where the closed polygon crosses y = yv."""
    xs = []
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 - yv) * (y2 - yv) < 0 or (y1 == yv and y2 != yv):
            xs.append(x1 + (x2 - x1) * (yv - y1) / (y2 - y1))
    return sorted(xs)


def b45():
    g = json.load(open(B45D / "geometry.json"))
    v = lambda k: g[k]["value"]
    s = v("span_mm") / 2
    cr, tle, tte = v("root_chord_mm"), v("tip_le_x_mm"), v("tip_te_x_mm")
    poly = [(0.0, -s), (tle, -s), (tte, -s), (cr, 0.0), (tte, s), (tle, s), (0.0, 0.0)]
    poly = [(0.0, 0.0), (tle, s), (tte, s), (cr, 0.0), (tte, -s), (tle, -s)]      # LE root, tip LE, tip TE, root TE, mirrored
    # flaps: hinge at local TE - 10 mm, y 30..80 (ASSUMED reading of Kai's description; b45_centre_flaps README)
    te = lambda y: cr + (tte - cr) * y / s
    fl = [(te(30) - 10, 30), (te(80) - 10, 80), (te(80), 80), (te(30), 30)]
    d = json.load(open(FLAPS / "results.json"))
    row66 = [r for r in d["sweep"] if r["loc"] == "nominal" and r["xcg"] == 66.0 and r["flap"] == 10.0][0]
    cg_lo, cg_hi, cg_t = 63.0, 68.0, 66.0
    np_ = g["avl_with_tip_fins"]["neutral_point_mm"]["value"]
    want("B45 NP", np_, 70.2, 0.05)
    E = Edge(poly, 0.0, -v("stick_nose_ahead_of_root_le_mm"), "stick")
    return dict(E=E, poly=poly, span=v("span_mm"), root=cr, flap=fl, flap_y=(30, 80), band=(cg_lo, cg_hi), cg=cg_t, np=np_,
                trim_cl=row66["cl_trim"], stick=(-v("stick_nose_ahead_of_root_le_mm"), cr), stick_w=v("stick_width_mm"),
                fin=(v("fin_chord_mm"), v("fin_thickness_mm"), tle, tte, s), geo=g)


def parse_svg_numbers(path):
    t = Path(path).read_text(encoding="utf-8")
    pick = lambda pat: float(re.search(pat, t).group(1))
    return dict(np=pick(r"AVL neutral point, bend 6: ([\d.]+) mm"), cg=pick(r"AVL trim CG, CL 0.3, bend 6: ([\d.]+) mm"),
                hinge=pick(r"hinge line \(bend UP[^)]*\): ([\d.]+) mm"))


def swift(span, folder, stated):
    import bird_outline_v4_swift as s
    import flight_day_page as fd
    g = s.build_geometry4(s.GLIDE)
    sc = span
    poly = [(p[0] * sc, p[1] * sc) for p in g["full_outline"]]
    xh = g["x_h"] * sc
    yb = g["wb"] * sc
    # tail fan = the contiguous run of outline vertices behind the hinge, closed along the hinge line
    near = lambda yy: min(range(len(poly)), key=lambda i: abs(poly[i][0] - xh) + abs(poly[i][1] - yy))
    i1, i2 = near(-yb), near(yb)
    tail = poly[i1:i2 + 1]
    assert len(tail) == 7 and abs(tail[0][0] - xh) < 0.01 and abs(tail[-1][0] - xh) < 0.01, tail
    nums = parse_svg_numbers(SW / folder / "drawing.svg")
    want(f"{folder} NP", nums["np"], stated["np"], 0.05)
    want(f"{folder} CG", nums["cg"], stated["cg"], 0.05)
    want(f"{folder} hinge", nums["hinge"], stated["hinge"], 0.05)
    want(f"{folder} hinge vs geometry", xh, stated["hinge"], 0.06)
    # outlines.csv cross-check of CG and NP (A_as_measured row, 250 mm values, scaled)
    A = [r for r in csv.DictReader(open(SW / "outlines.csv", encoding="utf8")) if r["outline"] == "A_as_measured"][0]
    k = span / 250.0
    cg_csv = float(A["trim CG mm at 10% margin (250)"]) * k
    np_csv = (g["x_le_mac"] + float(A["NP % MAC"]) / 100 * g["mac"]) * sc
    want(f"{folder} CG vs outlines.csv", cg_csv, stated["cg"], 0.1)
    want(f"{folder} NP vs outlines.csv", np_csv, stated["np"], 0.1)
    n = fd.swift_numbers()["foam" if span == 400 else "printed"]["10"]
    want(f"{folder} ballast", n["ballast"], stated["ballast"], 0.006)
    want(f"{folder} hole", n["station"], stated["hole"], 0.01)
    # CG rail (read-only arithmetic, same call as flight_day_page.swift_numbers): where does its front end sit relative to the sheet nose tip?
    import bird_outline_v2 as v2
    rr = s.fake_res(cg_csv / span, 0.1)
    rr["trim"] = {c: dict(x_cg_span=cg_csv / span, static_margin=0.1) for c in v2.TRIM_CLS}
    nfull = v2.size_numbers(g, rr, "foam" if span == 400 else "printed", 6.0)
    assert abs(nfull["ballast_g"] - stated["ballast"]) < 0.006
    rail_front = nfull["rail"]["x_lo"]
    E = Edge(poly, 0.0, rail_front, "rail")
    assert abs(E.nose - min(p[0] for p in poly)) < 0.01 and abs(E.tail - max(p[0] for p in tail)) < 0.01, (E.nose, E.tail)
    assert abs(E.nose - stated["nose"]) < 0.01 and abs(E.tail - stated["tail"]) < 0.01, (folder, E.nose, E.tail)
    return dict(E=E, rail_front=rail_front, thick=nfull["t"], poly=poly, tail=tail, span=span, hinge=nums["hinge"], cg=nums["cg"], np=nums["np"], ballast=n["ballast"], hole=n["station"],
                xh=xh, yb=yb, root=xh, nose=min(p[0] for p in poly), tail_end=max(p[0] for p in tail), n=n, mac=g["mac"] * sc, csv=(cg_csv, np_csv))


def zanonia():
    geo = json.load(open(ZAN / "zanonia_geometry.json"))
    rows = list(csv.DictReader(open(ZAN / "zanonia_outline.csv")))
    k = 155.0 / (geo["span_in"] * 25.4)
    le = [(float(r["x_LE_in (aft of centre LE)"]) * 25.4 * k, float(r["s_in (from centreline)"]) * 25.4 * k) for r in rows]
    te = [(float(r["x_TE_in"]) * 25.4 * k, float(r["s_in (from centreline)"]) * 25.4 * k) for r in rows]
    right = le + te[::-1]
    poly = [(x, y) for x, y in right] + [(x, -y) for x, y in right[::-1] if y > 0]
    nj = json.load(open(RUNS / "known_good/zanonia/numbers.json"))["scale"][0]
    want("Zanonia CG", nj["cg_mm"], 17.2, 0.05)
    want("Zanonia hole x", nj["hole_mm"], 4.0, 0.06)
    cg = geo["cg_x_in"] * 25.4 * k
    want("Zanonia CG (geometry.json scaled)", cg, 17.2, 0.05)
    av = json.load(open(ZAN / "avl_results.json"))["results"]
    r4 = [r for r in av if r["region"] == 0.3 and r["theta"] == 4.0][0]
    xnp = r4["Xnp_m"] * 1000 * k           # AVL ran on the scan-size outline (span 155.12 mm); scaled to 155
    hole_d = 4.0                           # the printed STL hole (measured from zanonia_plate_155mm.stl: radius 2.0 mm); the pattern hole (2.9 mm) was enlarged for a blu-tack plug
    return dict(E=Edge(poly, 0.0), poly=poly, span=155.0, root=nj["chord_root"], cg=cg, np=xnp, np_raw=r4["Xnp_m"] * 1000, hole=nj["hole_mm"], hole_d=hole_d, k=k,
                curl_from=0.7 * 155.0 / 2)


# ------------------------------------------------------------------------------------------------ svg renderer
class Draw:
    def __init__(self, poly, f, extra_x_lo=0.0, extra_x_hi=0.0, side=10.0, top=15.0, bottom=30.0, minw=0.0):
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        self.xlo = min(xs + [extra_x_lo])
        self.xhi = max(xs + [extra_x_hi])
        half = max(max(abs(y) for y in ys) + side, minw / 2)
        self.cx, self.top, self.f = half, top, f
        self.W = 2 * half
        self.H = top + (self.xhi - self.xlo) + bottom
        self.o = []
        self.poly = poly

    def P(self, x, y):
        return self.cx + y, self.top + (x - self.xlo)

    def add(self, s):
        self.o.append(s)

    def line(self, x1, y1, x2, y2, cls, w=0.3, dash=None):
        a, b = self.P(x1, y1)
        c, d = self.P(x2, y2)
        self.add(f'<line x1="{a:.2f}" y1="{b:.2f}" x2="{c:.2f}" y2="{d:.2f}" class="{cls}" stroke-width="{w}"' + (f' stroke-dasharray="{dash}"' if dash else "") + "/>")

    def poly_el(self, pts, cls, extra=""):
        self.add('<polygon points="' + " ".join("%.2f,%.2f" % self.P(x, y) for x, y in pts) + f'" class="{cls}"{extra}/>')

    def text(self, x, y, s, cls="tx", size=None, anchor="start", bold=False):
        a, b = self.P(x, y)
        self.add(f'<text x="{a:.2f}" y="{b:.2f}" class="{cls}" font-size="{size or self.f}" text-anchor="{anchor}"' + (' font-weight="bold"' if bold else "") + f">{html.escape(s)}</text>")

    def rawtext(self, sx, sy, s, cls="tx", size=None, anchor="start", bold=False):
        self.add(f'<text x="{sx:.2f}" y="{sy:.2f}" class="{cls}" font-size="{size or self.f}" text-anchor="{anchor}"' + (' font-weight="bold"' if bold else "") + f">{html.escape(s)}</text>")

    def ruler(self, lo, hi):
        f = self.f * 0.8
        k = math.ceil(lo / 5) * 5
        while k <= hi + 1e-9:
            big = k % 10 == 0
            self.line(k, -(2.2 if big else 1.2), k, 2.2 if big else 1.2, "ink", 0.22 if big else 0.16)
            if big:
                self.text(k + f * 0.35, -3.0, f"{k:d}", "tx mut", f, "end")
            k += 5

    def crosshair(self, x, r, cls, tcls):
        a, b = self.P(x, 0)
        self.add(f'<circle cx="{a:.2f}" cy="{b:.2f}" r="{r:.2f}" class="{cls} nofill" stroke-width="{r * 0.22:.2f}"/>')
        self.line(x, -1.9 * r, x, 1.9 * r, cls, r * 0.16)
        self.line(x - r * 1.9, 0, x + r * 1.9, 0, cls, r * 0.16)
        self.add(f'<circle cx="{a:.2f}" cy="{b:.2f}" r="{r * 0.16:.2f}" class="{tcls}"/>')

    def dim(self, x1, x2, y, cls, txt, tcls, size):
        """dimension arrow parallel to the centreline at lateral offset y, with dotted extension lines to the centreline."""
        h = size * 0.7
        self.line(x1, y, x2, y, cls, 0.35)
        for x, sg in ((x1, 1), (x2, -1)):
            self.line(x, y, x + sg * h, y - h * 0.35, cls, 0.35)
            self.line(x, y, x + sg * h, y + h * 0.35, cls, 0.35)
        self.line(x1, 0, x1, y, cls, 0.12, "1 1")
        self.line(x2, 0, x2, y, cls, 0.12, "1 1")
        self.text((x1 + x2) / 2 + size * 0.35, y - 1.0, txt, "tx " + tcls, size, "end", True)

    def svg(self, sid, title_pts=None):
        return (f'<svg id="{sid}" class="g" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.W:.2f} {self.H:.2f}" width="{self.W:.1f}mm" height="{self.H:.1f}mm" '
                f'style="--w:{self.W:.1f}mm" role="img" aria-label="{html.escape(sid)}">'
                f'<rect width="{self.W:.2f}" height="{self.H:.2f}" class="bgr"/>' + "".join(self.o) + "</svg>")


def measure(poly_svg_mm):
    xs = [p[0] for p in poly_svg_mm]
    return max(xs) - min(xs)


def finish_common(d, span, bar, instr_lines, title):
    """title, scale bar, instruction at the foot, all in svg mm coordinates."""
    f = d.f
    d.rawtext(4, f * 1.4, title, "tx", f * 1.15, "start", True)
    y0 = d.H - 4 - f * 1.4 * (len(instr_lines))
    for i, s in enumerate(instr_lines):
        d.rawtext(4, d.H - 4 - f * 1.35 * (len(instr_lines) - 1 - i), s, "tx" + (" tcg" if i == 0 else ""), f * (1.0 if i == 0 else 0.85), "start", i == 0)
    # scale bar bottom right
    x1 = d.W - 4
    x0 = x1 - bar
    yb = d.H - 6 - f * 1.2
    d.add(f'<line x1="{x0:.2f}" y1="{yb:.2f}" x2="{x1:.2f}" y2="{yb:.2f}" class="ink" stroke-width="0.6"/>')
    for t in range(0, int(bar) + 1, 10 if bar > 60 else 5):
        h = 2.2 if t % (50 if bar > 60 else 10) == 0 else 1.2
        d.add(f'<line x1="{x0 + t:.2f}" y1="{yb - h:.2f}" x2="{x0 + t:.2f}" y2="{yb:.2f}" class="ink" stroke-width="0.3"/>')
    d.rawtext(x1, yb - 3.2, f"scale bar {bar:g} mm: check with a ruler", "tx", f * 0.8, "end")


# ------------------------------------------------------------------------------------------------ edge-referenced positions
REQ = []          # every edge-referenced number string that must appear verbatim in the page (asserted after writing)


def req(s):
    REQ.append(s)
    return s


class Edge:
    """Positions measured from physical edges on the centreline: the nose tip (frontmost sheet point) and the tail tip (rearmost sheet point)."""

    def __init__(self, poly, root_le=0.0, front=None, front_name=None):
        xs = interp_x(poly, 1e-4)                      # crossings of the centreline: first = nose tip, last = tail tip
        self.nose, self.tail = xs[0], xs[-1]
        self.root_le, self.front, self.front_name = root_le, front, front_name

    def n(self, x):
        return x - self.nose

    def t(self, x):
        return self.tail - x

    def r(self, x):
        return x - self.root_le

    def fr(self, x):
        return x - self.front

    def head(self, x):
        return req(f"{self.n(x):.1f} mm back from the nose tip ({self.t(x):.1f} mm forward of the tail tip)")

    def sub(self, x):
        return req(f"= {self.r(x):.1f} mm behind the wing root LE")

    def row(self, x):
        out = f"<b>{req('%.1f' % self.n(x))}</b> mm back from the nose tip; {req('%.1f' % self.t(x))} mm forward of the tail tip; (= {req('%.1f' % self.r(x))} behind the wing root LE)"
        if self.front is not None:
            out += f"; {req('%.1f' % self.fr(x))} mm from the {self.front_name} front end (alternative)"
        return out


def edge_ticks(d, E, f, half):
    for x in (E.nose, E.tail):
        d.line(x, -half, x, half, "ink", 0.5)
    d.text(E.nose - 0.4, half + 1.5, "NOSE TIP: measure from here", "tx tflap", f * 0.95, "start", True)
    d.text(E.tail + f * 1.0, half + 1.5, "TAIL TIP: measure from here", "tx tflap", f * 0.95, "start", True)


def edge_arrows(d, E, items, f, y0, sp):
    """items: (x_from, x_to, line class, text class); drawn on the left of the centreline, one lane each, number = length."""
    for i, (xa, xb, cls, tcls) in enumerate(items):
        d.dim(xa, xb, -(y0 + i * sp), cls, req("%.1f" % abs(xb - xa)), tcls, f * 0.95)


def legend(d, lines, f, step, x0=None, y0=None):
    """lines: (text, text class, size multiplier, bold).  Default place: bottom margin under the lowest drawn point."""
    if x0 is None:
        x0 = d.xhi + f * 3.0
    for i, (txt, tcls, mult, bold) in enumerate(lines):
        if y0 is None:
            d.rawtext(4, d.top + (x0 - d.xlo) + i * step, txt, "tx " + tcls, f * mult, "start", bold)
        else:
            d.text(x0 + i * step, y0, txt, "tx " + tcls, f * mult, "start", bold)


# ---- individual drawings
def draw_b45(b):
    f = 5.0
    E = b["E"]
    d = Draw(b["poly"], f, extra_x_lo=b["stick"][0], extra_x_hi=172, side=14, top=16, bottom=34)
    d.poly_el(b["poly"], "ol")
    # CG band shaded across the span (clipped to the outline)
    d.add('<defs><clipPath id="clipB45"><polygon points="' + " ".join("%.2f,%.2f" % d.P(x, y) for x, y in b["poly"]) + '"/></clipPath></defs>')
    lo, hi = b["band"]
    a1, b1 = d.P(lo, -b["span"] / 2)
    a2, b2 = d.P(hi, b["span"] / 2)
    d.add(f'<rect x="{a1:.2f}" y="{b1:.2f}" width="{a2 - a1:.2f}" height="{b2 - b1:.2f}" class="band" clip-path="url(#clipB45)"/>')
    # stick (printed spine under the wing)
    sx0, sx1 = b["stick"]
    d.poly_el([(sx0, -b["stick_w"] / 2), (sx0, b["stick_w"] / 2), (sx1, b["stick_w"] / 2), (sx1, -b["stick_w"] / 2)], "stick")
    # tip fins (top view: thin edge), butt-glued at the tip edge
    fc, ft, tle, tte, s = b["fin"]
    for sg in (1, -1):
        d.poly_el([(tle, sg * s), (tte, sg * s), (tte, sg * (s + ft)), (tle, sg * (s + ft))], "fin")
    d.text(tle + 4, b["span"] / 2 + ft + 1.5, "tip fin", "tx tfin", f * 0.8, "end")
    # flaps
    for sg in (1, -1):
        d.poly_el([(x, sg * y) for x, y in b["flap"]], "flap")
    fx = (b["flap"][0][0] + b["flap"][2][0]) / 2
    d.text(b["flap"][2][0] + 6, 70, "centre flap (ASSUMED): 10 mm chord,", "tx tflap", f * 0.85)
    d.text(b["flap"][2][0] + 6 + f * 1.0, 70, "bent ~10 deg trailing edge UP", "tx tflap", f * 0.85)
    d.ruler(b["stick"][0], 170)
    d.line(b["stick"][0] - 2, 0, 172, 0, "ink", 0.15, "3 2")
    # NP, CG band edges, target line
    full = b["span"] / 2 + 8
    d.line(b["np"], -full, b["np"], full, "np", 0.55, "3 2")
    d.line(b["cg"], -full, b["cg"], full, "cg", 0.9)
    d.line(b["band"][0], -full, b["band"][0], full, "cg", 0.2, "1 1")
    d.line(b["band"][1], -full, b["band"][1], full, "cg", 0.2, "1 1")
    d.crosshair(b["cg"], 8.0, "cg", "tcgfill")
    d.text(b["band"][0] - 3.0, 17, req(f"CG {E.n(b['cg']):.1f} from nose tip"), "tx tcg", f * 1.25, "start", True)
    d.text(b['band'][0] - 9.5, 17, f"model band {E.n(b['band'][0]):.1f}-{E.n(b['band'][1]):.1f} mm from the nose tip (shaded), target tick {E.n(b['cg']):.1f}", "tx tcg", f * 0.85)
    d.text(b["np"] + 6.5, 17, req(f"NP {E.n(b['np']):.1f} from nose tip: NEVER balance behind this line"), "tx tnp", f * 0.95, "start", True)
    d.text(b["stick"][0] + 6, -8, "printed stick under the wing (nose forward)", "tx mut", f * 0.8, "end")
    edge_ticks(d, E, f, 8.0)
    d.line(E.front, -8.0, E.front, 8.0, "ink", 0.5)
    d.text(E.front - 0.4, 9.5, "STICK FRONT END (alternative zero)", "tx tflap", f * 0.95, "start", True)
    edge_arrows(d, E, [(E.nose, b["cg"], "cg", "tcg"), (E.nose, b["np"], "np", "tnp"), (b["cg"], E.tail, "cg", "tcg"), (E.nose, E.tail, "ink", "mut"), (E.front, b["cg"], "cg", "tcg")], f, 12.0, f * 2.9)
    legend(d, [("CG: " + E.head(b["cg"]), "tcg", 1.0, True),
               (req(f"From the STICK FRONT END instead: CG {E.fr(b['cg']):.1f} mm (the stick sticks out {-E.front:.1f} mm ahead of the nose tip)"), "tcg", 0.8, False),
               (E.sub(b["cg"]) + " (the sheet nose tip IS the wing root LE)", "mut", 0.8, False),
               ("NEVER BALANCE BEHIND the neutral point: " + E.head(b["np"]), "tnp", 0.78, True),
               (req(f"From the stick front end instead: NP {E.fr(b['np']):.1f} mm"), "tnp", 0.8, False)], f, 5.6, x0=E.front + 6, y0=24.0)
    finish_common(d, b["span"], 100,
                  ["Balance on a knife edge across the span at the green line.",
                   "Dotted green lines = model band edges; red dashed = neutral point (unstable behind it).  Measure where yours balances NOW first, then record it."],
                  "B45 foamboard, span 400 mm, top view, nose up")
    return d


def draw_swift(sw, label, foam):
    f = 5.0 if foam else 3.9
    E = sw["E"]
    NL = 8
    d = Draw(sw["poly"], f, extra_x_hi=sw["tail_end"] + 2, side=(12 if foam else 10), top=15, bottom=(34 if foam else 30) + NL * f * 1.3 + 3 * f)
    d.poly_el(sw["poly"], "ol")
    d.poly_el(sw["tail"], "tail")
    d.ruler(math.ceil(sw["nose"] / 10) * 10 - 5, sw["tail_end"] + 2)
    d.line(sw["nose"] - 1, 0, sw["tail_end"] + 2, 0, "ink", 0.15, "3 2")
    full = sw["span"] / 2 + (8 if foam else 6)
    # NP red, CG green, hinge orange
    d.line(sw["np"], -full, sw["np"], full, "np", 0.55 if foam else 0.45, "3 2")
    d.line(sw["cg"], -full, sw["cg"], full, "cg", 0.9 if foam else 0.7)
    d.line(sw["hinge"], -sw["yb"], sw["hinge"], sw["yb"], "bend", 1.4 if foam else 1.0)
    r = 7.0 if foam else 5.0
    d.crosshair(sw["cg"], r, "cg", "tcgfill")
    d.text(sw["cg"] - f * 0.5, r * 2.4, req(f"CG {E.n(sw['cg']):.1f} from nose tip"), "tx tcg", f * 1.25, "start", True)
    d.text(sw["np"] + f * 1.2, r * 2.4, req(f"NP {E.n(sw['np']):.1f} from nose tip: NEVER behind"), "tx tnp", f * 0.95, "start", True)
    d.text(sw["hinge"] + f * 1.25, sw["yb"] + 3, req(f"BEND LINE {E.n(sw['hinge']):.1f} from nose tip: tail TE UP 6 deg"), "tx tbend", f * 1.05, "start", True)
    d.text(sw["hinge"] + f * 2.5, sw["yb"] + 3, "(bend along this orange line only; wings stay flat)", "tx tbend", f * 0.8, "start")
    # nose hole
    a, b = d.P(sw["hole"], 0)
    d.add(f'<circle cx="{a:.2f}" cy="{b:.2f}" r="{1.6 if foam else 1.1}" class="hole" stroke-width="0.3"/>')
    d.text(sw["hole"] + f * 0.3, 11.0 if not foam else 12.0, f"nose hole {sw['hole']:g} mm ({E.n(sw['hole']):.1f} from nose tip)" + (f": {sw['ballast']:.2f} g" if foam else f": {sw['ballast']:.2f} g (printed)"), "tx mut", f * 0.8, "start")
    edge_ticks(d, E, f, 8 if foam else 6)
    edge_arrows(d, E, [(E.nose, sw["cg"], "cg", "tcg"), (E.nose, sw["np"], "np", "tnp"), (E.nose, sw["hinge"], "bend", "tbend"),
                       (sw["cg"], E.tail, "cg", "tcg"), (E.nose, E.tail, "ink", "mut")], f, 9.0 if foam else 7.0, f * 2.9)
    rf = sw["rail_front"]
    legend(d, [
        ("CG: " + E.head(sw["cg"]), "tcg", 1.1, True), (E.sub(sw["cg"]), "mut", 0.8, False),
        ("NEVER BALANCE BEHIND the neutral point: " + E.head(sw["np"]), "tnp", 1.0, True), (E.sub(sw["np"]) + " (AVL)", "mut", 0.8, False),
        ("THE ONLY BEND (tail trailing edge UP 6 deg): " + E.head(sw["hinge"]), "tbend", 1.0, True), (E.sub(sw["hinge"]) + "; bend along this orange line only, wings stay flat", "mut", 0.8, False),
        (req(f"Alternative zero, the CG rail front end (printed rail under the sheet): CG {E.fr(sw['cg']):.1f}, NP {E.fr(sw['np']):.1f}, bend line {E.fr(sw['hinge']):.1f} mm back from the rail front end."), "mut", 0.8, False),
        (req(f"The rail does NOT stick out: its front end is {E.n(rf):.1f} mm behind the nose tip, hidden under the sheet. Measure from the sheet nose tip."), "mut", 0.8, False)], f, f * 1.3)
    finish_common(d, sw["span"], 100 if foam else 50,
                  ["Balance on a knife edge across the span at the green line.",
                   "Shaded orange = tail fan, the only part that bends.  Body edge and wrist lines are NOT drawn: they are marker lines only, do not bend or crease there."],
                  label)
    return d


def draw_zan(z):
    f = 2.7
    E = z["E"]
    d = Draw(z["poly"], f, extra_x_hi=59.0, side=7, top=11, bottom=22 + 4 * f * 1.3 + 3 * f, minw=140)
    d.poly_el(z["poly"], "ol")
    # curl region: outer 30 percent of each semi-span
    d.add('<defs><clipPath id="clipZan"><polygon points="' + " ".join("%.2f,%.2f" % d.P(x, y) for x, y in z["poly"]) + '"/></clipPath></defs>')
    c0 = z["curl_from"]
    s = z["span"] / 2
    for sg in (1, -1):
        a1, b1 = d.P(-5, sg * c0)
        a2, b2 = d.P(60, sg * s)
        d.add(f'<rect x="{min(a1, a2):.2f}" y="{b1:.2f}" width="{abs(a2 - a1):.2f}" height="{b2 - b1:.2f}" class="curl" clip-path="url(#clipZan)"/>')
        d.line(0, sg * c0, 60, sg * c0, "bend", 0.25, "1.5 1")
    d.ruler(0, 40)
    d.line(-1.5, 0, 42, 0, "ink", 0.1, "2 1.5")
    full = s + 4
    d.line(z["np"], -full, z["np"], full, "np", 0.22, "2 1.2")
    d.line(z["cg"], -full, z["cg"], full, "cg", 0.4)
    d.crosshair(z["cg"], 3.2, "cg", "tcgfill")
    d.text(z["cg"] - 1.5, 7.5, req(f"CG {E.n(z['cg']):.1f} from nose tip"), "tx tcg", f * 1.3, "start", True)
    d.text(z["np"] + 5.0, 8.0, req(f"NP {E.n(z['np']):.1f} from nose tip"), "tx tnp", f * 0.95, "start", True)
    d.text(z["np"] + 8.5, 8.0, "NEVER behind (0.3 mm behind CG)", "tx tnp", f * 0.8, "start")
    edge_ticks(d, E, f, 4.0)
    edge_arrows(d, E, [(E.nose, z["cg"], "cg", "tcg"), (E.nose, z["np"], "np", "tnp"), (z["cg"], E.tail, "cg", "tcg"), (E.nose, E.tail, "ink", "mut")], f, 9.5, f * 2.9)
    legend(d, [("CG: " + E.head(z["cg"]), "tcg", 1.1, True), (E.sub(z["cg"]) + " (the nose, centre LE, is the nose tip)", "mut", 0.8, False),
               ("NEVER BALANCE BEHIND the neutral point: " + E.head(z["np"]), "tnp", 1.0, True), (E.sub(z["np"]) + " (AVL; essentially neutral)", "mut", 0.8, False)], f, f * 1.3)
    a, b = d.P(z["hole"], 0)
    d.add(f'<circle cx="{a:.2f}" cy="{b:.2f}" r="{z["hole_d"] / 2:.2f}" class="hole" stroke-width="0.2"/>')
    d.text(z["hole"] + 1.0, 12.0, f"nose hole at {z['hole']:.1f} mm (dia {z['hole_d']:.1f})", "tx mut", f * 0.8, "start")
    for sg in (1, -1):
        d.text(53.0, sg * (c0 + s) / 2, "curl region: outer 30%", "tx tbend", f * 0.8, "middle")
        d.text(56.5, sg * (c0 + s) / 2, "tips twisted TE UP, start 4 deg", "tx tbend", f * 0.8, "middle")
    finish_common(d, z["span"], 50,
                  ["Balance on a knife edge across the span at the green line (the raised cross).",
                   "Orange hatch = curl region (outer 30% of each semi-span) twisted trailing edge UP; the rest stays flat."],
                  "Zanonia printed, span 155 mm, top view, nose up")
    return d


# ------------------------------------------------------------------------------------------------ page
CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--fill:#27313c;--cg:#6cc08b;--np:#ff8a80;--bend:#ff8a5c;--tailfill:#3d3326;--bandfill:rgba(108,192,139,.22);--stick:#9aa7b4;--finc:#e89a55;--flapc:#e89a55;--flapfill:#4a3523;--holefill:#14181d;--curlfill:rgba(232,154,85,.25);--thead:#262e37}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 "Segoe UI",system-ui,sans-serif}
main{max-width:1100px;margin:0 auto;padding:28px 20px 64px;display:flex;flex-direction:column;gap:26px}
h1{font-size:28px;margin:0}h2{font-size:22px;margin:0;border-bottom:1px solid var(--line);padding-bottom:4px}p{margin:0;max-width:84ch}
section{display:flex;flex-direction:column;gap:12px}
.box{background:var(--panel);border:2px solid var(--accent);padding:12px 16px;display:flex;flex-direction:column;gap:8px}
.card{background:var(--panel);border:1px solid var(--line);padding:12px 14px;display:flex;flex-direction:column;gap:10px}
.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:15px}
.warn{border-left:3px solid var(--np);padding:2px 0 2px 12px;font-size:15px}
a{color:var(--link)}code{font-size:13.5px}
.src{font-size:12.5px;color:var(--muted);max-width:110ch}
table{border-collapse:collapse;font-size:14.5px}th,td{border:1px solid var(--line);padding:4px 8px;text-align:left;vertical-align:top}th{background:var(--thead)}
button{background:var(--panel);color:var(--ink);border:2px solid var(--accent);padding:8px 14px;font:inherit;cursor:pointer;width:max-content}
.scroll{overflow-x:auto}
svg.g{display:block;width:100%;height:auto;background:var(--bg)}
svg .bgr{fill:var(--bg)}svg .ol{fill:var(--fill);stroke:var(--ink);stroke-width:.5}
svg .ink{stroke:var(--ink)}svg .cg{stroke:var(--cg);fill:none}svg .np{stroke:var(--np);fill:none}svg .bend{stroke:var(--bend);fill:none}
svg .nofill{fill:none}svg .tcgfill{fill:var(--cg);stroke:none}
svg .band{fill:var(--bandfill)}svg .tail{fill:var(--tailfill);stroke:var(--bend);stroke-width:.4}
svg .stick{fill:none;stroke:var(--stick);stroke-width:.3;stroke-dasharray:2 1.5}
svg .fin{fill:var(--flapfill);stroke:var(--finc);stroke-width:.3}svg .flap{fill:var(--flapfill);stroke:var(--flapc);stroke-width:.4}
svg .hole{fill:var(--holefill);stroke:var(--ink)}svg .curl{fill:var(--curlfill)}
svg text{font-family:"Segoe UI",Arial,sans-serif;fill:var(--ink);paint-order:stroke;stroke:var(--bg);stroke-width:1.1px;stroke-linejoin:round}
svg text.mut{fill:var(--muted)}svg text.tcg{fill:var(--cg)}svg text.tnp{fill:var(--np)}svg text.tbend{fill:var(--bend)}svg text.tflap,svg text.tfin{fill:var(--accent)}
"""
PRINT_VARS = ("--bg:#fff;--panel:#fff;--line:#999;--ink:#000;--muted:#333;--link:#000;--accent:#000;--fill:#fff;--cg:#000;--np:#000;--bend:#000;"
              "--tailfill:#e6e6e6;--bandfill:rgba(0,0,0,.12);--stick:#000;--finc:#000;--flapc:#000;--flapfill:#ddd;--holefill:#fff;--curlfill:rgba(0,0,0,.15);--thead:#eee")
PRINT_CSS = (f"body.print1{{color-scheme:light;{PRINT_VARS}}}body.print1 svg.g{{width:var(--w);max-width:none}}body.print1 .noprint{{display:none}}"
             f"@page{{size:432mm 279mm;margin:5mm}}"
             f"@media print{{:root{{color-scheme:light;{PRINT_VARS}}}svg.g{{width:var(--w);max-width:none}}.noprint{{display:none}}"
             "main{padding:0;max-width:none;gap:0}section{break-after:page;page-break-after:always;gap:4px}.card{border:0;padding:0}h2{font-size:14px}}")


def tb(rows):
    return ("<div class='scroll'><table><tr><th>Mark</th><th>Value (mm along the centreline, FROM THE NOSE TIP first; then from the tail tip; then from the wing root LE)</th><th>Source</th></tr>"
            + "".join(f"<tr><td>{a}</td><td>{b}</td><td class='src'>{c}</td></tr>" for a, b, c in rows) + "</table></div>")


def edge_note(E, bend_tail_len=0.0, extra=""):
    """How to measure, per glider.  bend_tail_len = length of the part that is bent up (tail behind the hinge), 0 if flat."""
    t = ("<p class='note'><b>How to measure.</b> Measure along the flat sheet on the centreline, starting at the NOSE TIP (the frontmost point of the sheet, on the centreline); "
         "the TAIL TIP is the rearmost point of the sheet on the centreline. Measure from the sheet, not from the printed rail or stick end. ")
    if bend_tail_len:
        loss = bend_tail_len * (1 - math.cos(math.radians(6.0)))
        t += (f"The CG, neutral point and bend line are at or ahead of the hinge, so the 6 deg tail bend does not change them. Only distances to the tail tip are affected: "
              f"measured along the sheet surface they are unchanged, but a straight ruler laid over the bent-up tail reads {loss:.2f} mm less ({bend_tail_len:.1f} mm of tail x (1 - cos 6 deg)); "
              "take tail-tip distances with the tail laid flat, or work from the nose tip. ")
    else:
        t += "This glider has no bend at the centreline, so flat and assembled distances are the same. "
    return t + extra + "</p>"


def section(sid, title, d, rows, caption, check):
    return (f"<section><h2>{title}</h2><div class='card'><div class='scroll'>{d.svg(sid)}</div>"
            f"<p class='src'>Scale check (measured on the plotted outline): {check}</p>"
            f"{tb(rows)}<p class='src'>{caption}</p></div></section>")


def scale_check(poly, d, span, root, y_at, label):
    """measure on the SVG-plane points (back-transformed from the drawn coordinates)."""
    pts = [d.P(x, y) for x, y in poly]                 # (sx, sy) in svg mm
    sp = max(p[0] for p in pts) - min(p[0] for p in pts)
    # root chord: scanline at svg x = cx + y_at
    pp = [(p[1], p[0] - d.cx) for p in pts]            # (x along chord, y)
    xs = interp_x(pp, y_at)
    ch = (xs[-1] - xs[0]) if label != "swift" else None
    return sp, xs


def back2(d, poly):
    pts = [d.P(x, y) for x, y in poly]
    return [(p[1] - d.top + d.xlo, p[0] - d.cx) for p in pts]


def chk_edge(b, f4, p2, z):
    """Edge-referenced numbers, geometry-derived (Edge) against the figures computed by hand for Kai; assertion error if they disagree."""
    def eq(label, got, exp, tol=0.06):
        assert abs(got - exp) <= tol, f"{label}: {got:.3f} vs {exp:.3f}"
    for tag, s, ex in (("foam400", f4, dict(cg=(73.0, 71.5), hinge=93.5, np=77.7)), ("250", p2, dict(cg=(45.6, 44.7), hinge=58.5, np=48.6))):
        E = s["E"]
        eq(tag + " CG nose", E.n(s["cg"]), ex["cg"][0])
        eq(tag + " CG tail", E.t(s["cg"]), ex["cg"][1])
        eq(tag + " hinge nose", E.n(s["hinge"]), ex["hinge"])
        eq(tag + " NP nose", E.n(s["np"]), ex["np"])
    E = b["E"]
    eq("B45 nose", E.nose, 0.0)
    eq("B45 tail", E.tail, 118.5, 0.06)
    eq("B45 stick front", E.front, -47.4, 0.06)
    eq("B45 CG from stick front", E.fr(b["cg"]), 113.4, 0.06)
    eq("Zanonia nose", z["E"].nose, 0.0)
    assert 36.9 < z["E"].tail < 37.2, z["E"].tail


def main():
    b = b45()
    f4 = swift(400, "A_foam_span400", dict(np=41.7, cg=37.0, hinge=57.5, ballast=1.21, hole=-12.5, nose=-36.0, tail=108.46))
    p2 = swift(250, "A_printed_span250", dict(np=26.1, cg=23.1, hinge=36.0, ballast=1.72, hole=-2.5, nose=-22.5, tail=67.79))
    z = zanonia()
    chk_edge(b, f4, p2, z)

    d1 = draw_b45(b)
    d2 = draw_swift(f4, "Swift A foamboard, span 400 mm, top view, nose up", True)
    d3 = draw_swift(p2, "Swift A 250 mm (foamboard or printed), top view, nose up", False)
    d4 = draw_zan(z)

    chk = []
    # --- scale checks on the plotted (svg-plane) outlines
    def measure_all(d, poly, stated_span, stated_root, y_at, tag, tail_ok=False):
        pts = [d.P(x, y) for x, y in poly]
        span = max(p[0] for p in pts) - min(p[0] for p in pts)
        back = [(p[1] - d.top + d.xlo, p[0] - d.cx) for p in pts]      # back to (x,y) from drawn svg coordinates
        xs = interp_x(back, y_at)
        return span, xs

    s1, xs1 = measure_all(d1, b["poly"], 400, b["root"], 0.0001, "B45")
    r1 = xs1[-1] - xs1[0]
    s2, xs2 = measure_all(d2, f4["poly"], 400, f4["xh"], 20.0, "S4")
    r2 = min((p for p in back2(d2, f4["poly"])), key=lambda p: abs(p[0] - f4["xh"]) + abs(abs(p[1]) - f4["yb"]))[0]
    s3, xs3 = measure_all(d3, p2["poly"], 250, p2["xh"], 12.5, "S2")
    r3 = min((p for p in back2(d3, p2["poly"])), key=lambda p: abs(p[0] - p2["xh"]) + abs(abs(p[1]) - p2["yb"]))[0]
    s4, xs4 = measure_all(d4, z["poly"], 155, z["root"], 0.0001, "Z")
    r4 = xs4[-1] - xs4[0]
    results = [("B45", s1, 400.0, r1, b["root"], 0.0), ("Swift foam 400", s2, 400.0, r2, f4["xh"], 0), ("Swift printed 250", s3, 250.0, r3, p2["xh"], 0), ("Zanonia 155", s4, 155.0, r4, z["root"], 0)]
    ok_all = True
    lines = []
    for name, sm, ss, rm, rs, _ in results:
        ok = abs(sm - ss) <= 0.5 and abs(rm - rs) <= 0.5
        ok_all &= ok
        lines.append(f"{name}: span {sm:.2f} vs {ss:.1f}, root chord {rm:.2f} vs {rs:.2f} -> {'OK' if ok else 'FAIL'}")
    chk = lines

    def cs(i):
        name, sm, ss, rm, rs, _ = results[i]
        what = "root chord at the centreline" if i in (0, 3) else "root chord = x of the plotted hinge-line end vertex (the sheet has no outline point at x = 0 on the body edge: the nose blend is ahead of it)"
        return f"span measured {sm:.2f} mm, stated {ss:g} mm; {what} measured {rm:.2f} mm, stated {rs:.2f} mm (tolerance 0.5 mm)."

    Eb, E4, E2, Ez = b["E"], f4["E"], p2["E"], z["E"]

    def RAILNOTE(E, s):
        return (f"The printed CG rail does not stick out beyond the nose tip in this geometry: its front end is at x = {s['rail_front']:.1f}, {E.n(s['rail_front']):.1f} mm BEHIND the nose tip, under the sheet. "
                f"Alternative zero if you can see the rail front end: CG {E.fr(s['cg']):.1f} mm, bend line {E.fr(s['hinge']):.1f} mm, neutral point {E.fr(s['np']):.1f} mm back from it.")

    P = []
    a = P.append
    a("<h1>Balance points (CG) to mark on the gliders you fly today</h1>")
    a("<div class='box noprint'>"
      "<p><b>How to mark the CG.</b> Every position below is given in millimetres <b>from the NOSE TIP</b> (the frontmost point of the sheet, on the centreline) and also from the <b>TAIL TIP</b> (the rearmost point of the sheet, on the centreline): both are physical edges you can put a ruler on. "
      "The distance behind the wing root leading edge (centre LE) is kept as a small secondary line, and the ruler printed on each drawing still counts from the centre LE. "
      "Lay the glider flat, nose up the page like the drawing, measure back along the centreline from the nose tip to the green line, draw the line across the span, and balance the glider on a knife edge laid along it.</p>"
      "<p class='warn'><b>Swift gliders: the tail is the ONLY thing that bends.</b> One orange hinge line, tail trailing edge UP 6 deg. Do not fold at the body edge or the wrist: those lines on older drawings were markers only and are left off this page.</p>"
      "<p><button onclick=\"document.body.classList.add('print1');window.print();setTimeout(function(){document.body.classList.remove('print1')},500)\">print 1:1 (white)</button> "
      "Prints each glider on its own page, white background, black lines, true size (paper: tabloid/A3 landscape for the 400 mm gliders; set the printer to 100%, no fit to page). "
      "<b>Check the scale bar on the print with a ruler</b> before trusting it; if it is not exactly the stated length, the print scaled and the marks are wrong.</p></div>")
    a("<p class='src noprint'>Dark screen version below. Drawn at one true mm scale per glider; the ruler on the centreline has ticks every 5 mm and labels every 10 mm. "
      "Model values are MODEL (AVL, no thickness or stall), not measurements: the knife-edge balance on the real glider is what counts.</p>")

    a(section("b45", "1. B45 foamboard 400 mm", d1, [
        ("Nose tip / tail tip (measure from these)", f"nose tip = wing root LE (x = {Eb.nose:.1f}); tail tip = root trailing edge on the centreline (x = {Eb.tail:.1f}); sheet length {req('%.1f' % (Eb.tail - Eb.nose))}; the stick front end is {-Eb.front:.1f} ahead of the nose tip", f"GEOMETRY: <code>{rel(B45D/'geometry.json')}</code> root chord, stick_nose_ahead_of_root_le_mm"),
        ("CG target (green, big cross)", Eb.row(b['cg']) + f"; model band {Eb.n(b['band'][0]):.1f}-{Eb.n(b['band'][1]):.1f} from the nose tip (shaded)", f"MODEL: <code>{rel(FLAPS/'README.md')}</code> and <code>results.json</code> sweep (10 deg flaps, nominal flap location: trim CL {b['trim_cl']:.2f} at 66 mm, 0.24 at 63, 0.75 at 68). Band is where trim CL is 0.2-0.9 with 10 deg flaps."),
        ("AVL neutral point (red, never behind)", Eb.row(b['np']), f"AVL, <code>{rel(B45D/'geometry.json')}</code> avl_with_tip_fins (70.24; the flap study gets 70.29 at 10 deg flap)"),
        ("Centre flaps (two)", "10 mm chord at the trailing edge, y = 30-80 mm each side, trailing edge UP ~10 deg", "ASSUMED from Kai's description (no file measures them): <code>" + rel(FLAPS / 'README.md') + "</code>"),
        ("Outline", f"span {b['span']:g}, root chord {b['root']:.1f}, tip chord 59.3, quarter-chord sweep 25 deg, zero dihedral, tip LE {b['fin'][2]:.1f}, tip TE {b['fin'][3]:.1f}", f"GEOMETRY: <code>{rel(B45D/'geometry.json')}</code>"),
        ("Stick / fins", f"stick {b['stick'][0]:.1f} to {b['stick'][1]:.1f} (4 mm wide, under the wing); tip fins 24 mm tall, 59.3 chord, 5 mm thick (drawn as the thin edge)", f"GEOMETRY: same file")]
        , "<b>Measure where yours balances now first</b> and write it down before changing anything. Then move ballast until it balances at the green line. Behind the red line the model says the wing is unstable. "
          "The CG band is the model's trim window with the assumed flaps, not a promise.</p>" + edge_note(Eb, 0.0, "The printed stick DOES stick out ahead of the sheet nose tip, by " + f"{-Eb.front:.1f} mm; use it as the zero only if you cannot find the sheet nose (CG then {Eb.fr(b['cg']):.1f} mm from the stick front end), but the sheet nose tip is the primary reference.") + "<p class='src'>", cs(0)))
    a(section("swift400", "2. Swift A foamboard 400 mm", d2, [
        ("Nose tip / tail tip (measure from these)", f"nose tip at x = {E4.nose:.1f}, tail tip at x = {E4.tail:.1f} (root-LE coordinates); sheet length {req('%.1f' % (E4.tail - E4.nose))}", f"GEOMETRY: <code>build_geometry4(GLIDE)</code> x 400, centreline crossings of the outline"),
        ("CG (green, big cross)", E4.row(f4['cg']), f"AVL trim CG at CL 0.3, bend 6: <code>{rel(SW/'A_foam_span400/drawing.svg')}</code> label; agrees with <code>{rel(SW/'outlines.csv')}</code> (23.1 x 1.6 = {f4['csv'][0]:.1f}) and the 10-17 build record"),
        ("Neutral point (red, never behind)", E4.row(f4['np']), f"AVL, bend 6: same drawing.svg label; outlines.csv NP % MAC gives {f4['csv'][1]:.1f}"),
        ("THE ONLY BEND", E4.row(f4['hinge']) + f"; tail trailing edge UP 6 deg (hinge runs y = -{f4['yb']:.0f} to +{f4['yb']:.0f})", f"GEOMETRY: drawing.svg label and DXF layer HINGE_BEND_UP in <code>{rel(SW/'A_foam_span400/parts.dxf')}</code>"),
        ("Nose ballast hole", f"{f4['hole']:g} mm from the wing root LE ({E4.n(f4['hole']):.1f} from the nose tip), ballast {f4['ballast']:.2f} g (all-up {f4['n']['M']:.2f} g)", "CALCULATED: <code>bird_outline_v2.size_numbers</code> via <code>flight_day_page.swift_numbers</code>, ASSUMED masses; fly the glider as it is and weigh it")]
        , f"Outline: <code>{rel(SW/'bird_outline_v4_swift.py')}</code> <code>build_geometry4(GLIDE)</code> x 400 (same source as the A_foam_span400 DXF OUTLINE layer, 94 vertices). The 10-17 glider flew without a bend and tumbled; this page marks the same CG with the 6 deg bend added.</p>" + edge_note(E4, E4.tail - f4['hinge'], RAILNOTE(E4, f4)) + "<p class='src'>", cs(1)))
    a(section("swift250", "3. Swift A 250 mm (foamboard or printed)", d3, [
        ("Nose tip / tail tip (measure from these)", f"nose tip at x = {E2.nose:.1f}, tail tip at x = {E2.tail:.1f} (root-LE coordinates); sheet length {req('%.1f' % (E2.tail - E2.nose))}", f"GEOMETRY: <code>build_geometry4(GLIDE)</code> x 250, centreline crossings of the outline. The positions are geometric (same outline scaled, same margin), so they apply to a foamboard 250 as well as the printed one"),
        ("CG (green, big cross)", E2.row(p2['cg']), f"AVL trim CG at CL 0.3, bend 6: <code>{rel(SW/'A_printed_span250/drawing.svg')}</code> label; outlines.csv 'trim CG mm at 10% margin (250)' = 23.1"),
        ("Neutral point (red, never behind)", E2.row(p2['np']), f"AVL, bend 6: same drawing.svg label; outlines.csv NP % MAC gives {p2['csv'][1]:.1f}"),
        ("THE ONLY BEND", E2.row(p2['hinge']) + f"; tail trailing edge UP 6 deg (hinge runs y = -{p2['yb']:.0f} to +{p2['yb']:.0f})", f"GEOMETRY: drawing.svg label and DXF layer HINGE_BEND_UP in <code>{rel(SW/'A_printed_span250/parts.dxf')}</code>"),
        ("Nose ballast hole", f"{p2['hole']:g} mm from the wing root LE ({E2.n(p2['hole']):.1f} from the nose tip). PRINTED: ballast {p2['ballast']:.2f} g at the {p2['hole']:g} mm hole (all-up {p2['n']['M']:.2f} g). FOAMBOARD 250: ballast not computed - balance with blu-tack at the nose to the CG mark", "CALCULATED for the printed 250 only: <code>bird_outline_v2.size_numbers</code>, ASSUMED masses (PLA 1.24 g/cm3); weigh yours. The foamboard 250 mass is not modelled (size_numbers has no foam-250 case)")]
        , f"Outline: <code>build_geometry4(GLIDE)</code> x 250 (same source as A_printed_span250 DXF OUTLINE layer). "
          f"Foamboard note: 5 mm foamboard is about {100 * f4['thick'] / p2['xh']:.0f}% of the {p2['xh']:.0f} mm root chord at 250 mm span ({f4['thick']:g}/{p2['xh']:.0f}), "
          f"much thicker relative to the wing than at 400 mm ({100 * f4['thick'] / f4['xh']:.1f}% of {f4['xh']:.1f} mm) (ASSUMED 5 mm; measure).</p>" + edge_note(E2, E2.tail - p2['hinge'], RAILNOTE(E2, p2)) + "<p class='src'>", cs(2)))
    a(section("zanonia", "4. Zanonia printed 155 mm", d4, [
        ("Nose tip / tail tip (measure from these)", f"nose tip = centre LE (x = {Ez.nose:.1f}); tail tip = trailing edge on the centreline (x = {Ez.tail:.1f}); sheet length {req('%.1f' % (Ez.tail - Ez.nose))}", f"GEOMETRY: <code>{rel(ZAN/'zanonia_outline.csv')}</code> scaled to 155 mm span, centreline crossings of the outline"),
        ("CG (green, raised cross)", Ez.row(z['cg']), f"<code>{rel(ZAN/'zanonia_geometry.json')}</code> cg_x_in x 25.4 x scale; <code>results/runs/known_good/zanonia/numbers.json</code> cg_mm {17.2175:.2f}; <code>TEST_CARD.md</code> says 17.2"),
        ("AVL neutral point (red, never behind)", Ez.row(z['np']) + f" (AVL gives {z['np_raw']:.2f} on the 155.1 mm scan-size outline; scaled to 155)", f"MODEL: <code>{rel(ZAN/'avl_results.json')}</code> curl_30pct_4deg Xnp; RESULTS.md says it is 0.31 mm behind the cross, which is about 1% of MAC: essentially neutral, so there is no safe margin"),
        ("Nose hole", f"centre {z['hole']:.2f} mm behind the LE ({Ez.n(z['hole']):.1f} from the nose tip), diameter {z['hole_d']:.2f} mm", f"position: <code>{rel(ZAN/'zanonia_geometry.json')}</code> hole_x_in (from the scan); diameter: the printed STL (<code>zanonia_plate_155mm.stl</code>, 4.0 mm, enlarged from the pattern's 2.9 mm so a blu-tack plug fits)"),
        ("Curl region", "outer 30%% of each semi-span (|y| from %.1f to %.1f mm), tips twisted trailing edge UP, start 4 deg" % (z["curl_from"], 77.5), f"<code>{rel(ZAN/'TEST_CARD.md')}</code> (jig, curl search order 4, then 6, 8, 10, 12 / 2)")]
        , f"Outline: <code>{rel(ZAN/'zanonia_outline.csv')}</code> (traced from the published pattern), scaled to 155 mm span. Plate: <code>results/runs/known_good/zanonia/print/zanonia_plate_155mm.stl</code>. "
          "The drawn hole is the printed plate's 4 mm hole (the pattern's own hole is 2.9 mm).</p>" + edge_note(Ez, 0.0, "There is no rail or stick on this plate, so nothing sticks out ahead of the nose tip.") + "<p class='src'>", cs(3)))
    a("<p class='src noprint'>Generated by <code>rigid_glider/flight_day_cg_page.py</code>; reads the files named under each drawing; writes only this page.</p>")
    doc = ("<!DOCTYPE html>\n<html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><meta name='color-scheme' content='dark'>"
           "<title>CG marks 2026-10-20</title><style>" + CSS + PRINT_CSS + "</style></head><body><main>\n" + "\n".join(P) + "\n</main></body></html>\n")
    OUT.write_text(doc, encoding="utf-8")
    missing = [r for r in REQ if html.escape(r) not in doc]
    assert not missing, missing
    chk.append(f"edge-referenced numbers asserted present in the page: {len(REQ)} strings, 0 missing")
    return chk, ok_all


class _P(HTMLParser):
    VOID = {"meta", "br", "input", "link", "img", "hr", "line", "rect", "circle", "polygon", "path"}

    def __init__(self):
        super().__init__()
        self.stack, self.err = [], []

    def handle_starttag(self, tag, attrs):
        if tag not in self.VOID:
            self.stack.append(tag)

    def handle_startendtag(self, tag, attrs):
        pass

    def handle_endtag(self, tag):
        if tag in self.VOID:
            return
        if not self.stack or self.stack[-1] != tag:
            self.err.append(f"mismatch </{tag}> vs {self.stack[-1] if self.stack else None}")
            if tag in self.stack:
                while self.stack and self.stack.pop() != tag:
                    pass
        else:
            self.stack.pop()


if __name__ == "__main__":
    chk, ok = main()
    p = _P()
    p.feed(OUT.read_text(encoding="utf-8"))
    print("wrote", OUT)
    print("\n".join(chk))
    print("scale check:", "PASS" if ok else "FAIL")
    print("parse errors:", p.err[:5], "unclosed:", p.stack)
    print("source/stated mismatches:", MISMATCH or "none")
    sys.exit(0 if ok and not p.err and not p.stack else 1)
