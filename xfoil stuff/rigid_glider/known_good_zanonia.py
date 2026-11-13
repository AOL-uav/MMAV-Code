"""Known-good glider check: the Foam Zanonia Glider (endlessLift, Gary Hinze 2011, after Paul McIlrath 1983) through our AVL chain.

   py -B rigid_glider/known_good_zanonia.py digitize [--scan PATH]   stage 1: trace the pattern scan -> outline CSV, CG and hole, preview SVG, geometry (research/known_good/zanonia/)
   py -B rigid_glider/known_good_zanonia.py avl                      stage 2: AVL sweep of the tip curl and the verdict on the rule in README.md (needs stage 1)
   py -B rigid_glider/known_good_zanonia.py build                    stage 3: scale-up numbers, ASCII STL plates and bending jigs, masses, cardstock templates, index.html (needs stage 2)

The pass rule is written in research/known_good/zanonia/README.md BEFORE stage 2 is run; the code applies it and does not change it.  The pull-out drop is skipped by the rule when the
verdict is DISAGREE.  The scan is untrusted data: it is only read as an image (JPEG -> grey PGM by a PowerShell System.Drawing script, then traced here).
New files only; no existing script is modified or run (flat_test is imported read-only for its mesh helpers).  Standard library only.  AVL runs are serial and light.
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import argparse
import base64
import csv
import html
import json
import math
import re
import struct
import subprocess
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
ROOT = HERE.parent
RES = ROOT / "research/known_good/zanonia"
OUT = ROOT / "results/runs/known_good/zanonia"
DEFAULT_SCAN = Path(r"<HOME>\AppData\Local\Temp\claude\MAV-2026-Codes\e78baad8-7bd4-4c2e-8073-2f0b681b4bcd\scratchpad\zanonia_dl\pattern.jpg")
DPI = 300.0                       # MEASURED: the JPEG header of the scan says 300 x 300 dpi (read by System.Drawing)
CLOSE_R = 4.0                     # pixels: closes small gaps in the hand-drawn outline before the interior is found
PEN_T = 110                       # grey level below which a pixel is pen (the scan is nearly black and white)
IN = 25.4                         # mm per inch (exact)
# =============================================================================================== image access (stdlib + PowerShell for the JPEG decode)
def jpeg_to_pgm(scan: Path, pgm: Path):
    """Convert a JPEG to an 8-bit grey PGM with PowerShell System.Drawing (no JPEG decoder in the standard library)."""
    ps = RES / "jpeg_to_pgm.ps1"
    ps.write_text('param([string]$src, [string]$dst)\n'
                  'Add-Type -AssemblyName System.Drawing\n'
                  '$bmp = New-Object System.Drawing.Bitmap($src)\n'
                  '$w = $bmp.Width; $h = $bmp.Height\n'
                  '"size $w x $h  dpi $($bmp.HorizontalResolution) x $($bmp.VerticalResolution)"\n'
                  '$rect = New-Object System.Drawing.Rectangle(0, 0, $w, $h)\n'
                  '$data = $bmp.LockBits($rect, [System.Drawing.Imaging.ImageLockMode]::ReadOnly, [System.Drawing.Imaging.PixelFormat]::Format24bppRgb)\n'
                  '$stride = $data.Stride\n'
                  '$buf = New-Object byte[] ($stride * $h)\n'
                  '[System.Runtime.InteropServices.Marshal]::Copy($data.Scan0, $buf, 0, $buf.Length)\n'
                  '$bmp.UnlockBits($data)\n'
                  '$out = New-Object byte[] ($w * $h)\n'
                  'for ($y = 0; $y -lt $h; $y++) { $o = $y * $stride; $p = $y * $w\n'
                  '  for ($x = 0; $x -lt $w; $x++) { $i = $o + 3 * $x; $out[$p + $x] = [byte]((299 * $buf[$i + 2] + 587 * $buf[$i + 1] + 114 * $buf[$i]) / 1000) } }\n'
                  '$hdr = [System.Text.Encoding]::ASCII.GetBytes("P5`n$w $h`n255`n")\n'
                  '$fs = [System.IO.File]::Create($dst); $fs.Write($hdr, 0, $hdr.Length); $fs.Write($out, 0, $out.Length); $fs.Close()\n', encoding="ascii")
    r = subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ps), str(scan), str(pgm)], capture_output=True, text=True, timeout=300)
    if r.returncode or not pgm.is_file():
        raise RuntimeError(r.stdout + r.stderr)
    return r.stdout.strip()


def read_pgm(p: Path):
    d = p.read_bytes()
    i, toks = 0, []
    while len(toks) < 4:
        j = d.index(b"\n", i)
        toks += d[i:j].split()
        i = j + 1
    return int(toks[1]), int(toks[2]), d[i:]


def disc(r):
    n = int(math.ceil(r))
    return [(dx, dy) for dy in range(-n, n + 1) for dx in range(-n, n + 1) if dx * dx + dy * dy <= r * r]


# =============================================================================================== stage 1: trace the outline
def digitize(scan: Path):
    pgm = scan.with_suffix(".pgm")
    note = ""
    if not pgm.is_file():
        note = jpeg_to_pgm(scan, pgm)
    w, h, px = read_pgm(pgm)
    # --- connected pen components (8-neighbour)
    def comp(start, seen):
        st, c = [start], [start]
        seen.add(start)
        while st:
            q = st.pop()
            y, x = divmod(q, w)
            for dy in (-1, 0, 1):
                yy = y + dy
                if 0 <= yy < h:
                    for dx in (-1, 0, 1):
                        xx = x + dx
                        if 0 <= xx < w:
                            r = yy * w + xx
                            if px[r] < PEN_T and r not in seen:
                                seen.add(r)
                                st.append(r)
                                c.append(r)
        return c
    seed = next(1625 * w + x for x in range(900, 1100) if px[1625 * w + x] < PEN_T)         # a pen pixel on the leading edge near mid-span (found from the scan by eye)
    seen = set()
    pen = comp(seed, seen)
    # The hand-drawn pen line has gaps at the upper tip (visible in the scan): the top arc and a few dotted fragments are separate pieces.
    # Join them to the outline with straight strokes between the nearest pieces (JUDGEMENT; the gaps total about 5 mm of the tip, the original curve is not recoverable there).
    penset = set(pen)
    box = [(y, x) for y in range(680, 800) for x in range(1440, 1700) if px[y * w + x] < PEN_T and y * w + x not in penset]
    pieces, used = [], set()
    for y, x in box:
        q = y * w + x
        if q not in used:
            c = comp(q, used)
            pieces.append(c)
    pieces = [c for c in pieces if len(c) >= 3]
    arc = max(pieces, key=len)
    frags = [c for c in pieces if c is not arc]
    main_near = [q for q in pen if 650 <= q // w <= 820 and 1400 <= q % w <= 1700]
    def line(p, q):
        (y0_, x0_), (y1_, x1_) = divmod(p, w), divmod(q, w)
        n_ = max(abs(x1_ - x0_), abs(y1_ - y0_)) + 1
        for k in range(n_ + 1):
            x, y = round(x0_ + (x1_ - x0_) * k / n_), round(y0_ + (y1_ - y0_) * k / n_)
            for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
                r = (y + dy) * w + x + dx
                if r not in penset:
                    penset.add(r)
                    pen.append(r)
                    seen.add(r)
    def dist(p, q):
        return math.hypot(p % w - q % w, p // w - q // w)
    bridge = []
    # right end of the arc: chain through the dotted fragments until the main outline is reached
    p = max(arc, key=lambda q: q % w)
    pool = [c for c in frags]
    for _ in range(6):
        cand = [(min(dist(p, q) for q in c), i) for i, c in enumerate(pool)]
        qm = min(main_near, key=lambda q: dist(p, q))
        dm = dist(p, qm)
        if cand and min(cand)[0] < dm and min(cand)[0] < 40:
            _, i = min(cand)
            c = pool.pop(i)
            q = min(c, key=lambda r: dist(p, r))
            line(p, q)
            bridge.append((divmod(p, w)[::-1], divmod(q, w)[::-1]))
            p = max(c, key=lambda r: dist(q, r))
        else:
            line(p, qm)
            bridge.append((divmod(p, w)[::-1], divmod(qm, w)[::-1]))
            break
    # left end of the arc: nearest main-outline pixel
    p = min(arc, key=lambda q: q % w)
    qm = min(main_near, key=lambda q: dist(p, q))
    line(p, qm)
    bridge.append((divmod(p, w)[::-1], divmod(qm, w)[::-1]))
    for q in arc:
        if q not in penset:
            penset.add(q)
            pen.append(q)
    xs = [q % w for q in pen]
    ys = [q // w for q in pen]
    bx0, bx1, by0, by1 = min(xs), max(xs), min(ys), max(ys)
    # --- small components inside the outline: the CG cross, its letters, the hole
    inside = []
    for yy in range(by0, by1 + 1, 3):
        for xx in range(bx0 + 20, bx1 - 20, 3):
            q = yy * w + xx
            if px[q] < PEN_T and q not in seen:
                c = comp(q, seen)
                if len(c) > 150:
                    cx = [r % w for r in c]
                    cy = [r // w for r in c]
                    inside.append(dict(n=len(c), cx=sum(cx) / len(c), cy=sum(cy) / len(c), x0=min(cx), x1=max(cx), y0=min(cy), y1=max(cy), pts=c))
    # --- region: close pen gaps, flood the outside, take the enclosed region, grow it back to the pen centre line
    W2, H2 = bx1 - bx0 + 31, by1 - by0 + 31
    ox, oy = bx0 - 15, by0 - 15
    solid = bytearray(W2 * H2)
    d_close = disc(CLOSE_R)
    for q in pen:
        x, y = q % w - ox, q // w - oy
        for dx, dy in d_close:
            solid[(y + dy) * W2 + x + dx] = 1
    ext = bytearray(W2 * H2)
    st = [0]
    ext[0] = 1
    while st:
        q = st.pop()
        y, x = divmod(q, W2)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            xx, yy = x + dx, y + dy
            if 0 <= xx < W2 and 0 <= yy < H2:
                r = yy * W2 + xx
                if not ext[r] and not solid[r]:
                    ext[r] = 1
                    st.append(r)
    inner = bytearray(W2 * H2)
    for q in range(W2 * H2):
        if not ext[q] and not solid[q]:
            inner[q] = 1
    n_inner = sum(inner)
    # pen thickness measured on the leading-edge stroke at mid span
    runs, cur = [], None
    for x in range(bx0 - 2, bx1 + 3):
        dark = (1625 * w + x) in seen or px[1625 * w + x] < PEN_T
        if dark and cur is None:
            cur = x
        if not dark and cur is not None:
            runs.append(x - cur)
            cur = None
    pen_t = sum(runs[:2]) / 2.0 if len(runs) >= 2 else 4.5
    grow = CLOSE_R + pen_t / 2.0                                              # closing radius + half the pen thickness
    dg = disc(grow)
    mid = bytearray(W2 * H2)
    for q in range(W2 * H2):
        if inner[q]:
            y, x = divmod(q, W2)
            # only boundary pixels need to spread
            if not (inner[q - 1] and inner[q + 1] and inner[q - W2] and inner[q + W2]):
                for dx, dy in dg:
                    mid[(y + dy) * W2 + x + dx] = 1
            mid[q] = 1
    # --- per-row extremes of the mid-line region (rows run along the span, x is chordwise)
    rows = {}
    for y in range(H2):
        base = y * W2
        xs_ = [x for x in range(W2) if mid[base + x]]
        if xs_:
            rows[y + oy] = (min(xs_) + ox, max(xs_) + ox + 1)
    ymin, ymax = min(rows), max(rows)
    return dict(bridge=bridge, w=w, h=h, px=px, pen=pen, rows=rows, ymin=ymin, ymax=ymax, bbox=(bx0, bx1, by0, by1), inside=inside, pen_t=pen_t, n_inner=n_inner, note=note, mid=(mid, W2, H2, ox, oy))


def analyse(D):
    """Symmetrise about the best mirror axis; convert to inches (300 dpi); geometry numbers."""
    rows = D["rows"]
    s2i = lambda v: v / DPI
    def le(y): return rows[y][0] if y in rows else None
    best = None
    y_mid = (D["ymin"] + D["ymax"]) / 2.0
    c0 = int(round(y_mid))
    for k in range(-40, 41):
        yc2 = 2 * c0 + k                                                       # twice the axis row, so half-pixel axes are tried
        err, n = 0.0, 0
        for u2 in range(2, 1600, 4):
            ya, yb = (yc2 + u2) // 2, (yc2 - u2) // 2
            if ya in rows and yb in rows and (yc2 + u2) % 2 == 0:
                err += (rows[ya][0] - rows[yb][0]) ** 2 + (rows[ya][1] - rows[yb][1]) ** 2
                n += 1
        if n and (best is None or err / n < best[0]):
            best = (err / n, yc2 / 2.0)
    yc = best[1]
    asym_rms_px = math.sqrt(best[0] / 2.0)
    u_max = min(yc - D["ymin"], D["ymax"] - yc)
    pts = []                                                                   # (u px, LE px, TE px) symmetrised, u from the axis
    for u in range(0, int(u_max) + 1):
        a, b = int(round(yc + u)), int(round(yc - u))
        if a in rows and b in rows:
            pts.append((u, (rows[a][0] + rows[b][0]) / 2.0, (rows[a][1] + rows[b][1]) / 2.0))
    # centre LE reference: average of the first rows
    x_le0 = sum(p[1] for p in pts[:5]) / 5
    return dict(yc=yc, asym_rms_px=asym_rms_px, u_max=u_max, pts=pts, x_le0=x_le0)


def geometry(A):
    """Inches, from the symmetrised (u, LE, TE) points: span, area, MAC, AR, sweeps."""
    s = [(p[0] / DPI, (p[1] - A["x_le0"]) / DPI, (p[2] - A["x_le0"]) / DPI) for p in A["pts"]]          # (s, x_LE, x_TE) in inches, s from the centreline
    # trapezoid sums over the half span (rows are 1 px apart)
    dy = 1.0 / DPI
    area_half = sum((p[2] - p[1]) for p in s) * dy
    S = 2 * area_half
    c = [p[2] - p[1] for p in s]
    mac = sum(ci * ci for ci in c) * dy / area_half
    x_le_mac = sum(p[1] * ci for p, ci in zip(s, c)) * dy / area_half
    half = s[-1][0]
    span = 2 * half
    def tang(frac):
        i = int(frac * (len(s) - 1))
        a, b = max(i - 25, 0), min(i + 25, len(s) - 1)
        return math.degrees(math.atan2(s[b][1] - s[a][1], s[b][0] - s[a][0]))
    i80 = int(0.8 * (len(s) - 1))
    q = lambda p: p[1] + 0.25 * (p[2] - p[1])
    return dict(span_in=span, half_in=half, area_in2=S, mac_in=mac, x_le_mac_in=x_le_mac, ar=span ** 2 / S, root_chord_in=c[0], mean_chord_in=S / span,
                sweep_le_mid_deg=tang(0.5), sweep_le_25_deg=tang(0.25), sweep_le_75_deg=tang(0.75), sweep_le_overall_deg=math.degrees(math.atan2(s[i80][1], s[i80][0])),
                sweep_qc_overall_deg=math.degrees(math.atan2(q(s[i80]) - q(s[0]), s[i80][0])), tip_x_le_in=s[-1][1], tip_x_te_in=s[-1][2], s=s)


def find_marks(D, A):
    """Ballast hole and CG cross from the small pen components near the centre line (mirror axis).
    hole = the ring (34 x 34 px) nearest the leading edge; CG cross = the 64 x 45 px component next to it (the two letters 'C' and 'G' sit beside it and are ignored).
    The cross centre is where its two strokes cross: the column with the most pen pixels (the stroke along the span) and the row with the most (the stroke along the chord)."""
    w = D["w"]
    cand = []
    for c in D["inside"]:
        if abs(c["cy"] - A["yc"]) < 40:
            cand.append(c)
    cand.sort(key=lambda c: c["cx"])
    hole = next(c for c in cand if abs((c["x1"] - c["x0"]) - (c["y1"] - c["y0"])) <= 3 and 25 <= c["x1"] - c["x0"] <= 45)
    cross = next(c for c in cand if c is not hole and c["cx"] > hole["cx"])
    cols, rows_ = {}, {}
    for q in cross["pts"]:
        cols[q % w] = cols.get(q % w, 0) + 1
        rows_[q // w] = rows_.get(q // w, 0) + 1
    cx = max(cols, key=lambda k: (cols[k], -abs(k - cross["cx"])))
    cy = max(rows_, key=lambda k: (rows_[k], -abs(k - cross["cy"])))
    return dict(hole=(hole["cx"], hole["cy"], hole), cross=(cx + 0.5, cy + 0.5, cross), cross_centroid=(cross["cx"], cross["cy"]))


# =============================================================================================== outputs of stage 1
def write_png_gray(path: Path, w, h, rows):
    raw = b"".join(b"\x00" + bytes(r) for r in rows)

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def smooth(vals, k=6):
    n = len(vals)
    return [sum(vals[max(0, i - k):min(n, i + k + 1)]) / len(vals[max(0, i - k):min(n, i + k + 1)]) for i in range(n)]


def outline_table(A, G):
    """Symmetrised, lightly smoothed outline: (s, x_LE, x_TE) in inches, s from the centreline, x from the centre leading edge, every 4 px = 0.0133 in."""
    s = G["s"]
    le = smooth([p[1] for p in s], 5)
    te = smooth([p[2] for p in s], 5)
    out = []
    for i in range(0, len(s), 4):
        out.append((s[i][0], le[i], te[i]))
    if out[-1][0] < s[-1][0] - 1e-9:
        out.append((s[-1][0], le[-1], te[-1]))
    return out


def preview_svg(D, A, G, M, tab):
    """Dark-mode preview: the scan (inverted, 3x downsampled, cropped) with the traced outline (both halves), the mirror axis, the CG cross and the hole."""
    px, w = D["px"], D["w"]
    x0, x1, y0, y1 = D["bbox"][0] - 60, D["bbox"][1] + 120, D["bbox"][2] - 60, D["bbox"][3] + 60
    f = 3
    cw, ch = (x1 - x0) // f, (y1 - y0) // f
    rows = []
    for j in range(ch):
        row = bytearray(cw)
        for i in range(cw):
            tot = 0
            for dj in range(f):
                base = (y0 + j * f + dj) * w + x0 + i * f
                tot += sum(px[base:base + f])
            row[i] = 255 - tot // (f * f)
        rows.append(row)
    tmp = RES / "_preview_scan.png"
    write_png_gray(tmp, cw, ch, rows)
    b64 = base64.b64encode(tmp.read_bytes()).decode()
    tmp.unlink()
    X = lambda x: (x - x0) / f
    Y = lambda y: (y - y0) / f
    yc, xl0 = A["yc"], A["x_le0"]
    P = lambda sg: [(X(xl0 + t[1] * DPI), Y(yc + sg * t[0] * DPI)) for t in tab] + [(X(xl0 + t[2] * DPI), Y(yc + sg * t[0] * DPI)) for t in reversed(tab)]
    pl = lambda pts: " ".join(f"{a:.1f},{b:.1f}" for a, b in pts)
    mk = lambda cx, cy, r: (f"<circle cx='{X(cx):.1f}' cy='{Y(cy):.1f}' r='{r}' fill='none' stroke='#6cc08b' stroke-width='1.6'/><line x1='{X(cx) - 12:.1f}' y1='{Y(cy):.1f}' x2='{X(cx) + 12:.1f}' y2='{Y(cy):.1f}' stroke='#6cc08b'/>"
                            f"<line x1='{X(cx):.1f}' y1='{Y(cy) - 12:.1f}' x2='{X(cx):.1f}' y2='{Y(cy) + 12:.1f}' stroke='#6cc08b'/>")
    br = "".join(f"<line x1='{X(a[0]):.1f}' y1='{Y(a[1]):.1f}' x2='{X(b[0]):.1f}' y2='{Y(b[1]):.1f}' stroke='#ff5da2' stroke-width='1.2'/>" for a, b in D["bridge"])
    return (f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {cw} {ch}' width='{cw}' height='{ch}' style='background:#14181d'>"
            f"<image x='0' y='0' width='{cw}' height='{ch}' href='data:image/png;base64,{b64}' opacity='0.9'/>"
            f"<polygon points='{pl(P(1))}' fill='none' stroke='#e89a55' stroke-width='1.3'/><polygon points='{pl(P(-1))}' fill='none' stroke='#e89a55' stroke-width='1.3'/>{br}"
            f"<line x1='{X(xl0) - 40:.1f}' y1='{Y(yc):.1f}' x2='{X(xl0 + 2.2 * DPI):.1f}' y2='{Y(yc):.1f}' stroke='#7fb4ff' stroke-dasharray='4 3'/>"
            f"{mk(M['cross'][0], M['cross'][1], 8)}{mk(M['hole'][0], M['hole'][1], 5)}"
            f"<text x='8' y='14' fill='#dde3ea' font-family='Segoe UI,Arial' font-size='11'>scan (inverted) + trace</text>"
            f"<text x='8' y='27' fill='#e89a55' font-family='Segoe UI,Arial' font-size='11'>orange: traced outline</text>"
            f"<text x='8' y='40' fill='#6cc08b' font-family='Segoe UI,Arial' font-size='11'>green: CG cross, hole</text>"
            f"<text x='8' y='53' fill='#7fb4ff' font-family='Segoe UI,Arial' font-size='11'>blue dashed: mirror axis</text>"
            f"<text x='8' y='66' fill='#9aa7b4' font-family='Segoe UI,Arial' font-size='11'>span {G['span_in']:.2f} in at 300 dpi</text></svg>")


def stage_digitize(scan: Path):
    RES.mkdir(parents=True, exist_ok=True)
    D = digitize(scan)
    A = analyse(D)
    G = geometry(A)
    M = find_marks(D, A)
    xl0, yc = A["x_le0"], A["yc"]
    cg_in = ((M["cross"][0] - xl0) / DPI, (M["cross"][1] - yc) / DPI)
    hole_in = ((M["hole"][0] - xl0) / DPI, (M["hole"][1] - yc) / DPI)
    cg_pct = 100 * (cg_in[0] - G["x_le_mac_in"]) / G["mac_in"]
    tab = outline_table(A, G)
    with (RES / "zanonia_outline.csv").open("w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["s_in (from centreline)", "x_LE_in (aft of centre LE)", "x_TE_in", "chord_in"])
        for t in tab:
            wr.writerow([f"{t[0]:.4f}", f"{t[1]:.4f}", f"{t[2]:.4f}", f"{t[2] - t[1]:.4f}"])
    marks = [["CG cross", f"{cg_in[0]:.4f}", f"{cg_in[1]:.4f}", f"{100 * (cg_in[0] - G['x_le_mac_in']) / G['mac_in']:.1f}"],
             ["ballast hole (centre)", f"{hole_in[0]:.4f}", f"{hole_in[1]:.4f}", f"{100 * (hole_in[0] - G['x_le_mac_in']) / G['mac_in']:.1f}"]]
    with (RES / "zanonia_marks.csv").open("w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["mark", "x_in (aft of centre LE)", "y_in (from centre line, + = scan down)", "% of MAC behind the MAC leading edge"])
        wr.writerows(marks)
    (RES / "zanonia_preview.svg").write_text(preview_svg(D, A, G, M, tab), encoding="utf-8")
    geo = {k: v for k, v in G.items() if k != "s"}
    geo.update(cg_x_in=cg_in[0], cg_y_in=cg_in[1], hole_x_in=hole_in[0], hole_y_in=hole_in[1], cg_pct_mac=cg_pct, asym_rms_in=A["asym_rms_px"] / DPI, pen_t_px=D["pen_t"], dpi=DPI,
               hole_diam_in=(M["hole"][2]["x1"] - M["hole"][2]["x0"] + 1) / DPI, cross_centroid_x_in=(M["cross_centroid"][0] - xl0) / DPI, bridge=[[list(a), list(b)] for a, b in D["bridge"]])
    (RES / "zanonia_geometry.json").write_text(json.dumps(geo, indent=1), encoding="utf-8")
    print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in geo.items()}, indent=1))
    return geo


# =============================================================================================== stage 2a: AVL sweep of the tip curl
AVL_EXE = ROOT / "tools/avl/avl352.exe"
REGIONS = (0.2, 0.3, 0.4)                      # outer fraction of the semi-span that is curled
THETAS = (0.0, 2.0, 4.0, 6.0, 8.0, 10.0)       # tip incidence, deg (washout: trailing edge up)
N_SEC = 30
NCHORD, NSPAN = 10, 80                         # same panelling as bird_outline_v2.avl_deck2
SCAN_ALPHAS = (0.0, 4.0, 8.0)
SM_LO, SM_HI = 0.03, 0.25                      # pass rule (README section 3)
TRIM_CL_LO, TRIM_CL_HI = 0.2, 0.9
CURL_RANGE = (0.0, 10.0)


def load_outline():
    rows = list(csv.reader((RES / "zanonia_outline.csv").open(encoding="utf-8")))[1:]
    return [(float(r[0]), float(r[1]), float(r[2])) for r in rows]


def interp(tab, s):
    for a, b in zip(tab, tab[1:]):
        if a[0] <= s <= b[0]:
            t = (s - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
            return a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])
    return tab[-1][1], tab[-1][2]


def build_deck(tab, geo, f, theta, name, n_sec=None, nchord=None, nspan=None):
    """AVL deck in metres at the pattern size (AVL results are scale-free).  x from the centre leading edge; reference point = the marked CG."""
    k = 0.0254
    s_tip = tab[-1][0]
    s0 = (1.0 - f) * s_tip
    L = [name, "0.0", "0 0 0.0", f"{geo['area_in2'] * k * k:.9f} {geo['mac_in'] * k:.9f} {geo['span_in'] * k:.9f}", f"{geo['cg_x_in'] * k:.9f} 0.0 0.0",
         "SURFACE", "Wing", f"{nchord or NCHORD} 1.0 {nspan or NSPAN} 0.0", "YDUPLICATE", "0.0"]
    n_sec = n_sec or N_SEC
    for i in range(n_sec):
        s = s_tip * i / (n_sec - 1)
        xl, xt = interp(tab, s)
        ainc = -theta * max(0.0, (s - s0) / (s_tip - s0)) if f > 0 else 0.0
        L += ["SECTION", f"{xl * k:.9f} {s * k:.9f} 0.0 {(xt - xl) * k:.9f} {ainc:.6f}", "NACA", "0000"]
    return "\n".join(L) + "\n"


def _field(text, key):
    m = re.search(rf"\b{re.escape(key)}\s*=\s*(-?[0-9.]+(?:[Ee][-+]?[0-9]+)?)", text)
    return float(m.group(1)) if m else None


def avl_alphas(deck, wd: Path, name, alphas):
    wd.mkdir(parents=True, exist_ok=True)
    for p in wd.glob("p*.*"):
        p.unlink()
    (wd / f"{name}.avl").write_text(deck, encoding="ascii")
    keys = ["PLOP", "G", "", "OPER"]
    for i, a in enumerate(alphas):
        keys += [f"A A {a:.6f}", "X", "FT", f"p{i}.ft", "ST", f"p{i}.st"]
    keys += ["", "QUIT"]
    r = subprocess.run([str(AVL_EXE), f"{name}.avl"], input="\n".join(keys) + "\n", text=True, cwd=wd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
    (wd / "avl.log").write_text(r.stdout, encoding="utf-8", errors="replace")
    out = []
    for i, a in enumerate(alphas):
        ft_t, st_t = (wd / f"p{i}.ft").read_text(errors="replace"), (wd / f"p{i}.st").read_text(errors="replace")
        out.append(dict(alpha=a, CL=_field(ft_t, "CLtot"), Cm=_field(ft_t, "Cmtot"), Xnp=_field(st_t, "Xnp"), Clb=_field(st_t, "Clb"), Cnb=_field(st_t, "Cnb"), CLa=_field(st_t, "CLa")))
    return out


def lsq(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    return k, my - k * mx


def run_config(tab, geo, f, theta, base: Path, n_sec=None, nchord=None, nspan=None):
    name = f"curl_{int(round(100 * f))}pct_{theta:g}deg" if theta > 0 else "flat"
    deck = build_deck(tab, geo, f, theta, name, n_sec, nchord, nspan)
    wd = base / name
    scan = avl_alphas(deck, wd / "scan", name, SCAN_ALPHAS)
    CL = [p["CL"] for p in scan]
    Cm = [p["Cm"] for p in scan]
    k, cm0 = lsq(CL, Cm)                                              # Cm = Cm0 + k CL about the CG
    cla_deg, cl0 = lsq(list(SCAN_ALPHAS), CL)
    xnp = sum(p["Xnp"] for p in scan) / len(scan)
    cref = geo["mac_in"] * 0.0254
    sm = (xnp - geo["cg_x_in"] * 0.0254) / cref
    trims = (k < 0 and cm0 > 0)
    cl_trim = -cm0 / k if trims else None
    a_trim = (cl_trim - cl0) / cla_deg if trims else None
    if trims and 0.0 < cl_trim < 1.5:
        a_run, at = a_trim, "trim"
    else:
        a_run, at = (0.5 - cl0) / cla_deg, "CL 0.5 (not trimmed)"
    lat = avl_alphas(deck, wd / "lateral", name, [a_run])[0]
    return dict(name=name, region=f, theta=theta, Xnp_m=xnp, SM=sm, Cm0=cm0, dCm_dCL=k, CLa_per_deg=cla_deg, CL0=cl0, trims=bool(trims), CL_trim=cl_trim, alpha_trim=a_trim,
                lateral_at=at, lateral_alpha=a_run, lateral_CL=lat["CL"], Cnb=lat["Cnb"], Clb=lat["Clb"], Xnp_lat_m=lat["Xnp"], scan=scan)


def evaluate(results):
    """Apply the pre-written pass rule.  Returns (verdict, text lines, recommended config or None)."""
    flat = next(r for r in results if r["theta"] == 0)
    lines = []
    ok1 = SM_LO <= flat["SM"] <= SM_HI
    lines.append(f"Rule 1, flat plate: SM = {100 * flat['SM']:.1f}% MAC (need {100 * SM_LO:.0f} to {100 * SM_HI:.0f}%): {'met' if ok1 else 'NOT MET'}")
    trimmers = [r for r in results if r["theta"] > 0 and CURL_RANGE[0] < r["theta"] <= CURL_RANGE[1] and r["trims"] and TRIM_CL_LO <= r["CL_trim"] <= TRIM_CL_HI]
    lines.append(f"Rule 2: {len(trimmers)} of {sum(1 for r in results if r['theta'] > 0)} curl configurations trim about the CG at a CL between {TRIM_CL_LO} and {TRIM_CL_HI}")
    ok1b = all(SM_LO <= r["SM"] <= SM_HI for r in trimmers)
    lines.append(f"Rule 1, every trimming curl: SM from {100 * min([r['SM'] for r in trimmers] or [float('nan')]):.1f} to {100 * max([r['SM'] for r in trimmers] or [float('nan')]):.1f}% MAC: {'met' if ok1b else 'NOT MET'}")
    agree = ok1 and ok1b and len(trimmers) > 0
    rec = min(trimmers, key=lambda r: (abs(r["CL_trim"] - 0.5), r["theta"])) if trimmers else None
    return ("AGREE" if agree else "DISAGREE"), lines, rec


def stage_avl():
    geo = json.loads((RES / "zanonia_geometry.json").read_text(encoding="utf-8"))
    tab = load_outline()
    base = RES / "avl"
    results = []
    for f in REGIONS:
        for th in THETAS:
            if th == 0 and f != REGIONS[0]:
                continue
            r = run_config(tab, geo, f, th, base)
            results.append(r)
            print(f"  curl {int(100 * f)}% {th:g} deg: SM {100 * r['SM']:.1f}%  Cm0 {r['Cm0']:+.4f}  trim CL {r['CL_trim'] if r['CL_trim'] is None else round(r['CL_trim'], 3)}  Cnb {r['Cnb']:+.4f}  Clb {r['Clb']:+.4f}")
    verdict, lines, rec = evaluate(results)
    (RES / "avl_results.json").write_text(json.dumps(dict(results=results, verdict=verdict, lines=lines, recommended=rec and rec["name"]), indent=1), encoding="utf-8")
    with (RES / "avl_results.csv").open("w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["curl region (outer fraction of semi-span)", "tip incidence deg", "Xnp mm", "static margin % MAC at the marked CG", "Cm0 about the CG (CL = 0)", "dCm/dCL", "trim CL", "trim alpha deg", "lateral values at", "Cn_beta per rad", "Cl_beta per rad"])
        for r in results:
            wr.writerow([r["region"], r["theta"], f"{r['Xnp_m'] * 1000:.3f}", f"{100 * r['SM']:.2f}", f"{r['Cm0']:+.5f}", f"{r['dCm_dCL']:+.5f}", "" if r["CL_trim"] is None else f"{r['CL_trim']:.3f}",
                         "" if r["alpha_trim"] is None else f"{r['alpha_trim']:.2f}", r["lateral_at"], f"{r['Cnb']:+.5f}", f"{r['Clb']:+.5f}"])
    print("VERDICT:", verdict)
    for l in lines:
        print("  ", l)
    print("recommended:", rec and rec["name"])
    return results, verdict, lines, rec


# =============================================================================================== stage 2b (after the verdict): section-count check and the CG + curl test
def stage_sections():
    """Neutral point at other section counts and panelling (the flat plate and one curl), to show how far the 0.33 mm margin is from the model's own numerical resolution."""
    geo = json.loads((RES / "zanonia_geometry.json").read_text(encoding="utf-8"))
    tab = load_outline()
    base = RES / "avl_sections"
    cases = [(15, 10, 80, "15 sections"), (20, 10, 80, "20 sections"), (30, 10, 80, "30 sections (the sweep)"), (40, 10, 80, "40 sections, uniform spacing"), (50, 10, 80, "50 sections, uniform spacing"),
             (30, 16, 128, "30 sections, 16 x 128 panels"), (40, 16, 160, "40 sections, 16 x 160 panels")]
    rows = []
    ref = None
    for n, nc, ns, label in cases:
        for f, th, nm in ((0.0, 0.0, "flat"), (0.3, 4.0, "30% curl, 4 deg")):
            try:
                r = run_config(tab, geo, f, th, base / f"n{n}_c{nc}_s{ns}", n, nc, ns)
                rows.append(dict(label=label, n_sec=n, nchord=nc, nspan=ns, config=nm, Xnp_mm=r["Xnp_m"] * 1000, SM=r["SM"], Cm0=r["Cm0"], CL_trim=r["CL_trim"], ok=True))
            except Exception as e:                                          # AVL refuses some spacings
                rows.append(dict(label=label, n_sec=n, nchord=nc, nspan=ns, config=nm, ok=False, why=str(e)[:80]))
    refs = {r["config"]: r for r in rows if r["n_sec"] == 30 and r["nchord"] == 10 and r["ok"]}
    mac_mm = geo["mac_in"] * 25.4
    for r in rows:
        if r["ok"]:
            d = r["Xnp_mm"] - refs[r["config"]]["Xnp_mm"]
            r["dNP_mm"], r["dNP_pct"] = d, 100 * d / mac_mm
    (RES / "avl_sections.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    with (RES / "avl_sections.csv").open("w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["case", "sections", "panels chord x span", "configuration", "Xnp mm (155 mm span)", "NP shift vs 30 sections mm", "NP shift % MAC", "static margin % MAC", "Cm0", "trim CL"])
        for r in rows:
            if r["ok"]:
                wr.writerow([r["label"], r["n_sec"], f"{r['nchord']} x {r['nspan']}", r["config"], f"{r['Xnp_mm']:.3f}", f"{r['dNP_mm']:+.3f}", f"{r['dNP_pct']:+.2f}", f"{100 * r['SM']:.2f}", f"{r['Cm0']:+.5f}", "" if r["CL_trim"] is None else f"{r['CL_trim']:.3f}"])
            else:
                wr.writerow([r["label"], r["n_sec"], f"{r['nchord']} x {r['nspan']}", r["config"], "AVL refused: " + r["why"], "", "", "", "", ""])
    for r in rows:
        print(r["label"], r["config"], ("NP %.3f mm  shift %+.3f mm (%+.2f%% MAC)  SM %.2f%%" % (r["Xnp_mm"], r["dNP_mm"], r["dNP_pct"], 100 * r["SM"])) if r["ok"] else "REFUSED " + r["why"])
    return rows


def curl_for(results, region, cm0_needed):
    """Tip incidence that gives a wanted Cm0, by linear interpolation in the sweep table (flat plate = the 0 deg point); beyond 10 deg the last segment is extended."""
    pts = [(0.0, next(r for r in results if r["theta"] == 0)["Cm0"])] + sorted((r["theta"], r["Cm0"]) for r in results if r["theta"] > 0 and abs(r["region"] - region) < 1e-9)
    for (t0, c0), (t1, c1) in zip(pts, pts[1:]):
        if c0 <= cm0_needed <= c1:
            return t0 + (t1 - t0) * (cm0_needed - c0) / (c1 - c0), False
    (t0, c0), (t1, c1) = pts[-2], pts[-1]
    return t1 + (t1 - t0) * (cm0_needed - c1) / (c1 - c0), True


TARGET_CL = 0.4
CG_SHIFTS_MM = (0.0, 1.0, 2.0)


def stage_cg_curl():
    """For a trim CL of about 0.4: Cm0 needed = 0.4 x SM at each CG (trim CL = Cm0 / SM, Cm0 does not depend on the reference point), the curl from the table, then an AVL check at that CG and curl."""
    geo = json.loads((RES / "zanonia_geometry.json").read_text(encoding="utf-8"))
    tab = load_outline()
    ar = json.loads((RES / "avl_results.json").read_text(encoding="utf-8"))
    results = ar["results"]
    flat = next(r for r in results if r["theta"] == 0)
    mac_mm = geo["mac_in"] * 25.4
    rows = []
    for d in CG_SHIFTS_MM:
        sm = flat["SM"] + d / mac_mm
        need = TARGET_CL * sm
        for f in REGIONS:
            th, extrap = curl_for(results, f, need)
            g2 = dict(geo, cg_x_in=geo["cg_x_in"] - d / 25.4)
            r = run_config(tab, g2, f, th, RES / "avl_cg_curl" / f"fwd{d:g}mm", None)
            rows.append(dict(shift_mm=d, region=f, SM_calc=sm, Cm0_needed=need, theta_calc=th, extrapolated=extrap, SM_avl=r["SM"], Cm0_avl=r["Cm0"], CL_trim_avl=r["CL_trim"], Cnb=r["Cnb"], Clb=r["Clb"], lateral_at=r["lateral_at"]))
            print(f"fwd {d:g} mm region {int(100 * f)}%: SM {100 * sm:.2f}% need Cm0 {need:.4f} -> {th:.1f} deg{' (extrapolated)' if extrap else ''}; AVL check SM {100 * r['SM']:.2f}% trim CL {r['CL_trim'] and round(r['CL_trim'], 3)} Cn {r['Cnb']:+.4f} Cl {r['Clb']:+.4f}")
    (RES / "avl_cg_curl.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    with (RES / "avl_cg_curl.csv").open("w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["CG forward of the marked cross, mm", "curl region", "static margin % MAC (CALCULATED)", "Cm0 needed for trim CL 0.4 (CALCULATED)", "tip incidence needed, deg (CALCULATED, interpolated)", "extrapolated beyond 10 deg",
                     "AVL check: static margin %", "AVL check: trim CL", "AVL check: Cn_beta", "AVL check: Cl_beta"])
        for r in rows:
            wr.writerow([r["shift_mm"], f"{int(100 * r['region'])}%", f"{100 * r['SM_calc']:.2f}", f"{r['Cm0_needed']:.4f}", f"{r['theta_calc']:.1f}", "yes" if r["extrapolated"] else "no", f"{100 * r['SM_avl']:.2f}",
                         "" if r["CL_trim_avl"] is None else f"{r['CL_trim_avl']:.3f}", f"{r['Cnb']:+.4f}", f"{r['Clb']:+.4f}"])
    return rows


# =============================================================================================== stage 3: scaled outlines, plates, jigs, masses, templates, page
import flat_test as ft                                   # read-only: the kit's mesh helpers (prism, edge_report, mesh_stats) and constants

PLATE_T_MM = ft.THICKNESS_MM                             # 0.6 mm: the kit plate thickness (3 layers of 0.2 mm), flat_test.THICKNESS_MM
RHO_PLA = ft.PLA_DENSITY_G_CM3                           # 1.24 g/cm3, the project's PLA density (flat_test.PLA_DENSITY_G_CM3, ASSUMED there; typical datasheet value, not re-checked)
LW_PLA = (0.6, 0.8)                                      # g/cm3, ASSUMED range for foamed LW-PLA
FOAM_PLATE_AREAL = 100.0                                 # g/m2, ASSUMED for a disposable foam food plate (typical 60-120)
FOAM_BOARD_AREAL = ft.FOAM_AREAL_MASS_G_M2               # 300 g/m2, the project's foamboard assumption
CURL_REGION_GENERIC = 0.30                               # outer fraction of the semi-span, JUDGEMENT (README: no AVL recommendation on DISAGREE)
JIG_ANGLES_GENERIC = (2.0, 4.0, 6.0, 8.0, 10.0, 12.0)         # deg: 4, 6, 8 from README section 3, plus 2, 10, 12 added after the CG + curl test (the needed curl exceeds 8 deg at 1 mm forward)
SIZES_MM = (155.0, 250.0)


def rdp(pts, tol):
    if len(pts) < 3:
        return pts
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    dx, dy = x1 - x0, y1 - y0
    n = math.hypot(dx, dy) or 1e-12
    dmax, imax = -1.0, 0
    for i in range(1, len(pts) - 1):
        d = abs(dy * (pts[i][0] - x0) - dx * (pts[i][1] - y0)) / n
        if d > dmax:
            dmax, imax = d, i
    if dmax > tol:
        return rdp(pts[:imax + 1], tol)[:-1] + rdp(pts[imax:], tol)
    return [pts[0], pts[-1]]


class Scaled:
    """The traced outline scaled to a span (mm): LE / TE functions, a simplified closed polygon, area and centroid."""

    def __init__(self, tab, geo, span_mm):
        self.k = span_mm / (geo["span_in"] * 25.4)
        self.span = span_mm
        self.tab = [(s * 25.4 * self.k, xl * 25.4 * self.k, xt * 25.4 * self.k) for s, xl, xt in tab]
        self.half = self.tab[-1][0]
        self.geo = geo
        le = [(xl, s) for s, xl, xt in self.tab]
        te = [(xt, s) for s, xl, xt in self.tab]
        tol = 0.0003 * span_mm
        le_r = rdp([(xl, s) for s, xl, xt in self.tab], tol)
        te_r = rdp([(xt, s) for s, xl, xt in self.tab], tol)
        left = [(x, -s) for x, s in reversed(le_r[1:])]                    # LE from the -tip to the centre
        right_le = le_r
        # closed loop: LE from -tip to +tip, TE from +tip back to -tip
        loop = left + right_le + list(reversed([(x, s) for x, s in te_r])) + [(x, -s) for x, s in te_r[1:]]
        self.poly = ft.orient_ccw(loop)
        self.area = abs(ft.area_signed(self.poly))
        self.centroid = ft.polygon_centroid(self.poly)
        self.cg_x = geo["cg_x_in"] * 25.4 * self.k
        self.hole_x = geo["hole_x_in"] * 25.4 * self.k
        self.hole_d = geo["hole_diam_in"] * 25.4 * self.k
        self.mac = geo["mac_in"] * 25.4 * self.k
        self.x_le_mac = geo["x_le_mac_in"] * 25.4 * self.k

    def xle(self, s):
        return interp(self.tab, abs(s))[0]

    def xte(self, s):
        return interp(self.tab, abs(s))[1]


# --- ASCII STL
def write_stl_ascii(path: Path, tris, name):
    with path.open("w", encoding="ascii", newline="\n") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            m = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(f" facet normal {n[0] / m:.6e} {n[1] / m:.6e} {n[2] / m:.6e}\n  outer loop\n")
            for p in (a, b, c):
                f.write(f"   vertex {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")


def read_stl_ascii(path: Path):
    tris, cur = [], []
    for line in path.read_text(encoding="ascii").splitlines():
        t = line.split()
        if t and t[0] == "vertex":
            cur.append(tuple(float(v) for v in t[1:4]))
            if len(cur) == 3:
                tris.append(tuple(cur))
                cur = []
    return tris


# --- polygon with one hole: bridge, then ear clipping that ignores coincident vertices
def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def ear_clip(poly):
    idx = list(range(len(poly)))
    tris = []
    guard = 0
    while len(idx) > 3:
        n = len(idx)
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if _cross(a, b, c) <= 1e-12:
                continue
            ok = True
            for j in idx:
                if j in (i0, i1, i2):
                    continue
                p = poly[j]
                if p == a or p == b or p == c:
                    continue
                if _cross(a, b, p) >= -1e-12 and _cross(b, c, p) >= -1e-12 and _cross(c, a, p) >= -1e-12:
                    ok = False
                    break
            if ok:
                tris.append((i0, i1, i2))
                del idx[k]
                break
        else:
            raise ValueError("ear clipping failed")
    tris.append(tuple(idx))
    return tris


def seg_cross(p, q, r, s):
    d1, d2, d3, d4 = _cross(p, q, r), _cross(p, q, s), _cross(r, s, p), _cross(r, s, q)
    return d1 * d2 < 0 and d3 * d4 < 0


def bridge_hole(outer, hole):
    """outer CCW, hole CW -> one CCW polygon with a zero-width bridge; returns the polygon."""
    hi = max(range(len(hole)), key=lambda i: -hole[i][0])                 # the hole vertex nearest the leading edge (smallest x)
    h = hole[hi]
    cands = sorted(range(len(outer)), key=lambda i: math.dist(outer[i], h))
    for oi in cands:
        v = outer[oi]
        bad = False
        for i in range(len(outer)):
            if i == oi or (i + 1) % len(outer) == oi:
                continue
            if seg_cross(h, v, outer[i], outer[(i + 1) % len(outer)]):
                bad = True
                break
        if not bad:
            for i in range(len(hole)):
                if seg_cross(h, v, hole[i], hole[(i + 1) % len(hole)]):
                    bad = True
                    break
        if not bad:
            hs = hole[hi:] + hole[:hi]
            return outer[:oi + 1] + hs + [hs[0]] + [outer[oi]] + outer[oi + 1:]
    raise ValueError("no bridge found")


def circle_pts(cx, cy, r, n=32, cw=False):
    pts = [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    return list(reversed(pts)) if cw else pts


def plate_mesh(sc: Scaled, t, mark_len, mark_w, mark_h, hole_d):
    """Flat plate with a through hole at the pattern's hole position and a raised CG cross on the flight underside (the face at z = t, which is the top of the print)."""
    outer = sc.poly
    hole = circle_pts(sc.hole_x, 0.0, hole_d / 2, 32, cw=True)
    merged = bridge_hole(outer, hole)
    tris = []
    for i, j, k in ear_clip(merged):
        tris.append(((*merged[i], t), (*merged[j], t), (*merged[k], t)))
        tris.append(((*merged[k], 0.0), (*merged[j], 0.0), (*merged[i], 0.0)))
    for loop in (outer, hole):
        n = len(loop)
        for i in range(n):
            tris += ft._wall(loop[i], loop[(i + 1) % n], 0.0, t)
    bar = lambda x0, y0, x1, y1: ft.prism([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], t, t + mark_h)
    cx = sc.cg_x
    # the two bars of the cross share the centre square; build them as three solids so no solid overlaps another
    a = mark_w / 2
    marks = (bar(cx - mark_len / 2, -a, cx - a, a) + bar(cx + a, -a, cx + mark_len / 2, a) + bar(cx - a, -mark_len / 2, cx + a, mark_len / 2))
    return tris, marks


# --- 7-segment digits for the jig angle (drawn in (u, v): u along +y, v up the page = toward the leading edge = -x; mapped by a proper rotation so the prisms stay counter-clockwise)
SEG = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc", "7": "abc", "8": "abcdefg", "9": "abcdfg"}


def uv_rect(xb, ua, va, ub, vb, z0, z1):
    pts = [(xb - va, ua), (xb - va, ub), (xb - vb, ub), (xb - vb, ua)]
    return ft.prism(ft.orient_ccw(pts), z0, z1)


def digit_solids(ch, xb, u0, v0, w, h, st, z0, z1):
    """Raised 7-segment digit, lower-left corner (u0, v0), width w, height h, stroke st; xb is the chordwise position of v = 0."""
    segs = {"a": (u0, v0 + h - st, u0 + w, v0 + h), "d": (u0, v0, u0 + w, v0 + st), "g": (u0, v0 + h / 2 - st / 2, u0 + w, v0 + h / 2 + st / 2),
            "f": (u0, v0 + h / 2, u0 + st, v0 + h), "e": (u0, v0, u0 + st, v0 + h / 2), "b": (u0 + w - st, v0 + h / 2, u0 + w, v0 + h), "c": (u0 + w - st, v0, u0 + w, v0 + h / 2)}
    out = []
    for sname in SEG[ch]:
        out += uv_rect(xb, *segs[sname][:2], *segs[sname][2:], z0, z1)
    return out


def jig_mesh(sc: Scaled, theta, region, label, digits=True):
    """Bending jig: a block whose top face follows the curled tip.  The plate lies UPSIDE DOWN on it (flight upper face down, as in 'rolled upside down over a dowel'); inside the curl region the top face
    slopes down behind the leading edge by the local twist angle, which ramps linearly from 0 at the start of the region to theta at the tip.  Height field on a grid, watertight by construction."""
    h0 = 8.0 if sc.span < 200 else 10.0
    step = sc.span / 80.0
    xlo, xhi = -18.0 * sc.span / 155.0, max(sc.tab[i][2] for i in range(len(sc.tab))) + 4.0
    ylo, yhi = -(sc.half + 4.0), sc.half + 4.0
    nx, ny = int(math.ceil((xhi - xlo) / step)), int(math.ceil((yhi - ylo) / step))
    xs = [xlo + i * (xhi - xlo) / nx for i in range(nx + 1)]
    ys = [ylo + j * (yhi - ylo) / ny for j in range(ny + 1)]
    s0 = (1.0 - region) * sc.half

    def top(x, y):
        s = abs(y)
        if s <= s0 or s > sc.half:
            if s > sc.half:
                s = sc.half
            else:
                return h0
        phi = math.radians(theta * (s - s0) / (sc.half - s0))
        xl, xt = interp(sc.tab, s)
        d = min(max(0.0, x - xl), (xt - xl) + 3.0)
        return h0 - d * math.tan(phi)
    z = [[top(x, y) for y in ys] for x in xs]
    tris = []
    for i in range(nx):
        for j in range(ny):
            a, b, c, d = (xs[i], ys[j], z[i][j]), (xs[i + 1], ys[j], z[i + 1][j]), (xs[i + 1], ys[j + 1], z[i + 1][j + 1]), (xs[i], ys[j + 1], z[i][j + 1])
            tris += [(a, b, c), (a, c, d)]
            a0, b0, c0, d0 = (a[0], a[1], 0.0), (b[0], b[1], 0.0), (c[0], c[1], 0.0), (d[0], d[1], 0.0)
            tris += [(c0, b0, a0), (d0, c0, a0)]
    for i in range(nx):                                                  # walls: y = ylo and y = yhi
        for j, sgn in ((0, 1), (ny, -1)):
            p, q = (xs[i], ys[j]), (xs[i + 1], ys[j])
            zp, zq = z[i][j], z[i + 1][j]
            if sgn == 1:                                                  # outward normal -y
                tris += [((*p, 0.0), (*q, 0.0), (*q, zq)), ((*p, 0.0), (*q, zq), (*p, zp))]
            else:
                tris += [((*q, 0.0), (*p, 0.0), (*p, zp)), ((*q, 0.0), (*p, zp), (*q, zq))]
    for j in range(ny):                                                  # walls: x = xlo and x = xhi
        for i, sgn in ((0, 1), (nx, -1)):
            p, q = (xs[i], ys[j]), (xs[i], ys[j + 1])
            zp, zq = z[i][j], z[i][j + 1]
            if sgn == 1:                                                  # outward normal -x
                tris += [((*q, 0.0), (*p, 0.0), (*p, zp)), ((*q, 0.0), (*p, zp), (*q, zq))]
            else:
                tris += [((*p, 0.0), (*q, 0.0), (*q, zq)), ((*p, 0.0), (*q, zq), (*p, zp))]
    if not digits:
        return tris, dict(h0=h0, nx=nx, ny=ny, step=step)
    # raised digits on the flat margin in front of the plate (never under the plate): the angle, then a small degree ring
    k = sc.span / 155.0
    txt = f"{theta:g}"
    dw, dh, st = 5.0 * k, 9.0 * k, 1.3 * k
    gap = 2.0 * k
    total = len(txt) * (dw + gap) + 3.0 * k
    u = -total / 2
    xb = xlo + 3.0 * k + dh                                                 # chordwise position of v = 0 (the digit's baseline is the side away from the leading edge)
    for ch in txt:
        tris += digit_solids(ch, xb, u, 0.0, dw, dh, st, h0, h0 + 0.8)
        u += dw + gap
    r = 2.4 * k
    ring = [((u, dh - r, u + r, dh - 0.9 * k), 0), ((u, dh - r, u + 0.9 * k, dh), 0), ((u + r - 0.9 * k, dh - r, u + r, dh), 0), ((u, dh - 0.9 * k, u + r, dh), 0)]
    for (ua, va, ub, vb), _ in ring:
        tris += uv_rect(xb, ua, va, ub, vb, h0, h0 + 0.8)
    return tris, dict(h0=h0, nx=nx, ny=ny, step=step)


def mesh_ok(tris):
    n_e, bad, same, degen = ft.edge_report(tris, nd=4)
    st = ft.mesh_stats(tris)
    return dict(triangles=len(tris), edges=n_e, edges_not_shared_by_two=bad, repeated_directed=same, degenerate=degen, volume_mm3=st["volume"], centroid=st["centroid"])


def mass_rows(sc: Scaled, vol_mm3, centroid_x):
    """Plate mass, nose weight to reach the marked CG (default) and the CG 1 and 2 mm forward of it (test option), all-up mass and wing loading, for each material.
    Nose weight m = M (x_CG - x_c) / (x_hole - x_CG), x_c = the plate's centroid, positive = weight at the hole ahead of the centroid."""
    S_m2 = sc.area * 1e-6
    mats = [("PLA 100% solid", f"{RHO_PLA:g} g/cm3", vol_mm3 * 1e-3 * RHO_PLA, "ASSUMED (the project's value, flat_test.PLA_DENSITY_G_CM3)")]
    for r in sorted(LW_PLA, reverse=True):
        mats.append(("LW-PLA foamed", f"{r:g} g/cm3", vol_mm3 * 1e-3 * r, "ASSUMED range 0.6 to 0.8"))
    mats.append(("foam food plate (the original)", f"{FOAM_PLATE_AREAL:g} g/m2", sc.area * 1e-6 * FOAM_PLATE_AREAL, "ASSUMED (typical 60 to 120 g/m2; thickness about 1 to 2 mm, not 0.6)"))
    rows = []
    for nm, dens, m, tag in mats:
        row = dict(material=nm, density=dens, plate_g=m, tag=tag)
        for d in (0.0, 1.0, 2.0):
            cg = sc.cg_x - d
            nose = m * (cg - centroid_x) / (sc.hole_x - cg)
            tot = m + nose
            key = "" if d == 0 else f"_f{d:g}"
            row["nose_g" + key], row["total_g" + key], row["loading" + key] = nose, tot, tot * 1e-3 * 9.80665 / S_m2
        rows.append(row)
    return rows


def scale_rows(tab, geo, spans=(155.0, 250.0, 400.0)):
    out = []
    for sp in spans:
        sc = Scaled(tab, geo, sp)
        m = sc.area * 1e-6 * FOAM_BOARD_AREAL
        cx = sc.centroid[0]
        nose = m * (sc.cg_x - cx) / (sc.hole_x - sc.cg_x)
        out.append(dict(span=sp, chord_root=sc.tab[0][2] - sc.tab[0][1], mac=sc.mac, area_cm2=sc.area / 100.0, cg_mm=sc.cg_x, hole_mm=sc.hole_x, plate_g=m, nose_g=nose, total_g=m + nose,
                        loading=(m + nose) * 1e-3 * 9.80665 / (sc.area * 1e-6)))
    return out


# =============================================================================================== templates (cardstock, white, 1:1, Letter) - secondary to the printed plates
def template_svg(sc: Scaled, half, title, notes):
    W, H, mar = ft.LETTER_W, ft.LETTER_H, ft.TEMPLATE_MARGIN
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">', f'<rect x="0" y="0" width="{W}" height="{H}" fill="#ffffff"/>']
    t = lambda x, y, s, size=3.2, w="normal": o.append(f'<text x="{x:.2f}" y="{y:.2f}" font-family="Helvetica" font-size="{size}" fill="#000000" font-weight="{w}">{html.escape(s)}</text>')
    ln = lambda x1, y1, x2, y2, col="#000000", w=0.35, dash=None: o.append(f'<line x1="{x1:.3f}" y1="{y1:.3f}" x2="{x2:.3f}" y2="{y2:.3f}" stroke="{col}" stroke-width="{w}"' + (f' stroke-dasharray="{dash}"' if dash else "") + "/>")
    t(mar, 15, title, 5.4, "bold")
    for i, s in enumerate(notes):
        t(mar, 21 + 4.2 * i, s, 3.0)
    ys = 21 + 4.2 * len(notes) + 6
    o.append(f'<line id="scalebar" x1="{mar:.4f}" y1="{ys:.4f}" x2="{mar + 100:.4f}" y2="{ys:.4f}" stroke="#000000" stroke-width="0.5"/>')
    for i in range(11):
        ln(mar + 10 * i, ys - 2, mar + 10 * i, ys + 2)
    t(mar + 103, ys + 1.2, "100 mm: this bar must measure 100 mm on your print", 3.0)
    cx = mar + 6.0 if half else W / 2
    y0 = ys + 22
    X = lambda s_: cx + s_
    Y = lambda x_: y0 + x_
    pts = [(X(y), Y(x)) for x, y in sc.poly]
    if half:
        # keep the right half (s >= 0) only: clip the polygon at s = 0 by rebuilding from the table
        le = [(sc.tab[i][0], sc.tab[i][1]) for i in range(len(sc.tab))]
        te = [(sc.tab[i][0], sc.tab[i][2]) for i in range(len(sc.tab))]
        pts = [(X(s_), Y(x)) for s_, x in le] + [(X(s_), Y(x)) for s_, x in reversed(te)]
    o.append('<polygon points="' + " ".join(f"{a:.3f},{b:.3f}" for a, b in pts) + '" fill="#f4f6f8" stroke="#000000" stroke-width="0.4"/>')
    if half:
        ln(cx, Y(0) - 6, cx, Y(sc.tab[0][2]) + 6, "#000000", 0.35, "3 2")
        t(cx + 1.5, Y(0) - 3.0, "centre line: fold or flip here for the other half", 2.8)
    # curl region boundaries
    for f_, lab in ((0.2, "20%"), (0.3, "30%"), (0.4, "40%")):
        s_ = (1 - f_) * sc.half
        for sg in ((1,) if half else (1, -1)):
            xl, xt = interp(sc.tab, s_)
            ln(X(sg * s_), Y(xl) - 1, X(sg * s_), Y(xt) + 1, "#7a3fb0", 0.35, "1.2 1.2")
        t(X(s_) - 4, Y(interp(sc.tab, s_)[1]) + 5.5, f"curl {lab}", 2.4)
    # hole and CG mark
    n = 24
    hole = [(X(0) + (sc.hole_d / 2) * math.cos(2 * math.pi * i / n) * 0, 0)]
    o.append('<polygon points="' + " ".join(f"{X(0) + (sc.hole_d / 2) * math.sin(2 * math.pi * i / n):.3f},{Y(sc.hole_x) + (sc.hole_d / 2) * math.cos(2 * math.pi * i / n):.3f}" for i in range(n)) + '" fill="#ffffff" stroke="#000000" stroke-width="0.4"/>')
    t(X(0) + sc.hole_d / 2 + 2, Y(sc.hole_x) + 1, f"ballast hole, dia {sc.hole_d:.1f} mm", 2.6)
    ln(X(0) - 6, Y(sc.cg_x), X(0) + 6, Y(sc.cg_x), "#1b7f3f", 0.5)
    ln(X(0), Y(sc.cg_x) - 6, X(0), Y(sc.cg_x) + 6, "#1b7f3f", 0.5)
    t(X(0) + 8, Y(sc.cg_x) + 1, f"CG mark: {sc.cg_x:.1f} mm behind the leading edge ({100 * (sc.cg_x - sc.x_le_mac) / sc.mac:.1f}% of MAC)", 2.6)
    # millimetre ticks along the centre line, from the leading edge
    for mm in range(0, int(sc.tab[0][2]) + 1, 5):
        ln(X(0) - (3 if mm % 10 == 0 else 1.6), Y(mm), X(0), Y(mm), "#000000", 0.25)
        if mm % 10 == 0:
            t(X(0) - 12, Y(mm) + 1, str(mm), 2.2)
    o.append("</svg>")
    return "\n".join(o)


def build_templates(tab, geo, out: Path):
    s155, s250 = Scaled(tab, geo, 155.0), Scaled(tab, geo, 250.0)
    notes_common = ["Cardstock or foam-plate template, secondary to the printed plates. Print at 100 % (not fit to page); check the bar below.",
                    "Cut the solid line. Curl: roll each tip, flight-upper face DOWN, over a 3/16 in (4.8 mm) dowel (the page: 'just a slight curve').",
                    "Purple dotted lines: 20, 30 and 40% of the semi-span. Circle: ballast hole for clay or blu-tack.",
                    "Green cross: the CG marked on the published pattern (MEASURED-FROM-SCAN). Ticks along the centre line: mm from the leading edge."]
    p1 = template_svg(s155, False, "Zanonia glider, published 6 in pattern (span 155 mm), full outline, 1:1", notes_common)
    p2 = template_svg(s250, True, "Zanonia glider, 250 mm span, ONE HALF, 1:1 (flip on the centre line)", notes_common)
    paths = []
    for i, p in enumerate((p1, p2), 1):
        pp = out / f"template_page{i}.svg"
        pp.write_text(p, encoding="utf-8")
        paths.append(pp)
    pdf = out / "templates.pdf"
    subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(pdf)] + [str(p) for p in paths], check=True, stdout=subprocess.DEVNULL)
    return pdf, paths


# =============================================================================================== the page
CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37;--bad:#ff8a80;--good:#6cc08b}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 "Segoe UI",system-ui,sans-serif}main{max-width:1180px;margin:0 auto;padding:26px 20px 60px;display:flex;flex-direction:column;gap:16px}
h1{font-size:26px;margin:0}h2{font-size:19px;margin:0;border-bottom:1px solid var(--line);padding-bottom:5px}h3{font-size:15px;margin:6px 0 0}p,ul{margin:0;max-width:105ch}ul{padding-left:22px}
table{border-collapse:collapse;background:var(--panel);font-size:13px}th,td{border:1px solid var(--line);padding:3px 8px;text-align:left;vertical-align:top}th{background:var(--thead)}td.n{text-align:right;white-space:nowrap}
.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:14px}.box{background:var(--panel);border:2px solid var(--accent);padding:10px 16px}.bad{background:#3b1d1d}.good{background:#1d3b2a}.scroll{overflow-x:auto}code{font-size:13px}
.pill{display:inline-block;font-size:10.5px;font-weight:700;border:1px solid;border-radius:9px;padding:0 6px;letter-spacing:.03em}.svgwrap{max-width:520px}"""
TAGS = {"CITED": "#9aa7b4", "MEASURED-FROM-SCAN": "#6cc08b", "CALCULATED": "#7fb4ff", "MODEL": "#7fb4ff", "ASSUMED": "#e89a55", "JUDGEMENT": "#c39bff"}


def pill(tag):
    c = TAGS[tag]
    return f"<span class='pill' style='color:{c};border-color:{c}'>{tag}</span>"


def resolution_text(geo, results, sections):
    """The size of the DISAGREE against the drawing's resolution (all CALCULATED from files already made)."""
    flat = results[0]
    mac_mm = geo["mac_in"] * 25.4
    gap = flat["Xnp_m"] * 1000 - geo["cg_x_in"] * 25.4
    floor = SM_LO * mac_mm
    asym = geo["asym_rms_in"] * 25.4
    spread = max(abs(r["dNP_mm"]) for r in sections if r["ok"])
    return dict(gap_mm=gap, floor_mm=floor, asym_mm=asym, pixel_mm=25.4 / DPI, np_spread_mm=spread, mac_mm=mac_mm)


CAUSES = ["(i) the hand-drawn CG cross is approximate (its two strokes are a few tenths of a millimetre wide and the drawing's two halves differ by about the same);",
          "(ii) AVL's neutral point for a low-aspect-ratio swept flat plate: plate_balance suggests the real aerodynamic centre is 0 to 5% of the MAC ahead of AVL's, which alone is 0 to 1.6 mm at this size;",
          "(iii) the published glider is probably near-neutral and forgiving at its very low wing loading (about 1.7 N/m2), so it need not have a 3% margin to fly;",
          "(iv) the real foam plate is several millimetres thick on a 37 mm chord, which the flat-plate model ignores;",
          "(v) the real curl may be camber or dihedral rather than twist: that changes the trim, not the neutral point."]


def fwd_text():
    return ("The forward CGs are model-based and Kai's choice: a margin of about 1% is where the pull-out model cannot say, and measured plates put the aerodynamic centre ahead of AVL's, "
            "so the real margin at each CG is uncertain by about the size of the shifts tried. The knife-edge balance on the real part is what counts.")


def update_readme(sections):
    """Restore README section 3 to its pre-run wording (40 sections) and append the change note below, with the section-count table."""
    p = RES / "README.md"
    t = p.read_text(encoding="utf-8")
    t = t.replace("in 30 spanwise sections", "in 40 spanwise sections")
    k = t.find("\n## 5. Changes after the AVL run")
    if k >= 0:
        t = t[:k]
    rows = ["| case | configuration | Xnp, mm (155 mm span) | shift vs the 30-section run, mm | % MAC | static margin % MAC |", "|---|---|---|---|---|---|"]
    for r in sections:
        if r["ok"]:
            rows.append(f"| {r['label']} | {r['config']} | {r['Xnp_mm']:.3f} | {r['dNP_mm']:+.3f} | {r['dNP_pct']:+.2f} | {100 * r['SM']:.2f} |")
        else:
            rows.append(f"| {r['label']} | {r['config']} | AVL refused the deck | | | |")
    t += ("\n## 5. Changes after the AVL run (added 2026-10-19; sections 1 to 4 above are as written before the run)\n"
          "- **Section count.** Section 3 said 40 spanwise sections. The sweep was actually run with **30 equally spaced** sections. Reason: the first attempt used 40 sections with sine clustering toward the tip; AVL refused the deck "
          "(\"Cannot adjust spanwise spacing at section 34 ... Insufficient number of spanwise vortices\") and wrote no output, so no result existed when I changed the count. "
          "An earlier version of this README was edited to say 30 at that point; it is restored above to the original wording and the change is recorded here.\n"
          "- **Effect of the section count on the neutral point** (flat plate and the 30% / 4 deg curl, same CG; shift relative to the 30-section sweep; MAC = 32.6 mm):\n\n" + "\n".join(rows) + "\n\n"
          "40 equally spaced sections do run (the failure was the sine spacing, not the count), and give the same neutral point to 0.002 mm. The neutral point moves by at most 0.03 mm (0.1% of MAC) with the section count and 0.02 mm with finer panelling, "
          "so the verdict does not depend on this choice.\n"
          "- **Later additions** (not part of the pre-run rule): the section-count check above; a test of the CG moved 1 and 2 mm forward together with a larger curl (`avl_cg_curl.csv`); jigs at 2, 10 and 12 deg; nose weights at the forward CGs. None of them changes the verdict.\n")
    p.write_text(t, encoding="utf-8")


def build_page(geo, results, verdict, lines, rec, tab, scale, prints, jigs, mass_tabs, pdf_name):
    h = html.escape
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Zanonia known-good check</title><style>{CSS}</style></head><body><main>"]
    P.append("<h1>Foam Zanonia Glider through our AVL chain</h1>")
    P.append(f"<p class='note'>Source: endlessLift, Gary Hinze 2011, after Paul McIlrath 1983 {pill('CITED')}. Tags: {pill('MEASURED-FROM-SCAN')} traced from the 300 dpi pattern scan; {pill('CALCULATED')} arithmetic on that trace; {pill('MODEL')} AVL 3.52 (inviscid, flat plate, no airfoil); "
             f"{pill('ASSUMED')} densities and masses; {pill('JUDGEMENT')} choices. Nothing here was flown or built. Rule and geometry: <code>research/known_good/zanonia/README.md</code>.</p>")
    cls = "good" if verdict == "AGREE" else "bad"
    P.append(f"<div class='box {cls}'><h2 style='border:0'>Verdict on the pre-written rule: {verdict}</h2><p>" + "<br>".join(h(l) for l in lines) + "</p>"
             "<p>The rule was written before the AVL run (static margin at the marked CG 3 to 25% of MAC, and some curl of 0 to 10 degrees trimming at a CL of 0.2 to 0.9). "
             f"The static margin at the pattern's CG is <b>{100 * results[0]['SM']:.1f}% of MAC</b>: positive, but below the 3% floor, so the rule records a DISAGREE; per the rule nothing was adjusted and the pull-out run was not done. "
             "Observations (not used to change the verdict): the marked CG sits almost on AVL's neutral point; AVL's neutral point for flat plates is uncertain by several percent of the chord (plate_balance found the real aerodynamic centre ahead of AVL's), so a true margin could be zero or negative; the author reports a flight with 'a little wobble'.</p></div>")
    sec = json.loads((RES / "avl_sections.json").read_text(encoding="utf-8"))
    cgc = json.loads((RES / "avl_cg_curl.json").read_text(encoding="utf-8"))
    rz = resolution_text(geo, results, sec)
    P.append(f"<div class='box'><h2 style='border:0'>How big is the DISAGREE? Inside drawing resolution</h2><p>The neutral point is <b>{rz['gap_mm']:.2f} mm</b> behind the marked cross; the rule's floor (3% of MAC) is {rz['floor_mm']:.2f} mm "
             f"(about 1 mm); the traced outline's two halves differ by {rz['asym_mm']:.2f} mm rms; one scan pixel is {rz['pixel_mm']:.2f} mm; and the neutral point moves by at most {rz['np_spread_mm']:.2f} mm with the AVL section count (table below). "
             "The margin is the size of the drawing's own scatter. <b>It is not evidence that the published mark or the model is wrong.</b> Candidate explanations, none tested here:</p><ul>" + "".join(f"<li>{h(c)}</li>" for c in CAUSES) + "</ul></div>")
    P.append("<h2>Geometry from the scan</h2><table><tr><th>quantity</th><th>value</th><th>tag</th></tr>"
             f"<tr><td>span</td><td>{geo['span_in']:.3f} in ({geo['span_in'] * 25.4:.1f} mm); the pattern is called 6 in</td><td>{pill('MEASURED-FROM-SCAN')}</td></tr>"
             f"<tr><td>centre chord</td><td>{geo['root_chord_in']:.3f} in ({geo['root_chord_in'] * 25.4:.1f} mm)</td><td>{pill('MEASURED-FROM-SCAN')}</td></tr>"
             f"<tr><td>area, MAC, aspect ratio</td><td>{geo['area_in2']:.3f} in&sup2; ({geo['area_in2'] * 6.4516:.2f} cm&sup2;), {geo['mac_in']:.3f} in ({geo['mac_in'] * 25.4:.1f} mm), {geo['ar']:.2f}</td><td>{pill('CALCULATED')}</td></tr>"
             f"<tr><td>leading-edge sweep</td><td>tangent {geo['sweep_le_25_deg']:.0f} deg at 25% semi-span, {geo['sweep_le_mid_deg']:.0f} at 50%, {geo['sweep_le_75_deg']:.0f} at 75%; {geo['sweep_le_overall_deg']:.1f} deg centre to 80% semi-span; quarter-chord {geo['sweep_qc_overall_deg']:.1f} deg</td><td>{pill('MEASURED-FROM-SCAN')}</td></tr>"
             f"<tr><td>CG cross</td><td>{geo['cg_x_in']:.3f} in ({geo['cg_x_in'] * 25.4:.1f} mm) behind the centre leading edge = <b>{geo['cg_pct_mac']:.1f}% of MAC</b></td><td>{pill('MEASURED-FROM-SCAN')}</td></tr>"
             f"<tr><td>ballast hole</td><td>centre {geo['hole_x_in']:.3f} in ({geo['hole_x_in'] * 25.4:.1f} mm) behind the leading edge, dia {geo['hole_diam_in']:.3f} in ({geo['hole_diam_in'] * 25.4:.1f} mm)</td><td>{pill('MEASURED-FROM-SCAN')}</td></tr>"
             f"<tr><td>trace quality</td><td>pixel = 0.0847 mm; the two halves of the hand-drawn outline differ by {geo['asym_rms_in']:.3f} in rms; pen gaps at the upper tip bridged by straight strokes (about 5 mm of tip)</td><td>{pill('JUDGEMENT')}</td></tr></table>")
    P.append("<div class='svgwrap'>" + (RES / "zanonia_preview.svg").read_text(encoding="utf-8") + "</div>")
    P.append(f"<h2>AVL: tip curl at the marked CG {pill('MODEL')}</h2><p>Flat plate with the traced outline in 30 sections; the tip curl is a washout (trailing edge up) ramping linearly from 0 at the start of the region to the angle at the tip (JUDGEMENT; dihedral of a real curl ignored). "
             "Cm about the CG; Cm0 = the value at CL = 0; trim CL = Cm0 / SM (the intercept method of the README). Cn_beta and Cl_beta per radian at the trim point, or at CL 0.5 when no trim at CL up to 1.5 exists (marked).</p>")
    P.append("<div class='scroll'><table><tr><th>curl region</th><th>tip incidence</th><th>Xnp, mm</th><th>static margin</th><th>Cm0</th><th>trim CL</th><th>trim alpha</th><th>Cn_beta</th><th>Cl_beta</th><th>evaluated at</th></tr>")
    for r in results:
        okrow = r["trims"] and TRIM_CL_LO <= (r["CL_trim"] or 0) <= TRIM_CL_HI and r["theta"] > 0
        P.append(f"<tr class='{'good' if okrow else ''}'><td>{'none (flat)' if r['theta'] == 0 else str(int(100 * r['region'])) + '% outer'}</td><td class='n'>{r['theta']:g} deg</td><td class='n'>{r['Xnp_m'] * 1000:.2f}</td><td class='n'>{100 * r['SM']:.1f}%</td>"
                 f"<td class='n'>{r['Cm0']:+.4f}</td><td class='n'>{'-' if r['CL_trim'] is None else format(r['CL_trim'], '.2f')}</td><td class='n'>{'-' if r['alpha_trim'] is None else format(r['alpha_trim'], '.1f') + ' deg'}</td>"
                 f"<td class='n'>{r['Cnb']:+.4f}</td><td class='n'>{r['Clb']:+.4f}</td><td>{'trim' if r['lateral_at'] == 'trim' else 'CL 0.5, not trimmed'}</td></tr>")
    P.append("</table></div><p class='note'>Green rows trim at a CL of 0.2 to 0.9. Every row has a static margin of about 1%: the curl changes Cm0 (so the trim CL) but hardly moves the neutral point. With a margin that small, the trim CL is extremely sensitive to Cm0 (trim CL = Cm0 / SM), so the trim values are not precise. AVL's mode / eigen command was not used (it crashes).</p>")
    P.append(f"<h2>Neutral point against AVL section count and panelling {pill('MODEL')}</h2><div class='scroll'><table><tr><th>case</th><th>configuration</th><th>Xnp, mm</th><th>shift vs the 30-section sweep</th><th>static margin</th></tr>")
    for r in sec:
        if r["ok"]:
            P.append(f"<tr><td>{h(r['label'])}</td><td>{h(r['config'])}</td><td class='n'>{r['Xnp_mm']:.3f}</td><td class='n'>{r['dNP_mm']:+.3f} mm ({r['dNP_pct']:+.2f}% MAC)</td><td class='n'>{100 * r['SM']:.2f}%</td></tr>")
        else:
            P.append(f"<tr><td>{h(r['label'])}</td><td>{h(r['config'])}</td><td colspan='3'>AVL refused the deck</td></tr>")
    P.append("</table></div><p class='note'>The sweep used 30 equally spaced sections; the README's pre-run text said 40, and its note records why (the first 40-section attempt used sine clustering that AVL refused). 40 uniform sections do run and agree to 0.002 mm.</p>")
    P.append(f"<h2>CG and curl together: a trim CL of about {TARGET_CL} {pill('CALCULATED')} {pill('MODEL')}</h2><p>Trim CL = Cm0 / SM, and Cm0 (the curl) does not depend on the reference point; so for a wanted trim CL the Cm0 needed is {TARGET_CL} x SM at each CG. "
             "The curl angle is interpolated in the sweep table (CALCULATED; beyond 10 deg the last segment is extended, marked), then checked with AVL at that CG and curl (MODEL).</p>"
             "<div class='scroll'><table><tr><th>CG forward of the marked cross</th><th>static margin</th><th>Cm0 needed</th><th>curl region</th><th>tip incidence needed</th><th>AVL check: SM</th><th>AVL check: trim CL</th><th>Cn_beta</th><th>Cl_beta</th></tr>")
    for r in cgc:
        P.append(f"<tr><td class='n'>{r['shift_mm']:g} mm</td><td class='n'>{100 * r['SM_calc']:.1f}%</td><td class='n'>{r['Cm0_needed']:.4f}</td><td>{int(100 * r['region'])}% outer</td><td class='n'>{r['theta_calc']:.1f} deg{' (extrapolated)' if r['extrapolated'] else ''}</td>"
                 f"<td class='n'>{100 * r['SM_avl']:.1f}%</td><td class='n'>{r['CL_trim_avl']:.2f}</td><td class='n'>{r['Cnb']:+.4f}</td><td class='n'>{r['Clb']:+.4f}</td></tr>")
    P.append("</table></div><p class='note'>" + h(fwd_text()) + " Jigs exist at 2, 4, 6, 8, 10 and 12 deg over the outer 30%: the marked cross needs about 2 deg (30%); 1 mm forward about 9 deg (30%) or 7 deg (40%); "
             "2 mm forward about 16 deg (30%) or 12 deg (40%), more than the largest jig for the 30% region, and well beyond 'just a slight curve'.</p>")
    P.append(f"<h2>Scale-up (AVL results are scale-free: margins, Cm0, trim CL and Cn_beta are the same at any span)</h2><div class='scroll'><table><tr><th>span</th><th>centre chord</th><th>area</th><th>CG behind the leading edge</th><th>hole position</th><th>plate mass at foam 300 g/m&sup2; {pill('ASSUMED')}</th><th>nose weight to the marked CG</th><th>all-up mass</th><th>wing loading</th></tr>")
    for s in scale:
        P.append(f"<tr><td class='n'>{s['span']:g} mm</td><td class='n'>{s['chord_root']:.1f} mm</td><td class='n'>{s['area_cm2']:.1f} cm&sup2;</td><td class='n'>{s['cg_mm']:.1f} mm</td><td class='n'>{s['hole_mm']:.1f} mm</td><td class='n'>{s['plate_g']:.2f} g</td><td class='n'>{s['nose_g']:.2f} g</td><td class='n'>{s['total_g']:.2f} g</td><td class='n'>{s['loading']:.1f} N/m&sup2;</td></tr>")
    P.append("</table></div>")
    # printing
    cl_ = 0.45
    def dd(ws):
        return 3.0 * ws / (1.225 * 9.80665 * cl_)
    def vv(ws):
        return math.sqrt(2.0 * ws / (1.225 * cl_))
    mt = dict(mass_tabs)[155.0]
    fo, pla, lw = mt[3]["loading"], mt[0]["loading"], mt[2]["loading"]
    P.append("<div class='box'><h2 style='border:0'>A PLA print is not the known-good design</h2>"
             f"<p>The printed plate shares the outline only: it is <b>the first step of the morph</b>, not a copy of the glider that is known to fly. Wing loading: <b>{pla:.1f} N/m&sup2;</b> for 0.6 mm solid PLA against <b>{fo:.1f}</b> for the foam original (foam plate 100 g/m&sup2; ASSUMED), "
             f"and {lw:.1f} for LW-PLA at 0.6 g/cm&sup3;. At CL {cl_} (CALCULATED, 3 (W/S) / (rho g CL) for the first swoop and sqrt(2 (W/S) / (rho CL)) for speed): PLA {vv(pla):.1f} m/s and a first-swoop depth of about {dd(pla):.1f} m; "
             f"LW-PLA 0.6 g/cm&sup3; {vv(lw):.1f} m/s and {dd(lw):.1f} m; foam original {vv(fo):.1f} m/s and {dd(fo):.1f} m. So the PLA plate flies about {vv(pla) / vv(fo):.1f} times faster with about {dd(pla) / dd(fo):.0f} times the swoop depth, "
             "and has a different stiffness and pitch inertia. The true baseline is a <b>foam-plate replica at the published loading</b> (cut from the template pages, with clay in the hole); the printed plates come after it.</p></div>")
    P.append("<h2>3D-printable plates and bending jigs (PLA or LW-PLA)</h2>")
    P.append("<p>Files in <code>results/runs/known_good/zanonia/print/</code> (ASCII STL, millimetres). The kit's own STLs are binary STL from <code>flat_test.write_stl</code>; the same plate thickness (0.6 mm, 3 layers of 0.2 mm) and the same mesh helpers (<code>flat_test.prism</code>) are used here, written as ASCII as asked. "
             "<b>The AVL check was a DISAGREE, so the jigs are the generic angles of the README (4, 6, 8 deg) over a generic outer 30% of the semi-span, not an AVL recommendation.</b></p>")
    P.append("<table><tr><th>file</th><th>what</th><th>mesh check</th></tr>")
    for nm, what, chk in prints + jigs:
        P.append(f"<tr><td><code>{h(nm)}</code></td><td>{what}</td><td>{chk}</td></tr>")
    P.append("</table>")
    P.append("<ul><li><b>Plates:</b> flat outline traced from the scan, 0.6 mm thick, through hole at the pattern's hole position (4.0 mm at 155 mm span, 6.0 mm at 250 mm; the pattern's own hole is 2.9 mm scaled, enlarged so a blu-tack plug or a small screw-and-nut weight fits: JUDGEMENT), "
             "and a raised CG cross (0.2 mm, one layer) on the flight <b>underside</b>. Print as modelled: flight upper face on the bed, the cross is the top layer. No supports. A 0.6 mm plate is 1 to 2 perimeters of a 0.4 mm nozzle: print it with 100% infill or all perimeters.</li>"
             "<li><b>Jigs:</b> a block per angle and size. Lay the plate <b>upside down</b> (flight upper face down, as in the page's 'rolled upside down') with its leading edge on the flat top; inside the curl region the top face slopes down behind the leading edge by a twist that ramps from 0 to the printed angle at the tip. "
             "Soften the tips in hot water, lay the plate on the jig, press each tip down onto the slope and hold it until it cools. The angle is printed (raised digits and a degree mark) on the flat margin in front of the plate. One jig serves both tips.</li></ul>")
    P.append("<h3>Mass, nose weight and wing loading: marked CG (default) and 1 and 2 mm forward (test option)</h3><div class='scroll'><table><tr><th>span</th><th>material</th><th>density</th><th>plate mass</th>"
             "<th>marked CG: nose / all-up / loading</th><th>1 mm forward: nose / all-up / loading</th><th>2 mm forward: nose / all-up / loading</th><th>basis</th></tr>")
    for sp, rows in mass_tabs:
        for r in rows:
            cell = lambda k: f"{r['nose_g' + k]:.2f} g / {r['total_g' + k]:.2f} g / {r['loading' + k]:.1f} N/m&sup2;"
            P.append(f"<tr><td class='n'>{sp:g} mm</td><td>{h(r['material'])}</td><td>{r['density']}</td><td class='n'>{r['plate_g']:.2f} g</td><td class='n'>{cell('')}</td><td class='n'>{cell('_f1')}</td><td class='n'>{cell('_f2')}</td><td>{pill('ASSUMED')} {h(r['tag'])}</td></tr>")
    P.append("</table></div><p class='note'>Plate mass for the printed rows is the mesh volume (plate with its hole and CG cross) times the density. The nose weight is the mass at the hole that moves the CG from the plate's centroid to the marked CG: nose = M (x_CG - x_c) / (x_hole - x_CG). "
             "The PLA density 1.24 g/cm&sup3; is the project's value (<code>flat_test.PLA_DENSITY_G_CM3</code>, labelled ASSUMED there; it is the commonly quoted PLA figure, not re-checked against a datasheet). "
             "The nose weight is large because the plate's centroid sits well behind the marked CG: it is 70% of the plate mass at the marked CG, 84% at 1 mm forward and about 100% at 2 mm forward (the hole is fixed, so the lever shortens). The marked-CG column is the default; the forward columns are a test option. The knife-edge balance on the real part is what counts: weigh the plate and set the clay or blu-tack on a balance, not from the table. Printed plates are 4 to 7 times heavier per area than the original foam plate, so they fly faster; they are not dynamically similar.</p>")
    P.append("<h3>Notes</h3><ul><li><b>Plain PLA at 0.6 mm is the safer first print (JUDGEMENT).</b> Its mass is known from the density and the file, the plate is stiff, and hot-water bending is well understood. LW-PLA saves about half the mass but a 0.6 mm plate is only about one and a half 0.4 mm extrusion widths thick: the foaming is inconsistent from print to print, "
             "so the mass varies and the plate is fragile at the tip bend. Weigh every plate and set the nose weight on a balance, not from the table.</li>"
             "<li>Hot-water bending is a softening step. PLA softens at about 55 to 65 degC; the plan was 60 to 70 degC water. Stay in that range and for as short a time as needed: a 0.6 mm plate cooled on the jig keeps the shape, but a plate held hot too long, or hotter, can shrink, warp or lose flatness in the part that is not being bent. "
             "Bend only the tip region; keep the rest of the plate out of the water. LW-PLA softens at a similar temperature but can swell or soften unevenly.</li>"
             "<li>The curl angle is the tip's twist relative to the root. A rolled sheet also lifts its tip: the jig does the twist only, as in the AVL model.</li>"
             "<li>Cardstock template page (secondary): <a href='" + pdf_name + "'>templates.pdf</a>, white Letter at 1:1 with the 100 mm bar, the CG, the hole and the 20/30/40% curl lines (155 mm full outline; 250 mm one half).</li></ul>")
    P.append("<h2>Caveats</h2><ul><li>One CG (from a hand-drawn cross) and one anecdote of flight: the check is weak either way. AVL does not model stall, camber, a rolled tip's dihedral, or the foam plate's thickness and roughness at chord Reynolds numbers of 10,000 to 40,000.</li>"
             "<li>The trace is of a hand drawing (outline asymmetry 0.3 mm rms; about 5 mm of the upper tip bridged by straight strokes).</li></ul></main></body></html>")
    return "\n".join(P)


def stage_build():
    geo = json.loads((RES / "zanonia_geometry.json").read_text(encoding="utf-8"))
    tab = load_outline()
    ar = json.loads((RES / "avl_results.json").read_text(encoding="utf-8"))
    results = ar["results"]
    verdict, lines, rec = evaluate(results)
    sections = json.loads((RES / "avl_sections.json").read_text(encoding="utf-8"))
    update_readme(sections)
    PRINT = OUT / "print"
    PRINT.mkdir(parents=True, exist_ok=True)
    prints, jigs, mass_tabs, checks = [], [], [], {}
    for sp in SIZES_MM:
        sc = Scaled(tab, geo, sp)
        k = sp / 155.0
        hd = 4.0 if sp < 200 else 6.0                               # printed hole, JUDGEMENT (the pattern's own hole is 2.9 mm scaled)
        tris, marks = plate_mesh(sc, PLATE_T_MM, 8.0 * k, 0.9 * k, 0.2, hd)
        allt = tris + marks
        name = f"zanonia_plate_{int(sp)}mm.stl"
        write_stl_ascii(PRINT / name, allt, f"zanonia_plate_{int(sp)}mm")
        back = read_stl_ascii(PRINT / name)
        chk = mesh_ok(back)
        plate_only = mesh_ok(tris)
        checks[name] = dict(chk, plate_volume=plate_only["volume_mm3"], plate_centroid=plate_only["centroid"], area_mm2=sc.area)
        prints.append((name, f"flat plate, span {sp:g} mm, 0.6 mm thick, hole dia {hd:g} mm at {sc.hole_x:.1f} mm, raised CG cross at {sc.cg_x:.1f} mm", f"{chk['triangles']} triangles, {chk['edges_not_shared_by_two']} open edges, volume {chk['volume_mm3']:.0f} mm&sup3;"))
        mass_tabs.append((sp, mass_rows(sc, plate_only["volume_mm3"], plate_only["centroid"][0])))
        for th in JIG_ANGLES_GENERIC:
            jt, info = jig_mesh(sc, th, CURL_REGION_GENERIC, f"{th:g}")
            jn = f"zanonia_jig_{int(sp)}mm_{th:g}deg.stl"
            write_stl_ascii(PRINT / jn, jt, jn[:-4])
            jb = read_stl_ascii(PRINT / jn)
            jc = mesh_ok(jb)
            nd_ = mesh_ok(jig_mesh(sc, th, CURL_REGION_GENERIC, f"{th:g}", digits=False)[0])
            checks[jn] = dict(jc, block_open_edges=nd_['edges_not_shared_by_two'], block_volume=nd_['volume_mm3'], **info)
            jigs.append((jn, f"bending jig, {th:g} deg tip twist over the outer {int(100 * CURL_REGION_GENERIC)}% of the semi-span, {sp:g} mm plate; block height {info['h0']:g} mm; GENERIC angle (not AVL-recommended)", f"{jc['triangles']} triangles; block without digits: {nd_['edges_not_shared_by_two']} open edges (the raised digits are separate solids standing on the block)"))
    scale = scale_rows(tab, geo)
    pdf, _ = build_templates(tab, geo, OUT)
    (OUT / "index.html").write_text(build_page(geo, results, verdict, lines, rec, tab, scale, prints, jigs, mass_tabs, pdf.name), encoding="utf-8")
    (OUT / "numbers.json").write_text(json.dumps(dict(verdict=verdict, lines=lines, scale=scale, mass=[(sp, rows) for sp, rows in mass_tabs], mesh_checks=checks), indent=1, default=str), encoding="utf-8")
    md = ["# Zanonia known-good check: results (generated by rigid_glider/known_good_zanonia.py)", "",
          f"**Verdict on the pre-written rule (README section 3): {verdict}.**", ""] + [f"- {l}" for l in lines] + ["",
          "Per the rule, nothing was adjusted and the pull-out run was not done. Tables, the 3D-print files and the dark-mode page: `results/runs/known_good/zanonia/` (`index.html`, `print/`, `templates.pdf`).", "",
          "| curl region | tip incidence deg | static margin % MAC | Cm0 | trim CL | Cn_beta | Cl_beta |", "|---|---|---|---|---|---|---|"]
    for r in results:
        md.append(f"| {'flat' if r['theta'] == 0 else str(int(100 * r['region'])) + '%'} | {r['theta']:g} | {100 * r['SM']:.1f} | {r['Cm0']:+.4f} | {'-' if r['CL_trim'] is None else format(r['CL_trim'], '.2f')} | {r['Cnb']:+.4f} | {r['Clb']:+.4f} |")
    rz = resolution_text(geo, results, sections)
    cgc = json.loads((RES / "avl_cg_curl.json").read_text(encoding="utf-8"))
    md += ["", "## How big is the DISAGREE: inside drawing resolution", "",
           f"The neutral point is {rz['gap_mm']:.2f} mm behind the marked cross; the rule's floor (3% of MAC) is {rz['floor_mm']:.2f} mm (about 1 mm); the traced outline's two halves differ by {rz['asym_mm']:.2f} mm rms; one scan pixel is {rz['pixel_mm']:.2f} mm; "
           f"the neutral point moves by at most {rz['np_spread_mm']:.2f} mm with the AVL section count. **It is not evidence that the published mark or the model is wrong.** Candidate explanations, none tested here:", ""] + [f"- {c}" for c in CAUSES] + [
           "", "## Neutral point against AVL section count (MODEL)", "", "| case | configuration | Xnp mm | shift vs 30 sections, mm (% MAC) |", "|---|---|---|---|"]
    for r in sections:
        md.append(f"| {r['label']} | {r['config']} | {r['Xnp_mm']:.3f} | {r['dNP_mm']:+.3f} ({r['dNP_pct']:+.2f}%) |" if r["ok"] else f"| {r['label']} | {r['config']} | AVL refused | |")
    md += ["", f"## CG and curl together, trim CL about {TARGET_CL} (CALCULATED from the table; AVL check = MODEL)", "", "| CG forward, mm | SM % | Cm0 needed | region | tip incidence deg | AVL check SM % | AVL check trim CL | Cn_beta | Cl_beta |", "|---|---|---|---|---|---|---|---|---|"]
    for r in cgc:
        md.append(f"| {r['shift_mm']:g} | {100 * r['SM_calc']:.1f} | {r['Cm0_needed']:.4f} | {int(100 * r['region'])}% | {r['theta_calc']:.1f}{'*' if r['extrapolated'] else ''} | {100 * r['SM_avl']:.1f} | {r['CL_trim_avl']:.2f} | {r['Cnb']:+.4f} | {r['Clb']:+.4f} |")
    md += ["", "(*) extrapolated beyond the 10 deg of the sweep. " + fwd_text(), "", "## A PLA print is not the known-good design", "",
           "The printed plate shares the outline only (first step of the morph). Wing loading 12.4 N/m2 (PLA 0.6 mm) or 6.0 (LW-PLA 0.6 g/cm3) against 1.7 for the foam original (100 g/m2 ASSUMED): at CL 0.45 about 2.7 times the speed and about 7 times the first-swoop depth for PLA. The true baseline is a foam-plate replica at the published loading."]
    (RES / "RESULTS.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("verdict", verdict)
    for sp, rows in mass_tabs:
        for r in rows:
            print(sp, r["material"], r["density"], f"plate {r['plate_g']:.2f} nose {r['nose_g']:.2f} total {r['total_g']:.2f} loading {r['loading']:.1f}")
    for k_, v in checks.items():
        print(k_, {a: (round(b, 2) if isinstance(b, float) else b) for a, b in v.items() if a in ("triangles", "edges_not_shared_by_two", "repeated_directed", "degenerate", "volume_mm3", "plate_volume", "area_mm2")})
    for s in scale:
        print(s)
    return pdf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["digitize", "avl", "sections", "cgcurl", "build"])
    ap.add_argument("--scan", type=Path, default=DEFAULT_SCAN)
    a = ap.parse_args()
    if a.stage == "digitize":
        stage_digitize(a.scan)
    elif a.stage == "avl":
        stage_avl()
    elif a.stage == "sections":
        stage_sections()
    elif a.stage == "cgcurl":
        stage_cg_curl()
    else:
        stage_build()


if __name__ == "__main__":
    main()
