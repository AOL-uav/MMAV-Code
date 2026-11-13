"""Turn the 1:1 Letter SVG print templates into one vector PDF.

Stdlib only. Reads the template SVGs written by flat_test.py (rect, line,
polygon and text elements only) and writes a PDF whose pages are exactly
US Letter (612 x 792 pt), one SVG per page, with 1 mm in the SVG equal to
1 mm on paper. Nothing is rescaled or re-derived: every coordinate comes
straight from the SVG.

Usage:
    py -B rigid_glider/flat_templates_pdf.py OUT.pdf PAGE1.svg [PAGE2.svg ...]
"""
import sys
import xml.etree.ElementTree as ET

PAGE_W_PT = 612.0          # US Letter
PAGE_H_PT = 792.0
MM_TO_PT = 72.0 / 25.4
PAGE_W_MM = 215.9
PAGE_H_MM = 279.4
AVG_CHAR_EM = 0.5          # rough Helvetica width, only used for end-anchored labels


def _rgb(colour):
    named = {"white": "#ffffff", "black": "#000000"}
    c = named.get(colour, colour).lstrip("#")
    if len(c) == 3:
        c = "".join(ch * 2 for ch in c)
    return [int(c[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]


def _blend(colour, opacity):
    """Opacity over white paper, so no PDF transparency is needed."""
    return [1.0 - opacity * (1.0 - v) for v in _rgb(colour)]


def _num(el, key, default=0.0, total=None):
    v = el.get(key)
    if v is None:
        return default
    if v.endswith("%"):
        return float(v[:-1]) / 100.0 * total
    return float(v)


def _paint(el, path):
    """Return PDF operators that fill and/or stroke the given path."""
    out = []
    fill = el.get("fill", "#000")
    stroke = el.get("stroke")
    if fill != "none":
        out.append("%.4f %.4f %.4f rg" % tuple(_blend(fill, float(el.get("fill-opacity", 1)))))
    if stroke:
        out.append("%.4f %.4f %.4f RG" % tuple(_rgb(stroke)))
        out.append("%.3f w" % float(el.get("stroke-width", 1)))
        dash = el.get("stroke-dasharray")
        out.append("[%s] 0 d" % (dash.replace(",", " ") if dash else ""))
    op = {(True, True): "B", (True, False): "f", (False, True): "S"}.get(
        (fill != "none", bool(stroke)))
    if op:
        out.append(path + " " + op)
    return out


def _text_escape(s):
    s = s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    return s.encode("cp1252", "replace").decode("latin-1")


def page_stream(svg_path):
    root = ET.parse(svg_path).getroot()
    vb = [float(v) for v in root.get("viewBox").split()]
    if abs(vb[2] - PAGE_W_MM) > 0.01 or abs(vb[3] - PAGE_H_MM) > 0.01:
        raise ValueError("%s is not a US Letter template in mm" % svg_path)
    # mm with y down -> points with y up
    ops = ["%.6f 0 0 %.6f 0 %.3f cm" % (MM_TO_PT, -MM_TO_PT, PAGE_H_PT)]
    for el in root:
        tag = el.tag.split("}")[-1]
        if tag == "rect":
            x, y = _num(el, "x"), _num(el, "y")
            w = _num(el, "width", total=PAGE_W_MM)
            h = _num(el, "height", total=PAGE_H_MM)
            ops += _paint(el, "%.3f %.3f %.3f %.3f re" % (x, y, w, h))
        elif tag == "line":
            line = dict(el.attrib)
            line["fill"] = "none"
            el2 = ET.Element("line", line)
            ops += _paint(el2, "%.3f %.3f m %.3f %.3f l" % (
                _num(el, "x1"), _num(el, "y1"), _num(el, "x2"), _num(el, "y2")))
        elif tag == "polygon":
            pts = [p.split(",") for p in el.get("points").split()]
            path = " ".join("%.3f %.3f %s" % (float(px), float(py), "m" if i == 0 else "l")
                            for i, (px, py) in enumerate(pts)) + " h"
            ops += _paint(el, path)
        elif tag == "text":
            size = float(el.get("font-size"))
            text = el.text or ""
            x, y = _num(el, "x"), _num(el, "y")
            if el.get("text-anchor") == "end":
                x -= len(text) * size * AVG_CHAR_EM
            font = "/F2" if el.get("font-weight") == "bold" else "/F1"
            ops.append("%.4f %.4f %.4f rg" % tuple(_rgb(el.get("fill", "#000"))))
            ops.append("BT %s %.3f Tf 1 0 0 -1 %.3f %.3f Tm (%s) Tj ET" % (
                font, size, x, y, _text_escape(text)))
        else:
            raise ValueError("unsupported SVG element <%s> in %s" % (tag, svg_path))
    return "\n".join(ops).encode("latin-1")


def write_pdf(out_path, svg_paths):
    streams = [page_stream(p) for p in svg_paths]
    n = len(streams)
    # object numbers: 1 catalog, 2 pages, 3 Helvetica, 4 Helvetica-Bold,
    # then a page object and a content stream per page
    objs = {
        1: b"<< /Type /Catalog /Pages 2 0 R >>",
        2: ("<< /Type /Pages /Count %d /Kids [%s] >>" % (
            n, " ".join("%d 0 R" % (5 + 2 * i) for i in range(n)))).encode(),
        3: b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>",
        4: b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>",
    }
    for i, s in enumerate(streams):
        objs[5 + 2 * i] = (
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.0f %.0f] "
            "/Resources << /Font << /F1 3 0 R /F2 4 0 R >> >> /Contents %d 0 R >>"
            % (PAGE_W_PT, PAGE_H_PT, 6 + 2 * i)).encode()
        objs[6 + 2 * i] = b"<< /Length %d >>\nstream\n" % len(s) + s + b"\nendstream"
    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = {}
    for num in sorted(objs):
        offsets[num] = len(out)
        out += b"%d 0 obj\n" % num + objs[num] + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    for num in sorted(objs):
        out += b"%010d 00000 n \n" % offsets[num]
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, xref)
    with open(out_path, "wb") as f:
        f.write(out)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    write_pdf(sys.argv[1], sys.argv[2:])
    print("wrote", sys.argv[1], "pages:", len(sys.argv) - 2)
