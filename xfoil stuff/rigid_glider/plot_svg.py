"""Dependency-free SVG plots for the baseline screening results."""

from __future__ import annotations

from html import escape
from pathlib import Path


COLORS = ("#1769aa", "#c55127", "#28834f", "#793da4", "#ad850d")


def plot(path: Path, title: str, x_label: str, y_label: str,
         series: list[tuple[str, list[tuple[float, float]]]], zero_line: bool = False,
         marker_x: float | None = None) -> None:
    width, height = 760, 470
    left, right, top, bottom = 80, 28, 56, 66
    data = [(x, y) for _, points in series for x, y in points if y is not None]
    if not data:
        return
    xmin, xmax = min(x for x, _ in data), max(x for x, _ in data)
    ymin, ymax = min(y for _, y in data), max(y for _, y in data)
    if zero_line:
        ymin, ymax = min(ymin, 0), max(ymax, 0)
    xpad = (xmax - xmin) * 0.03 or 0.5
    ypad = (ymax - ymin) * 0.08 or 0.05
    xmin, xmax, ymin, ymax = xmin - xpad, xmax + xpad, ymin - ypad, ymax + ypad
    pw, ph = width - left - right, height - top - bottom
    sx = lambda x: left + (x - xmin) / (xmax - xmin) * pw
    sy = lambda y: top + ph - (y - ymin) / (ymax - ymin) * ph
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="{width/2}" y="29" text-anchor="middle" font-family="Arial" font-size="18">{escape(title)}</text>']
    for k in range(6):
        x, y = left + k * pw / 5, top + k * ph / 5
        xv, yv = xmin + k * (xmax - xmin) / 5, ymax - k * (ymax - ymin) / 5
        svg += [f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top+ph}" stroke="#e5e9ed"/>',
                f'<line x1="{left}" y1="{y:.1f}" x2="{left+pw}" y2="{y:.1f}" stroke="#e5e9ed"/>',
                f'<text x="{x:.1f}" y="{top+ph+20}" text-anchor="middle" font-family="Arial" font-size="11">{xv:.3g}</text>',
                f'<text x="{left-9}" y="{y+4:.1f}" text-anchor="end" font-family="Arial" font-size="11">{yv:.3g}</text>']
    svg += [f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+ph}" stroke="#222"/>',
            f'<line x1="{left}" y1="{top+ph}" x2="{left+pw}" y2="{top+ph}" stroke="#222"/>',
            f'<text x="{left+pw/2}" y="{height-12}" text-anchor="middle" font-family="Arial" font-size="13">{escape(x_label)}</text>',
            f'<text x="20" y="{top+ph/2}" text-anchor="middle" transform="rotate(-90 20 {top+ph/2})" font-family="Arial" font-size="13">{escape(y_label)}</text>']
    if zero_line and ymin < 0 < ymax:
        svg.append(f'<line x1="{left}" y1="{sy(0):.1f}" x2="{left+pw}" y2="{sy(0):.1f}" stroke="#555" stroke-dasharray="5 4"/>')
    if marker_x is not None and xmin <= marker_x <= xmax:
        svg.append(f'<line x1="{sx(marker_x):.1f}" y1="{top}" x2="{sx(marker_x):.1f}" y2="{top+ph}" stroke="#777" stroke-dasharray="4 3"/>')
    for index, (label, points) in enumerate(series):
        color = COLORS[index % len(COLORS)]
        valid = [(x, y) for x, y in points if y is not None]
        if len(valid) >= 2:
            coords = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in valid)
            svg.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2.5"/>')
        for x, y in valid:
            svg.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3" fill="{color}"/>')
        svg.append(f'<line x1="{left+index*145}" y1="{top+ph+40}" x2="{left+index*145+22}" y2="{top+ph+40}" stroke="{color}" stroke-width="3"/>')
        svg.append(f'<text x="{left+index*145+28}" y="{top+ph+44}" font-family="Arial" font-size="11">{escape(label)}</text>')
    svg.append('</svg>')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(svg), encoding="utf-8")
