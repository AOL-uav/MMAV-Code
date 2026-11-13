"""Postprocess one fixed-reflex AVL case with local-Cl XFOIL profile drag.

Run after reflex_study.py and reflex_xfoil.py. This is an additive estimate,
not a coupled viscous/VLM trim calculation.
"""

from __future__ import annotations

import json
from bisect import bisect_left
from math import log
from pathlib import Path

from avl import read_strips
from reflex_xfoil import at_cl


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/runs/rigid_glider/reflex"


def section_at_cl(data: dict, reynolds: float, target: float) -> dict:
    values = sorted(data)
    if not values[0] <= reynolds <= values[-1]:
        raise ValueError(f"Re {reynolds:.0f} outside XFOIL range")
    i = bisect_left(values, reynolds)
    if i < len(values) and reynolds == values[i]:
        result = at_cl(data[values[i]], target)
        if result is None:
            raise ValueError(f"Cl {target:.3f} not bracketed at Re {reynolds:.0f}")
        return result
    low, high = values[i-1], values[i]
    a, b = at_cl(data[low], target), at_cl(data[high], target)
    if a is None or b is None:
        raise ValueError(f"Cl {target:.3f} not bracketed at Re {low} or {high}")
    w = (log(reynolds)-log(low))/(log(high)-log(low))
    return {key: a[key]+w*(b[key]-a[key]) for key in ("alpha_deg", "cd", "cm")}


def main() -> None:
    study = json.loads((OUT / "static.json").read_text())
    nu = study["config"]["flight"]["nu_m2_s"]
    polar_rows = json.loads((OUT / "xfoil/summary.json").read_text())
    data = {(r["section"],r["angle_deg"]): {} for r in polar_rows}
    for r in polar_rows:
        data[(r["section"],r["angle_deg"])][r["reynolds"]] = r["rows"]
    evaluated = []
    for row in study["cases"]:
        feature = row["reflex"]
        if feature["chord_fraction"] != 0.10 or feature["angle_deg"] != 3 or row.get("trim_cl", 0) <= 0:
            continue
        wing = row["wing"]
        strips = read_strips(Path(row["trim_strip_file"]))
        weighted = 0.0
        missing = []
        for strip in strips:
            yfrac = abs(strip.y_m)/(wing["span_m"]/2)
            active = (feature["placement"] == "full" or
                      (feature["placement"] == "inner" and yfrac < feature["split_fraction"]) or
                      (feature["placement"] == "outer" and yfrac > feature["split_fraction"]))
            polar = data.get((wing["section"], 3.0 if active else 0.0))
            if polar is None:
                missing.append("missing foil")
                continue
            try:
                value = section_at_cl(polar, row["speed_m_s"]*strip.chord_m/nu, strip.cl)
                weighted += strip.area_m2*value["cd"]
            except ValueError as error:
                missing.append(str(error))
        profile = weighted/(wing["span_m"]**2/wing["aspect_ratio"]) if not missing else None
        evaluated.append({"name": row["name"], "trim_cl": row["trim_cl"],
                          "profile_cd": profile, "cdi_avl": row["cdi_avl"],
                          "ld_additive": (row["trim_cl"]/(profile+row["cdi_avl"])
                                          if profile is not None else None),
                          "strip_count": len(strips), "uncovered_strips": len(missing),
                          "coverage_errors": sorted(set(missing))[:4]})
    (OUT / "drag_estimates.json").write_text(json.dumps(evaluated, indent=2))
    for r in evaluated:
        print(r)


if __name__ == "__main__":
    main()
