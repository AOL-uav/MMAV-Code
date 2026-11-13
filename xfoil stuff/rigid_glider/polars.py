"""Read the historical XFOIL table without modifying or regenerating it."""

from __future__ import annotations

from bisect import bisect_left
from math import isfinite, log
from pathlib import Path


class PolarCoverageError(ValueError):
    pass


class SectionPolars:
    def __init__(self, database: Path, section: str, ncrit: int = 5):
        self.section = section
        rows: dict[int, dict[float, tuple[float, float]]] = {}
        with database.open(encoding="utf-8", errors="replace") as stream:
            for line in stream:
                if not line.startswith(section + " "):
                    continue
                fields = line.split()
                if len(fields) < 9 or int(fields[2]) != ncrit:
                    continue
                reynolds, alpha = int(fields[1]), float(fields[4])
                cl, cd = float(fields[5]), float(fields[6])
                if isfinite(cl) and isfinite(cd) and cd > 0:
                    rows.setdefault(reynolds, {})[alpha] = (cl, cd)
        if not rows:
            raise PolarCoverageError(f"no {section} polar rows at Ncrit={ncrit}")
        self.reynolds = sorted(rows)
        self.curves = {}
        for reynolds, values in rows.items():
            alpha = sorted(values)
            self.curves[reynolds] = (alpha, [values[a][0] for a in alpha], [values[a][1] for a in alpha])

    def _at_re(self, reynolds: int, alpha: float) -> tuple[float, float]:
        angles, cls, cds = self.curves[reynolds]
        i = bisect_left(angles, alpha)
        if i < len(angles) and abs(angles[i] - alpha) < 1e-8:
            return cls[i], cds[i]
        if i == 0 or i == len(angles):
            raise PolarCoverageError(f"alpha {alpha:.2f} outside {self.section} Re={reynolds} polar")
        lo, hi = i - 1, i
        if angles[hi] - angles[lo] > 1.0:
            raise PolarCoverageError(f"XFOIL gap >1 deg near alpha {alpha:.2f}, Re={reynolds}")
        weight = (alpha - angles[lo]) / (angles[hi] - angles[lo])
        return (cls[lo] + weight * (cls[hi] - cls[lo]), cds[lo] + weight * (cds[hi] - cds[lo]))

    def at(self, reynolds: float, alpha: float) -> tuple[float, float]:
        if not self.reynolds[0] <= reynolds <= self.reynolds[-1]:
            raise PolarCoverageError(f"Re {reynolds:.0f} outside stored {self.section} range")
        i = bisect_left(self.reynolds, reynolds)
        if i < len(self.reynolds) and abs(self.reynolds[i] - reynolds) < 1e-8:
            return self._at_re(self.reynolds[i], alpha)
        lo, hi = self.reynolds[i - 1], self.reynolds[i]
        cl0, cd0 = self._at_re(lo, alpha)
        cl1, cd1 = self._at_re(hi, alpha)
        weight = (log(reynolds) - log(lo)) / (log(hi) - log(lo))
        return cl0 + weight * (cl1 - cl0), cd0 + weight * (cd1 - cd0)

    def drag_at_local_cl(self, reynolds: float, local_cl: float, alpha_hint: float) -> tuple[float, float]:
        """Return (2D alpha, XFOIL CD) on a nearby attached-flow branch.

        Multiple branches can have the same Cl near separation. We restrict the
        search to -8..10 deg and choose the crossing nearest the AVL effective
        angle; absent coverage is an error, never silently extrapolated.
        """
        if not self.reynolds[0] <= reynolds <= self.reynolds[-1]:
            raise PolarCoverageError(f"Re {reynolds:.0f} outside stored {self.section} range "
                                     f"{self.reynolds[0]}..{self.reynolds[-1]}")
        sampled = []
        for step in range(-32, 41):
            alpha = step / 4
            try:
                cl, cd = self.at(reynolds, alpha)
            except PolarCoverageError:
                continue
            sampled.append((alpha, cl, cd))
        crossings = []
        for a, b in zip(sampled, sampled[1:]):
            if b[0] - a[0] > 0.5 or (local_cl - a[1]) * (local_cl - b[1]) > 0:
                continue
            if abs(b[1] - a[1]) < 1e-8:
                continue
            fraction = (local_cl - a[1]) / (b[1] - a[1])
            crossings.append((a[0] + fraction * (b[0] - a[0]), a[2] + fraction * (b[2] - a[2])))
        if not crossings:
            raise PolarCoverageError(f"Cl={local_cl:.3f} not bracketed on attached-flow {self.section} polar at Re={reynolds:.0f}")
        return min(crossings, key=lambda candidate: abs(candidate[0] - alpha_hint))
