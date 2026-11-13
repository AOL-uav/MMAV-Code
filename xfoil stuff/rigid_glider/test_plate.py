"""Unit tests for plate_section (geometry) and the plate_polars job grid (no solver calls). Run from rigid_glider/:
py -B -m unittest test_plate -v
"""
import sys
sys.dont_write_bytecode = True
import math
import unittest

import plate_section as ps
import plate_polars as pp
import reflex_small_polars as sp


def ellipse_residual(x, y, t):
    """0 when (x, |y|) lies on the LE or TE ellipse of a plate of thickness t (None on the flat part)."""
    h = t / 2
    if x < 5 * h:
        return ((5 * h - x) / (5 * h)) ** 2 + (y / h) ** 2 - 1
    if x > 1 - 10 * h:
        return ((x - (1 - 10 * h)) / (10 * h)) ** 2 + (y / h) ** 2 - 1
    return None


class GeometryTests(unittest.TestCase):
    def test_thickness_and_flat_part(self):
        for t in (0.043, 0.030):
            pts = ps.coordinates(t)
            ymax = max(y for _, y in pts)
            ymin = min(y for _, y in pts)
            self.assertAlmostEqual(ymax, t / 2, places=12)
            self.assertAlmostEqual(ymin, -t / 2, places=12)
            h = t / 2
            for x, y in pts:
                if 5 * h <= x <= 1 - 10 * h:
                    self.assertAlmostEqual(abs(y), h, places=12)

    def test_edge_ellipse_ratios(self):
        for t in (0.043, 0.030):
            h = t / 2
            pts = ps.coordinates(t)
            n_le = n_te = 0
            for x, y in pts:
                r = ellipse_residual(x, y, t)
                if r is not None:
                    self.assertAlmostEqual(r, 0.0, places=9)
                    n_le += x < 5 * h
                    n_te += x > 1 - 10 * h
            self.assertGreater(n_le, 100)
            self.assertGreater(n_te, 100)
            # semi-major axes: full half-thickness is reached exactly 5h from the LE and 10h before the TE
            self.assertAlmostEqual(ps.half_thickness_at(5 * h, t), h, places=12)
            self.assertLess(ps.half_thickness_at(5 * h - 1e-4, t), h)
            self.assertAlmostEqual(ps.half_thickness_at(1 - 10 * h, t), h, places=12)
            self.assertLess(ps.half_thickness_at(1 - 10 * h + 1e-4, t), h)
            self.assertEqual(ps.half_thickness_at(0.0, t), 0.0)
            self.assertAlmostEqual(ps.half_thickness_at(1.0, t), 0.0, places=12)
            # ratio of the two edge lengths is 2:1 (10:1 over 5:1)
            self.assertAlmostEqual(ps.TE_RATIO / ps.LE_RATIO, 2.0)

    def test_closed_contour_and_order(self):
        pts = ps.coordinates(0.043)
        self.assertEqual(pts[0], (1.0, 0.0))                       # upper TE first (reflex_xfoil convention)
        self.assertAlmostEqual(pts[-1][0], 1.0, places=12)
        self.assertAlmostEqual(abs(pts[-1][1]), 0.0, places=12)    # last point closes at the TE
        i_le = min(range(len(pts)), key=lambda i: pts[i][0])
        self.assertEqual(pts[i_le], (0.0, 0.0))
        self.assertTrue(all(y >= 0 for _, y in pts[:i_le]))        # upper surface first
        self.assertTrue(all(y <= 0 for _, y in pts[i_le:]))
        xs_upper = [x for x, _ in pts[:i_le + 1]]
        self.assertEqual(xs_upper, sorted(xs_upper, reverse=True))  # monotone TE -> LE

    def test_clean_section_symmetric(self):
        for t in (0.043, 0.030):
            pts = ps.coordinates(t)
            i_le = min(range(len(pts)), key=lambda i: pts[i][0])
            up, lo = pts[:i_le + 1][::-1], pts[i_le:]
            self.assertEqual(len(up), len(lo))
            for (xu, yu), (xl, yl) in zip(up, lo):
                self.assertAlmostEqual(xu, xl, places=12)
                self.assertAlmostEqual(yu, -yl, places=12)

    def test_named_sections(self):
        self.assertEqual(ps.THICKNESS, {"PLATE_43": 0.043, "PLATE_30": 0.030})
        self.assertEqual(ps.section_coordinates("PLATE_43", 0.15, 0.0), ps.coordinates(0.043))

    def test_thickness_too_large_rejected(self):
        with self.assertRaises(ValueError):
            ps.coordinates(0.14)         # 15 h = 1.05 chord: the two ellipses would overlap


class LipTests(unittest.TestCase):
    def test_unrotated_part_unchanged(self):
        t = 0.043
        clean, lip = ps.coordinates(t), ps.coordinates(t, 0.20, 10.0)
        self.assertEqual(len(clean), len(lip))
        for (x0, y0), (x1, y1) in zip(clean, lip):
            if x0 <= 0.80:
                self.assertEqual((x0, y0), (x1, y1))

    def test_hinge_location_and_rise(self):
        t = 0.043
        for cf, ang in ((0.15, 8.0), (0.20, 10.0)):
            xh = 1 - cf
            clean, lip = ps.coordinates(t), ps.coordinates(t, cf, ang)
            # trailing edge moves to (xh + cf cos a, cf sin a): rise cf sin(angle), TE-up positive
            xte, yte = lip[0]
            self.assertAlmostEqual(xte, xh + cf * math.cos(math.radians(ang)), places=12)
            self.assertAlmostEqual(yte, cf * math.sin(math.radians(ang)), places=12)
            self.assertGreater(yte, 0.0)
            # rigid rotation about the hinge on the chord line: distances to (xh, 0) preserved
            for (x0, y0), (x1, y1) in zip(clean, lip):
                if x0 > xh:
                    self.assertAlmostEqual(math.hypot(x0 - xh, y0), math.hypot(x1 - xh, y1), places=12)
            # the lower-surface TE follows the same rotation (closed contour keeps coincident end points)
            self.assertAlmostEqual(lip[-1][0], lip[0][0], places=12)
            self.assertAlmostEqual(lip[-1][1], lip[0][1], places=12)

    def test_lip_rise_values(self):
        # rise of the trailing edge in chords: 15% / 8 deg -> 0.15 sin 8 = 0.0209 ; 20% / 10 deg -> 0.0347
        self.assertAlmostEqual(0.15 * math.sin(math.radians(8)), 0.02088, places=4)
        self.assertAlmostEqual(ps.coordinates(0.043, 0.20, 10.0)[0][1], 0.03473, places=4)

    def test_matches_reflex_convention_on_a_plain_point(self):
        # same rotation formula as reflex_xfoil.coordinates, applied to a point with y = 0 aft of the hinge
        import reflex_xfoil as rx
        pts = rx.coordinates(0.06, 0.20, 10.0)
        self.assertAlmostEqual(pts[0][1], 0.20 * math.sin(math.radians(10)), places=9)     # NACA TE has y = 0 too
        self.assertAlmostEqual(ps.coordinates(0.043, 0.20, 10.0)[0][1], pts[0][1], places=9)


class GridTests(unittest.TestCase):
    def test_grid_is_the_small_scale_grid(self):
        self.assertIs(pp.RES, sp.RES)
        self.assertEqual(pp.RES, [15000, 20000, 30000, 40000, 60000, 80000, 100000])
        self.assertEqual(pp.NCRITS, [5, 9])
        self.assertEqual(pp.PANELS, 160)
        self.assertEqual(len(pp.GRID), 29)
        self.assertEqual((pp.GRID[0], pp.GRID[-1]), (-4.0, 10.0))

    def test_job_count(self):
        jobs = pp.all_jobs()
        # 4 sections (4.3 % clean, 3.0 % clean, 4.3 % 15/8, 4.3 % 20/10) x (1 inviscid + 2 Ncrit x 7 Re)
        self.assertEqual(len(jobs), 4 * (1 + 2 * 7))
        self.assertEqual(len(set(jobs)), len(jobs))
        self.assertTrue(all(j[5] == 160 for j in jobs))

    def test_inviscid_pair_for_every_section(self):
        jobs = pp.all_jobs()
        cases = {(j[0], j[1], j[2]) for j in jobs}
        self.assertEqual(cases, {("PLATE_43", 0.0, 0.0), ("PLATE_30", 0.0, 0.0), ("PLATE_43", 0.15, 8.0), ("PLATE_43", 0.20, 10.0)})
        for c in cases:
            self.assertEqual(sum(1 for j in jobs if (j[0], j[1], j[2]) == c and j[3] is None), 1)
            self.assertEqual(sum(1 for j in jobs if (j[0], j[1], j[2]) == c and j[3] is not None), 14)

    def test_lips_only_on_the_43_plate(self):
        for j in pp.all_jobs():
            if j[2] != 0.0:
                self.assertEqual(j[0], "PLATE_43")

    def test_viscous_script_is_the_existing_runner_script(self):
        s = sp.build_script(40000, 9, pp.PANELS)
        self.assertIn("VISC 40000\nVPAR\nN 9\n\nITER 400\n", s)
        self.assertIn("ASEQ 0 10 0.5", s)
        self.assertIn("ASEQ 0 -4 -0.5", s)
        self.assertNotIn("PPAR", s)

    def test_tables_interface_matches_arch_tables(self):
        for name in ("clean", "bent", "inv_curve"):
            self.assertTrue(callable(getattr(pp.PlateTables, name)))


if __name__ == "__main__":
    unittest.main()
