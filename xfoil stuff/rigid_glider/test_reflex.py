"""Fast geometry/regression checks for the isolated fixed-reflex branch."""

import unittest

from geometry import Wing
from reflex_geometry import Reflex, deck
from reflex_xfoil import coordinates


class ReflexGeometryTests(unittest.TestCase):
    def setUp(self):
        self.wing = Wing(1.2, 0.65, 25, 0, 0, "NACA_0008", 0.16,
                         aspect_ratio=7)

    def test_bent_section_trailing_edge_goes_up(self):
        clean = coordinates(.08, .10, 0)
        bent = coordinates(.08, .10, 3)
        self.assertGreater(bent[0][1], clean[0][1])
        self.assertAlmostEqual(bent[160][1], clean[160][1])  # unchanged nose

    def test_partial_span_control_is_on_bounding_sections_only(self):
        inner = deck(self.wing, "inner", Reflex(.10, 3, "inner"))
        outer = deck(self.wing, "outer", Reflex(.10, 3, "outer"))
        self.assertEqual(inner.count("CONTROL\n"), 4)
        self.assertEqual(outer.count("CONTROL\n"), 4)
        self.assertEqual(inner.count("SECTION\n"), 6)
        self.assertIn("reflL", inner)
        self.assertIn("reflR", outer)


if __name__ == "__main__":
    unittest.main()
