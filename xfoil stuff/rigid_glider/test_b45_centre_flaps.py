import sys, tempfile, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import b45_centre_flaps as b


class FlapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        b.OUT = Path(cls.tmp.name)                       # keep test AVL files out of the results folder
        cls.s0 = b.summarise(b.avl_points(b.CG_NOM, "nominal", 0.0, "t0"), b.CG_NOM)
        cls.s10 = b.summarise(b.avl_points(b.CG_NOM, "nominal", 10.0, "t10"), b.CG_NOM)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_flap_up_is_nose_up(self):
        self.assertGreater(self.s10["cm0"], self.s0["cm0"] + 0.005)      # Cm increases (nose up) with TE up
        self.assertLess(self.s10["cl0"], self.s0["cl0"])                 # TE up loses lift at alpha 0

    def test_unflapped_flat_wing_has_zero_cm0(self):
        self.assertLess(abs(self.s0["cm0"]), 0.005)

    def test_xnp_matches_existing_deck(self):
        self.assertAlmostEqual(self.s0["xnp"], b.EXISTING_XNP, delta=0.5)

    def test_deck_has_hinge_at_chord_minus_10mm(self):
        d = b.deck(68.0, "nominal")
        self.assertEqual(d.count("CONTROL"), 2)
        y = 80.0
        self.assertIn(f"{1 - 10 / b.chord(y):.9f}", d)


if __name__ == "__main__":
    unittest.main()
