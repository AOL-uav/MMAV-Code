# Foam Zanonia Glider as a known-good test of our AVL chain

Written 2026-10-19, **before any AVL run on this outline**. The pass rule in section 3 is fixed here and is not changed by anything that follows.

## 1. The glider (CITED, nothing here was measured on a built one)
- Source: endlessLift, "Foam Zanonia Glider", Gary Hinze 2011, after Paul McIlrath's 1983 Model Aviation design: https://www.endlesslift.com/foam-zanonia-glider/ [CITED: page text as given to me, not re-read].
- Tailless swept crescent cut from a foam food plate, no airfoil [CITED].
- Tips "curved upward by rolling them, upside down, over a 3/16" dowel ... just a slight curve" [CITED].
- Clay nose weight in a hole on the centre line just behind the leading edge; the CG is marked on the pattern [CITED].
- The page gives no mass and no CG number, and says it flies "pretty straight" with "a little wobble" [CITED]. So the only CG available is the cross on the pattern scan.
- Pattern scan (untrusted data, read as an image only): `pattern.jpg`, 2550 x 3507 px, the 6" pattern drawn rotated 90 degrees (span vertical, leading edge on the left).

## 2. Geometry traced from the scan (MEASURED-FROM-SCAN, 300 dpi = 0.0847 mm per pixel; the dpi is the JPEG header value)
The outline was traced by stdlib code (`rigid_glider/known_good_zanonia.py digitize`): the JPEG was converted to a grey PGM with PowerShell System.Drawing, the pen line was thresholded (the scan is almost black and white), the enclosed region found by flood fill, grown back to the pen centre line, and the two halves mirrored about the best axis and averaged. The pen line has gaps at the upper tip (about 40 px); they were bridged with straight strokes (JUDGEMENT; shown pink on the preview). Files: `zanonia_outline.csv`, `zanonia_marks.csv`, `zanonia_geometry.json`, `zanonia_preview.svg` (dark, overlaid on the scan).

| quantity | value | tag |
|---|---|---|
| span | 6.107 in (155.1 mm) | MEASURED-FROM-SCAN (the pattern is called "6 inch"; 1.8% over) |
| centre chord | 1.460 in (37.1 mm) | MEASURED-FROM-SCAN |
| planform area | 7.605 in2 (49.06 cm2) | CALCULATED from the trace |
| MAC | 1.285 in (32.6 mm), leading edge 0.400 in behind the centre leading edge | CALCULATED |
| aspect ratio b2/S | 4.90 | CALCULATED |
| leading-edge sweep | tangent 11.9 deg at 25% semi-span, 21.8 at 50%, 34.2 at 75%; 19.6 deg from the centre to 80% semi-span | MEASURED-FROM-SCAN (the eyeball "30 deg" matches only the outer leading edge) |
| quarter-chord sweep, centre to 80% | 18.0 deg | CALCULATED |
| CG cross | 0.678 in behind the centre leading edge (on the centre line, offset 0.005 in) = **21.6% of MAC** behind the MAC leading edge | MEASURED-FROM-SCAN |
| ballast hole | centre 0.156 in behind the centre leading edge, diameter 0.113 in (2.9 mm) | MEASURED-FROM-SCAN |
| asymmetry of the hand-drawn outline | 0.012 in rms between the two halves | CALCULATED |

## 3. Pass rule (fixed before the AVL run)
Our model **agrees** with the known-good design if both hold:
1. **Static margin.** At the pattern's CG, the AVL static margin SM = (Xnp - x_CG) / MAC is positive and between **3% and 25% of MAC**, for the flat plate (no curl) and for every curl configuration that is reported as trimming.
2. **Trim.** At least one tip curl in the sweep below, with tip incidence within **0 to 10 deg** (0 excluded as a "curl"), trims the plate about the pattern's CG (Cm = 0) at a trim lift coefficient between **0.2 and 0.9**.

If the static margin at the marked CG is negative, or no curl in range trims it, the verdict is **DISAGREE**: record it and stop analysing. No adjustment of the CG, the outline, the curl model, the panelling or the angle range to rescue it. On DISAGREE the AVL sweep table is still reported as it is; the pull-out run is skipped; the plates and templates of the flat outline (pure geometry) are still made, and the bending jigs are made for generic angles (4, 6, 8 deg) labelled as not AVL-recommended.

Definitions fixed now:
- Model: a flat plate with the traced outline in 40 spanwise sections, no thickness, no airfoil, no dihedral (AVL, `tools/avl/avl352.exe`, 10 chordwise x 80 spanwise panels, the same deck layout as `bird_outline_v2.avl_deck2`). Reference point = the marked CG, so Cm is the moment about the CG.
- Tip curl = twist: the outer 20%, 30% or 40% of the semi-span, incidence ramping linearly from 0 at the start of the region to **-theta at the tip** (washout, trailing edge up), theta = 0, 2, 4, 6, 8, 10 deg. A real roll over a dowel would also lift the tip (dihedral); that is ignored (JUDGEMENT).
- AVL runs at alpha 0, 4, 8 deg. Cm about the CG is fitted as Cm = Cm0 + k CL (Cm0 = the value at CL = 0). Trim CL = -Cm0 / k if k < 0 and Cm0 > 0. Xnp = the mean over the runs of that configuration.
- The AVL mode / eigen command is not used (it segfaults in this build).
- Recommended curl (for the bending jigs) = among the configurations that pass rule 2, the one whose trim CL is closest to 0.5 (JUDGEMENT: a typical glide lift coefficient for a flat plate); jigs at that angle and 2.5 deg either side.
- Static margin is evaluated with MAC = the traced planform's MAC, which is also AVL's reference chord.

## 4. Not claimed
A pass means AVL's neutral point and trim are consistent with a design known to fly. It does not validate AVL's lift, drag or stall for a foam plate at this Reynolds number (about 10,000 to 40,000), and it says nothing about the wobble the author reports. A fail would be a disagreement with one anecdote-level source (a page with no mass or CG number), not a refutation.

## 5. Changes after the AVL run (added 2026-10-19; sections 1 to 4 above are as written before the run)
- **Section count.** Section 3 said 40 spanwise sections. The sweep was actually run with **30 equally spaced** sections. Reason: the first attempt used 40 sections with sine clustering toward the tip; AVL refused the deck ("Cannot adjust spanwise spacing at section 34 ... Insufficient number of spanwise vortices") and wrote no output, so no result existed when I changed the count. An earlier version of this README was edited to say 30 at that point; it is restored above to the original wording and the change is recorded here.
- **Effect of the section count on the neutral point** (flat plate and the 30% / 4 deg curl, same CG; shift relative to the 30-section sweep; MAC = 32.6 mm):

| case | configuration | Xnp, mm (155 mm span) | shift vs the 30-section run, mm | % MAC | static margin % MAC |
|---|---|---|---|---|---|
| 15 sections | flat | 17.577 | +0.033 | +0.10 | 1.06 |
| 15 sections | 30% curl, 4 deg | 17.585 | +0.033 | +0.10 | 1.09 |
| 20 sections | flat | 17.556 | +0.011 | +0.03 | 1.00 |
| 20 sections | 30% curl, 4 deg | 17.564 | +0.011 | +0.03 | 1.02 |
| 30 sections (the sweep) | flat | 17.544 | +0.000 | +0.00 | 0.96 |
| 30 sections (the sweep) | 30% curl, 4 deg | 17.552 | +0.000 | +0.00 | 0.99 |
| 40 sections, uniform spacing | flat | 17.542 | -0.002 | -0.01 | 0.96 |
| 40 sections, uniform spacing | 30% curl, 4 deg | 17.550 | -0.002 | -0.01 | 0.98 |
| 50 sections, uniform spacing | flat | 17.544 | -0.000 | -0.00 | 0.96 |
| 50 sections, uniform spacing | 30% curl, 4 deg | 17.552 | +0.000 | +0.00 | 0.99 |
| 30 sections, 16 x 128 panels | flat | 17.528 | -0.016 | -0.05 | 0.91 |
| 30 sections, 16 x 128 panels | 30% curl, 4 deg | 17.536 | -0.016 | -0.05 | 0.94 |
| 40 sections, 16 x 160 panels | flat | AVL refused the deck | | | |
| 40 sections, 16 x 160 panels | 30% curl, 4 deg | AVL refused the deck | | | |

40 equally spaced sections do run (the failure was the sine spacing, not the count), and give the same neutral point to 0.002 mm. The neutral point moves by at most 0.03 mm (0.1% of MAC) with the section count and 0.02 mm with finer panelling, so the verdict does not depend on this choice.
- **Later additions** (not part of the pre-run rule): the section-count check above; a test of the CG moved 1 and 2 mm forward together with a larger curl (`avl_cg_curl.csv`); jigs at 2, 10 and 12 deg; nose weights at the forward CGs. None of them changes the verdict.
