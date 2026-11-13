# Section validation experiment: design draft

**Status: draft for Kai's review, 2026-10-11. Nothing here is decided.** The
test matrix is the accepted one in `REFLEX_RESULTS.md` section H; the
predictions it is compared against are in
`results/runs/rigid_glider/reflex_ext/predict/prediction_table.csv`. This
document adds what the matrix does not fix: how big the signals are, what
the rig must resolve, how to build the models, the order to run things in,
and how to read the result. No facility is assumed; none is recorded in this
project, so the rig requirements are written to be checked against whatever
is available.

**Scale caveat (2026-10-11).** The chords (0.14 and 0.21 m) and Reynolds
numbers (60k-100k) come from the 1.2 m reference wing. The aircraft scale is
now open and likely much smaller (0.2-0.5 m span, chords of roughly
0.05-0.12 m, Re possibly 15k-60k). The method and the signal-size formulas
in section 2 carry over; the matrix, chords and Re values must be re-derived
once the pure-wing model has produced a size class. Moment signals fall
with Re squared, so a Re 30k test has a quarter of the Re 60k signal.

## 1. What the experiment has to decide

Three questions, in order of value:

1. **Does a printed lip produce a nose-up section moment at Cl 0.20–0.30 and
   Re 60k–100k, and how much?** XFOIL and the inviscid model disagree by a
   factor of 2 to 40 here depending on the lip.
2. **What is the clean section's own moment and its slope dCm/dCl?** AVL
   assumes both are zero. XFOIL gives Cm of -0.004 to -0.017 and an
   aerodynamic-centre shift of 5–9% chord. This affects the tail branch as
   much as the reflex branch, so it is worth measuring even if the lip
   result is negative.
3. **What does the lip cost in lift?** XFOIL has the 15% / 8 degree lip
   needing alpha 3.6 degrees for Cl 0.25 against 1.8 clean.

Drag is not required. It is the smallest force, the hardest to measure, and
`REFLEX_RESULTS.md` already concludes that moment authority, not drag, is
what is in doubt.

## 2. Signal sizes

At a fixed Reynolds number the product V·c is fixed, so the pitching moment
per unit span does not depend on chord:

    M / b = Cm · ½ ρ (Re ν)²        L / b = Cl · ½ ρ (Re ν)² / c

with ρ = 1.225 kg/m³ and ν = 1.5e-5 m²/s:

| Re | Moment per unit Cm, per metre of span | V at c = 0.21 m | V at c = 0.14 m | q at 0.21 / 0.14 m |
|---:|---:|---:|---:|---:|
| 60k | 0.50 N·m | 4.3 m/s | 6.4 m/s | 11 / 25 Pa |
| 80k | 0.88 N·m | 5.7 m/s | 8.6 m/s | 20 / 45 Pa |
| 100k | 1.38 N·m | 7.1 m/s | 10.7 m/s | 31 / 70 Pa |

For a model of 0.40 m span (an assumption; scale linearly):

| Quantity | Cell | Value |
|---|---|---:|
| Lift at Cl 0.25 | Re 60k, c 0.21 m | 0.24 N |
| Lift at Cl 0.25 | Re 100k, c 0.14 m | 0.98 N |
| Largest lift in the sweep (Cl about 0.7) | Re 100k, c 0.14 m | 2.8 N |
| Moment for Cm = 0.040 (XFOIL, 15% / 8 lip) | Re 80k | 14 mN·m |
| Moment for Cm = 0.003 (XFOIL, 10% / 3 lip) | Re 80k | 1.1 mN·m |
| Moment for Cm = 0.029 (inviscid, 10% / 3 lip) | Re 80k | 10 mN·m |
| Moment for Cm = -0.012 (XFOIL, clean 0008) | Re 80k | -4.2 mN·m |

Consequences:

- The moments are milli-newton-metres. This is the binding constraint on
  the rig, not the lift.
- The smaller chord costs nothing in moment signal, triples the dynamic
  pressure and raises the lift signal by half. It is the easier model to
  measure, and it has lower blockage.
- Because the two chords are the same shape at the same Re, they are the
  same aerodynamic case. Running both does not test aerodynamics; it tests
  whether print layer height, trailing-edge thickness and bend radius, which
  do not scale with chord, matter. That is a manufacturing question and can
  come second.

## 3. What the rig must resolve

To tell the XFOIL and inviscid columns apart for every lip, including the
small one, at every Re:

| Quantity | Needed resolution | As a load, 0.40 m span, worst cell |
|---|---|---:|
| Cm (each of bent and clean) | ±0.0015 | ±0.3 mN·m at Re 60k |
| Cl | ±0.01 | ±9 mN (about 1 gram-force) at Re 60k, c 0.21 m |
| Alpha, relative | ±0.1 deg | |
| Alpha, absolute | ±0.25 deg | |
| Dynamic pressure | ±2% | ±0.2 Pa at Re 60k, c 0.21 m |

If only the large lips are to be resolved, ±0.005 in Cm (about 1 mN·m at Re
60k) is enough: the 15% / 8 lip's XFOIL and inviscid values differ by 0.04
or more. The 10% / 3 lip is then reported as "not distinguishable from
zero", which is still informative since its inviscid value is 0.029.

Other requirements:

- **Moment reference.** Resolve the moment about quarter chord. An error of
  d in where the balance axis is believed to be adds L·d: at L = 0.24 N,
  1 mm is 0.24 mN·m, which is the whole Cm budget at Re 60k. Measure the
  axis position to 0.5 mm or better, and prefer mounting the model with the
  balance axis at quarter chord so the correction is small.
- **Dynamic pressure at 11–31 Pa.** A typical pitot and manometer is poor
  at 11 Pa. This alone may rule out the 0.21 m chord at Re 60k, and it is
  another reason to run the 0.14 m chord first.
- **Freestream turbulence.** The predictions use Ncrit 5, which corresponds
  to a turbulence intensity of roughly 0.4% in the usual e^n correlation.
  Measure or look up the facility's value. A cleaner tunnel behaves like
  Ncrit 9 or more, and a rougher one will suppress the laminar separation
  that drives the whole XFOIL result. Still air in free flight is cleaner
  than Ncrit 5, so if the facility is noisy the result is optimistic for
  the real glider. Predictions at Ncrit 9 do not exist yet in this project.
- **Two-dimensionality.** Wall-to-wall with gaps under about 1 mm, or end
  plates. Model aspect ratio of 2 or more. Low-Re separation bubbles are
  sensitive to end effects; check with tufts or oil that the flow at
  mid-span is spanwise uniform at one condition before taking data.
- **Blockage.** Keep model frontal area at alpha 8 degrees under about 5%
  of the test-section area, or correct for it.
- **Tares.** Run the support alone at each speed. Take wind-off zeros before
  and after every sweep; at these loads zero drift is comparable to the
  signal.

## 4. Models

- **One forward body per section, interchangeable aft strips.** Build the
  forward 80% of chord once and attach the aft 20% as a strip, the same
  split `BUILD_ARTICLE.md` section 5 provides on the wing. The lip moment is
  a difference between bent and clean; using the same forward body for both
  removes model-to-model print differences from that difference. This is
  the single most important model decision.
- A 20% strip covers every lip in the matrix. The 10% and 15% lips are
  strips that are neutral forward of their bend.
- The clean strip is a strip too, so the joint is present in every case.
- Print the strips and the forward body with the process, orientation and
  layer height intended for the wing. Record surface finish and the
  measured trailing-edge thickness and bend radius.
- Measure each strip's lip angle after printing. A 1 degree error is a
  first-order effect in the model. Compare against the as-measured angle.
- The flat plate is a separate body at the real printed thickness, with
  the leading-edge shape the wing would use. There is no prediction for it;
  it is compared against the measured 0006 and 0008, not against a model.

## 5. Procedure

- Alpha -2 to 8 degrees. Steps of 0.25 degrees where Cl is 0.15–0.35 (about
  1 to 5 degrees depending on the lip), 0.5 degrees elsewhere.
- Sweep up, then down, in one run. The limited XFOIL check found no
  hysteresis; a real laminar bubble may show it.
- Three repeats per case, with the strip removed and refitted between
  repeats so that joint repeatability is inside the scatter.
- Clean and bent strips on the same forward body back to back at the same
  tunnel setting, clean first and last.
- Set Re from measured temperature and pressure, not from a nominal speed.
- At one Re per section, take oil-film or tuft pictures at alpha 0, 2 and 4
  on both surfaces to locate separation and reattachment relative to the
  hinge.

## 6. Order of work

Reduced first pass, 12 polars:

| Pass | Sections | Lips | Re | Chord | Polars |
|---|---|---|---|---|---:|
| 1 | NACA 0008 | clean, 10% / 3, 15% / 8, 20% / 10 | 60k, 80k, 100k | 0.14 m | 12 |
| 2 | NACA 0008 | 15% / 5 | 60k, 80k, 100k | 0.14 m | 3 |
| 3 | NACA 0006 | all five | 60k, 80k, 100k | 0.14 m | 15 |
| 4 | Plate | all five | 60k, 80k, 100k | 0.14 m | 15 |
| 5 | Best section | clean and best lip | 60k, 80k, 100k | 0.21 m | 6 |

Pass 1 answers questions 1 to 3 for the section the build article uses.
Stop and review after it; the later passes depend on what it shows.

## 7. Reading the result

For each section, lip and Re, interpolate measured Cm against measured Cl
and report at Cl 0.20, 0.25 and 0.30:

- lip moment, bent minus clean, with its scatter;
- clean Cm;
- dCm/dCl over Cl 0.15–0.35, clean and bent;
- alpha for that Cl.

Compare each with both the XFOIL and the inviscid column of section H.

Proposed interpretation, for Kai to accept or change:

| Outcome | Reading |
|---|---|
| Lip moment near the inviscid column for all lips | XFOIL's low-Re loss is not physical here. Small lips are viable; revisit the AVL lip sizing. The clean-section moment still has to be checked before trusting AVL trim. |
| Lip moment near XFOIL: small lips near zero, large lips at 0.02–0.05 | Small lips are dead. A large lip may trim, but needs a viscous 3D estimate, costs lift and drag, and moves the aerodynamic centre forward. Reflex stays a later interchangeable-strip test on the flying article. |
| Lip moment below XFOIL, or erratic between repeats | Reflex is not a usable trim mechanism at this scale. Close the branch. |
| Clean dCm/dCl measured outside about ±0.03 | The AVL static margins in `BUILD_ARTICLE.md` are off by more than 3% MAC from the section alone. Revise the first-flight CG before flying. |

A lip moment that is large but varies strongly with Cl (positive dCm/dCl)
is destabilising, and should not be read as a pass on moment alone.

## 8. Open items for Kai

- Which facility, and its speed range, test-section size, turbulence
  intensity and balance resolution. Section 3 is the checklist.
- Whether ±0.0015 in Cm is reachable. If not, accept resolving only the
  large lips.
- Model span.
- Whether to add Ncrit 9 predictions before testing.
- The interpretation table in section 7.
