# Flight day 2026-10-20: results (Kai's report, 2026-10-21)

Source: Kai's words in chat. OBSERVED, qualitative; no measured CGs, masses, periods or distances were reported, and no video has been handed over yet.

| glider | result (Kai's words, condensed) |
|---|---|
| B45 foamboard, centre flaps | Best. "Still quite good, semi heavy oscillations but not bad, good distance." |
| Swift A foamboard 400, tail bent | Second best. More sensitive to exact CG; ended up very close to the marked point (73.0 mm from nose tip). "Pretty heavy oscillations." Moving the CG slightly forward or aft did not help. Bending the tail less gave a nose dive. |
| Swift A foamboard 250 | "Kinda unstable" (details not given). |
| Swift A printed 250 | Sucked into the quad's props at release and shredded into about a dozen pieces. Not flown. Second printed glider lost to the props. |
| Zanonia printed | Not reported. |

Common to most: "stable but adopted a turn and keep turning" (a steady turn after the drop). Turn direction per glider not reported.

Kai's next aims: straight, stable, smooth flight; find out what the oscillations are (phugoid, stall porpoise, or a mix); test a larger printed glider (Swift A at 400 mm, printed in two halves glued at the centre, with the leading edge as the main spar like a bird's wing).

Still wanted from these flights: turn direction for each glider, whether the oscillation shrank, stayed or grew over the flight, rough period (seconds per swoop), whether the nose visibly stalled (mushed) at the top of each swoop, and any video.

## Follow-up answers (Kai, 2026-10-21)
- Prop strikes: random, plus a tether that was too short. Not a design problem; fix = longer tether.
- Swoops: "generally were stalls, didn't go away" (OBSERVED, by eye): reads as stall porpoise, persistent.
- Turn: probably the same direction each flight; foamboard imperfections almost certain.
- Video: all from beneath; no side view, so no period measurement.

## Further answers (Kai, 2026-10-21, later)
- Zanonia: not printed, so not flown.
- Swoops: shrank, but still present after about 10 swoops (small, not very noticeable). Kai's reading: possibly phugoid early, stall swoops at the end.
- Turns: probably the same direction each time; Kai attributes them to build asymmetry (foamboard hard to make symmetric, e.g. tail bent slightly more on one side).
- The two large foam gliders (B45, Swift A 400) flew best at the model's estimated balance point (REPORTED; CG not measured).
- Kai's assessment: the models are validated "reasonably enough" for the next design.
- Turn size: about 1-2 full turns per drop from a 25-30 m release (rough, by eye), part of the height lost in the pull-out. Rough implied radius 12-32 m (CALCULATED, using the model glide angle ~6.6 deg and ~7 m pull-out loss).
- Fin trim tab (bent and set before flight, passive): acceptable to Kai.
- Swift A foam 400 as flown had no vertical element at all (no fin of any kind) (KAI).

## Model comparison (2026-10-21, reviewer REVISE applied)
Drop sim `rigid_glider/swiftA_oscillation_study.py` -> `results/runs/rigid_glider/swiftA_oscillation/`: fails 2 of 3 pre-stated checks. At 73/6 it predicts a phugoid (~3.1 s, ratio 0.58/cycle) with one stall, not persistent stalls; it glides at 3-4 deg bend where Kai saw a dive. Trim Re ~17,400 is 3.5x below the 60,000 plate data. Its smooth region (forward CG, small bend) sits where it already failed; not a basis for settings. Next: measured test matrix (mass, balance CG, gauged bend re-checked per flight, side-view video, drop height and flight time).
