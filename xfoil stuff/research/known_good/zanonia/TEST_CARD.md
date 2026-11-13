# Zanonia printed replica: build, balance, fly

Written 2026-10-19, BEFORE any build or flight. The "works" rule in section 4 is fixed here; edit it only before the first flight, and say so if you do.

Aim: does the published Zanonia outline, at its published size and its marked CG, glide when printed? If yes, analyse why and carry the lessons into our gliders. If no, record it plainly (AVL already said the marked CG is near-neutral: margin 1% of MAC, RESULTS.md).

## 1. Parts (files already made, `results/runs/known_good/zanonia/print/`)
- Plate: `zanonia_plate_155mm.stl`, 155 mm span, 0.6 mm thick, 4 mm nose hole, raised CG cross on the underside. Print flight-upper face on the bed, no supports, 100% infill or all perimeters.
- Material: **LW-PLA preferred** (lighter; closer to the foam original). Solid PLA is the backup.
- Bending jigs: `zanonia_jig_155mm_{2,4,6,8,10,12}deg.stl`, tip twist (trailing edge up) over the outer 30% of each semi-span. Print 2, 4, 6 at least.

## 2. Expected masses (CALCULATED, densities ASSUMED; weigh and record the real ones)
| material | plate g | nose weight g | total g | wing loading N/m2 | vs foam original |
|---|---|---|---|---|---|
| LW-PLA 0.6 g/cm3 | 1.76 | 1.23 | 2.99 | 6.0 | 3.6x loading, ~1.9x speed |
| LW-PLA 0.8 g/cm3 | 2.34 | 1.64 | 3.99 | 8.0 | 4.8x, ~2.2x speed |
| PLA solid | 3.63 | 2.55 | 6.18 | 12.4 | 7.4x, ~2.7x speed |
| foam plate (the original, ASSUMED 100 g/m2) | 0.49 | 0.34 | 0.83 | 1.7 | 1x |

Higher loading means faster flight and a deeper first swoop; launch from a bit higher than you'd launch the foam one.

## 3. Build and balance
1. Print the plate. Weigh it (0.01 g scale if you have one).
2. Curl: warm the outer tips (hot water or a heat gun on low) and press the plate **upside down** on the **4 deg** jig until it cools. Check both tips match by eye along the span.
3. Balance: blu-tack or clay in the nose hole until the plate balances on a knife edge (ruler edge or pin) **at the raised CG cross** (17.2 mm behind the centre leading edge). Weigh the finished glider.
4. Note the warp: sight along the span. Any twist left vs right is recorded, not "fixed by trimming", until the first flights.

## 4. "It works" rule (fixed now)
Indoors or dead-calm air. Gentle, level, straight hand launch at shoulder height (about 1.5 m); 5 launches per setting.

**Works** = at the marked CG and some single jig curl between 2 and 12 deg, at least **4 of 5** launches give a steady glide: at most one swoop, then no repeated stall-dive cycle, no tumble, no nose-in, and a ground distance of at least **4 times the launch height** (about 6 m from 1.5 m). A steady turn counts; note its direction.

Curl search order: start at 4 deg. Dives / no pull-up: re-bend at 6, then 8, 10, 12. Stalls / porpoises: re-bend at 2. Stop at the first curl that meets the rule, then fly 5 more there to confirm.

**Does not work** = no curl from 2 to 12 deg meets the rule at the marked CG.

Only after the verdict at the marked CG, as side tests (not part of the verdict): CG 1 mm and 2 mm forward (more nose weight) with more curl, as listed in RESULTS.md; and two nose-down drops from about 2 m released at rest (our start condition): does it pull out, and how much height does it use?

## 5. Log (fill in)
| date | material | plate g | nose g | total g | CG mm (measured) | curl deg | launch | glides meeting rule / 5 | distance m | behaviour (swoop, stall, turn L/R, tumble) |
|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | |

Video of each setting (phone, side view) is the best evidence; note the file names here.

## 6. After the flights
Bring back the log and videos. Works: compare the measured CG and curl with AVL (margin, trim CL, the curl table in RESULTS.md) and list what transfers to our gliders. Does not work: record it next to the AVL DISAGREE; neither result is rescued by changing this rule after the fact.
