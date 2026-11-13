# Swift A foam 400 mm: pitch-oscillation study (model only, nothing here is a flight result)

## Pre-stated agreement check (written 2026-10-21 11:46, BEFORE any grid result was looked at)

The model agrees with Kai's 2026-10-20 flight if all three parts pass:

- (a) CG 73.0 mm from nose tip with a 6 deg tail bend pulls out of the nose-down drop and goes on to a glide: after the first level-off the flight path never returns steeper than -80 deg, the mean flight path over the last 5 s of the 30 s run is shallower than -45 deg, and the glider does not tumble. (A glide with a large oscillation still passes (a); the size of the oscillation is judged in (c).)
- (b) At CG 73.0 mm, bends of 3 deg AND 4 deg both fail (a) (they dive: no pull-out, tumble, or a steady flight path steeper than -45 deg).
- (c) At 6 deg bend, CG 72.0 and 74.0 mm both pass (a) and both keep a persistent oscillation: not classed "smooth", and the flight-path-angle amplitude over the last full cycle available is between 0.5 and 2 times that of the 73.0 mm case.

Each part is reported pass or fail on its own. Failing a part is a finding about the model, not a reason to change the check.

(Everything below this line was filled in after the run. The three statements above were not edited.)

## Result of the agreement check (base model)

- (a) **PASS**: 73/6: class persistent phugoid; pulls out True; mean flight path over the last 5 s -6.64 deg; returns steeper than -80 deg: False
- (b) **FAIL**: 73/3: smooth (glide True); 73/4: smooth (glide True)
- (c) **FAIL**: 72/6: persistent phugoid, last-full-cycle amplitude 0.786 deg; 74/6: porpoise, 2.11 deg; 73/6: 0.917 deg (ratios 0.857 and 2.29; allowed 0.5-2)
- Parts passing per variant: base: a P, b F, c F; effectiveness 75%: a P, b F, c F; effectiveness 50%: a P, b F, c F; Cm offset -0.008: a P, b F, c F; Cm offset +0.003: a P, b F, c P; AC: table c/4 on NP: a P, b F, c F; pitch inertia x0.5: a P, b F, c F; pitch inertia x2: a P, b F, c F

Note on (c): the pre-stated metric is the amplitude of the last full cycle in 30 s, which for a decaying oscillation is a small number (about 1-2 deg) and is very sensitive to the decay rate; the first-cycle amplitudes of 72, 73 and 74 mm are 38 deg, 42 deg, 48 deg. The verdict above uses the pre-stated metric.

## What the model says the oscillation is

- As flown (CG 73.0 mm, 6 deg) [MODEL]: pulls out after losing 6.93 m; class **persistent phugoid**; first-cycle period 3.02 s (mean of the first cycles 3.16 s); first-cycle flight-path amplitude 42.2 deg; amplitude ratio 0.58 per cycle (shrinks); time to stay within 2 deg of the final path 20.3 s; alpha passes the lift peak on 1 cycle(s) (the first swoop only).
- Reading: a lightly damped **phugoid** (linear damping ratio 0.086) excited by the nose-down release, with a stall at the top of the first swoop only; it is not a sustained stall porpoise at the flown setting. 'Persistent phugoid' is the stated-threshold label (decay ratio 0.58 per cycle is worse than the 0.5 that 'smooth' needs); the oscillation still dies out in the model in about 20 s.
- Predicted phugoid period at the as-flown case, for checking against video [MODEL]: linear 3.22 s (sensitivity range 3.11-4.32 s), simulation 3.16 s (range 3.04-4.37 s over the variants), trim speed 5.5 m/s, trim CL 0.292, glide angle -6.61 deg. Textbook T = pi sqrt(2) V / g = 2.49 s (23% shorter than the model because it ignores drag). Short-period period 0.24 s.
- A mix cannot be excluded: a stall in the first swoop is predicted, and the aft-CG / high-bend corner of the map is a true porpoise region.

## Conclusions that survive the sensitivity variants (data-driven)

- As flown (73 mm, 6 deg): it pulls out and glides (part a) in 8 of 8 variants; the class is persistent phugoid in 5, smooth in 2, porpoise in 1. A stall at the top of the first swoop occurs in 5 of 8; stalls on 2 or more cycles (true porpoise) in 1 of 8.
- The phugoid period at the flown setting is 3.11-4.32 s and its damping ratio 0.06-0.14 across the variants: the oscillation is always lightly damped; the as-flown run settles (flight path within a 5 deg band over the last 5 s of 30 s) in 7 of 8 variants.
- Part (b) (3 and 4 deg dive at 73 mm) passes in 0 of 8 variants: the model's dive limit at CG 73 mm is about 1 deg (base), 1.5 deg (effectiveness 75%), 2 deg (effectiveness 50%), 2.5 deg (Cm offset -0.008), 0.5 deg (Cm offset +0.003), whereas Kai reported a nose dive with 'less' bend than 6 deg (how much less was not recorded).
- Only combined changes reproduce the dive at 3 and 4 deg: bend effectiveness 0.5 together with a Cm offset of -0.008 gives dive at 3 deg, dive at 4 deg and smooth at 6 deg (73 mm); that combination is inside the stated ranges of each parameter but is a stacked guess, and at 6 deg it removes the large oscillation (classed smooth), which conflicts with the flight.
- Smooth region: 23 cells in the base model, 14 in all 8 variants, always at forward CG and small bend (base smooth cells: static margin 7.8-18.3 % MAC, trim CL 0.09-0.25). At 6 deg the smooth CGs (base) are 69, 70, 71 mm; the flown 73 mm is outside it in the base model and inside it in 2 of 8 variants.
- Variants that add any dive inside the grid: Cm offset -0.008, pitch inertia x2.
- Porpoise region: 22 cells in the base model; 15 cells are porpoise in at least 6 of 8 variants: CG 74: bend 8, 9, 10; CG 75: bend 6, 7, 8, 9, 10; CG 76: bend 4, 5, 6, 7, 8, 9, 10. It sits at aft CG / large bend (lowest static margin, highest trim lift).

## Map (base model); rows = bend (deg), columns = CG mm behind the nose tip

D dive, P stall porpoise, h persistent phugoid, S smooth (decay ratio per cycle in cases.csv and the HTML map)

```
bend\CG  69  70  71  72  73  74  75  76
     10    h   h   h   h   P   P   P   P
      9    S   h   h   h   P   P   P   P
      8    S   S   h   h   h   P   P   P
      7    S   S   h   h   h   P   P   P
      6    S   S   S   h   h   P   P   P
      5    S   S   S   S   h   h   P   P
      4    S   S   S   S   S   h   P   P
      3    S   S   S   S   S   S   h   P
```

- Smooth cells (base): CG 69: bend 3, 4, 5, 6, 7, 8, 9; CG 70: bend 3, 4, 5, 6, 7, 8; CG 71: bend 3, 4, 5, 6; CG 72: bend 3, 4, 5; CG 73: bend 3, 4; CG 74: bend 3.
- Smooth in all 8 variants: CG 69: bend 4, 5, 6, 7; CG 70: bend 3, 4, 5, 6; CG 71: bend 3, 4, 5; CG 72: bend 3, 4; CG 73: bend 3.
- Porpoise cells (base): CG 73: bend 9, 10; CG 74: bend 6, 7, 8, 9, 10; CG 75: bend 4, 5, 6, 7, 8, 9, 10; CG 76: bend 3, 4, 5, 6, 7, 8, 9, 10.
- Porpoise in all variants: none.
- Dive in the base model: none. Dive in at least one variant: CG 69: bend 3; CG 75: bend 7, 8, 9, 10; CG 76: bend 4, 5, 6, 7, 8, 9, 10.

## Sensitivity (what survives)

| variant | a | b | c | as-flown class | period s | decay/cycle | stall cycles | settle s | linear phugoid T s |
|---|---|---|---|---|---|---|---|---|---|
| base | P | F | F | persistent phugoid | 3.02 | 0.58 | 1 | 20.3 | 3.22 |
| effectiveness 75% | P | F | F | persistent phugoid | 3.5 | 0.51 | 0 | 19.2 | 3.64 |
| effectiveness 50% | P | F | F | smooth | 4.32 | 0.41 | 0 | 17 | 4.32 |
| Cm offset -0.008 | P | F | F | smooth | 3.52 | 0.5 | 0 | 17.8 | 3.67 |
| Cm offset +0.003 | P | F | P | persistent phugoid | 2.92 | 0.6 | 1 | 21.1 | 3.11 |
| AC: table c/4 on NP | P | F | F | persistent phugoid | 2.94 | 0.6 | 1 | 21.5 | 3.16 |
| pitch inertia x0.5 | P | F | F | persistent phugoid | 3.06 | 0.53 | 1 | 17.2 | 3.23 |
| pitch inertia x2 | P | F | F | porpoise | 3.1 | 0.69 | 2 | 28.4 | 3.22 |

Best few smooth cases (shortest settle time, base): 69/3, 71/3, 70/3, 70/4. Class per variant (base, effectiveness 75%, effectiveness 50%, Cm offset -0.008, Cm offset +0.003, AC: table c/4 on NP, pitch inertia x0.5, pitch inertia x2):
- 69/3: S S S D S S S S
- 71/3: S S S S S S S S
- 70/3: S S S S S S S S
- 70/4: S S S S S S S S

Bend scan at CG 73.0 mm below 3 deg (class per variant; columns 0, 0.5, 1, 1.5, 2, 2.5, 3 deg):
- base: D D S S S S S
- effectiveness 75%: D D D S S S S
- effectiveness 50%: D D D D S S S
- Cm offset -0.008: D D D D D S S
- Cm offset +0.003: D S S S S S S
- AC: table c/4 on NP: D D S S S S S
- pitch inertia x0.5: D D S S S S S
- pitch inertia x2: D D S S S S S

What would turn 3 and 4 deg at CG 73.0 mm into a dive (rows: bend effectiveness; columns: Cm offset 0, -0.008, -0.016, -0.024, -0.032):
- bend 3 deg:
  - eff 1: S S D D D
  - eff 0.75: S D D D D
  - eff 0.5: S D D D D
  - eff 0.4: S D D D D
  - eff 0.3: S D D D D
  - eff 0.2: D D D D D
- bend 4 deg:
  - eff 1: S S S D D
  - eff 0.75: S S D D D
  - eff 0.5: S D D D D
  - eff 0.4: S D D D D
  - eff 0.3: S D D D D
  - eff 0.2: S D D D D

## Specimen (as modelled), with source tags

| quantity | value | source |
|---|---|---|
| Outline | Swift A GLIDE outline (bird_outline_v4_swift.GLIDE), foamboard, span 400 mm | MODEL |
| Wing area S | 175.5 cm2 | MODEL |
| Aspect ratio b^2/S | 9.12 | MODEL |
| Mean aerodynamic chord | 47.4 mm (leading edge at 27.5 mm behind the root LE) | MODEL |
| Neutral point Xnp (AVL, 5 deg bend run) | 41.70 mm behind root LE = 77.70 mm behind nose tip = 30.0 % MAC | MODEL |
| Lift slope CLa | 5.010 per rad (0.0874 per deg) | MODEL |
| Pitch damping Cmq (about the CG, per q c/2V) | -3.346 | MODEL |
| Lift change per deg of tail bend | -0.01392 per deg | MODEL |
| Cm0 per deg of tail bend (about the NP) | +0.00501 per deg (v4 outlines.csv: +0.00501) | MODEL |
| CG that trims CL 0.3 at 6 deg bend | 36.95 mm behind root LE = 72.95 mm behind nose tip (flight-day pages: 36.96 mm) | CALCULATED |
| Tail hinge | 93.5 mm behind nose tip | MODEL |
| Mass: sheet | 6.651 g, centroid 52.0 mm behind root LE, own inertia 6990 g mm2 | CALCULATED |
| Mass: rail | 1.911 g, centroid 16.4 mm behind root LE, own inertia 928 g mm2 | CALCULATED |
| Mass: ballast | 1.210 g, centroid -12.5 mm behind root LE, own inertia 0 g mm2 | CALCULATED |
| All-up mass at CG 36.96 mm (73.0 behind nose tip) | 9.773 g (flight-day pages: 9.773 g; Kai's 'about 9.77 g'); NOT WEIGHED | CALCULATED |
| Ballast | 1.210 g at the -12.5 mm hole (rail fixed as built; moving the CG changes this mass) | CALCULATED |
| Pitch inertia about the CG | 13188 g mm2 = 1.319e-05 kg m2 (sheet as uniform plate + rail bar + ballast point) | CALCULATED |
| Wing loading | 5.46 N/m2 | CALCULATED |
| Normal/axial force and moment vs alpha | Ananda 2012 plate table, run 0420ga, wing AR5_taper1, Re 59999.7 (lowest tested), made symmetric | MEASURED |
| Carry-over of the plate table to the Swift outline | tapered/swept outline and foam surface are NOT what was tested; computed Re = V x MAC / nu with MAC 47.4 mm, nu 1.5e-05 m2/s: as flown 73/6 trim V 5.50 m/s gives Re 17,391, fastest point of the first swoop V 10.04 m/s gives Re 31,729; trim Re over the 64 base glide cases 11,740-32,737; the table's lowest Re is 60,000, so the as-flown trim Re is 3.5x below it (peak 1.9x below) | ASSUMED |
| First lift peak (table) | alpha 9.5 deg, CL 0.64; the stall test compares alpha - tau x bend with this angle | MEASURED |
| Table AC; AC alternative | AC moved to the AVL NP (default); alternative: table quarter chord on the NP | ASSUMED |
| Air | rho 1.225 kg/m3, nu 1.5e-5 m2/s, still air, no quad downwash, no unsteady aerodynamics | ASSUMED |
| CG of the flown glider | 73.0 mm behind the nose tip ('very close to the marked point', Kai, not measured) | ASSUMED |
| Tail bend of the flown glider | 6 deg planned; the foam hinge's real bend angle and springback were not measured | ASSUMED |
| Start | released from rest, nose straight down (theta -90 deg), 30 s simulated, RK4 dt 0.002 s | ASSUMED |

## Definitions and thresholds (chosen by the study, stated here)

- Pulls out: flight path shallower than -20 deg after release (and no full rotation). Height lost to first level-off = height at that moment.
- Settled: flight path stays within a 5 deg band over the last 5 s of the 30 s run.
- Cycle: flight-path maximum to the next one (extrema need 0.5 deg); cycle amplitude = half of (maximum minus the following minimum); decay ratio per cycle = (A_n / A_1)^(1/(n-1)) with n up to 6; shrinks below 0.8, grows above 1.1, else persists.
- Smooth: pulls out, settles, cycle-3 amplitude < 25 % of cycle 1, no stall after the first cycle.
- Stall porpoise: alpha minus the bend's alpha shift passes the plate table's first lift peak on 2 or more cycles after pull-out (a cycle = flight-path maximum to the next; the swoop top lies inside it).
- Persistent phugoid: a glide that is neither smooth nor a porpoise (may still decay slowly).
- Dive: fails the glide test of part (a).
- Linear modes: numpy eigenvalues of a central-difference Jacobian of the 4-state model (vx, vz, theta, q) at the equilibrium glide; phugoid = the lower-frequency complex pair, short period = the higher one.

## Assumptions and limits

- Moving the CG is modelled as changing the ballast mass at the fixed -12.5 mm hole (rail fixed as built); the all-up mass therefore changes with CG (about 9.6-10.4 g over the grid). Pitch inertia is recomputed with it.
- Aerodynamics: Ananda plate table (AR 5 taper 1 run, lowest Re) for the whole Swift outline, AC at the AVL neutral point; the carry-over to the tapered swept foam wing and to the flight Reynolds numbers (computed Re = V x MAC / nu with MAC 47.4 mm, nu 1.5e-05 m2/s: as flown 73/6 trim V 5.50 m/s gives Re 17,391, fastest point of the first swoop V 10.04 m/s gives Re 31,729; trim Re over the 64 base glide cases 11,740-32,737; the table's lowest Re is 60,000, so the as-flown trim Re is 3.5x below it (peak 1.9x below)) is ASSUMED. The simulation is quasi-steady (no added mass, no unsteady stall, no quad downwash, no wind).
- The tail bend is applied as AVL's lift and moment change per degree (linear, from 0 and 5 deg runs); the real foam hinge bend and its springback were not measured (the effectiveness 75 % / 50 % variants stand in for that).
- Lateral behaviour (the turn Kai saw) is not modelled.
- No flight data beyond Kai's words; no period or amplitude of the real oscillation has been measured.

## Files

index.html (dark-mode page), cases.csv (base grid), sensitivity_cases.csv, modes.csv, cases_raw.json, avl/. Re-run: `py -B rigid_glider/swiftA_oscillation_study.py`; tests: `py -B rigid_glider/test_swiftA_oscillation_study.py`.
