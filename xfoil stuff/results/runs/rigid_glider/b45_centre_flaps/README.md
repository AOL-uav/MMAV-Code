# B45 centre trailing-edge flaps: does AVL agree with Kai's flight observation?

## Question and method (written before the run)

**Observation (MEASURED-by-Kai, anecdotal, no video, not a measurement):** the B45 foamboard flat wing glided "quite well" after Kai added two
small trailing-edge flaps, one each side near the centre, about 10 mm chordwise x 50 mm spanwise, bent UP about 10 deg (trailing edge up =
reflex), "probably only 3 cm from the centreline", balanced at "the guessed point". Before the flaps the wing presumably did not glide well
(UNKNOWN, not observed by us).

**Question:** is that consistent with our AVL model of the B45 wing?

**Model (MODEL, AVL 3.52, inviscid vortex lattice, flat camber line, thickness ignored: a 5 mm foamboard plate is treated as a zero-thickness flat plate).**
- Planform from `rigid_glider/flat_test.py wing_geometry(400, 4.5)` (GEOMETRY): span 400, AR 4.5, taper 0.5, quarter-chord sweep 25 deg,
  zero dihedral, root chord 118.5, tip chord 59.3, MAC 92.18 (MAC LE 48.03 mm aft of root LE). Tip fins as in the existing
  B45 deck (24 mm tall, tip chord, centred on the plate plane; GEOMETRY). Panelling 20 chordwise x ~32 spanwise per half wing, as the existing deck.
- Flaps = AVL CONTROL surfaces on the wing, symmetric (both up). Hinge line at local chord minus 10 mm. Nominal location: inboard edge y = 30 mm,
  outboard edge y = 80 mm (ASSUMED reading of "3 cm from centreline"). Alternate: centred at y = 30 mm, i.e. y = 5 to 55 mm (ASSUMED). Sections are added at the flap edges.
  Sign: deflection defined positive = trailing edge UP; checked by confirming Cm about the CG increases (nose up) with up deflection.
- Nominal CG (ASSUMED): the B45 template/flat_test mark a CG TEST RANGE band (JUDGEMENT) of 57-78 mm aft of root LE, not a single point; the nominal is the
  middle setting of that band (the setting the page uses for its ballast figures), found by reading `flat_test.py`/`geometry.json`. Reference point for
  moments = this CG. The CG is also swept over the band and 5 mm beyond it.
- Runs: alpha 0, 4, 8 deg; flap 0, 5, 10, 15 deg up; nominal and alternate location. Reported per case: Xnp (mm aft of root LE), static margin (% MAC),
  Cm0 (Cm at alpha = 0 about the CG), dCm0/d(flap deg), trim CL (Cm = 0, linear in alpha through the three alphas).
  CG sweep at 10 deg flap: static margin, trim CL, trim airspeed at the B45 assumed mass (all-up with ballast at the middle setting, ASSUMED, from flat_test). Cn_beta, Cl_beta at nominal.
- Sanity: flap-0 Xnp compared with the existing B45 avl_work/geometry.json neutral point (with fins).

**Pre-written rule (not to be changed after the run).** AVL AGREES with the observation if, at the nominal CG and nominal flap location:
1. the flat wing with no flap has Cm0 near zero (|Cm0| < 0.005) and so no positive trim CL at positive static margin; AND
2. the 10 deg up flaps give a positive static margin AND a trim CL between 0.2 and 0.9.
Otherwise AVL DISAGREES (it does not predict a glide with the flaps, or predicts that the unflapped wing would already trim). The alternate location is reported
as a sensitivity; it does not change the verdict unless the two locations land on different sides, in which case the verdict is called "location-dependent".

## Caveats (stated before the run)
- AVL is inviscid. A real flat plate's aerodynamic centre may be 0-5 % MAC ahead of AVL's (earlier plate_balance work), which moves the true static margin.
- A 5 mm foamboard flap bent 10 deg is a crude reflex (thick edge, paper crease, unknown actual angle and extent); AVL treats it as a thin hinged flap with ideal effectiveness.
- The flight is one anecdote with no video; "quite well" has no number, and the unflapped behaviour is unknown.
- Every number below is tagged MODEL / ASSUMED / GEOMETRY / MEASURED-by-Kai.


## Results (MODEL unless tagged; run by rigid_glider/b45_centre_flaps.py)

Nominal CG 68 mm aft of root LE (ASSUMED, middle of the 57-78 mm band, JUDGEMENT); MAC 92.18 mm (GEOMETRY); mass 17.50 g (ASSUMED).

### Flap sweep at nominal CG (MODEL)

| location | flap up deg | Xnp mm | SM % MAC | Cm0 | dCm0/dflap per deg | Cm_alpha /deg | trim alpha deg | trim CL | trim V m/s |
|---|---|---|---|---|---|---|---|---|---|
| nominal | 0 | 70.24 | 2.43 | -0.0000 | n/a | -0.00159 | -0.01 | -0.000 | n/a |
| nominal | 5 | 70.27 | 2.46 | 0.0103 | 0.00206 | -0.00161 | 6.39 | 0.380 | 4.55 |
| nominal | 10 | 70.30 | 2.50 | 0.0206 | 0.00206 | -0.00164 | 12.59 | 0.748 | 3.24 |
| nominal | 15 | 70.33 | 2.53 | 0.0309 | 0.00206 | -0.00166 | 18.61 | 1.105 | 2.67 |
| alternate | 0 | 70.26 | 2.45 | -0.0000 | n/a | -0.00160 | -0.01 | -0.000 | n/a |
| alternate | 5 | 70.29 | 2.48 | 0.0087 | 0.00174 | -0.00162 | 5.35 | 0.311 | 5.04 |
| alternate | 10 | 70.31 | 2.51 | 0.0174 | 0.00174 | -0.00165 | 10.58 | 0.613 | 3.59 |
| alternate | 15 | 70.34 | 2.54 | 0.0261 | 0.00174 | -0.00167 | 15.68 | 0.908 | 2.95 |

Cn_beta 0.0095 /rad, Cl_beta -0.0261 /rad at nominal (flap 0, mean over alpha 0/4/8; MODEL).

### CG sweep at 10 deg up (MODEL)

| location | CG mm | SM % MAC | trim CL | trim V m/s |
|---|---|---|---|---|
| nominal | 52 | 19.76 | 0.095 | 9.13 |
| nominal | 53 | 18.68 | 0.100 | 8.88 |
| nominal | 54 | 17.60 | 0.106 | 8.62 |
| nominal | 55 | 16.53 | 0.113 | 8.35 |
| nominal | 56 | 15.45 | 0.121 | 8.07 |
| nominal | 57 | 14.37 | 0.130 | 7.79 |
| nominal | 58 | 13.29 | 0.141 | 7.49 |
| nominal | 59 | 12.21 | 0.153 | 7.18 |
| nominal | 60 | 11.13 | 0.168 | 6.85 |
| nominal | 61 | 10.05 | 0.186 | 6.51 |
| nominal | 62 | 8.97 | 0.208 | 6.15 |
| nominal | 63 | 7.89 | 0.237 | 5.77 |
| nominal | 64 | 6.81 | 0.274 | 5.36 |
| nominal | 65 | 5.73 | 0.326 | 4.92 |
| nominal | 66 | 4.65 | 0.401 | 4.43 |
| nominal | 67 | 3.58 | 0.522 | 3.88 |
| nominal | 68 | 2.50 | 0.748 | 3.24 |
| nominal | 69 | 1.42 | 1.317 | 2.45 |
| nominal | 70 | 0.34 | 5.535 | 1.19 |
| nominal | 71 | -0.74 | -2.519 | n/a |
| nominal | 72 | -1.82 | -1.026 | n/a |
| nominal | 73 | -2.90 | -0.644 | n/a |
| nominal | 74 | -3.98 | -0.469 | n/a |
| nominal | 75 | -5.06 | -0.369 | n/a |
| nominal | 76 | -6.14 | -0.304 | n/a |
| nominal | 77 | -7.22 | -0.259 | n/a |
| nominal | 78 | -8.30 | -0.225 | n/a |
| nominal | 79 | -9.37 | -0.199 | n/a |
| nominal | 80 | -10.45 | -0.179 | n/a |
| nominal | 81 | -11.53 | -0.162 | n/a |
| nominal | 82 | -12.61 | -0.148 | n/a |
| nominal | 83 | -13.69 | -0.136 | n/a |
| alternate | 52 | 19.77 | 0.078 | 10.06 |
| alternate | 53 | 18.70 | 0.082 | 9.79 |
| alternate | 54 | 17.62 | 0.087 | 9.50 |
| alternate | 55 | 16.54 | 0.093 | 9.20 |
| alternate | 56 | 15.46 | 0.100 | 8.90 |
| alternate | 57 | 14.38 | 0.107 | 8.58 |
| alternate | 58 | 13.30 | 0.116 | 8.25 |
| alternate | 59 | 12.22 | 0.126 | 7.91 |
| alternate | 60 | 11.14 | 0.138 | 7.56 |
| alternate | 61 | 10.06 | 0.153 | 7.18 |
| alternate | 62 | 8.98 | 0.171 | 6.78 |
| alternate | 63 | 7.90 | 0.195 | 6.36 |
| alternate | 64 | 6.82 | 0.225 | 5.91 |
| alternate | 65 | 5.75 | 0.268 | 5.43 |
| alternate | 66 | 4.67 | 0.330 | 4.89 |
| alternate | 67 | 3.59 | 0.429 | 4.29 |
| alternate | 68 | 2.51 | 0.613 | 3.59 |
| alternate | 69 | 1.43 | 1.076 | 2.71 |
| alternate | 70 | 0.35 | 4.410 | 1.34 |
| alternate | 71 | -0.73 | -2.102 | n/a |
| alternate | 72 | -1.81 | -0.851 | n/a |
| alternate | 73 | -2.89 | -0.532 | n/a |
| alternate | 74 | -3.97 | -0.388 | n/a |
| alternate | 75 | -5.05 | -0.305 | n/a |
| alternate | 76 | -6.12 | -0.251 | n/a |
| alternate | 77 | -7.20 | -0.213 | n/a |
| alternate | 78 | -8.28 | -0.186 | n/a |
| alternate | 79 | -9.36 | -0.164 | n/a |
| alternate | 80 | -10.44 | -0.147 | n/a |
| alternate | 81 | -11.52 | -0.133 | n/a |
| alternate | 82 | -12.60 | -0.122 | n/a |
| alternate | 83 | -13.68 | -0.112 | n/a |

### Sanity and verdict (appended after the run)

- Sign check (MODEL): 10 deg up flap raises Cm0 from -0.0000 to +0.0206 (nose up) and lowers CL at alpha 0 from 0.000 to -0.077; unittest confirms.
- Xnp sanity: flap-0 Xnp 70.24 mm (this run, mean of alpha 0/4/8) vs 70.24 mm existing B45 avl_work (geometry.json, with fins): agree to 0.01 mm. (MODEL)
- **Rule 1 (no flap):** Cm0 = -0.0000, SM = +2.4 % MAC at CG 68 mm, trim CL = 0: PASS (the unflapped model trims at zero lift, i.e. a dive/no glide, consistent with "did not glide well", which itself is UNKNOWN).
- **Rule 2 (10 deg flaps):** nominal location SM +2.5 %, trim CL 0.75 (alpha 12.6 deg), inside 0.2-0.9: PASS. Alternate: SM +2.5 %, trim CL 0.61: PASS.
- **Verdict: AVL AGREES with the observation under the pre-written rule (weak test: rule 1 holds by symmetry of a flat plate; rule 2 passes only for CG about 62.6-68.7 mm at the nominal location, 63.4-68.6 mm at the alternate).** Consistent with the flight under the model, not confirmed by it. Aft of about 69 mm the model needs trim CL above 0.9; aft of 70.3 mm the wing is statically unstable. (Qualifier added after review, 2026-10-19; the rule above is unchanged.) The 12.6 deg trim alpha at 68 mm is a linear extrapolation beyond the alpha 0/4/8 runs.
- Trim CL for 10 deg flaps is about 0.35-0.5 at CG 66-67 mm (nominal location 0.40-0.52, SM 3.6-4.7 %; alternate 0.33-0.43) (corrected after review; first written as 0.4-0.6); trim speed there about 3.9-4.4 m/s at 17.5 g (ASSUMED). Trim CL is extremely sensitive to CG
  (e.g. nominal: 0.24 at 63 mm, 0.40 at 66, 1.3 at 69 mm) because SM is small; behind Xnp (about 70.3 mm) trim CL is negative/meaningless (statically unstable).
- Caveats on the agreement: trim CL 0.75 at the nominal CG needs alpha 12.6 deg on a thick flat plate where separation is likely (AVL cannot see it); if the real aerodynamic centre is up to 5 % MAC (4.6 mm) ahead of AVL's, the
  true static margin at 68 mm would range from +2.4 % to about -2.6 % MAC (MODEL + ASSUMED), so the observed CG may have been closer to the front of the band than 68 mm. Flap effect is linear in deflection and
  area (dCm0/dflap 0.0021 /deg nominal, 0.0017 /deg alternate), so a flap smaller or less bent than described still gives the same sign but proportionally less trim CL. Anecdote, no video: this is consistency, not confirmation.
