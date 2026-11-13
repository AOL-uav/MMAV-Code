# Bird planforms in gliding: real data for a flat-sheet glider

Compiled 2026-10-13. Web search, reading and figure measurement only. Nothing here was flown, built or modelled by us.
Companion to `research/bird_control/sources.md`, which covers control, the robots, and swift/jackdaw glide performance. That material is not repeated here.

## How to read this

Every number carries a tag:

- **[full text]** - read in the paper, its supplement, or an open copy of it.
- **[abstract]** - read in the abstract or a search-result excerpt of the abstract only.
- **[digitised]** - measured by me from a published figure. The method is given in section 1.2. Treat as plus or minus 5-10%.
- **[secondary]** - a database, field guide, web page, or one paper describing another.
- **[my calc]** - arithmetic done here from tagged numbers.
- **NOT FOUND** - looked for, not found. No number is offered.

Papers whose body could not be opened: Lentink 2007 (Nature) and its supplement, Videler 2004 (Science), Vazquez 1994, Matloff 2020, Thomas & Taylor 2001, both Sachs papers, Stowers & Lentink 2015, Withers 1981, Tucker & Parrott 1970, Pennycuick 1968. For Muir 2017 the bioRxiv preprint was read, not the journal version.

The single most useful source turned out to be open access: **Lentink & de Kat 2014 (PLoS ONE)**. Its figure 4A is a measured chord distribution of three real, fully extended swift wing pairs, and its figure 2 shows surface scans of real swift wings at three sweep settings with a scale bar. Most of section 1 comes from those two figures.

---

## 1. The real swift wing in gliding

### 1.1 Whole-bird numbers

| Quantity | Value | Tag and source |
|---|---|---|
| Mass | 0.042 kg (wind-tunnel juveniles); 0.038 kg (mean of 230 birds) | [full text] Henningsson 2014 Table 1; Alerstam 2007 supplement |
| Maximum span | 0.39 m (gliding bird), 0.38 m (flapping bird); 0.40 m (mean of 14) | [full text] same |
| Wing area, including the body strip between the wings | 0.016 m2 (gliding bird), 0.014 m2 (flapping bird); 0.0168 m2 (mean of 13) | [full text] same |
| Mean chord (area / span) | 41 mm (gliding bird), 37 mm (flapping bird) | [full text] Henningsson 2014 |
| Aspect ratio | 9.8 and 10.3 (two tunnel birds); 9.5 from the 138-species data set | [full text]; [my calc] for 9.5 |
| Mean chord of the wings alone (no body) | 36 mm, averaged over three sweep settings | [full text] Lentink & de Kat 2014 |
| Body frontal area | 0.0011 m2 | [full text] Henningsson 2014 |
| Body width as a fraction of span | about 0.10 (37 mm on 390 mm), assuming a round body | [my calc] from frontal area |
| Body length | 16-17 cm, so about 0.42 of span | [secondary] Wikipedia citing commonswift.org |
| Wing length, wrist to tip of the folded wing | 175 mm adults (169-181), 169 mm juveniles | [secondary] BTO BirdFacts ringing data |
| Tail span (width) in glide | 0.09 m at 7 m/s, falling to 0.04 m at 11 m/s | [full text] Henningsson 2011 |
| Tail width as a fraction of span | 0.23 spread, 0.10 furled | [my calc] |
| Tail length | longest tail feather 77-91 mm, about 0.19-0.23 of span | [secondary] Featherbase, seen in a search excerpt only |
| Tail fork depth | **NOT FOUND** as a number. A 2023 ringing paper (Jukema et al.) reports it; not opened | - |

### 1.2 Chord along the span, wing fully extended

Source: Lentink & de Kat 2014, figure 4A. The figure plots chord divided by mean chord against position along the span for three freeze-dried swift wing pairs set at "5 degrees sweep" (their name for fully extended), with an ellipse fitted (R2 = 0.96, error 6.7%). The paper states: "Fully extended swift wings are to a good approximation elliptical" [full text].

I read the thick black mean curve off the figure by pixel position and averaged left and right wings. The wing pairs were glued together without a body, so position 0 is the wing root, not the bird's centreline.

| Position along one wing (0 = root, 1 = tip) | Chord / mean chord, measured swift [digitised] | Chord / mean chord, exact ellipse [my calc] |
|---|---|---|
| 0.04 (junction, includes inner feathers) | 1.48 | 1.27 |
| 0.10 | 1.42 | 1.27 |
| 0.20 | 1.29 | 1.25 |
| 0.30 | 1.27 | 1.21 |
| 0.40 | 1.20 | 1.17 |
| 0.50 | 1.13 | 1.10 |
| 0.60 | 0.96 | 1.02 |
| 0.70 | 0.84 | 0.91 |
| 0.80 | 0.69 | 0.76 |
| 0.90 | 0.45 | 0.55 |
| 0.96 | 0.28 | 0.36 |

The authors excluded the middle fifth of the span (the junction) from their ellipse fit.

What this says in plain words:

- **The widest chord is at the root, not at the wrist.** The chord stays within about 10% of its maximum over the inner 40% of each wing, then tapers.
- **The outer wing is not thin.** At 80% of the wing the chord is still 0.69 of the mean chord, about half the root chord. At 90% it is 0.45.
- Compared with a true ellipse the swift is a little fuller inboard and a little narrower over the outer 40%, by 0.06-0.10 of the mean chord. That is the "pointed" character, and it is modest.

### 1.3 Outline at three sweep settings

Source: Lentink & de Kat 2014, figure 2A-C, which shows surface scans of real swift wing pairs at "5, 30 and 50 degrees" sweep with a 50 mm scale bar. I separated each outline from the background by pixel analysis and measured span, area and the leading and trailing edge lines. Cautions: one specimen per panel, possibly not the same specimen; I assumed the one scale bar applies to all three panels; dried wings; no body.

| | 5 deg (extended) | 30 deg | 50 deg | Tag |
|---|---|---|---|---|
| Span of the wing pair | 354 mm | 314 mm | 235 mm | [digitised] |
| Span relative to extended | 1.00 | 0.89 | 0.66 | [digitised] |
| Area relative to extended | 1.00 | 0.94 | 0.84 | [digitised] |
| Mean chord measured along the flight direction | 35 mm | 37 mm | 44 mm | [digitised] |
| Aspect ratio of the wing pair | 10.1 | 8.5 | 5.3 | [my calc] |
| Angle of the straight line from root leading edge to tip | 7-15 deg | 31 deg | 50 deg | [digitised] |
| Leading-edge sweep, inner part (0.1-0.3 of the wing) | about 9 deg **forward** | 11 deg back | 23 deg back | [digitised] |
| Leading-edge sweep, outer part (0.5-0.9) | 6 deg rising to 17 deg | 37 deg | 55 deg | [digitised] |
| Leading-edge sweep, last 25% | 26 deg | 44 deg | 61 deg | [digitised] |
| Trailing-edge sweep, outer half | about 0 (straight across) | 25 deg | 51 deg | [digitised] |

Checks: my measured mean chord (35 mm) agrees with the paper's stated 36 mm, and my root-to-tip line angles reproduce the paper's labels for 30 and 50 degrees, so the paper's "sweep angle" is roughly the angle of the root-to-tip line.

Chord measured along the flight direction, as a fraction of each setting's own mean chord [digitised]:

| Position along the wing | 5 deg | 30 deg | 50 deg |
|---|---|---|---|
| 0.1 | 1.44 | 1.32 | 1.19 |
| 0.3 | 1.27 | 1.17 | 1.07 |
| 0.5 | 1.07 | 1.02 | 0.95 |
| 0.7 | 0.78 | 0.83 | 0.89 |
| 0.8 | 0.63 | 0.70 | 0.89 |
| 0.9 | 0.41 | 0.56 | 0.74 |
| 0.95 | 0.26 | 0.44 | 0.57 |

Plain words:

- Extended, the swift wing is a near-ellipse with a **straight trailing edge over the outer half** and a **leading edge that curves back** more and more toward the tip. The inner leading edge runs slightly forward from the body to about 30-40% of the wing, which is where the leading edge is furthest forward.
- Swept back, the chord along the flight direction becomes **nearly constant** along the wing (1.2 down to 0.9 of the mean over the inner 80%), ending in a blunt-pointed tip. The scythe shape is a swept, almost parallel-sided blade, not a broad root with a thin tip.
- Sweeping to 50 degrees removes a third of the span but only about a sixth of the area.

### 1.4 What posture does a gliding swift actually hold?

- The live swift in the Lund tunnel held a span of 0.90 of maximum at 7 m/s, falling linearly to 0.85 at 11 m/s (fit: span ratio = -0.0139 U + 1.0115 at the flattest glide) [full text, Henningsson 2011].
- Wing area followed span: S = 0.0514 b - 0.0044 (m2, m) [full text, same]. So 85% span keeps about 81% of the area [my calc].
- Mean chord fell slightly with speed: c = -0.0006 U + 0.044 (m) [full text, same].
- A span ratio of 0.85-0.90 matches the "30 degrees" scan (0.89) far better than the "5 degrees" scan [my calc]. So at its best-glide speed the swift is **not** fully extended. The two studies define maximum span differently, so treat this as approximate.
- Lentink 2007: extended wings are best for slow glides and turns, swept wings for fast ones; best choice of sweep "can halve sink speed or triple turning rate" [abstract].

### 1.5 Arm wing and hand wing

No paper found states the swift's wrist position as a fraction of span. What was found:

- "a very long hand section and a short arm section" [full text, Henningsson 2011].
- Birds "with longer hand wings than arm wings (like the swift)" [full text, Harvey 2022, about a black swift]. That paper has an arm-to-hand ratio per species in an extended data table, which was not readable in my copy.
- Wing length (wrist to tip) 175 mm against a half span of 195-200 mm gives a hand wing no longer than 0.88 of the half span [my calc from BTO and Alerstam; an upper limit, because the hand does not point straight sideways].
- In the 30 and 50 degree scans the leading edge changes slope between 0.2 and 0.4 of the wing, 30-35 mm out from the root. I take this as the wrist region [digitised, rough].

Estimate, with the body included: **body 0.10, arm 0.10-0.15, hand 0.75-0.80 of the half span.** The wrist therefore sits at about 0.20-0.25 of the half span from the centreline. This is an estimate from the lines above, not a published number.

- Bone lengths (humerus, ulna, hand): **NOT FOUND.**
- Real 3D scans of swift wings as downloadable geometry: **NOT FOUND.** The Lentink & de Kat scans exist only as figures.

### 1.6 Tail and body

- Tail width in glide: 0.09 m at 7 m/s to 0.04 m at 11 m/s, non-linear [full text, Henningsson 2011].
- Lift coefficient at 7 m/s is 0.96 on wing area alone and 0.81 if the spread tail's area is added, so the spread tail adds roughly 18% to the lifting area at the slowest speed [my calc from full text].
- Tail vortices were sometimes nearly as strong as the wingtip vortices, but the tail's contribution to lift is small because its span is small [full text, same].
- The spread tail spoils the downwash pattern. Span efficiency in gliding averaged 0.41 (best 0.55 at 8.0 m/s); removing the tail's wake from the calculation raised it by 12% on average [full text, Henningsson 2014].
- Body drag coefficient on frontal area: 0.30 at 7 m/s, 0.22 at 11 m/s. Body drag was 23% of total drag at 7 m/s and 46% at 11 m/s [full text, Henningsson 2011].
- Wing root vortices were seen behind a flapping swift, suggesting the two wings act "to some extent aerodynamically detached" with the body carrying less lift than the wings [full text, Henningsson, Muijres & Hedenstrom 2011].
- Profile drag coefficient of the wings: lowest 0.011 at 10 m/s [full text, Henningsson 2011].

### 1.7 Leading-edge vortex

- Videler 2004 used a rigid model swift wing in a water tunnel with the wing swept 60 degrees and saw a leading-edge vortex over the hand wing at angles of attack of 5-10 degrees. The hand wing has a sharp leading edge formed by the narrow front vane of the outer primary; the arm wing has a rounded one [secondary: as described in Muir 2017 and in press coverage].
- Lentink 2007 tested real wings at up to 50 degrees sweep and found the vortex only at higher angles, 11 degrees and above at Re 25,000 and 14 degrees and above at Re 50,000 [secondary: as described in Muir 2017].
- The two disagree on whether a gliding swift uses the vortex at its normal low angles. A flat sheet with a sharp edge is closer to the Videler model than to a real wing.

---

## 2. Other birds a flat-sheet glider could imitate

Morphology and cruise speed for all rows except those marked come from one data set, the supplement to Alerstam et al. 2007 [full text], which averages many birds. Aspect ratio, wing loading, mean chord and Reynolds number are [my calc] from those. Wing area includes the body strip. **The speeds are radar-tracked flapping cruise speeds, not glide speeds.**

| Bird | Mass (kg) | Span (m) | Area (m2) | Aspect ratio | Wing loading (N/m2) | Mean chord (mm) | Cruise speed (m/s) | Re at cruise |
|---|---|---|---|---|---|---|---|---|
| Common swift | 0.038 | 0.40 | 0.0168 | 9.5 | 22 | 42 | 9.7 | 27,000 |
| House martin | 0.015 | 0.29 | 0.0104 | 8.1 | 14 | 36 | 9.7 | 23,000 |
| Barn swallow | 0.016 | 0.32 | 0.0136 | 7.5 | 12 | 43 | 10.0 | 28,000 |
| Arctic tern (area from 1 bird) | 0.110 | 0.80 | 0.0571 | 11.2 | 19 | 71 | 10.9 | 52,000 |
| Common kestrel | 0.203 | 0.73 | 0.0708 | 7.5 | 28 | 97 | 10.1 | 65,000 |
| Hobby | 0.238 | 0.74 | 0.0667 | 8.2 | 35 | 90 | 11.3 | 68,000 |
| Black-headed gull | 0.283 | 0.97 | 0.0976 | 9.6 | 28 | 101 | 11.9 | 80,000 |
| Jackdaw | 0.245 | 0.65 | 0.0684 | 6.2 | 35 | 105 | 12.5 | 88,000 |
| Peregrine | 0.789 | 1.02 | 0.1257 | 8.3 | 62 | 123 | 12.1 | 99,000 |
| Carrion crow | 0.566 | 0.91 | 0.1378 | 6.0 | 40 | 151 | 13.5 | 136,000 |
| Laggar falcon (one tunnel bird) | 0.57 | 1.01 | 0.132 | 7.7 | 42 | 131 | - | - |
| Harris' hawk (one tunnel bird) | 0.70 | 1.02 | 0.190 | 5.5 | 36 | 186 | - | - |
| Storm petrel (species not stated) | 0.033 | 0.43 | 0.0244 | 7.7 | 13 | 56 | - | - |

Laggar falcon and Harris' hawk rows: [full text] Rosen & Hedenstrom 2001 Table 2, quoting Tucker & Parrott 1970 and Tucker & Heine 1990. Storm petrel row: [secondary] a table excerpt from a 2026 Marine Ornithology appendix; unverified. A second house martin measurement, one tunnel bird: 16.9 g, half span 14.8 cm, area 112 cm2, mean chord 3.8 cm, aspect ratio 7.8, wing loading 14.8 N/m2 [full text, Rosen, Spedding & Hedenstrom 2007].

Shape, tail and glide performance:

| Bird | Hand-wing length / half span (upper limit) | Planform character | Tail | Measured glide |
|---|---|---|---|---|
| Common swift | 0.88 | near-elliptical when extended, pointed, curved-back leading edge, no slots | short, forked, spread at low speed | L/D 12.5 at 9.5 m/s, Re 18,400-29,500 [full text] |
| Barn swallow | 0.79 | pointed, no slots | long, deeply forked with streamers | NOT FOUND |
| House martin | 0.76 | pointed, no slots | short, shallow fork | NOT FOUND |
| Arctic tern | 0.68 | long, narrow, pointed | long, deeply forked | NOT FOUND |
| Common kestrel | 0.68 | pointed falcon wing | long, fan | NOT FOUND |
| Black-headed gull | 0.63 | long, pointed, strongly cambered | short, square | NOT FOUND for this species |
| Jackdaw | 0.72 | broad, rounded, slotted tips | medium, fan; fully furled above 9 m/s | L/D 12.6 at 8.5 m/s, Re 38,000-76,000 [full text] |
| Harris' hawk | NOT FOUND | broad, slotted tips | long, fan | L/D 10.9, minimum sink 0.81 m/s [abstract] |
| Laggar falcon | NOT FOUND | pointed | medium | L/D NOT FOUND (paper not opened) |
| Black vulture | NOT FOUND | broad, deeply slotted | short | L/D 11.6 at 13.9 m/s [abstract excerpt, Parrott 1970] |

Notes on this table:

- The hand-wing column is BTO ringing wing length divided by half the Alerstam span [my calc, mixed sources]. It overstates the sideways share of the hand, but the ranking is real: swift, then swallow and martin, then the rest.
- The planform and tail descriptions are general ornithology knowledge, not numbers from a paper [secondary].
- Fulmar, shearwater, pigeon: glide ratios **NOT FOUND** in anything I could open.
- Only the swift, house martin and barn swallow fly at our Reynolds number. Every other bird in the table flies at two to five times our Reynolds number.

---

## 3. Why bird planforms are the way they are in gliding

**Lift distribution.** An elliptical chord distribution gives the least induced drag for an untwisted wing, and the extended swift wing is close to one [full text, Lentink & de Kat 2014]. But the live bird does not achieve an efficient lift distribution: span efficiency in gliding averaged 0.41, against 0.62 in flapping, because the body and the spread tail disturb the downwash [full text, Henningsson 2014]. The authors conclude the wing is tuned more for flapping than for gliding.

**Pointed versus rounded tips.** Dried wings of ten species tested as fixed surfaces differed mainly in drag, not in maximum lift. Crows, with slotted tips, had the lowest minimum drag; the authors link this to reduced induced drag from slotting. Wing shape alone "cannot ... predict the gliding performance" [full text, Lees 2016]. Best L/D of these dried, body-less wings was only 2.8-4.0. For swift-type pointed tips the measured argument is about speed: a pointed, sweepable hand wing lets the bird trade span and area for speed [abstract, Lentink 2007].

**Role of the hand wing.** It is the part that sweeps. In raptors, swept wings gave about 11% more vertical force than extended wings in a glide test, with similar best force ratios; extended wings were clearly better only for slow flapping [abstract, Klaassen van Oorschot 2016]. The hand wing's sharp leading edge can also hold a leading-edge vortex (section 1.7).

**Tail: lifting surface or stabiliser?** In the swift and the jackdaw the tail is spread at low speed and furled at high speed [full text, Henningsson 2011; Rosen & Hedenstrom 2001]. The jackdaw's tail area fell linearly from 6 to 9 m/s and was fully furled above; fully spread it was 2.1 times the furled area [full text]. This is a low-speed lift and control surface. In gliding raptors the tail carries positive lift, the opposite of an aircraft stabiliser (already in `sources.md`, Usherwood 2020).

**Body.** A large drag item (a quarter to nearly half of total drag in the swift) and a poor lifter (section 1.6).

**Pitch stability.**

- Across 22 species, the best-supported centre of gravity position is 10% of body length behind the shoulder joint [full text, Harvey 2022].
- The fitted evolutionary optimum for the most stable wing posture is a static margin of +26% of the maximum root chord, and for the least stable posture -15% [full text, Harvey 2022]. Birds are built to cross between stable and unstable by moving elbow and wrist. That analysis ignores the tail and uses gull-wing aerodynamics for all species.
- Rules of thumb from theory: a passively stable glider should have wings swept forward in slow flight, tips twisted down when swept back (washout) or up when swept forward, and extra lifting surfaces set at a different angle from the wing (tail or canard). A centre of gravity below the wing and drag-based stability matter more for animals than for aircraft. Birds may resemble tailless aircraft [abstract, Thomas & Taylor 2001].

**Yaw stability without a fin.** Small fliers have enough yaw "stiffness" from the wing alone, because yaw inertia falls faster with size than the aerodynamic restoring moment does. Yaw damping demands also fall with size. The spiral mode is only partly affected [abstract, Sachs 2005 and Sachs 2007]. The papers say the wing can supply the yawing moment "if it has suitable features"; the abstracts do not list them. Against this, PigeonBot II needed active feedback to damp Dutch roll with no fin (in `sources.md`). Whether a flat swept plate at 20-40 cm span is yaw-stable without a fin is **NOT FOUND**.

**Twist and camber.** Measured spanwise twist for a gliding swift: **NOT FOUND.** Measured camber of a real swift wing: **NOT FOUND**; the swift-like model wing in Lentink & de Kat 2014 had 4.5% camber at mid-chord and 2.8% thickness [full text, a model, not a measurement of a bird].

**What a flat sheet cannot copy.**

| Bird feature | Can a flat sheet copy it? |
|---|---|
| Outline: chord distribution, sweep, tip shape, tail outline | Yes |
| Sharp leading edge of the hand wing | Yes; a thin sheet is sharp everywhere, including the arm wing, where the bird's edge is rounded |
| Camber | No, unless the sheet is bent or printed curved |
| Twist or washout along the span | No, unless the sheet is deliberately warped |
| Thickness change from thick arm wing to thin hand wing | No |
| Feather roughness, 1-2% of chord on the hand wing [full text, Lentink & de Kat 2014] | No; its effect on a flat plate is unknown |
| Porous or slotted feathers, separated tip primaries | No, though slots can be cut |
| Change of span, sweep and tail spread with speed | Not in a fixed sheet; only by a folding mechanism |
| Tail carrying lift and being trimmed in flight | No; a fixed tail must be set once |
| Active stabilisation | No |

---

## 4. Reynolds numbers

| Flier | Chord Reynolds number | Tag |
|---|---|---|
| Our glider | 15,000-40,000 | project figure |
| Swift, live, gliding at 7-11 m/s | 18,400-29,500 | [full text] Henningsson 2011 |
| Swift, typical gliding range | 12,000-77,000 | [full text] Lentink & de Kat 2014 |
| Swift at its most efficient glide, about 10 m/s | about 24,000 | [full text] same |
| House martin, barn swallow at cruise | 23,000-28,000 | [my calc] |
| Jackdaw gliding at 6-11 m/s | 38,000-76,000 | [full text] Rosen & Hedenstrom 2001 |
| Kestrel, gull, crow at cruise | 65,000-136,000 | [my calc] |

We are at swift, swallow and martin Reynolds numbers, and below crows, gulls and falcons.

Wind-tunnel data at or near our Reynolds number:

- **Real swift wings**, Re 12,000-67,000. The flow stays laminar over much of the wing despite feather roughness of 1-2% of chord: 69% of the wing area laminar at best glide [full text, Lentink & de Kat 2014]. The paper also compares smooth and rough model wings at Re 12,000-48,000 in its figure 4; I did not extract that comparison. Lift coefficient of the real wings reached about 0.8-1.1 [full text, Henningsson 2011 quoting Lentink 2007].
- **Flat-plate swift-shaped wing**, Re 15,800-65,500, water flume, 50 degrees sweep, aspect ratio 6, sharp bevelled edge. A double or triple leading-edge vortex appeared in all conditions tested. No forces were measured, and the authors note their model tapers more strongly than a real swift wing [full text of preprint, Muir 2017].
- **3D-printed swift-like wing**, Re 26,000, curved sweep, aspect ratio 3.4, angles of attack 0, 20 and 30 degrees. Leading-edge vortices similar to a delta wing. No forces [full text, Lambert 2019].
- **Flat plate, 5% cambered plate and Eppler 387 aerofoil**, Re 10,000 and 20,000, aspect ratio 6 and two-dimensional. At Re 10,000 the flat plate had a **higher L/D than the Eppler 387 aerofoil**, through lower drag. At Re 20,000 the Eppler was better than the flat plate and about equal to the cambered plate. The 5% cambered plate made more lift than the Eppler at most angles. The plate was 3.6% thick with a 5:1 elliptical nose [full text, McArthur 2008 thesis]. The L/D values are in figures I could not render: **numbers NOT FOUND.**
- **Eppler 387 wing, aspect ratio 6, Re 10,000-20,000.** The flow separates and does not reattach. Span efficiency is 0.27 by one definition and 0.76 by the other, both far from the textbook 0.8-0.95. One figure quotes a best L/D of about 5.4 for the two-dimensional section [full text, Spedding & McArthur 2010; I could not tie the 5.4 to a stated Reynolds number].
- **Flat-plate wings, aspect ratio 2-5, taper 0.5-1**, Re 60,000-160,000, above our range. Minimum drag coefficient 0.01-0.02. The high-L/D band of angle of attack is narrow, with sharp drops either side. Taper has little effect on lift. The aerodynamic centre moves aft, to near mid-chord, as stall approaches [full text, Ananda, Sukumar & Selig 2015].
- **Dried wings of ten bird species** at 8-16 m/s. Maximum lift coefficient 0.7-1.1, minimum drag coefficient 0.08-0.14, best L/D 2.8-4.0, with no body [full text, Lees 2016].
- Withers 1981 (bird wings as fixed aerofoils) and Okamoto 1996 (flat and curved plates near Re 10,000): cited everywhere, **bodies not opened, no numbers taken.**

Not found: force data (lift, drag, pitching moment) for a flat plate cut to a real bird outline at Re 15,000-40,000. Only flow pictures exist.

---

## 5. How birds fold the wing

Kept to geometry and mechanism. Most of the classic anatomy papers could only be read as abstracts, so the mechanism description below is at abstract level.

**Joints.** Three in series. The shoulder (body to upper arm) swings the whole wing fore-aft and up-down and sets its pitch. The elbow (upper arm to forearm) and the wrist (forearm to hand) fold in the plane of the wing, in opposite senses, so the wing closes like a letter Z. The primaries are carried on the hand, the secondaries on the forearm.

**Elbow and wrist are linked.**

- The wing "can automatically synchronize flexion or extension of the elbow and wrist". The forearm's two bones, radius and ulna, lie side by side; as the elbow bends, the radius is pushed lengthwise along the ulna toward the wrist and folds the hand. This is the "drawing parallels" action: a four-bar parallelogram made by the two forearm bones [abstract, Vazquez 1994].
- All 61 species tested could couple elbow and wrist motion [full text, Baliga 2019].
- In a jackdaw, wing area against span was the same in flight and when the wing was flexed by hand, "indicating that the wing can be flexed only along one geometrical trajectory". The fit is S = 0.0665 b2 + 0.0316 b + 0.0156 (m2, m, including body) [full text, Rosen & Hedenstrom 2001]. **In effect the planform has one degree of freedom.**
- Caution: recent pigeon work reports the bone motion is strongly out of plane and that the wrist bones are needed to fit it, so the flat four-bar is a simplification [abstract, SICB meeting abstract; authors not recorded].

**What holds the wing open.** A ligament runs along the leading edge of the skin flap in front of the elbow (the propatagium), from the shoulder to the wrist. Its middle section is highly stretchable and supports the leading edge across a range of extensions. A muscle acting on it coordinates automatic extension of elbow and wrist [abstract excerpt, a 1994 paper on the great horned owl propatagium; the Brown, Baumel & Klemm 1995 paper on this was not found]. Aerodynamic load also keeps the wing extended; no number found.

**How the feathers move.**

- Elastic tissue at the feather bases spreads the feathers passively as the skeleton moves. No actuator per feather [abstract, Matloff 2020].
- Neighbouring feathers lock with one-way micro-hooks when the wing extends, which stops gaps opening, and unlock when it flexes [abstract, Matloff 2020].
- Small smooth muscles and ligaments link the flight feather bases to each other and to the bone, and may hold the folded wing in place [abstract, Hieronymus 2016].
- Folded, the primaries slide under one another into a stack and the stack lies over the secondaries along the body. No measured stack geometry was found.

**How much the shape changes.**

| State | Span | Area | Tag |
|---|---|---|---|
| Swift, extended | 1.00 | 1.00 | [digitised] |
| Swift, 30 deg sweep | 0.89 | 0.94 | [digitised] |
| Swift, 50 deg sweep | 0.66 | 0.84 | [digitised] |
| Swift, live, gliding 7-11 m/s | 0.90-0.85 | about 0.87-0.81 | [full text] span; [my calc] area |
| Jackdaw, span halved | 0.50 | 0.53 | [my calc] from the fit above |
| Jackdaw, live, gliding 6-11 m/s | 0.95-0.70 | follows span, slope 0.97 | [full text] |
| Swift, fully folded | NOT FOUND | NOT FOUND | - |

A swift's folded wing is 169-181 mm long from wrist to tip, longer than its 160-170 mm body, so the folded tips must reach past the tail [my calc from secondary numbers].

**Stiffness depends on posture.** Out-of-plane bending and twisting at the wrist are most restricted when the wing is fully extended [full text, Baliga 2019]. The extended wing is the stiff one.

**Passive unfolding does not transfer to a glider.** Stowers & Lentink 2015 built a wing with a free wrist hinge that unfolds by itself within one to two wingbeats, with fold ratios of 2 to 2.5, and re-opens after hitting an obstacle. The opening force is the centrifugal load from flapping [abstract]. A glider has no such load.

**Folding changes balance little, stability a lot.** Elbow and wrist motion barely move the whole bird's centre of gravity. The wing's own centre of gravity shifts by only 10% of the half span in a black swift, the least of 22 species, because its hand wing is long and light [full text, Harvey 2022]. But sweep moves the centre of lift strongly (see `sources.md`).

---

## 6. What a flat-sheet 20-40 cm glider can honestly copy from a swift, and what it cannot

**INTERPRETATION. Everything in this section is my reading of the sources above. None of it is tested.**

### On the earlier "swift-like v2" shape

The complaint was a broad inner wing with a very thin outer wing. The data split that complaint in two:

- A broad inner wing **is** real for a fully extended swift. The chord is greatest at the root and stays near that value over the inner 40% of the wing.
- A very thin outer wing is **not** real. At 80% of the wing the chord is still about half the root chord, and at 90% about a third. If v2 was thinner than that outboard, it was thinner than the bird.
- The more important point: a swift gliding at its best speed is not fully extended. It holds 85-90% of full span, which is close to the "30 degree" outline, and there the chord is more even along the wing and the hand wing is swept about 40 degrees.

### What can be copied

1. The outline, since it is measured.
2. The aspect ratio, 9.5-10 extended, falling to about 8.5 at the glide posture.
3. A sharp leading edge on the swept hand wing.
4. The tail outline and its two widths.
5. The Reynolds number. Ours is the swift's own.

### What cannot be copied, and what that costs

1. **Camber and twist.** A flat plate makes less lift and stalls earlier. Expect a best L/D well below the swift's 12.5. No flat-plate force data for a bird outline at our Reynolds number was found, so no figure is offered.
2. **Pitch trim and stability.** The bird trims with tail lift and constant adjustment. A flat swept plate with no twist has no built-in nose-up trim moment. It will need the centre of gravity forward plus something that pushes the tail end down or the tips up: a tail set at a slight negative angle, or the trailing edge bent up at the tips. This follows the Thomas & Taylor rules; the amounts are not in any source.
3. **Yaw.** The Sachs argument says a small swept wing can do without a fin. PigeonBot II says a bird shape without a fin needed active damping. Plan for tip fins or a small fin until a test shows otherwise.
4. **The body.** A quarter to nearly half of a swift's drag. A flat sheet has almost none, which is one thing in our favour.
5. **Speed-dependent shape.** The bird changes span, sweep and tail spread with speed. A fixed sheet must pick one posture. For a nose-down release and pull-out, the swept posture is the one the bird uses at speed.

### Proposed swift proportions

Lengths are fractions of the half span s (centreline to tip) or of the full span b. Chords are fractions of the mean chord (wing area divided by span) and are measured along the flight direction.

| Item | Extended (slow) shape | Glide shape (about 30 deg) | Source |
|---|---|---|---|
| Aspect ratio | 9.5-10 | about 8.5 | [full text] Henningsson 2014, Alerstam 2007; [my calc] |
| Body or centre-section half width | 0.10 s | 0.11 s | [my calc] from frontal area |
| Arm wing, body side to wrist | 0.10-0.15 s | same length, more angled | estimate, section 1.5 |
| Hand wing, wrist to tip | 0.75-0.80 s | 0.75-0.80 of the reduced half span | estimate, section 1.5 |
| Span relative to extended | 1.00 | 0.85-0.90 | [full text] Henningsson 2011; [digitised] |
| Area relative to extended | 1.00 | 0.81-0.94 | [my calc]; [digitised] |
| Chord at root | 1.30-1.45 | 1.30 | [digitised] Lentink & de Kat 2014 fig 4A, fig 2 |
| Chord at wrist | 1.27-1.30 | 1.20 | [digitised] |
| Chord at mid-hand (0.6 of the wing) | 0.95 | 0.95 | [digitised] |
| Chord at 0.8 of the wing | 0.69 | 0.70 | [digitised] |
| Chord at 0.9 of the wing | 0.45 | 0.56 | [digitised] |
| Tip | pointed, rounded off over the last 4% | pointed | [digitised] |
| Where the leading edge is furthest forward | 0.3-0.4 of the wing | at the shoulder | [digitised] |
| Arm-wing leading edge | about 9 deg forward | about 10 deg back | [digitised] |
| Hand-wing leading-edge sweep | 6 deg inboard, curving to 25 deg at the tip | 37-44 deg | [digitised] |
| Hand-wing trailing edge | straight across | about 25 deg back | [digitised] |
| Hand-wing sweep range, full | - | up to 55-61 deg at the "50 deg" setting | [digitised] |
| Tail width | 0.23 b spread (slow) | 0.10 b furled (fast) | [full text] Henningsson 2011; [my calc] |
| Tail length | 0.19-0.23 b | same | [secondary] search excerpt only |
| Tail fork | NOT FOUND | NOT FOUND | - |
| Centre of gravity | 10% of body length behind the shoulder, as a starting point | same | [full text] Harvey 2022, all-species value, not swift-specific |

Suggested use: draw the **glide shape** for a fixed article. Draw both if the wing is to fold, with one hinge line near 0.20-0.25 s as the wrist and a single linked motion, since the real wing follows one path.

### Other birds worth a look

- **House martin and barn swallow** are the only other birds at our Reynolds number. They have lower aspect ratios (7.5-8.1), much lower wing loading (12-14 N/m2 against the swift's 22-26) and a shorter hand wing. They are closer than the swift to a 10-30 g glider. Their planform chord distributions were not found.
- **Jackdaw and crow** have a low aspect ratio (6) and slotted tips, and fly at three to five times our Reynolds number. They are a weaker match, and the slots cannot be copied in a way known to work at our scale.

### Open items

Not found, and worth a measurement of our own (a museum skin, or a photograph of a gliding bird from directly below, would settle most of them): wrist position as a fraction of span; tail length and fork depth; spanwise twist and camber; fully folded span and area; chord distributions for swallow and martin; any force data for a flat plate cut to a bird outline at Re 15,000-40,000.

---

## Sources

Read in full (open access or open copy):

- Lentink D, de Kat R (2014). Gliding swifts attain laminar flow over rough wings. PLoS ONE 9(6):e99901. doi:10.1371/journal.pone.0099901. Figures 2 and 4A digitised.
- Henningsson P, Hedenstrom A (2011). Aerodynamics of gliding flight in common swifts. J Exp Biol 214:382-393. doi:10.1242/jeb.050609. Text read through the journal page; Tables 1 and 2 are images and were not read.
- Henningsson P, Hedenstrom A, Bomphrey RJ (2014). Efficiency of lift production in flapping and gliding flight of swifts. PLoS ONE 9(2):e90170. doi:10.1371/journal.pone.0090170
- Henningsson P, Muijres FT, Hedenstrom A (2011). Time-resolved vortex wake of a common swift flying over a range of flight speeds. J R Soc Interface 8:807-816. doi:10.1098/rsif.2010.0533
- Alerstam T, Rosen M, Backman J, Ericson PGP, Hellgren O (2007). Flight speeds among bird species: allometric and phylogenetic effects. PLoS Biol 5(8):e197. doi:10.1371/journal.pbio.0050197. Species table is Protocol S1, doi:10.1371/journal.pbio.0050197.sd001
- Rosen M, Hedenstrom A (2001). Gliding flight in a jackdaw: a wind tunnel study. J Exp Biol 204:1153-1166. doi:10.1242/jeb.204.6.1153
- Rosen M, Spedding GR, Hedenstrom A (2007). Wake structure and wingbeat kinematics of a house-martin Delichon urbica. J R Soc Interface 4:659-668. doi:10.1098/rsif.2007.0215. Copy: https://sites.usc.edu/drydenwt/files/2022/06/RSH_JRSI_07.pdf
- Harvey C, Baliga VB, Wong JCM, Altshuler DL, Inman DJ (2022). Birds can transition between stable and unstable states via wing morphing. Nature 603:648-653. doi:10.1038/s41586-022-04477-8
- Baliga VB, Szabo I, Altshuler DL (2019). Range of motion in the avian wing is strongly associated with flight behavior and body mass. Sci Adv 5:eaaw6670. doi:10.1126/sciadv.aaw6670
- Lees JJ, Dimitriadis G, Nudds RL (2016). The influence of flight style on the aerodynamic properties of avian wings as fixed lifting surfaces. PeerJ 4:e2495. doi:10.7717/peerj.2495
- Muir RE, Arredondo-Galeana A, Viola IM (2017). The leading-edge vortex of swift wing-shaped delta wings. R Soc Open Sci 4:170077. doi:10.1098/rsos.170077. Preprint read: doi:10.1101/099713
- Lambert WB, Stanek MJ, Gurka R, Hackett EE (2019). Leading-edge vortices over swept-back wings with varying sweep geometries. R Soc Open Sci 6:190514. doi:10.1098/rsos.190514
- Spedding GR, McArthur J (2010). Span efficiencies of wings at low Reynolds numbers. J Aircraft 47(1):120-128. doi:10.2514/1.44247. Copy: https://sites.usc.edu/drydenwt/files/2022/06/SMac_JA_10.pdf (page range not confirmed)
- McArthur J (2008). Aerodynamics of wings at low Reynolds numbers: boundary layer separation and reattachment. PhD thesis, University of Southern California. https://sites.usc.edu/drydenwt/files/2023/05/mcarthur-thesis-2008.pdf
- Ananda GK, Sukumar PP, Selig MS (2015). Measured aerodynamic characteristics of wings at low Reynolds numbers. Aerosp Sci Technol 42:392-406. doi:10.1016/j.ast.2014.11.016

Abstract or excerpt only:

- Lentink D, Muller UK, Stamhuis EJ, de Kat R, van Gestel W, Veldhuis LLM, Henningsson P, Hedenstrom A, Videler JJ, van Leeuwen JL (2007). How swifts control their glide performance with morphing wings. Nature 446:1082-1085. doi:10.1038/nature05733
- Videler JJ, Stamhuis EJ, Povel GDE (2004). Leading-edge vortex lifts swifts. Science 306:1960-1962. doi:10.1126/science.1104682 (DOI not confirmed)
- Thomas ALR, Taylor GK (2001). Animal flight dynamics I. Stability in gliding flight. J Theor Biol 212:399-424. doi:10.1006/jtbi.2001.2387
- Sachs G (2005). Yaw stability in gliding birds. J Ornithol 146:191-199. doi:10.1007/s10336-005-0078-5
- Sachs G (2007). Why birds and miniscale airplanes need no vertical tail. J Aircraft 44(4):1159-1167. doi:10.2514/1.20175
- Vazquez RJ (1994). The automating skeletal and muscular mechanisms of the avian wing (Aves). Zoomorphology 114:59-71. doi:10.1007/BF00574915 (volume and pages not confirmed)
- Matloff LY, Chang E, Feo TJ, Jeffries L, Stowers AK, Thomson C, Lentink D (2020). How flight feathers stick together to form a continuous morphing wing. Science 367:293-297. doi:10.1126/science.aaz3358 (volume and pages not confirmed)
- Hieronymus TL (2016). Flight feather attachment in rock pigeons (Columba livia): covert feathers and smooth muscle coordinate a morphing wing. J Anat 229:631-656.
- Stowers AK, Lentink D (2015). Folding in and out: passive morphing in flapping wings. Bioinspir Biomim 10:025001. doi:10.1088/1748-3190/10/2/025001
- Klaassen van Oorschot B, Mistick EA, Tobalske BW (2016). Aerodynamic consequences of wing morphing during emulated take-off and gliding in birds. J Exp Biol 219:3146-3154.
- Tucker VA, Heine C (1990). Aerodynamics of gliding flight in a Harris' hawk, Parabuteo unicinctus. J Exp Biol 149:469-489.
- Parrott GC (1970). Aerodynamics of gliding flight of a black vulture Coragyps atratus. J Exp Biol 53:363-374 (pages not confirmed).

Secondary:

- BTO BirdFacts, wing length and weight from ringing data: https://www.bto.org/understanding-birds/birdfacts/swift and the matching pages for swallow, house-martin, kestrel, jackdaw, black-headed-gull, arctic-tern
- Wikipedia, Common swift: https://en.wikipedia.org/wiki/Common_swift
- Featherbase, Common swift (page did not load; a search excerpt was used): https://www.featherbase.info/en/species/Apus/apus

Named but not opened; no number taken: Withers 1981 (J Exp Biol 90:143-162); Okamoto, Yasuda & Azuma 1996 (J Exp Biol 199:281-294); Tucker & Parrott 1970; Pennycuick 1968; Laitone 1997 (Exp Fluids); Brown, Baumel & Klemm 1995; Jukema et al. 2023 (Ringing & Migration, doi:10.1080/03078698.2023.2262804).
