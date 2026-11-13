# Bird-wing layouts for a small passive glider: literature notes

Compiled 2026-10-12. Web research only. Scope: gliding (not flapping) birds from swift to crow size, and thin plates at chord Reynolds number (Re) roughly 10,000-40,000.

## How to read these notes

Every claim carries an access tag saying how much of the source I actually read:

- **[full text]** I read the paper text (open-access HTML or PDF converted to text).
- **[abstract only]** I read only the publisher abstract.
- **[secondary]** I did not reach the paper; the statement comes from another paper, a repository record, or a search-result summary that describes it. Treat as a lead to check, not as a verified number.
- **[derived]** A number I calculated from values in the source (formula given). Not stated by the source.
- **[unverified, general knowledge]** Not confirmed from any source in this session.

Reference numbers in square brackets, e.g. [R7], point to the list at the end.

Two limits apply throughout. First, several Journal of Experimental Biology papers put their morphology tables in images or behind a paywall, so some masses and spans are missing. Second, I could not open the full text of the three papers most often cited for thin plates at this scale (Pelletier & Mueller 2000, Torres & Mueller 2004, Okamoto & Azuma 2011); what is said about them below is second-hand.

---

## 1. Gliding birds, small to crow size: numbers

### 1.1 Table

L/D is the lift-to-drag ratio, equal to the glide ratio in steady flight. "Tilting tunnel" means a live bird gliding in a wind tunnel tilted until the bird just holds station; L/D is read from the tilt angle.

| Species | Mass | Span | Wing area | Aspect ratio | Wing loading | Speed at best glide | Chord Re | Best L/D | How measured | Source and access |
|---|---|---|---|---|---|---|---|---|---|---|
| House martin (*Delichon urbicum*) | 16.9 g | 29.6 cm (semi-span 14.8 cm) | 112 cm² | 7.8 | 14.8 N/m² | Not measured in glide; tunnel flights at 4-10 m/s | Mean chord 3.8 cm; local Re "always less than 2 x 10^4" | **No glide polar found** | Flapping-wake study; morphology table only | Rosén, Spedding & Hedenström 2007 [R1], full text |
| Common swift (*Apus apus*), live bird | Not readable (table is an image) | 0.39 m max, 0.30 m at high speed | 0.016 m² max, 0.011 m² min | 9.8 (paper); 9.5 [derived, b²/S] | Not readable | 9.4-9.5 m/s (min sink at 8.1 m/s) | 18,400 at 7 m/s to 29,500 at 11 m/s | **12.5** (fitted; highest single point 12.7 at 10 m/s) | Tilting tunnel, one juvenile bird, 7-11 m/s | Henningsson & Hedenström 2011 [R2], full text (first ~55%) |
| Common swift, wings only | n/a | n/a | n/a | n/a | n/a | about 9.0 m/s | Tunnel range 12,000-67,000 | **10.0** | Preserved wings on a balance | Lentink et al. 2007 [R3]: abstract only for the paper; the 10.0 figure is [secondary] via Hedrick et al. 2018 [R4] |
| Small passerine (zebra finch) | - | - | - | - | - | - | - | Body-only L/D 1.4-1.5 during bounds (wings folded) | PIV on live birds plus specimens | Tobalske et al. 2009 [R5], [secondary] (search summary of abstract). **No wings-out glide polar found for any small passerine.** |
| Barn swallow | - | - | - | - | - | - | - | **None found** | - | Only behaviour data: Bruderer et al. 2001 [R6], [secondary] |
| Jackdaw (*Corvus monedula*) | 0.215 kg (0.211-0.221) | 0.67 m max | 0.0652 m² max | 6 (as cited in [R2]); 6.9 [derived] | 32 N/m² [derived, mg/S] | 8.3-8.5 m/s (min sink 0.62 m/s at 7.4 m/s) | 38,000-76,000 over 6-11 m/s | **12.6** at 8.5 m/s | Tilting tunnel, one bird | L/D, speeds, Re: Rosén & Hedenström 2001 [R7], abstract only. Mass/span/area: KleinHeerenbrink et al. 2016 [R8], full text (first half), a different individual |
| Jackdaw, wing only | 0.25 kg | semi-span 0.29 m | 0.055 m² | 6.07 | - | 8-16 m/s tested | order 10^5 (not given per species) | **4.0** | Dried, pinned wing on a balance | Lees, Dimitriadis & Nudds 2016 [R9], full text |
| Magpie, jay, wing only | 0.21 kg; 0.17 kg | semi-span 0.25; 0.22 m | 0.050; 0.045 m² | 4.97; 4.35 | - | 8-16 m/s tested | order 10^5 | **3.7; 3.1** | Dried wings | Lees et al. 2016 [R9], full text |
| Pigeon (*Columba livia*) | Not in abstract | Not in abstract | Not in abstract | Falls with speed | - | L/D stays near maximum up to at least twice the minimum gliding speed | Not in abstract | **5.5-6.0** | Tilting tunnel, trained birds | Pennycuick 1968 [R10], abstract only |
| Woodpigeon, wing only | 0.49 kg | semi-span 0.35 m | 0.077 m² | 6.36 | - | 8-16 m/s tested | order 10^5 | **4.0** | Dried wing | Lees et al. 2016 [R9], full text |
| Crow or rook | - | - | - | - | - | - | - | **None found** | - | The jackdaw is the only corvid with a live glide polar that I could find |
| Laggar falcon (*Falco jugger*) | Not in abstract | Not in abstract | Not in abstract | - | - | 12.5 m/s (tested 6.6-15.9 m/s) | Not in abstract | **about 10** | Tilting tunnel, one bird | Tucker & Parrott 1970 [R11], abstract only |
| Kestrel (*Falco tinnunculus*) | Not in abstract | - | - | - | - | winds about 8.7 +/- 1.5 m/s | - | Minimum glide angle "estimated" 5 deg, i.e. L/D about 11 [derived, 1/tan 5 deg] | Field observation of birds hanging over a sea dike plus measured wind profiles; an estimate, not a direct measurement | Videler & Groenewold 1991 [R12], abstract only |
| Harris' hawk (for scale) | 0.702 kg | 1.02 m | - | - | - | - | - | **10.9** (min sink 0.81 m/s) | Tilting tunnel | Tucker & Heine 1990 [R13], [secondary] (search summary of abstract) |
| Gull (glaucous-winged x western hybrids), wings only | - | - | - | - | - | 10 m/s tested | 92,000-179,000 (root chord) | **about 5** | Cadaver wings on a load cell | Harvey et al. 2019 [R14], full text. Authors say this probably understates what a live gull achieves. **No live-gull glide polar found.** |

### 1.2 Extra numbers worth having

- Swift lift coefficient fell from 0.96 at 7 m/s to 0.48 at 11 m/s (wing area reference); total drag coefficient from 0.10 to 0.043; minimum wing profile drag coefficient 0.011 at 10 m/s; body drag coefficient 0.30 to 0.22. The induced-drag factor was assumed (k = 1.1), not measured. The bird shortened its span and furled its tail (tail span 0.09 m at 7 m/s, 0.04 m at 11 m/s) as speed rose [R2, full text].
- A discussion passage in [R2] gives a wing loading of "26" for the comparison with the jackdaw; I could not tell from the extracted text which bird or unit it refers to. Not used.
- Jackdaw: the tail made positive lift in most conditions; body drag coefficient about 0.2 with tail folded and above 0.4 with tail spread; tunnel walls cut induced drag by about 10-15 % for a bird of this size, so tilting-tunnel L/D values for birds this large are somewhat flattering [R8, full text].
- Withers 1981 [R15, abstract only]: fixed bird wings at Re of order 10^4 (the abstract's range reads "1-5 x 10^4"; the dash was garbled in my copy, and one later review misquotes it as 10^5). Across species: minimum drag coefficient 0.03-0.13, maximum lift coefficient 0.8-1.2, maximum L/D 3 to 17. Lees et al. [R9] report that the top of that range belongs to gliding specialists (swift) and that non-specialists gave 3.8-6.

### 1.3 Pattern in the table

- Live birds in tilting tunnels: L/D about 10-12.6 (swift, jackdaw, falcon, hawk), pigeon about 6.
- The same kinds of wings tested alone as dried, fixed surfaces: L/D about 3-5, apart from swift wings (about 10).
- Lees et al. [R9] attribute part of the gap to testing wings without the body; Harvey et al. [R14] say their cadaver-wing value is probably low. Neither paper isolates the cause. Dried wings lose the shape a live bird holds, so the low numbers should not be read as "a rigid copy of the outline gives L/D 3-5" nor as proof that it would do better.

### 1.4 Closest match to a 25-40 cm, 10-40 g glider

- **House martin** matches on size: 29.6 cm span, 16.9 g, 3.8 cm chord, aspect ratio 7.8, wing loading 14.8 N/m², Re below 20,000 [R1]. It has no published glide polar.
- **Common swift** is the closest species that does have a measured polar: 39 cm span, Re 18,400-29,500, L/D 12.5 at about 9.5 m/s [R2]. Its best-glide speed is above the 5-8 m/s target; at 7 m/s the fitted polar is below its peak and the bird flies at a lift coefficient near 0.96.
- The jackdaw (Re 38,000-76,000) overlaps only the top of the glider's Re range and is 5-20 times heavier.

---

## 2. Which wing features are credited with good performance, and how strong is the evidence

Ratings: **well-supported** = measured forces, more than one study, at relevant Re. **Mixed** = measured, but studies disagree or the benefit is conditional. **Speculative** = proposed, or shown only by flow pictures, modelling, or at a much higher Re.

| Feature | Rating | Evidence |
|---|---|---|
| Thin section with camber | **Well-supported** | Laitone 1997 [R16, abstract only]: for all Re below 70,000 the best profile tested was a thin plate with 5 % circular-arc camber; it had the highest L/D at every turbulence level, and thin plates were far less sensitive to Re and turbulence than a NACA 0012, which degraded badly below 50,000. McArthur 2008 [R17, full text]: at Re 10,000 and 20,000 a 5 % cambered plate out-lifted the Eppler 387 sailplane section at most angles; at 10,000 even the uncambered flat plate had higher L/D than the Eppler 387. Sunada et al. 1997 [R18, secondary] reach the same conclusion at Re 4,000: thin, sharp leading edge, about 5 % camber. |
| Sharp or thin leading edge | **Well-supported** (same sources) | Laitone found a NACA 0012 lifted better when run backwards (sharp edge first) at Re about 20,000, as summarised in [R17]. Sunada 1997 [R18] lists a sharp leading edge among the traits of good sections at Re 4,000. |
| Feather roughness as a turbulator | **Mixed (helps only in a narrow band)** | Lentink & de Kat 2014 [R19, full text]: swift upper-surface roughness is 1-2 % of chord; laminar flow covered 69 % (SD 13 %, n = 3) of the wing in efficient glides; on model wings with tape strips up to 1.3 % chord high, roughness lowered drag near glide lift coefficients at Re about 12,000-24,000 but raised drag above about 24,000. Van Bokhorst et al. 2015 [R20, abstract only]: on 3D-printed swift hand-wing models the smooth wing formed open laminar separation bubbles reaching past the trailing edge, while the rough wing kept attached flow; the abstract text I saw does not give the size of the force benefit. Both studies use swift-shaped, cambered, thin models; neither tested a flat plate. |
| Swept-back hand wing and leading-edge vortex (LEV) | **Mixed, leaning unsupported as a lift benefit in gliding** | Videler et al. 2004 [R21]: a vortex was seen over a 60 deg swept swift hand-wing model in a water tunnel at Re 3,700-37,000 and angle of attack 10 deg; no forces were measured ([secondary] via [R17] and news coverage). Lentink et al. 2007 [R3]: on real swift wings the vortex appeared only at high angles and did not raise lift, because sweeping back cut wing area ([secondary], as summarised in [R17]); the abstract says extended wings are better for slow glides and swept wings for fast glides and turns. McArthur 2008 [R17, full text]: four simple 5 % cambered plates, aspect ratio 6, sweep 0/20/40/60 deg, at Re 15,000 - the **unswept wing had the highest L/D at nearly all angles**; a small vortex formed on the 60 deg wing near 15 deg but left no mark on the force curves. |
| Alula (the "thumb" feathers) | **Well-supported, but only at high angle of attack** | Lee et al. 2015 [R22, full text]: magpie wings at Re 20,000-130,000; lift rose 1.3-12.7 % (mean 6.1 %) on one wing, only above roughly 20-40 deg angle of attack depending on the wing; stall delayed 5-10 deg on one wing; drag unchanged; one of four wings not statistically significant. Mechanism: a streamwise vortex from the alula tip. This is a slow-flight and landing device; the study shows no effect at normal glide angles. |
| Slotted (separated) primaries at the tip | **Mixed** | For: Tucker 1993 [R23]: a feathered tip on a Clark Y base wing cut drag by about 12 % against an unslotted tip; Tucker 1995 [R24]: clipping a Harris' hawk's tip feathers raised the induced drag factor from 0.56 to 1.10 (both [secondary]; the publisher pages blocked me). Sachs 2006 [R25, secondary] computes that slotted tips also add yaw stability. Against, or neutral: KleinHeerenbrink et al. 2016 [R8, full text] report their jackdaw wake data do not support separated primaries controlling induced drag. Lees et al. 2016 [R9] link the corvids' low drag to slotting, but only by association. All of this is for hawk-to-jackdaw size (Re about 40,000-150,000); swifts, swallows and martins, the birds at the glider's scale, have pointed unslotted tips. |
| Crescent or lunate planform | **Speculative at this scale** | Van Dam 1987 [R26, abstract only]: backward curvature raises induced efficiency above the flat elliptical wing through a non-planar wake (panel-method computation). Later reports put the computed gain near 8 %, a typical 4 % in another analysis, and a measured gain of about 3 % at Re 1.7 x 10^6 ([secondary]). I found no measurement at Re 10^4-10^5. At this Re induced drag is not the only large term, so a few per cent on induced drag is a small prize. A published rebuttal (Smith & Kroo 1993) is often cited; I could not locate it to confirm. |
| Wing twist / washout | **Mixed** (well documented as geometry; function not demonstrated) | Cheney et al. 2021 [R27, abstract]: barn owl, tawny owl and goshawk share a spanwise twist pattern, an inverse relation between twist and peak camber, and wings held below the shoulder; camber falls as speed rises. Durston et al. 2019 [R28, abstract]: gliding barn owl had positive camber and "longitudinal anhedral". Thomas & Taylor 2001 [R29, abstract only] predict that a statically stable animal with swept-back wings should have washout. But the raptors with measured geometry were judged unstable (Section 3), so observed twist in birds is not evidence that it is there for stability. |
| Tail as a lifting surface | **Well-supported** (for raptors and jackdaw, Re about 10^5) | Usherwood et al. 2020 [R30, full text]: in barn owl, tawny owl and goshawk the downwash behind body and tail exceeded that behind the wings; the pattern fitted a "minimise viscous (profile) drag" model (24 % RMS error) far better than a "minimise induced drag" model (247 %). Song et al. 2022 [R31, abstract]: observed barn-owl tail postures sit near minimum total drag in CFD. KleinHeerenbrink et al. 2016 [R8]: jackdaw tail lifts in most conditions but spreading it roughly doubles body drag coefficient. Tail lift theory: Thomas 1993 treats the tail as a slender delta wing; Evans 2003 [R32] found that holds only for spread below 60 deg and angle of attack below 20 deg ([secondary]). |
| Tail as a pitch stabiliser (aircraft-style) | **Not supported for the birds measured** | Usherwood et al. [R30]: the tail carries positive lift, which is the opposite of the down-loaded tail that gives a conventional aircraft its pitch stability; the authors do not present a full stability analysis. Chen et al. 2025 [R33, abstract]: a drag-minimising model barn owl is aerodynamically unstable at all speeds. |
| Flexible or porous feathers | **Speculative for steady gliding** | Carruthers et al. 2007 [R34, secondary]: steppe-eagle underwing coverts pop up passively at high angle of attack like a leading-edge flap; observational video, during landing and gusts. Bench studies of hinged flaps report lift gains near stall ([secondary]). I found no force measurement showing a gliding benefit at cruise angles, and could not locate the often-cited feather air-transmissivity paper (Müller & Patone 1998). |

---

## 3. Stability: are gliding birds passively stable?

### 3.1 Short answer from the literature

The published work does not support "birds are passively stable, so copy the bird". It supports three narrower statements:

1. Theory and wing-only tests say a bird's **wings alone can be statically stable in pitch** in some postures, without the tail.
2. For the birds whose **in-flight shape was actually measured** (barn owl, peregrine), the analyses found them **strongly unstable in pitch**, flying on active control.
3. Most species studied can **move between stable and unstable** by flexing elbow and wrist. That is a moving-wing capability a rigid glider does not have.

### 3.2 Pitch stability, source by source

**Thomas & Taylor 2001, J. Theor. Biol. [R29, abstract only].** Theoretical pitching-moment analysis. Claims: flying animals have more inherent stability than had been assumed; measured positions of centre of gravity and lift in birds suggest the wings alone may give longitudinal static stability, making birds more like tailless aircraft than tailed ones. Pendulum stability (centre of gravity below the wing) and drag-based stability matter more for animals than for aircraft. Rules of thumb for a *stable* flier: wings swept forward in slow flight; washout if the wings are swept back and wash-in if swept forward; any surface behind the wing set nose-down relative to it, any surface ahead set nose-up. They also note that static margin is only meaningful for an animal once the trimmed angle of attack is stated. I did not read the derivations.

**Harvey et al. 2019, J. R. Soc. Interface, gull wings [R14, full text].** Cadaver wings, 10 m/s, Re 92,000-179,000, elbow angle varied 34-149 deg.
- Pitch stiffness: the slope dCm/dCL was negative (statically stable) for every wing, from about -0.03 (flexed) to -0.75 (extended), with moments taken about the shoulder.
- Trim: most wings had a **negative** zero-lift pitching moment, so they are "stable" but cannot balance at a positive lift without help. Only the most extended wings had a positive zero-lift moment.
- The authors conclude fully extended wings "may alone be sufficient" for passive stability, on the condition that the whole bird's centre of gravity lies less than about 12 cm behind the shoulder, which they did not measure.
- Higher tunnel turbulence slightly reduced stability. Tail and body were not tested.

**Harvey & Inman 2022, PNAS, gull dynamic stability [R35, full text].** Model with a furled, fixed tail. Of 1,457 trimmed wing configurations, 1,331 were statically stable and those were also dynamically stable; the 126 unstable ones mostly had extended wrists. Short-period frequency 8.9-41.0 rad/s, phugoid 0.45-1.10 rad/s, phugoid heavily damped. The tail had a minor stabilising effect and could not trim the bird on its own. The authors suggest a trimmed, stable gull would damp small disturbances passively.

**Harvey et al. 2022, Nature, 22 species [R36, full text].** Mass properties from 36 cadaver specimens; neutral point *scaled from the gull study*; tail aerodynamics excluded from the main model; shoulder motion excluded; lift slope assumed the same for all species.
- 17 of 22 species could switch between stable and unstable using elbow and wrist alone; 4 were stable throughout; 1 unstable throughout. (Which species fall in which class is only in the figures.)
- Evolutionary optima: maximum static margin about +26 % of maximum root chord; minimum about -15 %.
- The range of static margin shrinks as body mass rises.
- The authors say switching "probably requires substantially different control algorithms", and that whether birds actually switch in flight "remains to be seen".
- This is a geometric and inertial estimate, not a flight measurement.

**Durston et al. 2019, J. Exp. Biol., barn owl and peregrine [R28, abstract only].** Wing surface measured by photogrammetry in free glides. The owl's camber and longitudinal anhedral suggest, by flight-dynamics theory, that it was longitudinally statically unstable. The peregrine's configuration was not clear either way.

**Durston & Windsor, conference abstract (SICB) [R37, full text of abstract].** Same birds; measured shape plus CT-scan mass properties in a vortex-lattice model. Both owl and peregrine were "highly longitudinally statically unstable" in all three flights each. Phugoid and roll modes stable, spiral mode mildly unstable, longitudinal mode highly unstable, compared by the authors to combat aircraft and needing very fast corrections. A search summary of related work gives a pitch-divergence doubling time typically under 50 ms [secondary]. This is a conference abstract, not a peer-reviewed paper.

**Usherwood et al. 2020 [R30] and Chen et al. 2025 [R33].** See Section 2. Raptor tails carry positive lift for drag reasons; the modelled drag-minimising barn owl is unstable at every speed, less so at high speed.

**Harris' hawk, Weston, Zhu, Taylor & Harvey [R38, secondary; news coverage and search summary only].** 3D-printed models of measured hawk postures in a wind tunnel: tucked-wing configurations were statically stable; spread configurations had a nonlinear pitching-moment curve. Reported as a shift from unstable to stable as the hawk tucks to pass a gap. Year and full details not confirmed.

**Papers I was asked about but could not match.** "Cheney et al. on barn owl tail": the closest items are Cheney et al. 2021 [R27], Usherwood et al. 2020 [R30] (Cheney is second author), Song et al. 2022 [R31] and Chen et al. 2025 [R33]. "KleinHeerenbrink et al. 2022 on tail": I found no such paper; KleinHeerenbrink et al. 2016 [R8] covers jackdaw tail lift and drag, and a 2022 Nature paper by that author is about perching manoeuvres, which I did not open.

### 3.3 Why the studies disagree

They are not measuring the same thing. The "can be stable" results come from wings alone or from geometric models that leave out tail lift and use an assumed neutral point. The "is unstable" results come from the shape that live raptors actually adopted in glides, including a lifting tail. Both can be true: the bird has stable postures available and, in the recorded glides, was not using one. No study I found measured the pitch stability of a swift, swallow or martin, the birds nearest the glider's size.

### 3.4 Yaw and roll with no vertical fin

All of this is from G. Sachs and is analytical or computational, not flight-tested on birds.

- Sachs 2005, J. Theor. Biol. [R39, abstract only]: sideslip-induced yawing moment depends on sweep, aspect ratio and lift coefficient. A computed gull wing showed substantial yaw stability that grows with lift coefficient and peaks near best-glide conditions; the same wing with sweep removed produced much smaller yawing moments. Sweep in the *outer* wing is what counts.
- Sachs 2006, J. Theor. Biol. [R25, secondary]: sweep within slotted tip feathers adds to the yawing moment.
- Sachs 2005, J. Ornithol. [R40, secondary]: by scaling, birds' dynamic yaw stiffness is adequate even when static yaw stability is very small.
- Sachs 2007, J. Aircraft [R41, abstract only]: small fliers get adequate Dutch-roll frequency with very little static directional stability; the wing alone can supply the yawing moment and the yaw damping, because required damping falls with size; removing the fin affects the spiral mode only partly. The paper explicitly extends this to aircraft "a few centimetres long". (The 2007 paper is in Journal of Aircraft; a 2007 J. Theor. Biol. paper by Sachs on tail effects is cited by others but I could not find it.)
- The one flight-derived data point: Durston & Windsor [R37] found the spiral mode mildly unstable and roll subsidence stable in owl and peregrine.
- I found nothing published on roll stability (dihedral effect) of gliding birds beyond that, other than the observation that raptors glide with wings held below the shoulder (anhedral) [R27], which in aircraft terms reduces roll stability.

### 3.5 What this means for a glider with no control system (plain statement)

- A bird is not a proven passively stable design. The best-measured gliding birds were pitch-unstable and relied on fast active control. Copying a raptor's glide posture, in particular a cambered wing plus a lifting tail, copies a layout whose stability in the bird comes from the nervous system.
- The bird results that do point to passive stability are for **wings alone, fully extended**, and they come with a trim problem: stable slope but nose-down zero-lift moment in most postures [R14]. A fixed wing has to solve trim by geometry or by where the mass sits.
- The absence of a vertical fin in birds is supported by analysis as workable at small scale **if the outer wing is swept back** [R39, R41]. That is the one bird-layout feature with a stated passive-stability rationale that transfers directly to a rigid planform. It is computed, not demonstrated on a rigid glider in these sources.

---

## 4. Engineered precedents

### 4.1 Passive gliders at this scale

**Flat plate with the centre of mass in the right place: Li, Goodwill, Wang & Ristroph 2022, J. Fluid Mech. [R42, full text, open access].** The most directly relevant paper found.
- Paper sheets 15 cm span x 5 cm chord, with small side fins and copper tape on the leading edge, launched in air. With the centre of mass (CoM) displaced forward of mid-chord by 0.08 of chord the sheet tumbled; 0.14, bounding (swoops and climbs); **0.24, steady glide**; 0.31, steep dive.
- Controlled repeat with acrylic plates in water (20.3 cm span, 2.54 cm chord, 0.15 cm thick, Re 1,000-10,000): steady gliding for CoM between 0.22 and 0.29 of chord ahead of mid-chord. That is **21-28 % of chord back from the leading edge** [derived: 0.5 minus the offset].
- Measured mechanism (static torque tests at Re 2,000-4,000): on a thin plate the centre of pressure moves **aft** as angle of attack rises over the relevant range, so a nose-up disturbance produces a nose-down moment. Classical thin-aerofoil theory predicts no such movement (neutral stability about quarter-chord).
- Authors' conclusion: placing the CoM correctly is "a minimal strategy for stabilizing gliding flight that does not require tails or other additional aerodynamic surfaces". They state they know of no earlier study demonstrating free-flight gliding stability of plates.
- Limits: two-dimensional analysis of a rectangular plate; pitch only; the side fins are there for lateral behaviour and are not analysed; Re is below Kai's range (1,000-10,000 against 12,000-40,000). They cite Pelletier & Mueller 2000, Okamoto & Azuma 2011 and Shields & Mohseni 2012 as showing the same static stability about quarter-chord for thin plates "at somewhat higher Re".
- A follow-up, Pomerenk & Ristroph 2025 [R43, abstract only], maps equilibria and their stability and states that dynamic stability "does not follow simple, universal formulas".

**Alsomitra (Javan cucumber) seed, a natural tailless glider: Azuma & Okuno 1987 [R44, secondary].** As quoted in [R42]: centre of mass near quarter-chord, glide ratio 3 to 4. Other figures in circulation (descent angle about 12 deg, sink 0.41 m/s) come from a popular-science summary and are unverified. A 2023 Edinburgh thesis (Certini) [R45, abstract only] recorded 15 seeds: most flew a **helical** path, some a straight oscillating one; the seed flies without a fin or control, and the stability mechanism "is still not fully understood". A search summary adds that the seed is stable in pitch and roll but not directionally [secondary]. So the natural example of a finless passive glider mostly circles.

**Bird-shaped paper, foam or balsa chuck gliders.** I found no peer-reviewed study of the stability of bird-outline toy gliders. This is a real gap, not an oversight in searching one database; the only near hit was a student slide deck.

### 4.2 Bird-inspired aircraft (all actively controlled)

- **Lentink et al. 2007 swift wings [R3]:** wing-only force data and a glide model; not an aircraft.
- **LisHawk: Ajanic et al. 2020, Science Robotics [R46, abstract only].** Goshawk-inspired drone with feathered folding wing and morphing tail. Claims wider speed range, agility and stability from combined wing and tail morphing. The preprint notes flights were flown by a human pilot and so kept to the stable regime. No passive-stability data.
- **PigeonBot: Chang et al. 2020, Science Robotics [R47, secondary].** 40 real pigeon feathers driven by four wrist and finger actuators; asymmetric morphing starts turns. Propeller-driven and radio-controlled.
- **Harris' hawk 3D-printed models [R38]:** fixed-shape wind-tunnel models; tucked shape stable. The only fixed-geometry bird-shaped stability test I found, and at hawk scale.

### 4.3 Planform studies on thin plates

- **Zimmerman and inverse-Zimmerman planforms: Torres & Mueller 2004, AIAA J. [R48].** I could not read the abstract or text. Confirmed [secondary]: Re around 100,000; aspect ratio 0.5-2; maximum lift coefficient and stall angle rise sharply once aspect ratio is 1 or below (tip-vortex lift); a laminar separation bubble was seen on the flat plates. The comparison between planforms, and where the aerodynamic centre sits for each, I could not verify. These are low-aspect-ratio micro-air-vehicle shapes at 2-10 times Kai's Re.
- **Ananda, Sukumar & Selig 2015 [R49, full text]:** ten flat-plate wings, aspect ratio 2-5, taper ratio 0.5-1 with straight quarter-chord line, Re 60,000-160,000. Taper had minimal effect on lift slope and no clear effect on maximum lift. Detail in Section 6.
- **McArthur 2008 swept plates [R17, full text]:** sweep 0-60 deg at constant aspect ratio 6, Re 15,000. Sweep lowered both lift and drag; unswept had the best L/D. No pitching moment measured, so nothing on stability.
- **Okamoto & Azuma 2011, AIAA J. [R50, secondary: abstract fragments]:** thin rectangular-section plates at Re about 10,000; elliptical, rectangular and triangular planforms of several aspect ratios plus one swept parallelogram wing of aspect ratio 4; lift, drag and pitching moment to 90 deg. Reported: high-aspect-ratio wings show nonlinear lift even at low angles; low-aspect-ratio wings have a small lift slope and large maximum lift; drag and pitching moment also nonlinear. This is the paper most likely to hold pitching-moment data for non-rectangular plates at Kai's Re, and I could not get it.

### 4.4 What gave passive stability in the sources

Only two arrangements are actually demonstrated to glide stably with no control, both in pitch only: a thin plate with its mass at roughly 21-28 % chord [R42], and the Alsomitra seed with mass near quarter-chord [R44, via R42]. No source demonstrated a bird-shaped fixed planform, with or without a tail, gliding stably at this scale.

---

## 5. Folding

**How a bird wing folds.** The radius and ulna run side by side between elbow and wrist and slide past each other like the bars of a parallel ruler ("drawing parallels"), so flexing the elbow automatically flexes the wrist and extending one extends the other; the wrist bends mainly toward the ulna side, which lets the hand fold back along the forearm [R51, secondary: search summaries of an 1870s anatomy paper, a Springer abstract, and a 2010 Proc. R. Soc. B paper]. A 2023 Zoological Letters paper [R52, secondary] describes the same ulna-radius sliding as an interlocking fold mechanism; a recent pigeon motion-capture and CT study, which I could not identify precisely, reports that the simple four-bar model fits poorly. The flight feathers fan over one another as the skeleton moves: elastic tissue at the feather bases redistributes them passively, and hooked microstructures lock neighbouring feathers together in one direction so that no gap opens (Matloff et al. 2020, Science [R53, secondary]).

**Small engineered wings that fold this way.**
- PigeonBot [R47]: real feathers on an elastic band, wrist and finger joints.
- LisHawk [R46]: artificial feathers that fan and overlap, with a folding tail.
- Stowers & Lentink 2015, Bioinspir. Biomim. [R54, secondary]: an unactuated hand wing on a wrist hinge that unfolds by centrifugal force within one or two wingbeats and folds on impact. It depends on flapping, so the unfolding mechanism does not carry over to a glider.

None of these is a passive glider; all carry actuators or rely on flapping.

---

## 6. Flat and thin cambered plates at very low Reynolds number

### 6.1 Data sources

| Source | Re | Aspect ratio | Section and thickness | Key results | Where the data are | Access |
|---|---|---|---|---|---|---|
| Li et al. 2022 [R42] | 2,000-4,000 (torque tests); free flight 1,000-10,000 | 6 (15 x 2.5 cm plate) and 8 | Flat acrylic, 0.076 cm thick = 3 % of chord [derived] | Normal-force, lift and drag coefficients and **centre-of-pressure position against angle of attack from 0 to 90 deg**; centre of pressure moves aft with angle; stable trim exists for pivot or CoM up to about 0.3 chord ahead of mid-chord | Figs 4-6; open access | full text |
| McArthur 2008 (USC thesis) [R17] | 10,000 and 20,000 (plus swept wings at 5,000 and 15,000) | 6, and 2D with end plates | Flat plate 3.6 % thick (1.3 mm on 3.5 cm chord), 5:1 elliptical leading edge, trailing edge tapered over last 18 %; same plate bent to 5 % camber; Eppler 387 | 5 % cambered plate gives the highest lift; flat plate beats Eppler 387 on L/D at Re 10,000; lift curve of the flat plate changes slope near 3-4 deg; **L/D plots for the aspect-ratio-6 wings are drawn on axes that top out at 10** at both Re (I could not read exact peaks from the text); no pitching moment | Figs 5.1-5.9 (rectangular), 5.40-5.41 (swept); literature comparison in Figs 5.7-5.8; free PDF | full text |
| Laitone 1997, Exp. Fluids [R16] | 20,000-70,000 | 6, rectangular | Thin wedge (stand-in for a flat plate), 5 % circular-arc plate, NACA 0012 | 5 % cambered plate best in L/D and lift throughout; thin plates insensitive to Re and turbulence. As summarised in [R17]: measured lift slope more than 20 % above lifting-line prediction at Re 40,000; force resolution +/- 0.1 mN; tunnel turbulence about 0.02 % | Fig. numbers not seen; Springer, paywalled | abstract only, plus [secondary] detail from [R17] |
| Pelletier & Mueller 2000, J. Aircraft [R55] | 60,000-200,000 | 0.5-3 semi-span models, plus 2D | Flat and 4 % cambered plates, 5:1 elliptical leading edge. **Thickness is reported inconsistently by citing papers: 1.93 % [R17 p. 40], 1.3 % [R17 p. 17], 2.6 % [R49].** | Cambered plates better; no hysteresis; no reattaching separation bubble seen; lift slope agrees with lifting-line theory; trailing-edge shape and tunnel turbulence matter little; quarter-chord pitching moment was measured. One number confirmed: minimum drag coefficient about 0.0175 for aspect ratio 3 at Re 80,000 [R49] | Not seen | [secondary] only |
| Ananda, Sukumar & Selig 2015, Aerosp. Sci. Technol. [R49] | 60,000-160,000 | 2, 3, 4, 5; taper 0.5, 0.75, 1 | Flat plate 4.3 % thick (3.81 mm on 88.9 mm chord), 5:1 elliptical leading edge, 10:1 elliptical trailing edge | See 6.2 | Figs 6-22 and Table 1; full data set in Ananda's thesis (their Ref. 4); author-hosted PDF | full text (first 13 of 15 pages) |
| Okamoto & Azuma 2011, AIAA J. [R50] | about 10,000 | several | Thin rectangular-section plates, several planforms | Lift, drag, pitching moment to 90 deg; nonlinear lift for high aspect ratio; see 4.3 | AIAA, paywalled | [secondary] |
| Sunada et al. 1997, J. Fluids Eng. [R18] | 4,000 | - | Many sections | Best: thin, sharp leading edge, about 5 % camber near mid-chord; planform has little effect at this Re | Not seen | [secondary] |
| Lentink & de Kat 2014 [R19] | 12,000-67,000 | swift-shaped model, 360 mm span, 38.1 mm chord | 2.8 % thick, 4.5 % camber | Roughness effect on laminar flow and drag (Section 2) | Open access | full text |
| Schmitz 1942, Selig et al. flat plates, Mueller 1999 lecture notes | 42,000 and up | 2D | Flat plates | Appear in McArthur's comparison figure 5.7; McArthur judges Schmitz's low-Re points unreliable (balance resolution about 1 mN) | [R17] Fig. 5.7 | [secondary] |

### 6.2 Ananda, Sukumar & Selig 2015 in detail (the best-documented set I could read)

All for the 4.3 % thick flat plate, Re 60,000-160,000, which is above Kai's range.

- **Maximum lift coefficient 0.55-0.70** for all ten wings, rising only slightly with Re and nearly independent of aspect ratio between 2 and 5.
- **Stall** near 10 deg for aspect ratio 3 at Re 60,000; post-stall lift stays roughly flat to 30 deg rather than collapsing.
- **Lift slope** rises with aspect ratio and with Re, converging on the classical finite-wing value as Re rises; taper has minimal effect.
- **Minimum drag coefficient 0.01-0.02**. For scale, laminar flat-plate skin friction alone would be 0.0066-0.0108 over this Re range.
- **L/D** has a narrow peak in angle of attack with steep falls either side, i.e. a narrow best-glide window. Peak L/D generally rises with Re. (Values are in their Fig. 10; I could not read them from text.)
- **Pitching moment about quarter-chord** is close to zero before stall, then goes strongly nose-down at and after stall, more so for low aspect ratio and strong taper.
- **Aerodynamic centre** sits at 20-27 % of chord at low angle, moves aft over the narrow band where L/D peaks, then shifts toward about 50 % of chord as stall approaches.
- **Oswald efficiency** from 0.3 to about 0.8, rising with Re; the textbook 0.6-0.9 does not hold at low Re.
- No hysteresis. A leading-edge laminar separation bubble was seen by oil flow at Re 60,000.
- Uncertainties: 3.3 % on lift, 2.7 % on drag, 4.6 % on moment coefficient. Tunnel turbulence below 0.1 %.

### 6.3 Points that recur across the plate literature

- An uncambered thin plate has essentially zero pitching moment about quarter-chord before stall [R49], and its centre of pressure moves aft as angle of attack rises [R42, R49]. That aft movement is the passive pitch-stability mechanism identified in [R42].
- Camber buys lift and L/D [R16, R17, R55], but none of the sources I could read gives the pitching moment of a cambered plate at Re 10,000-40,000. (Standard aerodynamics says positive camber adds a nose-down moment; I did not verify a measured value at this Re, so treat that as unverified, general knowledge.)
- Results in the band Re 20,000-60,000 differ between wind tunnels. McArthur [R17] reviews this: free-stream turbulence above about 0.1 % and even the tunnel's acoustic resonance change measured forces on conventional sections; thin plates are less affected.
- Lifting-line theory: Pelletier & Mueller found it adequate at Re 60,000-200,000; Laitone found lift slopes over 20 % above it at Re 40,000; McArthur found no significant disagreement at 10,000-20,000 [R17].

### 6.4 Gap

I found **no measured pitching-moment data for a thin plate with a swept, tapered or bird-like planform at Re 10,000-40,000** that I could actually read. Okamoto & Azuma 2011 [R50] is the likely holder.

---

## 7. What this implies, and does not imply, for a passive 25-40 cm glider

**Everything in this section is my interpretation of the sources above, not a finding of any one of them.**

### Supported by the sources

1. **Section shape matters more than outline at this Re.** Thin plates beat conventional thick sections below about Re 50,000, and a thin plate with about 5 % camber beats a flat one in lift and L/D [R16, R17, R18]. The 0.6 mm printed plate is the kind of section this literature favours. The 5 mm foamboard is, on any chord under about 12 cm, thicker in proportion than every plate in Section 6 (which run 1.3-4.3 %), so those data may not describe it.
2. **Where the mass sits is the demonstrated pitch-stability lever for a thin plate.** A flat rectangular plate glides steadily with its centre of mass roughly 21-28 % of chord from the leading edge; further aft it flutters, tumbles or bounds, further forward it dives [R42]. The window is narrow.
3. **Expected performance.** Published aspect-ratio-6 flat and cambered plates at Re 10,000-20,000 show L/D below 10 [R17]; plates and the Alsomitra seed that glide passively show glide ratios of 3-4 [R42, R44]. A live swift reaches 12.5 [R2] with a cambered, feathered, actively trimmed wing. The sources give no basis for expecting a flat-plate copy of a swift outline to approach the swift's figure.
4. **Swept-back outer wing as a substitute for a fin** has an analytical basis at small scale [R39, R41]. It also costs L/D on simple plates at Re 15,000 [R17]. Both effects are documented; their balance for a given design is not.

### Not supported, or contradicted

5. **"Copy the bird and it will be stable" is not supported.** The gliding birds with measured in-flight shape were pitch-unstable [R28, R37, R33]. Bird stability, where it exists, is posture-dependent and switchable [R36].
6. **A bird-style lifting tail is a drag device in the bird, not a stabiliser** [R30, R31]. A tail that carries positive lift behind the wing works against pitch stability by aircraft reasoning, and the authors of [R30] say so. Thomas & Taylor's rule for a stable flier is the reverse: a rear surface set nose-down relative to the wing [R29].
7. **Leading-edge-vortex lift from a swept hand wing** has no force evidence at glide angles [R17, R3].
8. **Alula, slotted tips, covert flaps** act at high angle of attack or at larger scale [R22, R23, R24, R34]. Nothing found shows a benefit for a steady glide at Re 12,000-40,000.
9. **Crescent planform**: a few per cent of induced drag at best, shown only by computation and at Re above 10^6 [R26].
10. **Surface roughness** helped swift-shaped cambered models only at Re 12,000-24,000 and hurt above that [R19]. It has not been tested on a flat plate in these sources.

### Open questions the literature does not settle

- Whether the centre-of-mass stability window found for a rectangular plate [R42] holds for a swept or tapered plate, and at Re 12,000-40,000 rather than 1,000-10,000.
- Whether cambering a plate closes or shifts that window. Gull wings, which are cambered, were stable in slope but untrimmable in most postures [R14].
- Lateral behaviour of a finless plate glider. The one natural example mostly spirals [R45].
- The pitch stability of the birds nearest this scale (swift, swallow, martin), which nobody appears to have measured.

---

## 8. Things I could not verify (consolidated)

- Swift body mass and wing loading in [R2]; pigeon, falcon and kestrel mass, span and area (tables behind paywalls or in images).
- Any glide polar for house martin, barn swallow, a small passerine with wings spread, a crow or rook, or a live gull.
- The text of Pelletier & Mueller 2000, Torres & Mueller 2004, Okamoto & Azuma 2011, Tucker 1993, Tucker 1995, Azuma & Okuno 1987, Videler et al. 2004, and the full text of Thomas & Taylor 2001 and Lentink et al. 2007.
- Plate thickness in Pelletier & Mueller 2000 (three different values in citing papers).
- Exact L/D peaks in McArthur's and Ananda's figures (plots only).
- Smith & Kroo 1993 and Müller & Patone 1998 (not located).
- The Harris' hawk gap-flight paper's year and details [R38].
- The pigeon study said to challenge the four-bar wing-folding model (not identified).
- The two citations in the brief that I could not match to a paper (Section 3.2).

---

## References

[R1] Rosén M, Spedding GR, Hedenström A (2007). Wake structure and wingbeat kinematics of a house-martin *Delichon urbica*. J. R. Soc. Interface 4:659-668. https://sites.usc.edu/drydenwt/files/2022/06/RSH_JRSI_07.pdf

[R2] Henningsson P, Hedenström A (2011). Aerodynamics of gliding flight in common swifts. J. Exp. Biol. 214:382-393. https://journals.biologists.com/jeb/article/214/3/382/33508/Aerodynamics-of-gliding-flight-in-common-swifts

[R3] Lentink D, Müller UK, Stamhuis EJ, de Kat R, van Gestel W, Veldhuis LLM, Henningsson P, Hedenström A, Videler JJ, van Leeuwen JL (2007). How swifts control their glide performance with morphing wings. Nature 446:1082-1085. https://doi.org/10.1038/nature05733 (abstract: https://research.wur.nl/en/publications/how-swifts-control-their-glide-performance-with-morphing-wings)

[R4] Hedrick TL, Pichot C, de Margerie E (2018). Gliding for a free lunch: biomechanics of foraging flight in common swifts (*Apus apus*). J. Exp. Biol. 221:jeb186270. https://hal-univ-rennes1.archives-ouvertes.fr/hal-01928514/file/Hedrick_et_al_-_J_Exp_Biol-2018.pdf

[R5] Tobalske BW, Hearn JWD, Warrick DR (2009). Aerodynamics of intermittent bounds in flying birds. Exp. Fluids 46:963-973. https://umimpact.umt.edu/en/publications/aerodynamics-of-intermittent-bounds-in-flying-birds/

[R6] Bruderer L, Liechti F, Bilo D (2001). Flexibility in flight behaviour of barn swallows (*Hirundo rustica*) and house martins (*Delichon urbica*) tested in a wind tunnel. J. Exp. Biol. 204:1473-1484. https://journals.biologists.com/jeb/article/204/8/1473/8803/

[R7] Rosén M, Hedenström A (2001). Gliding flight in a jackdaw: a wind tunnel study. J. Exp. Biol. 204:1153-1166. https://journals.biologists.com/jeb/article/204/6/1153/8689/Gliding-flight-in-a-jackdaw-a-wind-tunnel-study

[R8] KleinHeerenbrink M, Warfvinge K, Hedenström A (2016). Wake analysis of aerodynamic components for the glide envelope of a jackdaw (*Corvus monedula*). J. Exp. Biol. 219:1572-1581. https://doi.org/10.1242/jeb.132480

[R9] Lees JJ, Dimitriadis G, Nudds RL (2016). The influence of flight style on the aerodynamic properties of avian wings as fixed lifting surfaces. PeerJ 4:e2495. https://pmc.ncbi.nlm.nih.gov/articles/PMC5075716/

[R10] Pennycuick CJ (1968). A wind-tunnel study of gliding flight in the pigeon *Columba livia*. J. Exp. Biol. 49:509-526. https://journals.biologists.com/jeb/article/49/3/509/21318/A-Wind-Tunnel-Study-of-Gliding-Flight-in-the

[R11] Tucker VA, Parrott GC (1970). Aerodynamics of gliding flight in a falcon and other birds. J. Exp. Biol. 52:345-367. https://journals.biologists.com/jeb/article/52/2/345/21503/Aerodynamics-of-Gliding-Flight-in-A-Falcon-and

[R12] Videler J, Groenewold A (1991). Field measurements of hanging flight aerodynamics in the kestrel *Falco tinnunculus*. J. Exp. Biol. 155:519-530. https://journals.biologists.com/jeb/article/155/1/519/6382/Field-Measurements-Of-Hanging-Flight-Aerodynamics

[R13] Tucker VA, Heine C (1990). Aerodynamics of gliding flight in a Harris' hawk, *Parabuteo unicinctus*. J. Exp. Biol. 149:469-489. https://doi.org/10.1242/jeb.149.1.469

[R14] Harvey C, Baliga VB, Lavoie P, Altshuler DL (2019). Wing morphing allows gulls to modulate static pitch stability during gliding. J. R. Soc. Interface 16:20180641. https://pmc.ncbi.nlm.nih.gov/articles/PMC6364660

[R15] Withers PC (1981). An aerodynamic analysis of bird wings as fixed aerofoils. J. Exp. Biol. 90:143-162. https://journals.biologists.com/jeb/article/90/1/143/22971/An-Aerodynamic-Analysis-of-Bird-Wings-as-Fixed

[R16] Laitone EV (1997). Wind tunnel tests of wings at Reynolds numbers below 70 000. Exp. Fluids 23:405-409. https://doi.org/10.1007/s003480050128

[R17] McArthur J (2008). Aerodynamics of wings at low Reynolds numbers: boundary layer separation and reattachment. PhD dissertation, University of Southern California. https://sites.usc.edu/drydenwt/files/2023/05/mcarthur-thesis-2008.pdf

[R18] Sunada S, Sakaguchi A, Kawachi K (1997). Airfoil section characteristics at a low Reynolds number. J. Fluids Eng. 119:129-135. https://doi.org/10.1115/1.2819098 (DOI not checked; record seen at https://acnpsearch.tweb-dev.unibo.it/singlejournalindex/4655425)

[R19] Lentink D, de Kat R (2014). Gliding swifts attain laminar flow over rough wings. PLoS ONE 9:e99901. https://pmc.ncbi.nlm.nih.gov/articles/PMC4070913/

[R20] van Bokhorst E, de Kat R, Elsinga GE, Lentink D (2015). Feather roughness reduces flow separation during low Reynolds number glides of swifts. J. Exp. Biol. 218:3179-3191. https://doi.org/10.1242/jeb.121426 (abstract: https://eprints.soton.ac.uk/381461)

[R21] Videler JJ, Stamhuis EJ, Povel GDE (2004). Leading-edge vortex lifts swifts. Science 306:1960-1962. https://doi.org/10.1126/science.1104682

[R22] Lee S, Kim J, Park H, Jabłoński PG, Choi H (2015). The function of the alula in avian flight. Sci. Rep. 5:9914. https://pmc.ncbi.nlm.nih.gov/articles/PMC4423427/

[R23] Tucker VA (1993). Gliding birds: reduction of induced drag by wing tip slots between the primary feathers. J. Exp. Biol. 180:285-310. https://journals.biologists.com/jeb/article/180/1/285/6536/Gliding-Birds-Reduction-of-Induced-Drag-by-Wing

[R24] Tucker VA (1995). Drag reduction by wing tip slots in a gliding Harris' hawk, *Parabuteo unicinctus*. J. Exp. Biol. 198:775-781. https://journals.biologists.com/jeb/article/198/3/775/6908/Drag-Reduction-by-Wing-Tip-Slots-in-a-Gliding

[R25] Sachs G (2006). Effect of slotted wing tips on yawing moment characteristics. J. Theor. Biol. 239. https://portal.fis.tum.de/en/publications/effect-of-slotted-wing-tips-on-yawing-moment-characteristics/

[R26] van Dam CP (1987). Efficiency characteristics of crescent-shaped wings and caudal fins. Nature 325:435-437. https://www.nature.com/articles/325435a0

[R27] Cheney JA, Stevenson JPJ, Durston NE, Maeda M, Song J, Megson-Smith DA, Windsor SP, Usherwood JR, Bomphrey RJ (2021). Raptor wing morphing with flight speed. J. R. Soc. Interface 18:20210349. https://pmc.ncbi.nlm.nih.gov/articles/PMC8277465

[R28] Durston NE, Wan X, Liu JG, Windsor SP (2019). Avian surface reconstruction in free flight with application to flight stability analysis of a barn owl and peregrine falcon. J. Exp. Biol. 222:jeb185488. https://research-information.bris.ac.uk/en/publications/avian-surface-reconstruction-in-free-flight-with-application-to-f/

[R29] Thomas ALR, Taylor GK (2001). Animal flight dynamics I. Stability in gliding flight. J. Theor. Biol. 212:399-424. https://doi.org/10.1006/jtbi.2001.2387 (abstract: https://ora.ox.ac.uk/objects/uuid:39c37f27-3793-45e2-9b0e-156495ace658)

[R30] Usherwood JR, Cheney JA, Song J, Windsor SP, Stevenson JPJ, Dierksheide U, Nila A, Bomphrey RJ (2020). High aerodynamic lift from the tail reduces drag in gliding raptors. J. Exp. Biol. 223:jeb214809. https://pmc.ncbi.nlm.nih.gov/articles/PMC7033732

[R31] Song J, Cheney JA, Bomphrey RJ, Usherwood JR (2022). Virtual manipulation of tail postures of a gliding barn owl (*Tyto alba*) demonstrates drag minimization when gliding. J. R. Soc. Interface 19:20210710. https://pmc.ncbi.nlm.nih.gov/articles/PMC8833102

[R32] Evans MR (2003). Birds' tails do act like delta wings but delta-wing theory does not always predict the forces they generate. Proc. R. Soc. B 270:1379-1385. https://pmc.ncbi.nlm.nih.gov/articles/pmid/12965029

[R33] Chen C, Cheney JA, Usherwood JR, Bomphrey RJ, Song J (2025). The stability implications of drag minimization by tail action modelled in the gliding barn owl (*Tyto alba*). Journal not confirmed; record https://eprints.soton.ac.uk/509858 and data https://datadryad.org/dataset/doi:10.5061/dryad.tmpg4f59f

[R34] Carruthers AC, Thomas ALR, Taylor GK (2007). Automatic aeroelastic devices in the wings of a steppe eagle *Aquila nipalensis*. J. Exp. Biol. 210:4136-4149. https://doi.org/10.1242/jeb.011197 (DOI not checked; summary at https://intelligent-earth.ox.ac.uk/node/1030706)

[R35] Harvey C, Inman DJ (2022). Gull dynamic pitch stability is controlled by wing morphing. PNAS 119:e2204847119. https://pmc.ncbi.nlm.nih.gov/articles/PMC9477410

[R36] Harvey C, Baliga VB, Wong JCM, Altshuler DL, Inman DJ (2022). Birds can transition between stable and unstable states via wing morphing. Nature 603:648-653. https://pmc.ncbi.nlm.nih.gov/articles/PMC8942853

[R37] Durston NE, Windsor SP. Quantifying the flight stability of free-gliding birds of prey. SICB meeting abstract 138-8 (year not stated on the page). https://sicb.org/?p=8244

[R38] Weston, Zhu, Taylor, Harvey. Stability shifts in gliding flight: hawks morph from an unstable to stable state when navigating a gap. J. R. Soc. Interface (year not confirmed). News: https://engineering.ucdavis.edu/news/hawk-study-shows-potential-lessons-bird-flight

[R39] Sachs G (2005). Aerodynamic yawing moment characteristics of bird wings. J. Theor. Biol. 234:471-478. https://doi.org/10.1016/j.jtbi.2004.12.001

[R40] Sachs G (2005). Yaw stability in gliding birds. J. Ornithol. 146. https://portal.fis.tum.de/en/publications/yaw-stability-in-gliding-birds/

[R41] Sachs G (2007). Why birds and miniscale airplanes need no vertical tail. J. Aircraft 44:1159-1167. https://doi.org/10.2514/1.20175

[R42] Li H, Goodwill T, Wang ZJ, Ristroph L (2022). Centre of mass location, flight modes, stability and dynamic modelling of gliders. J. Fluid Mech. 937:A6. Open access. https://doi.org/10.1017/jfm.2022.89

[R43] Pomerenk O, Ristroph L (2025). Aerodynamic equilibria and flight stability of plates at intermediate Reynolds numbers. J. Fluid Mech. 1014:A24. https://arxiv.org/abs/2408.08864

[R44] Azuma A, Okuno Y (1987). Flight of a samara, *Alsomitra macrocarpa*. J. Theor. Biol. 129:263-274. https://doi.org/10.1016/S0022-5193(87)80001-2 (DOI not checked)

[R45] Certini D (2023). Flight of *Alsomitra macrocarpa*. PhD thesis, University of Edinburgh. https://era.ed.ac.uk/handle/1842/39840

[R46] Ajanic E, Feroskhan M, Mintchev S, Noca F, Floreano D (2020). Bioinspired wing and tail morphing extends drone flight capabilities. Sci. Robot. 5:eabc2897. Preprint https://arxiv.org/abs/2002.02421

[R47] Chang E, Matloff LY, Stowers AK, Lentink D (2020). Soft biohybrid morphing wings with feathers underactuated by wrist and finger motion. Sci. Robot. 5:eaay1246. https://doi.org/10.1126/scirobotics.aay1246 (DOI not checked)

[R48] Torres GE, Mueller TJ (2004). Low-aspect-ratio wing aerodynamics at low Reynolds numbers. AIAA J. 42:865-873. https://doi.org/10.2514/1.439

[R49] Ananda GK, Sukumar PP, Selig MS (2015). Measured aerodynamic characteristics of wings at low Reynolds numbers. Aerosp. Sci. Technol. 42:392-406. https://m-selig.web.engr.illinois.edu/pubs/AnandaSukumarSelig-2015-AeroSciTech-LowReWings.pdf

[R50] Okamoto M, Azuma A (2011). Aerodynamic characteristics at low Reynolds number for wings of various planforms. AIAA J. 49:1135-1150. https://doi.org/10.2514/1.J050071

[R51] Wing-folding anatomy, secondary sources: Sullivan C et al. (2010). The asymmetry of the carpal joint and the evolution of wing folding in maniraptoran theropod dinosaurs. Proc. R. Soc. B. https://pmc.ncbi.nlm.nih.gov/articles/PMC2880093 ; and a Springer abstract on automatic elbow-wrist coordination, https://link.springer.com/doi/10.1007/BF00574915

[R52] Interlocking wing-folding mechanism in extant birds (2023). Zoological Letters. https://doi.org/10.1186/s40851-023-00204-x (authors not confirmed)

[R53] Matloff LY, Chang E, Feo TJ, Jeffries L, Stowers AK, Thomson C, Lentink D (2020). How flight feathers stick together to form a continuous morphing wing. Science 367:293-297. https://doi.org/10.1126/science.aaz3358

[R54] Stowers AK, Lentink D (2015). Folding in and out: passive morphing in flapping wings. Bioinspir. Biomim. 10:025001. https://doi.org/10.1088/1748-3190/10/2/025001 (DOI not checked)

[R55] Pelletier A, Mueller TJ (2000). Low Reynolds number aerodynamics of low-aspect-ratio, thin/flat/cambered-plate wings. J. Aircraft 37:825-832. https://doi.org/10.2514/2.2676
