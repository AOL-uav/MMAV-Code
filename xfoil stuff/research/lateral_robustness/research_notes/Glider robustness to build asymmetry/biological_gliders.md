# Passive stability in biological gliders (birds, Alsomitra seed, gliding animals): what it says about a finless bird-planform glider that circles and swoops

Scope note: this file builds on, and does not repeat, `research/bird_research_notes.md` (Sections 3-4: Thomas & Taylor 2001, Harvey 2019/2022, Durston 2019, Sachs 2005-2007, Li et al. 2022 plate CoM window, Certini thesis) and `research/birds/planform_and_gliding.md`. Those findings are cited here only by reference [BRN §x] where needed for the argument. New material is mostly: Wei et al. 2022 (dihedral and spiral mode, flight-tested), Sachs 2008, Chang et al. 2024 PigeonBot II (finless yaw by tail reflex), Certini et al. 2026 Alsomitra straight vs helical, flying snakes, flying fish, Draco.

Evidence tags used on every claim: **[measured]** = peer-reviewed measurement on animals or physical models; **[model]** = peer-reviewed analysis/simulation; **[secondary]** = only abstract, dataset description, press or search summary read; **[hobbyist]**; **[speculation]** = my inference. Birds are actively controlled; claims are marked **passive** or **active** where it matters.

Source-access limits: I could not read the full texts of Sachs 2005/2007, Thomas & Taylor 2001, Azuma & Okuno 1987, Park & Choi 2010, Durston et al. 2019 (lateral numbers), Yeaton et al. 2020, Clark et al. 2021. Two PDFs fetched (ECCOMAS 2022 Certini abstract; Clark et al. 2021 ICB) arrived as compressed binary and were not parsed; nothing was saved to `sources/` and nothing was executed.

## 1. Are birds passively laterally/directionally stable in gliding?

### Takeaway
Analysis says a bird-like wing with no fin *can* be directionally stable enough at small scale, provided the outer wing is swept back, and that the spiral mode is set mainly by spanwise dihedral; but the only measured-shape bird analyses found lateral stability that varies flight to flight, and the one finless bird robot needed active reflexes to damp Dutch roll. "Birds are passively laterally stable" is not established; "a swept, dihedralled finless wing can be" is supported by models and one flight test.

### Cited Findings
- Directional (yaw) stiffness of a finless bird wing comes from sweep, mainly in the outer wing; it grows with lift coefficient and peaks near best glide; the same gull wing unswept gives much smaller yawing moments **[model; passive]** — Sachs 2005, J. Theor. Biol. 234:471, summarised in [BRN §3.4] — [doi](https://doi.org/10.1016/j.jtbi.2004.12.001)
- Small fliers achieve adequate Dutch-roll frequency even with very small static directional stability; required yaw damping falls with size, so the wing alone can supply it; removing the fin only partly affects the spiral mode; explicitly extended to aircraft of centimetre scale **[model, secondary: abstract]** — [Sachs 2007, J. Aircraft 44:1159](https://doi.org/10.2514/1.20175); [TUM record](https://portal.fis.tum.de/en/publications/why-birds-and-miniscale-airplanes-need-no-vertical-tail/)
- Sachs 2008: birds' lack of a fin affects yaw stability and damping and therefore the Dutch roll; "suitable wing features" can give the Dutch roll adequate frequency and damping; birds have a short roll-mode time constant and high achievable roll rates **[model, secondary: abstract]** — [Sachs 2008, VSDIA proc. pp. 15-26](https://portal.fis.tum.de/en/publications/what-can-be-learned-from-the-lateral-directional-stability-charac/)
- Frigatebird model (span 2.19 m, AR 11.4, 1.5 kg; base dihedral inner 38 deg, middle -3 deg, outer 0 deg), 9,724 dihedral distributions at 8-12 m/s: **all** had convergent Dutch roll (76.5 % met level-1) and convergent roll mode; only **60.7 %** had a convergent spiral; dihedral distribution is "the key factor" for a convergent spiral; roll mode barely affected by dihedral **[model; passive]** — [Wei, Lin, Huang, Song, Smith 2022, Aircr. Eng. Aerosp. Technol. 94(11):29-44, doi 10.1108/AEAT-12-2021-0364](https://www.emerald.com/insight/content/doi/10.1108/AEAT-12-2021-0364/full/html)
- Same paper, finless bungee-launched unpowered model matched to the frigatebird: hands-off glide at about 12 m/s stayed laterally convergent; identified Dutch roll at 10 m/s: wn 1.87 rad/s, damping 0.445 (theory 2.63 rad/s, 0.179); authors blame build error and the neglected body side force/Cn_beta for the mismatch **[measured, flight test; passive]** — [Wei et al. 2022](https://www.emerald.com/insight/content/doi/10.1108/AEAT-12-2021-0364/full/html). Span and mass of the test aircraft not in the text read.
- Free-gliding barn owl and peregrine (measured shape, CT mass, AVL): lateral-directional stability, especially Dutch roll and spiral, varied between flights and species; peregrine changed camber, twist, sweep and dihedral with speed, changing stability; authors suggest neural feedback may be too slow, implying passive stabilisation through structural compliance **[model on measured geometry; secondary for lateral numbers]** — [Durston et al. 2019, J. Exp. Biol. 222:jeb185488](https://research-information.bris.ac.uk/en/publications/avian-surface-reconstruction-in-free-flight-with-application-to-f/); [SICB abstract](https://sicb.org/?p=8244)
- PigeonBot II (finless, real feathers): its lack of a vertical tail causes a Dutch roll *instability* that it damps by reflexive wing and tail morphing; wings handle larger roll disturbances, tail handles small yaw disturbances; six controlled DOF (four tail, two wing); autonomous outdoor flight **[measured robot; active]** — [Chang, Chin, Lentink 2024, Sci. Robot., doi 10.1126/scirobotics.ado4535](https://citedrive.com/en/discovery/bird-inspired-reflexive-morphing-enables-rudderless-flight); [Physics Today](https://physicstoday.aip.org/news/birdlike-robot-flies-steady-without-a-vertical-tail)
- Raptors glide with wings held below the shoulder (anhedral) [BRN §3.4, Cheney 2021] — reduces dihedral effect by aircraft reasoning **[measured geometry; function not shown]**.

### Inferences
- [speculation] The two analyses do not conflict: Sachs/Wei concern swept, high-dihedral or high-lift wings where Dutch roll converges; PigeonBot is a pigeon-shaped, comparatively unswept wing with a body, where it did not. For a fixed glider the transferable recipe is *outer-wing sweep for Cn_beta + dihedral for spiral convergence*; neither alone is enough.
- [speculation] Circling caused by build asymmetry is a trim problem more than an instability: even a laterally stable glider with a small built-in rolling/yawing moment settles into a steady turn. Stability sets *how tight* the turn is per unit of asymmetry: larger Cl_beta (dihedral) and Cn_beta (sweep) both resist the bank/sideslip the asymmetry creates, so the turn radius grows. A divergent spiral instead lets a tiny asymmetry tighten into a spiral dive. The project's flat Zanonia AVL deck shows Cl_beta -0.059, Cn_beta +0.008 (`known_good/zanonia/RESULTS.md`): weakly stable directionally, so modest asymmetry would show up as a turn.
- [speculation] Frigatebird spiral convergence needed tens of degrees of inner-wing dihedral; a flat plate with zero dihedral sits on the spiral-divergent side unless sweep supplies effective dihedral (swept wings at positive CL give Cl_beta < 0 proportional to CL·sin(2Λ) in standard theory).

### Gaps
- No measured or modelled lateral-directional stability of a swift, swallow or martin (the glider's scale) was found.
- Wei et al.'s test aircraft dimensions and Cn_beta/Cl_beta numbers were not in the readable text; Sachs's quantitative criteria (required Cn_beta vs size) not read.

## 2. Bird tail roles without a vertical fin (yaw, twist, spread)

### Takeaway
In birds, yaw correction without a fin is largely active: the tail is tilted/twisted in proportion to yaw rate to make side force; passively the tail's yaw contribution is small and the wing's swept outer panel does the work. In pitch, the measured bird tail lifts (not down-loaded) and is a drag/trim device, not an aircraft-style stabiliser [BRN §2 R30-R33].

### Cited Findings
- Birds set tail tilt according to yaw rate; the tilted tail's side lift gives a restoring yaw torque; feathers return to neutral once flight steadies; four tail motions (rotation/twist, lateral deviation, pitch, spread) **[measured behaviour + robot; active]** — [Physics Today on Chang et al. 2024](https://physicstoday.aip.org/news/birdlike-robot-flies-steady-without-a-vertical-tail)
- Gull model: tail had a minor stabilising effect in pitch and could not trim the bird by itself **[model]** — Harvey & Inman 2022 PNAS [BRN §3.2](https://pmc.ncbi.nlm.nih.gov/articles/PMC9477410)
- Tail of owls/goshawk carries positive lift and fits a minimum-profile-drag model **[measured]**; drag-minimising barn-owl model is unstable at all speeds **[model]** — [BRN §2, R30, R33]
- Thomas & Taylor 2001 rule for a stable flier: a surface behind the wing set nose-down relative to it **[model]** — [doi 10.1006/jtbi.2001.2387](https://doi.org/10.1006/jtbi.2001.2387), [BRN §3.2]
- Sachs 2007: the wing alone can provide the required yaw moment and damping **[model, secondary]** — [J. Aircraft](https://doi.org/10.2514/1.20175)

### Inferences
- [speculation] For the glider's short, body-blended, bent-up tail: the bent-up portion acts as a reflex (nose-down set) surface, consistent with Thomas & Taylor's stable-flier rule, but a short, flat-ish tail near the CG has a tiny yaw arm; any yaw stability must come from wing sweep (and, weakly, from upturned tail edges acting as small fins). A *twisted* tail (one side higher) is exactly the bird's yaw-control input, so tail-bend asymmetry is a prime candidate for the circling.

### Gaps
- No measurement of passive (feathers fixed) yaw derivatives of a bird tail found.

## 3. Alsomitra (Zanonia) seed: straight vs circling, and pitch oscillation

### Takeaway
The seed is the natural finless, uncontrolled glider, and most seeds circle: 11 of 15 tracked seeds descended helically, 4 flew straight with oscillation. A 2026 dataset ties straight flight to low planform circularity plus a particular chordwise CoM eccentricity, and finds two straight-flight regimes, steady gliding and dynamically unstable "bounding" (phugoid-like swoops), with peak efficiency near the boundary.

### Cited Findings
- 15 seeds tracked by depth cameras; 11 helical, 4 straight oscillatory paths; motion modelled as tumbling wing coupled with fluttering wing; preferred flight orientation for every seed (not a symmetric 2D plate); spanwise membrane deformation under load shifts the aerodynamic centre; low-order models with non-uniform mass reproduce vertical-plane oscillations; flight regime in MAV range **[measured; passive]** — [Certini 2023 PhD thesis, Edinburgh](https://era.ed.ac.uk/handle/1842/39840); [ECCOMAS 2022 abstract (secondary via search)](https://congress.cimne.com/eccomas2022/admin/files/fileabstract/a812.pdf)
- Planform circularity is the principal morphological determinant: high circularity -> helical descent, low circularity -> straight flight (rare); circularity + chordwise CoM eccentricity define the behaviour space; among straight flyers two regimes: stable gliding and dynamically unstable bounding; 2D-model efficiency peaks in the bounding regime and measured eccentricities cluster there, but real seeds are most efficient while gliding, which the authors attribute to 3D shape stabilising the 2D instability **[measured + model; secondary: dataset abstract]** — [Certini, Cummins, Kessler, Komendantskaya, Yang, Giorgio-Serchi, Viola 2026, Edinburgh DataShare doi 10.7488/ds/8109](https://datashare.ed.ac.uk/handle/10283/9191). Related journal paper not identified.
- Azuma & Okuno 1987: CoM near quarter-chord, glide ratio 3-4 **[model/measured, secondary via Li et al. 2022]** — [BRN §4](https://doi.org/10.1016/S0022-5193(87)80001-2). Popular descriptions of a phugoid cycle (climb, stall, dip, accelerate) and 12 deg descent / 0.41 m/s are unverified **[secondary]** — [HandWiki](https://handwiki.org/wiki/Biology:Alsomitra_macrocarpa)
- Thin-plate gliders: steady glide only for CoM in a narrow window (~21-28 % chord for a rectangle); aft of it they flutter/bound, forward they dive **[measured]** — Li et al. 2022 JFM [BRN §4](https://doi.org/10.1017/jfm.2022.89)
- Hobbyist Zanonia foam glider flies "pretty straight" with "a little wobble"; project AVL gives static margin about 1 % MAC at the marked CG **[hobbyist; model]** — `known_good/zanonia/README.md`, `RESULTS.md`

### Inferences
- [speculation] The seed is the closest biological analogue to the glider, and its lesson is uncomfortable: a finless crescent flier circles by default, and straight flight is a narrow, shape- and CoM-dependent outcome. Pitch "swoops" correspond to the seed's bounding regime, which sits at aft CoM / small static margin; moving CoM forward toward the steady-glide regime is the biologically and experimentally supported cure (Li et al., Certini), at some cost in glide ratio.
- [speculation] Low circularity = more elongated, higher-aspect, more swept crescent; that is the direction the bird planform already goes.

### Gaps
- Why high circularity causes helical flight (mechanism) was not in the readable text; no Cn_beta/Cl_beta measurement of a seed found. The search summary claim "stable in pitch and roll but not directionally" [BRN §4] remains unverified.

## 4. Gliding animals (snakes, fish, Draco, squirrels, colugos)

### Takeaway
Most gliding animals are reported to rely on active posture/tail control; passive stability evidence exists mainly in pitch (flying fish fin "biplane"). None provides a passive lateral-stability recipe beyond what birds and the seed give.

### Cited Findings
- Flying snake: without undulation simulated glides become unstable (plummet or tumble); undulation enhances rotational stability even without feedback control **[model on 3D motion capture; open-loop active motion]** — [Yeaton, Ross, Baumgardner, Socha 2020, Nature Physics, doi 10.1038/s41567-020-0935-4](https://rossaoe.s4.es.cloud.vt.edu/papers/yeaton-ross-baumgardner-socha-2020.pdf); [APS 2018 abstract](https://meetings.aps.org/Meeting/DFD18/Event/332329)
- Flying fish (dried Cypselurus hiraii, wind tunnel, 6-axis balance, body -15 to 45 deg): pectoral fins raise L/D and longitudinal static stability; lift and L/D fall as pectoral-fin lateral dihedral increases; pectoral + pelvic fins together gave longer glide and more longitudinal static stability than pectoral alone (a tandem/"biplane" layout) **[measured on fixed specimens; passive]** — [Park & Choi 2010 J. Exp. Biol. 213:3269, doi 10.1242/jeb.046052 (via Nature highlight)](https://www.nature.com/articles/467254d); [APS 2006 abstract](https://meetings-archive.aps.org/dfd/2006/la/1/); [IUTAM 2007 paper (secondary)](https://eng-web77-v02.ocio.monash.edu/intranet/proceedings/iutam2007/pages/pdf/S1_Park.pdf). Max L/D about 4.7 cited by a 2019 CFD follow-up **[secondary]**.
- Draco: 3D-printed models from glide photos, wind tunnel lift/drag, simulation with PID tail control: active tail improves glide distance and stability **[model; active]** — [Clark et al. 2021, Integr. Comp. Biol.](https://biomechanics.ucr.edu/Clark%20et%20al%202021%20ICB.pdf); low-AR patagia give gradual stall via tip vortices **[measured/CFD, secondary]** — [Lau, Ma, Huang 2023 Phys. Fluids (EBSCO)](https://www.ebsco.com/articles/zoology/540fb491-37d2-5b97-b5f6-7f13bae13149/the-aerodynamic-effects-of-forelimb-pose-on-the-gliding-flight-of-draco-lizards)
- Flying squirrels (Glaucomys) are thought to use the tail to stabilise pitch **[secondary, via Clark et al. 2021 summary]**; squirrel-inspired drone: passive membrane deformation and tail/body morphing improved stability, limb morphing improved roll stability **[measured robot; secondary]** — [TU Delft repository](https://repository.tudelft.nl/record/uuid:27515743-028c-42ef-9d82-f33bf1244934)
- Colugos: no stability study found.

### Inferences
- [speculation] Flying fish support a tandem lifting-surface layout (fore + aft lifting surfaces) for passive pitch stability; the glider's bent-up tail is the opposite sign (nose-down reflex) and works like a tailless flying wing instead.

### Gaps
- No passive lateral stability measurements for any gliding mammal, lizard or frog were found.

## 5. Quantitative morphology numbers available

### Takeaway
Few numbers transfer; the useful ones are frigatebird dihedral distributions, swift sweep range and the seed/plate CoM windows.

### Cited Findings
- Frigatebird: dihedral inner 38 deg (range tested 28-44), middle -3 (-9 to 12), outer 0 (-6 to 19); 60.7 % spiral-convergent **[model]** — [Wei et al. 2022](https://www.emerald.com/insight/content/doi/10.1108/AEAT-12-2021-0364/full/html)
- Swift sweep settings "5, 30, 50 deg"; outer-wing sweep is what matters for yaw **[measured geometry; model]** — [BRN; planform_and_gliding.md], [Sachs 2005](https://doi.org/10.1016/j.jtbi.2004.12.001)
- Gull phugoid 0.45-1.10 rad/s, heavily damped; short period 8.9-41 rad/s **[model]** — [Harvey & Inman 2022](https://pmc.ncbi.nlm.nih.gov/articles/PMC9477410)
- Owl/peregrine pitch divergence time-to-double typically under 50 ms **[model on measured shape, secondary]** — [Durston et al. 2019](https://research-information.bris.ac.uk/en/publications/avian-surface-reconstruction-in-free-flight-with-application-to-f/)
- Plate CoM window 21-28 % chord; Alsomitra CoM near 25 % chord **[measured]** — [Li et al. 2022](https://doi.org/10.1017/jfm.2022.89)

### Inferences (implications for the 0.15-0.5 m flat, swept, finless glider) — all [speculation] built on the above
1. Circling: treat as asymmetry × lateral stiffness. Raise Cn_beta with outer-panel sweep (and small upturned tips acting as fins, as on the Zanonia toy) and raise Cl_beta with dihedral (tip curl-up or polyhedral); the Wei result says spiral convergence specifically needs dihedral. Then remove the dominant asymmetry, likely tail twist or tip twist (a bird's tail twist is its yaw *control*).
2. Pitch swoops: the seed and plate evidence place swoops (bounding) at aft CoM / near-zero margin; the project's Zanonia AVL margin of about 1 % MAC is in that zone. Move CoM forward a few % MAC and accept lower glide ratio; bird phugoids are damped mainly because their stable configurations have real margin.
3. Do not count on bird-like geometry for passive stability: measured gliding birds were pitch-unstable and lateral stability varied; the finless robot pigeon needed reflexes.

### Gaps
- No tail-volume numbers for birds' passive yaw contribution; no swift dihedral measurement; no stability data for finless bird-outline rigid gliders at Re 10^4.
