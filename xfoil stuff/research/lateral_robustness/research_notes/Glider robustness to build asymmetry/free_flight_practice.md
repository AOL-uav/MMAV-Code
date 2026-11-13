# Free-flight aeromodelling practice: lateral-directional trim and robustness to warps

Evidence tags: [MEAS] measured or peer-reviewed; [MODEL] model or analysis; [HOBBY] hobbyist practice (magazine columns, forums, plans); [SPEC] speculation or my inference.
Nearly all free-flight sources are [HOBBY], meaning practitioner columns in AMA *Model Aviation*. The Beron-Rawdon series is the most analytical of them: it is engineering-style analysis plus flight tests of one RC sailplane, so it counts as [MODEL] + [HOBBY] test, not peer-reviewed.
No files were downloaded to `sources\`. One FAI PDF was fetched by the tool and cached automatically outside the project, and it was unreadable. Nothing was executed.

## Q1. How do free-flighters handle warps and asymmetry? Trim procedure, and which adjustments are roll-type vs yaw-type

### Takeaway
Free-flight practice treats unintended asymmetric warps as defects. They steam them out or pin them on a shim. The circle is then set deliberately with a few small, separate "knobs": inboard wash-in or tip clay (roll-type), stab tilt (a yaw/roll coupling, the preferred glide-turn knob), and rudder tab (yaw-type, which practitioners avoid for glide trim because it is speed-sensitive). The quantitative anchor is a typical F1B glide-circle setup that uses only **0.5° wash-in on one main panel**, which brackets the 0.1-0.3° figure in the project's model study.

### Cited Findings
- [HOBBY] Remove warps by steaming: hold the surface aligned while it cools. Keep about 1/8 in. of tip washout and eliminate all other warps. — [Model Aviation, FF Old Timers](https://library.modelaviation.com/article/free-flight-old-timers-45)
- [HOBBY] Case report: a glider grew 1/8 in. of unintended right-wing wash-in, which pushed it into a gentle left turn. The fix was to mist the wing and pin it to the board on a washout shim. — [Model Aviation, Free Flight Sport 2003/06](https://library.modelaviation.com/article/free-flight-sport-200306) (via search snippet; full page not fetched)
- [HOBBY] Typical F1B (Wakefield) glide setup: right main panel 0.5° wash-in, left main panel flat, both tip panels 0.5° washout, plus slight right rudder (often an all-moving fin "because it is less speed-sensitive"). — [L. Joyner, FF Duration, Model Aviation 2005/08](https://library.modelaviation.com/article/ff-duration-30)
- [HOBBY] Wash-in and rudder can be balanced against each other. Slight rudder can offset slight wash-in, or large amounts of both can offset each other. Indoor models in small sites use "extreme" wash-in plus lots of rudder, which works but is inefficient; outdoor models use the minimum of each. — [Joyner 2005/08](https://library.modelaviation.com/article/ff-duration-30)
- [HOBBY] Tip weight as a roll-type trim: a small clay blob under the inboard tip adds glide turn with little effect on the climb, because CG shifts matter more at low speed. — [Joyner 2005/08](https://library.modelaviation.com/article/ff-duration-30)
- [HOBBY] "Reverse warp" setup: slight inboard washout with slight rudder gives an open glide that tightens in thermals and straightens in sink. It must be flown in open circles, because a tight glide turn leads to a spiral dive. — [Joyner 2005/08](https://library.modelaviation.com/article/ff-duration-30)
- [HOBBY] HLG trim (J. Haught, 1995): "Avoid using the rudder for glide trim; it is speed-sensitive." Instead, set the glide circle with built-in stab tilt (left side high, seen from the rear). Pitch trim is done by bending the rear ~1/2 in. of the stab about 1/32 in., and nose clay is adjusted in pencil-eraser-sized amounts. — [Model Aviation 1995/06, Hand-Launched Gliders Part II](https://library.modelaviation.com/article/hand-launched-gliders-part-ii)
- [HOBBY] Stab tilt of 3-5° for the glide turn. Rudder trim only enough to stop an abrupt yaw, because too much "can cause an unrecoverable dive." — [D. DeLoach, Making FF models fly, Model Aviation 2020/12](https://www.modelaviation.com/node/2076)
- [HOBBY] Glide trim is tested on a 10-20 ft hill on a calm morning or evening, aiming for a floating glide just short of the stall. — [DeLoach 2020](https://www.modelaviation.com/node/2076)
- [HOBBY] Catapult glider tip (McKiou): if launch and turn won't both work, add wash-in to the left tip by bending the trailing edge down on the outer 1/4 of the wing, but too much causes tip stall. He uses about 15° dihedral. — [NAR, Kevin McKiou's Glider Trimming Tips](https://nar.org/?p=2840) (snippet)
- [HOBBY] Washout must be equal on both tips "otherwise, a roll or aileron effect is created." — [AMA Flight School, basic glider physics: washout](https://www.amaflightschool.org/diy/basic-glider-physics-washout)
- [MODEL]/[HOBBY] Beron-Rawdon's first spiral-stability test step: "Ensure the model has no asymmetric warps and neither wing is heavier than the other." — [Beron-Rawdon, Spiral Stability and the Bowl Effect, Model Aviation 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [HOBBY] Horten (1949), on small tailless gliders: build twist "in excess, to compensate for deficiencies" from amateur construction. Losses from building errors "will not be able to be avoided without using special assembling and checking devices." — [R. Horten, "Tailless" Small Gliders, 1949, TWITT reprint](https://www.rcsoaringdigest.com/TWITT/Horten_Oct_49.html)

### Inferences
- [SPEC] Roll-type knobs are wing-half incidence (wash-in/out), tip clay, and the dihedral break. Yaw-type knobs are the rudder/tip-fin tab and sweep asymmetry. Stab tilt is mixed: it tilts the tail lift vector sideways and so produces a yaw moment, but practitioners report it as less speed-sensitive than rudder. The 1995 and 2005 columns agree that rudder-only trim changes with speed. That matters for this project's nose-down drop, where speed varies a lot before the glide settles.
- [SPEC] Free-flight practice implies a built-in roll asymmetry of about 0.5° is a deliberate turn setting on large, spirally stable models. A 0.1-0.3° accidental difference producing 12-32 m circles on small, low-spiral-margin models is therefore plausible, not anomalous.
- [SPEC] The practical free-flight answer to warps is to measure and correct each airframe (steam or shim, then test-glide). It is not to design them away, and that approach does not transfer to a "passive, robust to manufacturing variation" requirement without some per-airframe trim step.

### Gaps
- No source gave measured glide-circle diameter as a function of wash-in for any model class.
- The 2003/06 Free Flight Sport case was seen only as a snippet.

## Q2. Dihedral, polyhedral and tip-dihedral rules; spiral-stability coefficient; fin area and moment arm

### Takeaway
The free-flight community's quantitative rule is Beron-Rawdon's spiral-stability number **SS = EDA(deg) × (lv/b) / CL**. SS ≥ ~5.7 means "almost certain" spiral stability, and ~4.3 was marginal. Free-flight models use EDA 5-15°, lv/b 0.35-1.0, and fin area 5-8% of wing area. Tip dihedral counts for much more than its share of span: outer half ≈ 0.65 of the roll moment.

### Cited Findings
- [MODEL] Definition: SS = (EDA × lv/b) / Cl. "When SS is about 5.7 or greater the plane is almost certain to be spirally stable." Near 5.7 it depends mainly on vertical-tail size. Well below 5.7 the model is almost sure to be unstable. A test sailplane was acceptable at SS ≈ 4.3. — [Beron-Rawdon 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [MODEL] Typical values: EDA 10-15° (extremes 5-20°). lv/b is 0.25-0.35 for RC sailplanes and 0.35 up to 1.0 for free flight. Thermalling Cl is 0.6-1.0 for RC and about 0.4-1.4 for free flight. Higher Cl lowers SS, and Cl ∝ 1/V². Wing loading, aspect ratio, scale and ballast do not change SS at constant Cl. — [Beron-Rawdon 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [MODEL] Worked example: EDA 15° with lv/b 0.4 is stable for Cl ≤ ~1.05. Low-dihedral (3°) aileron ships would need lv/b > 1.0 or artificial stabilisation. — [Beron-Rawdon 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [HOBBY flight test] Tutor 2 m sailplane (lv/b 0.324): EDA 15° tracked well (slight Dutch roll), 12° was just stable, and 5.7° rolled into a circle over ~15 s and settled at ~20° bank. At Cl ≈ 0.4 (faster) the 5.7° wing flew straight. — [Beron-Rawdon 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [MODEL] EDA is defined as the V-dihedral angle giving the same roll moment per unit sideslip. To compute it, multiply each panel's spanwise roll-moment fraction by that panel's dihedral (measured from horizontal) and sum. Break at 0.5 semi-span: outer panel fraction 0.65, inner 0.35. So a flat centre with 10° tips gives EDA = 6.5°, and 5° inboard/10° outboard gives 8.25°. — [Beron-Rawdon, Dihedral Part 1, Model Aviation 1988/08](https://library.modelaviation.com/article/dihedral)
- [MODEL] Sweep has no EDA. Its roll-due-to-sideslip scales with total lift (∝ CL and g-load), is zero at 0 g, and reverses when inverted. — [Beron-Rawdon 1988/08](https://library.modelaviation.com/article/dihedral)
- [HOBBY estimate] Foote (1956): sweepback is ~10-15% as effective as equal dihedral at high speed but ≥30% near the stall. It also adds weathercock yaw stability through tip drag. "Don't go to extremes." — [D. Foote, Catching Thermals, Air Trails Annual 1956](https://airplanesandrockets.com/magazines/air-trails/catching-thermals-air-trails-annual-1956.htm)
- [MODEL] Spread dihedral beats concentrated: a tip/main dihedral ratio of about 3 with the break at mid-semispan is "near-optimal." The performance cost is tiny: a V of 10° per side loses ~0.6% of best L/D, and 15° loses ~1.3%. Thermal-sailplane recommendation: EDA ≥ 10° (15° preferred), vertical-tail arm 35-45% of span, vertical-tail area 5-6% of wing area, light tips and tail. — [Beron-Rawdon, Dihedral Part 4, Model Aviation 1988/11](https://library.modelaviation.com/article/dihedral-3)
- [HOBBY] Free-flight dihedral 5-15°, with calm-air and indoor models needing less. Fin area 5-8% (of wing area, though the article doesn't state the reference). Ideal fin is ~5% larger than the size that gives a mild Dutch roll. Too big a fin causes spiral divergence and wandering, so a larger fin needs more dihedral. Horizontal tail volume for duration models is 1.0-1.8. — [DeLoach 2020](https://www.modelaviation.com/node/2076)
- [MODEL] Small verticals gain extra spiral stability from differential (tip) drag yawing outboard, and side area forward of the CG also helps spiral stability. Circling flight with added pitch trim raises speed, which lowers CL and raises SS. "Unstable models can become stable at a smaller circle radius and higher speed." — [Beron-Rawdon 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [MEAS, wind tunnel free-flight] NASA Langley 1/10-scale tailless glider: more vertical area improved lateral stability, Cnβ ≥ ~0.002/deg was judged necessary, and lateral damping was very low. — [NASA NTRS 20090026465](https://ntrs.nasa.gov/citations/20090026465) (search snippet only)

### Inferences
- [SPEC] By Beron-Rawdon's rule, a flat (EDA 0°) flying wing has SS ≈ 0 from dihedral. Any spiral stability must come from sweep (CL-proportional), from tip fins sitting above the CG, and from tip drag. Tip fins on a short lv/b (~0.1-0.2 for a flying wing) are a weak lever. Flat wings at glide CL ~0.5-0.8 therefore sit far below the free-flight SS ≥ 5.7 band, which matches the observed near-neutral spiral mode and the large sensitivity to 0.1-0.3° twist.
- [SPEC] Illustration only; the SS rule was derived for tailed models. To reach SS ≈ 5.7 at CL 0.6 with lv/b ≈ 0.15 needs EDA ≈ 23°. That is impractical, so a tailless flat wing probably cannot reach free-flight-grade spiral stability from dihedral alone. Bent-up tips (tip dihedral on the outer half counts 0.65×) are the most efficient way to add EDA.
- [SPEC] Spiral stability does not by itself make the model fly straight when there is a built-in rolling moment. It sets how tight the equilibrium circle is for a given asymmetry. More SS means a larger radius per degree of twist.

### Gaps
- I found no published SS calibration for tailless models or for sweep/tip-fin contributions.
- Martin Simons, Pressnell, Hartill and Zaic texts were not reachable online in this pass.

## Q3. Why free-flight models circle deliberately, and what it takes to fly straight

### Takeaway
Free-flight models circle on purpose to stay inside the field and in thermals, and because a gently turning spirally-stable model is easier to trim than a perfectly straight one. The only free-flight class that flies straight, F1E slope gliders, uses an active **magnetic-compass rudder**, and its practitioners say asymmetric wing warps hurt straight flight.

### Cited Findings
- [HOBBY] Beron-Rawdon: a model with adequate spiral stability "holds a straight line indefinitely" and does not tighten when trimmed for a circle, while a low-stability one "spirals in." Few full-size aircraft are spirally stable at typical CL; they rely on the pilot. — [Beron-Rawdon 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [HOBBY] For best endurance the glide circle should be "as large as possible." — [Joyner 2005/08](https://library.modelaviation.com/article/ff-duration-30)
- [HOBBY] F1E: a strong magnet on a free pivot moves the rudder. The rudder is aligned to the intended course before launch, and "if the model deviates from the set course during flight, the compass will compensate." It was introduced by Hans Gremmer in 1951 and has been the core of F1E since 1977. — [FAI CIAM Flyer 4-2024](https://fai.org/sites/default/files/ciam_flyer_4-2024-print_ok.pdf) and [CIAM Flyer 5-2015](https://fai.org/news/ciam-flyer-5-2015-f1e-fly-fai-championship-class) (both via search snippets; the PDF and page could not be read)
- [HOBBY] The CIAM 5-2015 flyer reportedly says asymmetric wing warps hurt straight flight, and that dedicated F1E designs beat converted F1A gliders. — [CIAM Flyer 5-2015, search snippet](https://fai.org/news/ciam-flyer-5-2015-f1e-fly-fai-championship-class)
- [HOBBY] A German free-flight form uses a weather-vane arrangement on the rudder to keep a straight line. — [CIAM Flyer 4-2024, snippet](https://fai.org/sites/default/files/ciam_flyer_4-2024-print_ok.pdf)
- [HOBBY] Chuck-glider guidance stresses tail alignment "so the plane flies straight" and dihedral to self-level. — [Flite Test simple chuck gliders](https://flitetest.com/articles/how-to-make-brilliantly-simple-chuck-gliders)
- [HOBBY, low quality] Paper-airplane guides attribute veer mainly to fold asymmetry and fix it with tiny trailing-edge bends, one change at a time. No quantitative data. — [Fold'N Fly, steering](https://cdn.foldnfly.com/lounge/steer-aim.php)

### Inferences
- [SPEC] Free flight has no passive design rule that guarantees straight flight from an unchecked airframe. Straight flight comes from per-model trimming (a hill glide, then bend tabs) or from an active heading reference (a magnet). For a drop-launched batch of gliders, a likely passive compromise is high spiral margin plus one adjustable bendable tab per airframe, set by a hand/hill glide test.
- [SPEC] Accepting a large, deliberate, *known-direction* circle is also an option. It is the standard free-flight strategy, and it makes the path predictable even though it is not straight.

### Gaps
- No straight-glide distance competition rules or data were found, beyond paper-airplane distance records, which were not researched.

## Q4. Pendulum and other passive mechanical heading devices

### Takeaway
Pendulums have been used on scale free-flight models for roll correction, with mixed results. In a coordinated turn the pendulum is centred by centrifugal force, so it senses sideslip and transients, not bank. Practitioners recommend driving ailerons rather than rudder. No miniature (gram-scale) data were found.

### Cited Findings
- [HOBBY] Haight (1980) used roll pendulums driving ailerons, including twin horizontal pendulums in the wings with up-only aileron on the high wing. They worked at the 1977 Nationals, widening a tightening turn. He warns against rudder pendulums, which can over-correct into a spiral, and recommends heavy wire with a 15-20° hang below the pivot. — [B. Haight, The Pits and the Pendulum, Model Aviation 1980/04](https://library.modelaviation.com/article/pits-and-pendulum)
- [HOBBY] Known failure: "At speed in a turn, centrifugal force pulls the pendulum toward neutral, leaving the controls centered." A first fast installation failed this way. Moulton's warning that many pendulum fits gave no benefit is quoted. — [Haight 1980](https://library.modelaviation.com/article/pits-and-pendulum)
- [HOBBY] A 1999 column says rudder is the easiest surface to drive, but spiral instability, not yaw, is what causes trouble, so it recommends ailerons. A 1998 Kinner Sportster rudder pendulum allowed a wide glide circle and only picked up a low wing when bank was excessive. — [Model Aviation, Free Flight Sport and Scale 30 (1998/10)](https://library.modelaviation.com/article/free-flight-sport-and-scale-30) (snippet)
- [MODEL/HOBBY] An RC gyro on yaw at ~50% gain made a zero-dihedral sailplane fly straight and straighten out of circles. At 100% gain it oscillated at ±15° and 1 Hz. — [Beron-Rawdon 1990/09](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)
- [HOBBY] Dethermalisers (pop-up tail on a timer or RC) end flights. They are not heading devices. — [CIAM Flyer 5-2015 snippet](https://fai.org/news/ciam-flyer-5-2015-f1e-fly-fai-championship-class)
- [HOBBY patent] Free-floating, pivoted wing-tip fins with stops have been proposed for tailless aircraft. — [US2539357A](https://patents.google.com/patent/US2539357)

### Inferences
- [SPEC] In a steady coordinated circle, the apparent gravity is along the body vertical, so a pendulum (or any accelerometer-type sensor) cannot detect steady bank. This is basic physics and matches Haight's "stuck pendulum." A passive pendulum therefore cannot remove a steady 12-32 m circle caused by twist. Only a heading reference (magnet, as in F1E) or a rate sensor (gyro) can. An 8-25 g glider can't carry a magnet-rudder of F1E type without significant mass, but its size isn't known at this scale.

### Gaps
- I found no mass or size data for F1E magnet units, and no data on any pendulum below ~100 g.

## Q5. Tailless and flying-wing free-flight models: yaw stability without a fin

### Takeaway
Tailless free-flight relies on sweep (tip drag and lift-dependent roll), tip fins or tip rudders, and generous washout. Builders are warned that construction asymmetry is the main enemy. Directional stability is typically weak and lateral damping low.

### Cited Findings
- [HOBBY] Horten 1949: about 20° spar sweep. Sweep gives about twice the longitudinal damping of a conventional aircraft. Wing-tip brakes double as rudders. Build in extra twist to cover construction errors. — [Horten 1949](https://www.rcsoaringdigest.com/TWITT/Horten_Oct_49.html)
- [HOBBY] Foote 1956: sweep acts like fin area through tip drag but is "much weaker than a rudder" for directional stability. It can reduce the rudder area needed. — [Foote 1956](https://airplanesandrockets.com/magazines/air-trails/catching-thermals-air-trails-annual-1956.htm)
- [HOBBY] Foote 1956: warping one half to a higher-incidence tip makes that tip stall first in disturbances, so the model turns that way. Swept tips stall first, so on a swept model near the stall a tiny asymmetry decides turn direction. — [Foote 1956](https://airplanesandrockets.com/magazines/air-trails/catching-thermals-air-trails-annual-1956.htm)
- [HOBBY] Lippisch Storch tip fins were cambered in the same sense as the main wing (toe-in style), a feature said to be reproduced on later swept jets. — [Teichfuss Cicogna/Storch background, Wikipedia](https://en.wikipedia.org/wiki/Teichfuss_Cicogna) (summary via search; tertiary)
- [MEAS] NASA free-flight tunnel, tailless glider model: Cnβ ≥ 0.002/deg needed, with very low lateral damping. — [NTRS 20090026465](https://ntrs.nasa.gov/citations/20090026465) (snippet)

### Inferences
- [SPEC] For the swift-like planform with no fin, Foote's tip-stall mechanism is a likely amplifier. If the glide CL is near tip stall, a 0.1-0.3° twist difference can choose a stall-first tip and produce a turn. Flying slightly faster (lower CL), or adding washout so the tips sit further from stall, reduces this sensitivity. That washout should be symmetric and generous, per Horten.

### Gaps
- No free-flight tailless design rules (fin volume, sweep) with numbers were found beyond Horten's 20° and NASA's Cnβ. Nurflugel.com and the TWITT archive have more, but were not mined in this pass.

## Q6. Quantitative warp-to-turn data for small gliders

### Takeaway
I found no measured warp-vs-turn-radius data for small gliders. The closest anchors are hobbyist: 0.5° main-panel wash-in sets an F1B glide circle, 1/8 in. of accidental wash-in gave a "gentle" turn, and the Tutor 5.7° EDA tests show a slow roll-in over ~15 s to ~20° bank.

### Cited Findings
- [HOBBY] 0.5° wash-in, one panel, plus slight rudder is the deliberate F1B glide-circle trim. — [Joyner 2005](https://library.modelaviation.com/article/ff-duration-30)
- [HOBBY] 1/8 in. of unintended wash-in caused a gentle left turn. Span and chord were not given, so it can't be converted to degrees. — [Free Flight Sport 2003/06 snippet](https://library.modelaviation.com/article/free-flight-sport-200306)
- [HOBBY test] At EDA 5.7° the Tutor rolled to ~20° bank in ~15 s at Cl ≈ 0.9 with no deliberate asymmetry. — [Beron-Rawdon 1990](https://library.modelaviation.com/article/spiral-stability-and-bowl-effect)

### Inferences
- [SPEC] Free-flight practice suggests turn sensitivity to twist falls as spiral margin rises. With 0.5° used on purpose on long-tailed, high-dihedral F1B models, a 0.1-0.3° error on a flat, finless, short-arm model producing 12-32 m circles is consistent.

### Gaps
- No controlled warp-vs-radius measurement was found in hobby or academic sources searched. That needs a dedicated experiment or simulation.
