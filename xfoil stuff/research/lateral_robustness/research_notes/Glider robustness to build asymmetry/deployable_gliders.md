# Deployable, air-dropped, printed and foldable unpowered micro gliders: what has been built, and what it says about passive straight glide

Scope: fixed-wing gliders of about 0.15-0.5 m span and grams to tens of grams, unpowered, dropped from multirotors, balloons or aircraft. Seed and samara fliers are covered only briefly. Written 2026-10-21.

Evidence tags: **[MEAS]** measured or peer-reviewed experiment; **[MODEL]** simulation or analysis; **[HOBBY]** hobbyist or company claim, not independently checked; **[SPEC]** speculation, mine or the authors'.

Local copies (untrusted data, read as text only, nothing executed) are in `research/lateral_robustness/sources/deployable_gliders/`: `wiesemuller2022_frontiers.html`, `arxiv2603.18861.html` (+ `_abs.html`), `icas2012_649.pdf`, `earwig_pmc10665516.html`.

Not repeated here, because the project already has it: Li, Goodwill, Wang & Ristroph 2022 (JFM 937 A6, doi 10.1017/jfm.2022.89), the weighted-paper-plane tumble/glide/dive model in `tumble_model/README.md`; and the endlessLift / Hinze foam Zanonia (155 mm span, "pretty straight" with "a little wobble") in `known_good/zanonia/`.

---

## 1. Air-dropped and swarm sensor gliders: what was built, how big, and did they fly straight?

### Takeaway
The closest comparable published systems are the Kim et al. 2026 (Univ. Tokyo / NTT) 25 cm PCB flying-wing glider launched from 30 m and the Wiesemüller et al. 2022 (Imperial / Empa) 1.25-1.75 g Alsomitra-shaped wafer-paper gliders launched by a multicopter-style release gripper. **Neither got repeatable straight glide.** Both saw a mix of straight glide and spiralling, and both put it down to small manufacturing asymmetries (and wind). Both groups then reframed the spiralling as useful, because it spreads the sensors out. No published group I found showed a passive, untrimmed batch of identical micro gliders flying reliably straight.

### Cited Findings
**Kim, Honjo, Iizuka, Okubo, Endo, Matsubara, Kawahara, Morita, Sasatani: "A Passive Elastic-Folding Mechanism for Stackable Airdrop Sensors", arXiv 2603.18861 (19 Mar 2026)**
- **[MEAS]** Two flying-wing glider prototypes, each 25 cm span. The airframe is the PCB itself, carrying temperature/humidity/GPS/IMU sensors, a battery and LoRa. Dihedral and elevon fold angles are set by heat-shrink polyolefin hinges. — [arXiv 2603.18861](https://arxiv.org/abs/2603.18861)
- **[MEAS]** Launched from a 30 m platform at Fukushima Robot Test Field with a rubber-band launcher on an aluminium frame "to ensure consistent launches". The onboard IMU showed the airframe "passively performed a pull-up maneuver". No mass, straightness or heading data were reported for the real flights. — [arXiv 2603.18861](https://arxiv.org/html/2603.18861)
- **[MEAS]** Hinge fold angles of 10-100 deg were achieved, with a standard deviation of 4 deg. — [arXiv 2603.18861](https://arxiv.org/abs/2603.18861)
- **[MODEL]** A Monte Carlo of airframes with "slightly different elevon angles" (left/right asymmetry from hinge fabrication), dropped from 10 km, showed "many of the airframes descended in a turning pattern due to the difference in their left and right angles". The turning radius depended on each airframe's deviation, and landings spread over about 10 km diameter. The authors present this as a feature for dispersal. — [arXiv 2603.18861](https://arxiv.org/html/2603.18861)
- Reading of the above, about this project: a 4 deg (1 sigma) scatter in a printed or folded control-surface angle is enough, in their model, to make most units turn.

**Wiesemüller, Meng, Hu, Farinha, Govdeli, Nguyen, Nyström, Kovač: "Transient bio-inspired gliders with embodied humidity responsive actuators for environmental sensing", Front. Robot. AI 9:1011793 (2022), doi 10.3389/frobt.2022.1011793**
- **[MEAS]** Alsomitra-template tailless gliders, cut from potato-starch wafer paper on a plotter cutter and then moulded wet between two mould halves to give a 3D shape. Section is a reflexed Fauvel airfoil thinned to 1.5 % thickness. Final geometries weigh 1.25-1.75 g. Humidity actuators of 25-35 mm diameter sit near the leading edge to move the CG forward, plus 0.3 g of extra leading-edge payload "to further improve stability". Design goal: carry 0.5-2 g and be launched from a multicopter. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[MEAS]** A release gripper with pan-tilt was used to vary the release angle and imitate drone release. 16 indoor trials per design were run at release angles of 25-55 deg and pitched down. "These trials show little dependence of the gliding trajectory on the glider's release conditions." — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[MEAS]** Two modes were seen indoors: "a straight gliding motion and a spiral downwards motion". The XFLR5 root locus had predicted a spiral mode. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[SPEC, authors]** The variability "is likely a direct consequence of slight variations in manufacturing or warping of the glider with changes in humidity, which can create slightly asymmetrical dihedral or changes in the center of gravity position". — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[MEAS]** Outdoors (mean crosswind 0.53 m/s, 20 s gust 2.4 m/s) they saw forward glide, mixed glide-then-spiral, and pure spiral. Spiralling was triggered by changing wind, and the 2.4 m/s gusts were more than 70 % of the simulated glide speed. Their conclusion: "higher payloads are necessary to overcome wind disturbances for any length of forward gliding to occur before the glider descends into a spiral dive". — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[MODEL]** CFD predicts glide angles of 6-7 deg at alpha 4-8 deg. A small separation bubble on the reflex gave more pitch-down moment, and so more stability, than XFLR5 predicted. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)

**Girardi, Wu, Fukatsu, Shigemune, Mintchev: "Biodegradable gliding paper flyers fabricated through inkjet printing", IROS 2024, pp. 10334-10341**
- **[MEAS, secondary only]** Inkjet-printed cellulose-paper gliders that degrade in about 105 days in moist soil. A carbon-black ink trace on the elevon senses its deflection. Electronics are in a sealed housing. I could only see this through a review and a bibliography listing; no size, mass or flight-straightness data was obtained. — [Advanced Science review](https://advanced.onlinelibrary.wiley.com/doi/10.1002/advs.202509194); [researchr listing](https://researchr.org/alias/stefano-mintchev)

**Otherlab APSARA (DARPA ICARUS, "vanishing" gliders)**
- **[HOBBY / press]** About 1 m cardboard-and-tape glider from a flat-pack laser-cut, scored kit (about 30 min assembly), carrying a 1 kg payload. It is **actively** steered (GPS, autopilot, servos) and spirals down to a controlled crash. Claims: lands within 10 m of the target, about 150 km range from high-altitude release. No drop-test or tolerance data were published. — [IEEE Spectrum](https://spectrum.ieee.org/otherlab-apsara-aerial-delivery-system)
- **[HOBBY / press]** Early drops were made from off-the-shelf drones below 400 ft, then from 1,000 ft at an Army facility. One report gives an 8:1 glide ratio. — [Smithsonian Air & Space](https://www.smithsonianmag.com/air-space-magazine/vanishing-drones-180961990/); [NPR/WBEZ](https://www.wbez.org/technology/2017/03/02/someday-disposable-drones-may-deliver-a-payload-then-vanish)
- Relevance: it shows the low-cost-airframe route, but it gets heading by closed-loop control, not passive trim. It is not evidence for passive straightness.

**UW (Iyer, Gollakota) airdropped devices: not gliders, but the same deployment idea**
- **[MEAS]** Iyer et al., MobiCom 2020: 98 mg sensor, released by a 37 mg magnetic-pin release using 450 µJ. It survived 22 m drops onto tile because it flutters and spins about the corner battery. — [paper PDF](https://sensordrop.cs.washington.edu/files/airdropping.pdf)
- **[MEAS]** Iyer, Gaensbauer, Daniel, Gollakota, Nature 603:427 (2022), "Wind dispersal of battery-free wireless devices": dandelion-inspired, about 30x the 1 mg dandelion seed. 75 shapes were tested for lowest terminal velocity. The devices travel up to 100 m in moderate breeze when dropped from a drone and land solar-side-up 95 % of the time. — [UW EE spotlight](https://www.ee.washington.edu/spotlight/dandelion-sensors); [project page](https://dandelions.cs.washington.edu)
- **[MEAS]** Johnson et al., Science Robotics 8:eadg4276 (2023), "Solar-powered shape-changing origami microfliers": about 400 mg Miura-ori fliers. Flat, they tumble chaotically like an elm leaf; folded, they fall steadily like a maple leaf. The fold takes about 25 ms and is triggered by a pressure sensor, timer or Bluetooth. Dropped from 40 m in light breeze they travel about a football field. — [UW News](https://www.washington.edu/news/2023/09/13/battery-free-robots-use-origami-to-change-shape-in-mid-air/)

**Balloon-dropped gliders (NOAA Glidersonde, EPFL R2Home, Rutgers/WPI AURORA, HAB hobby gliders)**
- **[HOBBY / agency]** All of these are actively guided and much heavier; for example, Glidersonde is under 1 m span and about 2.0 kg. They tell us nothing about passive straightness. — [NOAA NSSL](https://www.nssl.noaa.gov/projects/glidersonde/glider.php); [WPI MQP](https://digitalcommons.wpi.edu/mqp-all/650)

**Seed fliers (brief)**
- **[MEAS, abstract only]** Certini (PhD thesis, Edinburgh 2023): depth-camera tracking of 15 Alsomitra seeds found 11 helical and 4 straight-with-oscillation. Every seed had a preferred orientation (the wing has distinct upper and lower faces) and deformed spanwise under load. — [ERA thesis record](https://era.ed.ac.uk/handle/1842/39840)
- **[MEAS/MODEL]** Certini, Cummins, Kessler, Komendantskaya, Yang, Giorgio-Serchi, Viola (2026 dataset, doi 10.7488/ds/8109): planform circularity is the main factor separating the modes. High-circularity seeds go helical, low-circularity seeds fly straight, and chordwise CG eccentricity is the second axis. Straight fliers split into stable gliding and unstable bounding. — [Edinburgh DataShare](https://datashare.ed.ac.uk/handle/10283/9191)
- **[MEAS]** Wiesemüller et al. note that real Alsomitra seeds "show large variation in shape and size", and that the irregular aft 44 % of the chord was left out of their design. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[MEAS]** EPFL (Kovač et al.) 1.5 g microglider: rudder control by a 0.2 g SMA actuator, glide ratio up to 5.6. — [EPFL poster](https://infoscience.epfl.ch/record/110258/files/mko_Poster_ICRA_3.pdf)

### Inferences
- [SPEC] Every passive micro-glider programme found reports the same two-mode result, straight or spiral, with nominally identical builds splitting between them. That matches the project's observations (B45 flat wing glided; the Swift foam 400 oscillated and turned). It is the norm at this scale, not a sign of a project-specific error.
- [SPEC] Both research groups gave up on straightness and used the spiral for dispersal. Neither published a tolerance budget, a trim procedure or a per-unit trimming step. The project's goal of repeatable straight glide is therefore more ambitious than the closest published work.
- [SPEC] The Wiesemüller wind result scales directly. At the project's about 5.5 m/s glide speed, a 2.4 m/s gust is about 44 % of airspeed, which is less severe than their more than 70 % but still large. Wing loading (mass) is a robustness lever, which is what they concluded.

### Gaps
- No real-flight heading or turn-rate data was published for the Kim 25 cm gliders (only the IMU pitch trace). The mass was not given in the text I read.
- I found no peer-reviewed study that flew several nominally identical passive micro gliders and reported heading or turn-rate scatter. The Kim result is simulated; the Wiesemüller result is qualitative (two modes, no counts).
- I could not get Girardi et al. (IROS 2024) or the Certini 2026 paper text, so their geometry, mass and per-flight statistics are unknown.

## 2. Manufacturing tolerance and variability of low-cost or printed small airframes

### Takeaway
Direct measurements are scarce. The two quantitative pointers are a 4 deg (1 sigma) fold-angle scatter in self-folded hinges that, in simulation, makes most units turn (Kim 2026), and a 500-plane robotic study showing that nominally identical paper planes fall into discrete, probabilistic behaviour classes (Obayashi et al. 2023). Practitioner guidance on 3D-printed gliders is consistent: print-induced and removal-induced warp breaks symmetry and makes them turn.

### Cited Findings
- **[MEAS]** Kim et al. 2026: hinge fold-angle standard deviation of 4 deg. Elastic recovery after flat stacking is slow: about 85 % of the fold angle 30 min after release regardless of 1-3 h flat storage, and "up to 30 seconds to reach 70 % of the nominal geometry". — [arXiv 2603.18861](https://arxiv.org/html/2603.18861)
- **[MEAS]** Obayashi, Junge, Ilic, Hughes, Sci. Rep. 13:4212 (2023), doi 10.1038/s41598-023-31395-0: a robot folded and flew about 500 paper planes. Trajectories cluster into nose dive, glide and recovery glide. The geometry-to-behaviour map is "highly nonlinear and probabilistic", so one design gives several outcomes. — [EPFL Infoscience](https://infoscience.epfl.ch/record/295813); [Interesting Engineering](https://interestingengineering.com/innovation/robot-fabricates-flies-paper-planes-aerodynamics)
- **[SPEC, authors]** Wiesemüller 2022 attribute straight-versus-spiral scatter to manufacturing variation and humidity warping, which can create asymmetric dihedral or a CG shift (see section 1). — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[MEAS]** Wiesemüller 2022: cast actuator films had thickness standard deviations of about 0.1-2.5 µm on about 27-38 µm films (up to about 7 %). That is a material variation, not a flight one. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[HOBBY]** Toothpick glider tuning: a twisted or warped wing makes the glider spin or turn, and the remedy is balancing lift between the wing halves. — [Centennial of Flight "Toothpick Airforce" tuning](https://centennialofflight.net/toothpick/tuning.htm)
- **[HOBBY]** A maker print guide for a 3D-printed glider says printing at 45 deg (or any non-right-angle axis) causes asymmetric wing warping under load, that wings are easily deformed on removal from the bed, and that they "must be absolutely symmetrical for proper flight". This was seen only via search summary, and I could not pin the exact page among the results. — [MakerWorld F47 mini glider](https://makerworld.com/en/models/801394-f47-mini-glider)

### Inferences
- [SPEC] Removal from the bed, humidity and stacking all relax or set thin structures over minutes to hours, so trim should be checked just before flight rather than at build. Kim's slow elastic recovery is a real example of geometry still changing during flight.
- [SPEC] The Obayashi and Li results together suggest that near a mode boundary (CG near the glide/dive edge, or a marginal spiral mode), build scatter shows up as discrete flip-flopping between modes rather than as smooth small changes. Designing well inside a mode matters more than tightening tolerance.

### Gaps
- I found no study measuring the flight effect of FDM print warp or twist on a small glider (deg of twist against turn rate). There is also none on the effect of layer orientation on asymmetric stiffness of 0.4-0.8 mm printed plates.
- I did not extract numerical scatter from Obayashi et al. (full PDF not read).

## 3. Dispersion of glide paths across nominally identical builds

### Takeaway
The only quantitative "dispersion" numbers are simulated (Kim 2026: 10 km landing scatter from 10 km altitude, driven by elevon-angle asymmetry) or are outcome counts for natural seeds (Certini: 11 of 15 helical). Neither gives turn rate or heading scatter for a gram-scale built glider.

### Cited Findings
- **[MODEL]** Kim 2026 Monte Carlo: from 10 km altitude, about 10 km landing-diameter scatter, caused by left/right elevon asymmetry with Dryden turbulence and a horizontal wind model. — [arXiv 2603.18861](https://arxiv.org/html/2603.18861)
- **[MEAS]** Certini thesis: 15 Alsomitra seeds, 11 helical and 4 straight. — [ERA](https://era.ed.ac.uk/handle/1842/39840)
- **[MEAS]** Wiesemüller 2022: across 16 trials per design, two modes, with little dependence on release angle. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)

### Inferences
- [SPEC] Because release angle hardly mattered for Wiesemüller but the build did, the project's turning dispersion is more likely to come from airframe asymmetry than from the release. This is consistent with Kim's model, where asymmetry alone produced the scatter.

### Gaps
- No published heading, turn-rate or landing-scatter statistics exist for repeated drops of one gram-scale glider, or for a batch of identical ones. The project's own drop log would be first-of-kind data at this scale.

## 4. Pull-out after nose-down release, and release disturbance

### Takeaway
Passive pull-out after a drop is documented: the Kim 2026 25 cm glider from 30 m, and the Fujita 2012 0.9 kg deployable-wing UAV, which finished pull-up within about 3 s. Nobody gives altitude-loss numbers at the project's scale. Hobby balloon-glider builders found that a nose-first release at low height dives into the ground, and that changing the release attitude fixed it. I found no measured study of multirotor downwash on a released micro glider.

### Cited Findings
- **[MEAS]** Kim 2026: an IMU pitch trace shows passive pull-up after a 30 m rubber-band launch. No altitude loss or time was given in the text. — [arXiv 2603.18861](https://arxiv.org/html/2603.18861)
- **[MEAS]** Fujita (Tohoku), ICAS 2012 paper 649, "Evaluation of dynamic behavior of a deployable wing UAV by drop test": 0.9 kg, 1.2 m span, 0.2 m chord model, balloon-dropped from "some dozen meters". Deployed-state drop: large pitch rate and normal acceleration at about 2 s = pull-up. Folded-state drop: wings deployed at 0.69-1.51 s and pull-up finished at 2.83 s ("all maneuvers finished within 3 seconds"). The right wing deployed slightly before the left, giving small lateral-acceleration and roll-rate pulses, and the airplane still flew stably. — [ICAS 2012](https://icas.org/ICAS_ARCHIVE/ICAS2012/PAPERS/649.PDF)
- **[HOBBY]** Hackaday 2020, a 2.2 m balloon-glider project with quadcopter test drops from about 90 ft using a nichrome-melted fishing-line release: it "often dropped nose-down and dived into the ground". After the release was changed, the builder reports dropping at about -20 deg instead of 0 deg, with four consecutive successful launches. Commenters suggested releasing just below horizontal, using a drogue or parachute first, or adding a small kick. Another commenter noted the glider must fall further to reach its trimmed true airspeed in thin air. — [Hackaday](https://hackaday.com/2020/04/17/dropping-a-glider-from-18000-feet/)
- **[HOBBY / patent]** A stratospheric-glider patent quotes about 10 m of recovery height for a 28 m span glider at sea level, and says it scales with size and altitude. — [US patent 12428173](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12428173)
- **[MEAS]** Wiesemüller 2022: release angle (25-55 deg and pitched down) had little effect on the indoor trajectory of a 1.25-1.75 g glider. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)

### Inferences
- [SPEC] With the fixed nose-down start, a statically stable, well-damped glider pulls out by itself. The Kim and Fujita data agree, and the Hackaday failure was a vehicle released nose-first at only about 27 m. A crude calculation (MODEL) for the project's case: reaching a 5.5 m/s glide from rest takes at least v²/2g ≈ 1.5 m of fall plus the pull-out arc. The Fujita 3 s figure is for a heavy 1.2 m airframe and should not be scaled down as is.
- [SPEC] The downwash of a 25-30 m-altitude quad is a few m/s directly under the rotors. It most likely adds a pitch/yaw kick in the first fraction of a second, comparable to the Wiesemüller release variations, which they found mattered little. Untested here.

### Gaps
- I found no measured pull-out height loss for gram-scale gliders, and no paper quantifying multirotor downwash on payload release at this scale.

## 5. Thin-membrane / 3D-printed wing construction and mass per area

### Takeaway
Published gram-scale gliders use 1.5 %-thick moulded wafer paper (Wiesemüller), PCB laminate (Kim) or paper. Printed hobby gliders use single-wall LW-PLA or vase mode. I found no source giving areal density for printed sheet wings. Solid-sheet values follow from density times thickness (calculated below).

### Cited Findings
- **[MEAS]** Wiesemüller 2022: wafer paper cut on a plotter, then wet-moulded under clamps to a 1.5 %-thick reflexed section. Whole glider 1.25-1.75 g including actuators. — [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1011793/full)
- **[MEAS]** Kim 2026: an FR4 PCB is the airframe. Hinges are a 350 µm polyolefin sheet (from heat-shrink tube) laminated with polyimide, with the angle set by the PCB gap (g = Rθ, R = t_FR4 + t_PO). — [arXiv 2603.18861](https://arxiv.org/html/2603.18861)
- **[HOBBY]** A 345 mm span single-print 3D-printed hand-launch glider is listed at 16 g, with LW-PLA optional. A 5 g hand-throw printed design exists, with no area given. — [MakerWorld 3380216](https://makerworld.com/en/models/3380216); [MakerWorld 2690576](https://makerworld.com/models/2690576)
- **[HOBBY]** Printed RC gliders are about 15-17 g/dm² wing loading with vase-mode or single-wall LW-PLA. These are much larger airframes. — [3DLabPrint Bungee Prop guide](https://www.air-rc.com/assets/pdf/Bungee-Prop_3114.pdf); [Aloft Seagull](https://alofthobbies.com/products/3d-printed-seagull-glider)

### Inferences
- [MODEL, calculation] A solid PLA plate (1.24 g/cm³) weighs about 0.74 kg/m² at 0.6 mm and about 0.50 kg/m² at 0.4 mm. For scale, a 250 mm × 56 mm plate (0.014 m²) is about 10.4 g at 0.6 mm. This matches the F45 plate's 10.33 g in `tumble_model/README.md`. Halving the thickness, or using ribs plus a membrane, is the main lever on wing loading.

### Gaps
- No peer-reviewed numbers were found for printed thin-plate wing areal density, layer counts, LE spar sizing, or the warp of 0.4-0.8 mm FDM plates.

## 6. (Scope addition) Passive deployment triggers for folding wings on drop-launched gliders

### Takeaway
In practice, folding wings on dropped or launched UAVs are deployed by stored energy (torsion springs, rubber, elastic hinges) released at launch, sometimes helped by aerodynamic load. Fujita's measured caution is that aerodynamic load can **oppose** spring deployment above a threshold airspeed (about 6.2-6.8 m/s for his 0.9 kg model) and that deployment works best at low positive angle of attack. I found **no** fielded example of a bistable snap-through wing triggered by dynamic pressure. Bistable/spring-origami structures (Faber 2018) and tape springs give the latches, but in published UAVs the trigger is thrust or acceleration, an actuator, or a timer/sensor.

### Cited Findings
**Spring and elastic hinges (most common)**
- **[MEAS]** Fujita 2012: a commercial torsion spring hinge gives 0.6 N·m deployed and 1.2 N·m folded. The calculation said "this wing can deploy in any attitude as long as the airspeed is lower than 6.8 m/s". A wind tunnel on a 0.4 × 0.2 m Clark-Y panel found the minimum maximum-deployable airspeed was 6.2 m/s, varying strongly with alpha, with easiest deployment at alpha 0-15 deg. — [ICAS 2012](https://icas.org/ICAS_ARCHIVE/ICAS2012/PAPERS/649.PDF)
- **[MEAS]** Fujita 2012 failure: in the spring-hinge drop test the right wing did not deploy and the airplane crashed. The cause was a deployment stopper that "worked in resting state but did not work with flow", i.e. it latched on the bench but not under air load. — [ICAS 2012](https://icas.org/ICAS_ARCHIVE/ICAS2012/PAPERS/649.PDF)
- **[MEAS]** Fujita 2012 alternative: a servo-and-wire wing whose folded-state hinge torque is near the gravity torque, so it "is hard to deploy without help of the aerodynamic force". It deployed in about 0.8 s during the drop, asymmetrically (right first), and still pulled up. — [ICAS 2012](https://icas.org/ICAS_ARCHIVE/ICAS2012/PAPERS/649.PDF)
- **[MEAS/MODEL]** Fujita's table of methods: folding is light and reliable and can use aerodynamic force to deploy; inflatable stows well and deploys fast but needs high-pressure gas (leak risk); extending (telescoping) is heavy. — [ICAS 2012](https://icas.org/ICAS_ARCHIVE/ICAS2012/PAPERS/649.PDF)
- **[MEAS]** Kim 2026: an elastic heat-set hinge flattened for stacking springs back on release. It is passive, but slow (70 % of nominal geometry takes up to 30 s; about 85 % at 30 min) and the angle scatter is 4 deg. — [arXiv 2603.18861](https://arxiv.org/html/2603.18861)
- **[HOBBY / patent]** Tube-launched UAV patent US 10112698: a multi-part spring-loaded hinge held by one pin and a ramped guide, sized to open the wings fast enough after a launch of over 100 g, and fitting a 3.5 in tube. The patent claims fewer parts gives better reliability. — [Google Patents US10112698](https://patents.google.com/patent/US10112698)
- **[HOBBY / company]** Swift Engineering Fortus: a 7 lb, 6.5 ft span UAV that folds into a 6 in tube. The main wing is spring-deployed and the outboard panel unfolds aerodynamically. "Hundreds" of launches are claimed, with no failure rate given. — [Swift Engineering](https://www.swiftengineering.com/?p=1731)

**Tape springs**
- **[HOBBY/eng. reference]** A tape-spring hinge is two-state: flat-folded for stowing, then snaps straight and locks. Free deployment can be "swift and produce large snap-through shock and backlash". Spacecraft practice tunes the tape geometry or adds an SMA brake to control it. — [Firgelli tape-spring explainer](https://www.firgelliauto.com/blogs/mechanisms/tape-spring-hinge); [NASA NTRS 20120008634](https://ntrs.nasa.gov/citations/20120008634)

**Bistable / snap-through and origami**
- **[MEAS]** Faber, Arrieta, Studart, Science 359:1386 (2018), doi 10.1126/science.aap7753, "Bioinspired spring origami": earwig wing joints act as resilin springs, giving bistable folded and deployed states that each hold load with no energy input. A 3D-printed multi-material demo self-folded and locked. The paper does not demonstrate a gliding wing. — [Purdue news](https://www.purdue.edu/newsroom/releases/2018/Q1/origami-folds-of-insect-wing-can-help-improve-machine-functions.html); [ScienceDaily](https://www.sciencedaily.com/releases/2018/03/180322140854.htm)
- **[MEAS]** "Soft Hybrid Aerial Vehicle via Bistable Mechanism", arXiv 2011.00426: the snap-through force was tuned to be low enough for the vehicle's motors to trigger but high enough to prevent accidental switching. The trigger is thrust and acceleration, not dynamic pressure. — [arXiv 2011.00426](https://arxiv.org/pdf/2011.00426)
- **[MODEL/MEAS]** Rotational bistable morphing-wing work (Commun. Eng. 2025) reports reversible chordwise shape change. Its operating envelope depends strongly on hinge position, and it cites earlier aero-load-controlled bistable laminates without mechanical locks. — [Nature Commun. Eng.](https://www.nature.com/articles/s44172-025-00495-2)
- **[MODEL]** "Concept for a Deployable Wing": a glass-fibre skin and rib wing that uses the Brazier bending instability to snap between extended and folded states (analytic plus FEM). — [ResearchGate](https://www.researchgate.net/publication/281119142_Concept_for_a_Deployable_Wing)
- **[MEAS]** Earwig-inspired origami foldable wing (Front. 2023, PMC10665516): fold ratio about 5-7. It opens and folds at 4 m/s in a wind tunnel, but deployment uses a pouch-motor actuator and folding uses a motor, so it is not passive. — [PMC10665516](https://pmc.ncbi.nlm.nih.gov/articles/PMC10665516/)

**Shape memory, timed and sensor triggers**
- **[MEAS]** Johnson 2023 microfliers: fold actuation in about 25 ms, triggered by an onboard pressure sensor (altitude), a timer, or Bluetooth. This is the closest working example of an altitude- or time-triggered shape change on a dropped flier. — [UW News](https://www.washington.edu/news/2023/09/13/battery-free-robots-use-origami-to-change-shape-in-mid-air/)
- **[MEAS]** NWPU 2020 NiTi-SMA-wire non-explosive wing release: 21-33 ms per wing to deploy, cabin impact loads up to about 380 g. The platform type was not confirmed. — [NWPU Pure](https://pure.nwpu.edu.cn/en/publications/design-analysis-and-prototype-testing-of-a-non-explosive-self-dep/fingerprints)
- **[MEAS]** Knowles et al. 2004: heated tubular SMA springs deploying wing surfaces on small air vehicles; feasibility shown. — [Missouri S&T Scholars' Mine](https://scholarsmine.mst.edu/mec_aereng_facwork/1825)
- **[HOBBY / patent]** An SMA locking strip holding a projectile wing until release (US 7090163). Metal/shape-memory-polymer roll-up wings for tube-launched vehicles (US 8528863). — [US 7090163](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7090163); [US 8528863](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8528863)
- **[MEAS]** UW 2020 release: a 37 mg coil-pulled magnetic pin using 450 µJ, i.e. a carrier-side release pin at gram scale. — [paper PDF](https://sensordrop.cs.washington.edu/files/airdropping.pdf)

**Static line / carrier-side pull**
- **[HOBBY]** The Hackaday balloon glider uses a nichrome-melted fishing line as its carrier release. — [Hackaday](https://hackaday.com/2020/04/17/dropping-a-glider-from-18000-feet/)
- I found no documented static-line wing deployment at gram scale.

### Inferences (numbers are my calculations, tagged MODEL; design ideas tagged SPEC)
- [MODEL] Dynamic pressure q = ½ρV² at ρ = 1.2 kg/m³: 5.5 m/s gives 18 Pa, 10 m/s gives 60 Pa, 15 m/s gives 135 Pa. A 20 cm² (0.002 m²) drag tab at C_D ≈ 1.2 gives 0.04 N at glide, 0.14 N at 10 m/s, and 0.32 N at 15 m/s. The glider's weight (10-20 g) is 0.1-0.2 N. So a tab trigger at about 60-100 Pa must resist about 0.1 N or less of normal handling force. That is a light, finger-sensitive latch.
- [SPEC] In a nose-down dive at near-zero lift the wing panels carry little normal load. An airspeed trigger would need a dedicated drag tab or flap, or would rely on the pull-out g-load (for example, 2-3 g briefly) acting on panel mass. Fujita's result also warns that, once flying, air load can push against deploying panels. For the project's fold axis (bird-like, spanwise), the wing should be designed so lift helps open it (dihedral-direction fold that lift unfolds) or so the panel opens while lift is near zero, i.e. during the dive, as Fujita found best at low alpha.
- [SPEC] Fujita's failure lesson carries over directly: test the over-centre or lock mechanism under flow (wind tunnel, fan, or car-roof run), not only on the bench.
- [SPEC] For a 10-20 g glider, the simplest robust options in the literature are (a) an elastic or rubber-band hinge held folded by the carrier release itself (a static-line style: the release pin or line holds the wing closed, and opening the clamp both drops it and frees the wings), or (b) a timed burn-wire or SMA release. A dynamic-pressure bistable trigger is novel at this scale; no reliability data exists.
- [SPEC] Expect asymmetric deployment timing (Fujita: right wing about 0.1 s before left, giving roll pulses), and fold-angle scatter of a few degrees (Kim: 4 deg 1 sigma). The section 1-3 evidence says that scatter turns gliders, so the lock must index to a hard stop, not to the hinge's elastic rest angle.

### Gaps
- No measured trigger thresholds, reliability statistics or failure rates for passive wing deployment below about 1 kg.
- No published dynamic-pressure-triggered bistable wing for a glider.
- No data on how repeatably a folding wing returns to the same incidence and dihedral; this matters most for straight flight.
