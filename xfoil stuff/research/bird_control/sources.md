# Bird-style glide control: source check

Compiled 2026-10-13. Web search and reading only. Nothing here was measured or modelled by us.

## How to read this

Verdicts: **VERIFIED**, **CORRECTED**, **PARTLY VERIFIED**, **NOT FOUND**.

Each number carries a tag saying how far it was checked:

- **[full text]** - read in the paper itself (open-access copy or preprint).
- **[abstract]** - read in the paper's abstract only; the body was not accessible (paywall or bot block).
- **[press]** - university press release or lab web page.
- **[secondary]** - news article, blog or retailer listing. Treat as unconfirmed.
- **[my calc]** - arithmetic done here from tagged numbers.

Papers whose body could not be opened: Lentink 2007 (Nature), Chang 2020 and Chang 2024 (Science Robotics), Henningsson & Hedenstrom 2011 (JEB), Tucker 1992, Thomas & Taylor 2001, Durston 2019. For LisHawk the arXiv preprint was read, not the journal version.

---

## 1. How gliding birds control pitch and roll

**Claim:** birds control pitch and roll in gliding largely by sweeping the wrist/hand-wing fore-aft (symmetric for pitch, asymmetric for roll), with the tail as a further control.

**Verdict: PARTLY VERIFIED, and too simple as stated.**

What the sources support:

- Wing extension/flexion strongly changes pitch stability. In gull wings, changing the **elbow** angle alone moved the pitch-stability derivative dCM/dCL from about -0.03 to -0.75, and moved the aerodynamic centre about 12 cm (about 60% of mean chord) while the centre of gravity moved about 2 cm [full text, Harvey 2019].
- Elbow and wrist together "effectively control lift, pitching moment and static margin, but other mechanisms are required to trim" [abstract + full text, Harvey 2021]. Maximum static margin shift reported: 24% of maximum chord [full text].
- Across 22 species, elbow and wrist range of motion is enough to switch between stable and unstable flight in 17 species; 4 were always stable, 1 always unstable. Elbow and wrist barely move the centre of gravity or pitch inertia but change roll and yaw inertia a lot [full text, Harvey 2022].
- Swifts: choosing the right sweep "can halve sink speed or triple turning rate"; extended wings are better for slow glides and turns, swept wings for fast ones [abstract, Lentink 2007]. This paper is about performance, not about pitch/roll control moments.
- Rule of thumb from theory: stable fliers are expected to sweep the wings forward in slow flight; wings alone may give birds longitudinal static stability, so birds may resemble tailless aircraft [abstract, Thomas & Taylor 2001].

What needs correcting:

- **"Wrist sweep" is not the tested variable in the key gull paper.** Harvey 2019 varied the elbow, held the wrist at maximum extension and did not vary sweep; it names sweep only as a likely further factor.
- **Symmetric sweep = pitch, asymmetric sweep = roll is a robot design rule, not a measured bird control law.** No source found here measures a bird's pitch or roll control moments from sweep in flight. The evidence for roll comes from robots: asymmetric wrist and finger motion initiated turns on PigeonBot, which the authors call "evidence that birds may use their fingers to steer" [abstract, Chang 2020]. Ajanic 2022 states birds may use wing folding and wing twisting, alone or together, for roll, and found on their drone that asymmetric wing pitching (twist) beats asymmetric folding in cruise [full text].
- **Wing pitch at the shoulder matters.** In gliding raptors "shoulder pitch was critical to maintaining control over the wing's angle of attack" [full text, Cheney 2021].
- **The tail is not a passive stabiliser in the aircraft sense.** In gliding barn owl, tawny owl and goshawk the tail carries positive lift, which is the opposite of what a stabilising tailplane does; the authors reject the tail as a passive pitch stabiliser under the measured conditions and note birds have "highly capable sensing and active control, presumably reducing the demand for passive aerodynamic stability" [full text, Usherwood 2020]. The tail does change strongly with speed: tail spread fell by 22-29 deg per m/s and tail angle of attack by about 6-13 deg per m/s [full text, Cheney 2021].
- **Birds can be statically unstable.** A gliding barn owl's geometry suggests it was longitudinally statically unstable [secondary summary of Durston 2019; body not read]. Together with Harvey 2022 this means bird control leans on fast active feedback.

Sources:

- Lentink D, Muller UK, Stamhuis EJ, de Kat R, van Gestel W, Veldhuis LLM, Henningsson P, Hedenstrom A, Videler JJ, van Leeuwen JL (2007). How swifts control their glide performance with morphing wings. Nature 446:1082-1085. doi:10.1038/nature05733
- Harvey C, Baliga VB, Lavoie P, Altshuler DL (2019). Wing morphing allows gulls to modulate static pitch stability during gliding. J R Soc Interface 16:20180641. doi:10.1098/rsif.2018.0641
- Harvey C, Baliga VB, Goates CD, Hunsaker DF, Inman DJ (2021). Gull-inspired joint-driven wing morphing allows adaptive longitudinal flight control. J R Soc Interface 18:20210132. doi:10.1098/rsif.2021.0132
- Harvey C, Baliga VB, Wong JCM, Altshuler DL, Inman DJ (2022). Birds can transition between stable and unstable states via wing morphing. Nature 603:648-653. doi:10.1038/s41586-022-04477-8
- Usherwood JR, Cheney JA, Song J, Windsor SP, Stevenson JPJ, Dierksheide U, Nila A, Bomphrey RJ (2020). High aerodynamic lift from the tail reduces drag in gliding raptors. J Exp Biol 223:jeb214809. doi:10.1242/jeb.214809
- Cheney JA, Stevenson JPJ, Durston NE, Maeda M, Song J, Megson-Smith DA, Windsor SP, Usherwood JR, Bomphrey RJ (2021). Raptor wing morphing with flight speed. J R Soc Interface 18:20210349. doi:10.1098/rsif.2021.0349
- Thomas ALR, Taylor GK (2001). Animal flight dynamics I. Stability in gliding flight. J Theor Biol 212:399-424. doi:10.1006/jtbi.2001.2387
- Durston NE, Wan X, Liu JG, Windsor SP (2019). Avian surface reconstruction in free flight with application to flight stability analysis of a barn owl and peregrine falcon. J Exp Biol 222:jeb185488. doi:10.1242/jeb.185488
- Tucker VA (1992). Pitching equilibrium, wing span and tail span in a gliding Harris' hawk, Parabuteo unicinctus. J Exp Biol 165:21-41. Citation confirmed; abstract not read, so no finding is quoted from it.

Note on authorship: the tail paper is led by Usherwood, not Cheney. Cheney leads the 2021 wing-morphing-with-speed paper.

---

## 2. The robots

**Claim:** all four are powered, much larger than our glider, and rely on an autopilot.

**Verdict: powered - VERIFIED. Much larger - VERIFIED for mass, weaker for RoboSwift span. Rely on an autopilot - CORRECTED: true for PigeonBot II only.**

| | PigeonBot (2020) | PigeonBot II (2024) | LisHawk (2020) | RoboSwift (2008) |
|---|---|---|---|---|
| Span | NOT FOUND | NOT FOUND | NOT FOUND in the preprint text; wing+tail area 0.131-0.209 m2, mean chord 0.161 m [full text] | about 50 cm [press] |
| Mass | about 280 g [secondary] | about 300 g [secondary] | 284 g ready to fly [full text] | 80 g design figure [press, 2007]; "under 100 g" at first flight [press, 2008] |
| Powered | Yes, propeller [secondary] | Yes, two motors with 76 mm propellers [secondary] | Yes, brushless motor, 7x6 propeller, 10 min flight time [full text] | Yes, electric motor with folding propeller for gliding [press] |
| Actuated | 4 servo-driven wrist and finger joints moving 40 real feathers, 42 degrees of freedom [abstract]; conventional tail and rudder [secondary] | 9 servos morphing wings and tail, 52 real feathers, no vertical tail [secondary] | 5 servos: 2 wing sweep (one per side, 9 g each), 1 tail spread, 1 elevator, 1 rudder (7.1 g each) [full text] | Wing sweep by 4 overlapping feathers per wing; roll by sweeping one wing more than the other [press] |
| Autopilot / closed loop | NOT FOUND. Described as remote-controlled [secondary] | **Yes, required.** Reflexive controller damps the Dutch roll instability caused by having no vertical tail; flew autonomously [abstract]. Teensy 4.0 plus PixRacer running ArduPilot [secondary] | **No.** "No autopilot or other autonomous flight enhancing measures were implemented during flight"; the PixRacer was for logging only; flown by hand over RC [full text] | NOT FOUND. Press text refers to a "pilot" being trained |
| Demonstrated | Asymmetric wrist and finger motion both initiate turns; feathered wing morphs under load and survives crashes [abstract] | Rudderless flight with bird-like wing and tail reflexes, tuned in a wind tunnel, then flown outdoors [abstract] | Speed range 4.0 to 7.6+ m/s by morphing; 59% thrust reduction at 12 m/s when tucked; roll by asymmetric sweep; tighter pull-ups [full text] | First flight March 2008 [press]. No peer-reviewed flight data found |

Notes:

- LisHawk is the clearest counter-example to "needs an autopilot". It was flown manually, but only in its stable configurations. The extended-wing, tucked-tail shape was avoided as "difficult to control due to its large pitch instability", and the authors say an autopilot would be needed to use the unstable regime [full text].
- LisHawk wing sweep range: 45 deg (tucked) to 130 deg (extended); wing area change 41%; symmetric sweep shifts the centre of gravity 18.6 mm [full text].
- The follow-up drone LisEagle is larger: 1.52 m span, 711 g, again flown by a pilot with a Pixhawk 4 for logging [full text, Ajanic 2022].
- RoboSwift has no paper that could be found; everything is from TU Delft and Wageningen press releases.

Sources:

- Chang E, Matloff LY, Stowers AK, Lentink D (2020). Soft biohybrid morphing wings with feathers underactuated by wrist and finger motion. Sci Robot 5:eaay1246. doi:10.1126/scirobotics.aay1246
- Chang E, Chin DD, Lentink D (2024). Bird-inspired reflexive morphing enables rudderless flight. Sci Robot 9:eado4535. doi:10.1126/scirobotics.ado4535
- Ajanic E, Feroskhan M, Mintchev S, Noca F, Floreano D (2020). Bioinspired wing and tail morphing extends drone flight capabilities. Sci Robot 5:eabc2897. doi:10.1126/scirobotics.abc2897. Preprint read: https://arxiv.org/abs/2002.02421
- Ajanic E, Feroskhan M, Wuest V, Floreano D (2022). Sharp turning maneuvers with avian-inspired wing and tail morphing. Commun Eng 1:34. doi:10.1038/s44172-022-00035-2
- RoboSwift press releases: TU Delft, July 2007, https://www.sciencedaily.com/releases/2007/07/070720205338.htm ; Wageningen, March 2008, https://www.sciencedaily.com/releases/2008/03/080304192104.htm
- Secondary, PigeonBot mass and tail: https://www.hackster.io/news/stanford-university-researchers-create-biohybrid-morphing-wing-for-pigeonbot-feathered-drone-fd4e739d49f4 ; https://spectrum.ieee.org/pigeonbot-uses-real-feathers-to-explore-how-birds-fly
- Secondary, PigeonBot II hardware: https://www.hackster.io/news/taking-flight-lessons-from-nature-6ca57390a3b4 ; https://physicstoday.aip.org/news/birdlike-robot-flies-steady-without-a-vertical-tail ; mass from https://dronexl.co/2024/12/01/revolutionary-drone-design-mimic-pigeon

---

## 3. Unpowered gliders under about 50 g that steer by wing sweep or fold

**Verdict: NOT FOUND.** No published passively stable or radio-controlled unpowered glider under about 50 g that uses wing sweep or fold morphing as its flight control was found. This is an absence in a limited search, not proof that none exists.

Closest work found:

| Vehicle | Mass / size | Powered | Folding or morphing | Control | Tag |
|---|---|---|---|---|---|
| EPFL SMA microglider (Kovac et al., ICRA 2007) | 1.5 g, span 24 cm, length 22 cm, about 1.5 m/s | No | None | Rudder driven by a 0.2 g shape memory alloy actuator; steers to a light | [press, lab page] |
| EPFL jumpglider (Kovac et al., ROBIO 2011) | 16.5 g | No (jumps, then glides) | Compared three folding-wing mechanisms with a rigid wing; folding is for stowage, not steering | Steered gliding | [secondary summary] |
| Stanford jumpglider (Desbiens et al. 2014, Bioinspir Biomim 9:025009, doi:10.1088/1748-3182/9/2/025009) | 30 g, 70 cm wing | No | Wing pivots freely in the climb, locks at the top | None found | [secondary] |
| Berkeley microglider (Wood et al. 2007, IEEE Robot Autom Mag 14(2):82-91) | under 3 g | No | None | Piezoelectric elevons, on-board optical flow | [press, lab page] |
| NRL CICADA | about 35 g, glide ratio about 3.5:1 | No, dropped from aircraft | None | GPS autopilot | [secondary] |
| Earwig-inspired origami wing (Ishiguro, Kawasetsu, Motoori, Paik, Hosoda 2023, Front Robot AI 10:1255666, doi:10.3389/frobt.2023.1255666) | 11.4 g wing, 125 mm span per wing, folds to about 1/7 area | Wind tunnel only, no free flight | Pneumatic opening, motor closing | Not a flight control | [full text] |
| Origami drop-deployable micro gliders (Wei, Xu, Lee, ASME IMECE2023-109491) | NOT FOUND | No | Self-unfolding after release, five wing shapes | NOT FOUND | [abstract fragment] |

Related but powered: Di Luca, Mintchev, Heitz, Noca, Floreano (2017), Bioinspired morphing wings for extended flight envelope and roll control of small drones, Interface Focus 7:20160092, doi:10.1098/rsfs.2016.0092 - asymmetric feather folding replaces ailerons [abstract].

EPFL lab page: https://lis2.epfl.ch/CompletedResearchProjects/BiomimeticJumpingMicroglider/ . Berkeley lab page: https://people.eecs.berkeley.edu/~ronf/MicroGlider/index.html

---

## 4. Smallest practical actuators

All numbers in this section are retailer or manufacturer listings read through search results, not opened and checked page by page. **[secondary]** throughout.

| Actuator | Listed mass | Notes |
|---|---|---|
| AGF C017CLS rotary servo | 1.7 g | 0.075 kg-cm at 4.2 V; 13.5 x 6.2 x 16.0 mm |
| Spektrum 1.7 g linear long-throw servo | 1.7 g | 9.1 mm throw, 3.2-4.2 V; listed as discontinued by one retailer |
| Spektrum AS2000 linear servo | 1.5 g | 7.4 mm throw; made for the AR6400 receiver bricks |
| Pichler DS15 | 1.5 g | 21.4 x 12 x 6 mm, 3.7-5.0 V |
| Magnetic coil actuators | about 1 g each | From a 2004 Model Aviation indoor column |
| Shape memory alloy actuator (EPFL research part) | 0.2 g | Not a hobby product [press] |

One retailer warns that a nominal 1.5 g linear servo is "more like 3 g with wires".

**Do sub-50 g radio-controlled gliders exist in the hobby? VERIFIED, weakly.**

- A 2004 Model Aviation article describes an indoor RC glider class with a 30 g minimum weight and 30 inch maximum span, and a converted glider of 20.6 g before two 1 g actuators were added [secondary].
- One current kit listing, a micro discus-launch glider, quotes 43-47 g [secondary, seller's copy].

No source was found giving a full weight breakdown for a sub-50 g glider built on 1.7 g servos.

Sources: https://alofthobbies.com/products/agf-c017cls-ultra-micro-servo-0-075-kg-1-0-oz-in-0-05-sec-1-7g ; https://www.superdroidrobots.com/store/product=1196 ; https://library.modelaviation.com/article/small-field-flying-200409 ; https://library.modelaviation.com/article/introduction-indoor-radio-control-200407

---

## 5. Gliding bird data for scaling

| | Common swift | Jackdaw |
|---|---|---|
| Mass | 0.042 kg [full text, Henningsson 2014 Table 1, gliding bird] | 0.1844 kg mean [full text] |
| Max wingspan | 0.39 m [full text, 2014] | 0.595 m measured in the hand [full text] |
| Wing area | 0.016 m2 [full text, 2014] | 0.0593 m2 including body strip [full text] |
| Aspect ratio | 9.8 [full text, 2014] | 5.97 [full text] |
| Mean chord | 0.041 m [full text, 2014] | - |
| Wing loading | about 25.8 N/m2 [my calc] | about 30.5 N/m2 [my calc] |
| Speeds tested | 7-11 m/s [abstract] | 6-11 m/s [full text] |
| Max L/D | 12.5 at 9.5 m/s [abstract] | 12.6 at 8.5 m/s [full text] |
| Best glide speed | 9.4 m/s [abstract] | 8.3 m/s [full text] |
| Min sink speed | at 8.1 m/s [abstract]; sink rate not read | at 7.4 m/s, sinking 0.62 m/s [full text] |
| Reynolds number | about 26,000 at 9.5 m/s [my calc from chord] | 38,000-76,000 over 6-11 m/s [full text] |
| Morphing with speed | Wingspan falls linearly with speed; tail span falls nonlinearly [abstract] | Wingspan falls linearly with speed; tail fully furled above 9 m/s [full text] |

Cautions:

- Each data set is **one bird** in a tilting wind tunnel.
- The swift morphology comes from the 2014 paper, which describes the gliding bird of the 2011 study; the 2011 body itself was not read.
- The jackdaw paper also uses a maximum span of 0.569 m in one figure caption (span measured in flight images) alongside the 0.595 m in-hand value.
- Swift model result: cost-effective gliding at 8-10 m/s, agility peaks at 15-25 m/s [abstract, Lentink 2007].
- Flapping swift L/D is 7.7 against 12.5 gliding [full text, Henningsson 2014, citing the earlier papers].

Sources:

- Henningsson P, Hedenstrom A (2011). Aerodynamics of gliding flight in common swifts. J Exp Biol 214:382-393. doi:10.1242/jeb.050609
- Henningsson P, Hedenstrom A, Bomphrey RJ (2014). Efficiency of lift production in flapping and gliding flight of swifts. PLoS ONE 9:e90170. doi:10.1371/journal.pone.0090170
- Rosen M, Hedenstrom A (2001). Gliding flight in a jackdaw: a wind tunnel study. J Exp Biol 204:1153-1166. doi:10.1242/jeb.204.6.1153. Read from https://lup.lub.lu.se/search/files/2545997/624228.pdf

---

## What this means for a 10-30 g passive glider (INTERPRETATION, not sourced fact)

Everything below is my reading of the sources above. None of it is tested.

1. **Bird-style morphing is a control-system technique, not a stability technique.** The bird papers show birds sitting near neutral or unstable and relying on fast feedback. A glider with no sensors or computer has to be designed stable first; sweep can then be used as a trim or mode setting.
2. **Sweep is a strong pitch lever, so treat it carefully.** In gulls and in LisHawk, extending or sweeping the wing moved the stability margin by tens of percent of chord, and LisHawk's extended-wing shape was unstable enough that the pilot avoided it. On a small glider, a few degrees of unintended sweep or fold slop could move it from stable to unstable.
3. **The LisHawk result is encouraging for a no-autopilot design.** A 284 g aircraft steered by asymmetric sweep was hand-flown with no stabilisation, as long as it stayed in stable configurations. That is the closest published precedent, at roughly ten times our mass.
4. **Nobody appears to have published our case.** An unpowered, under-50 g glider steered by sweep or fold was not found. That makes it open ground, and it also means there is no reference design to copy.
5. **Actuator mass is the hard limit.** Two 1.5-1.7 g servos plus receiver and cell is likely 6-10 g once wires are counted, which is most of a 10 g glider and a third of a 30 g one. This points to the upper end of the mass range for any controlled version, or to a single actuator.
6. **Do not expect bird glide ratios.** Swift and jackdaw reach L/D near 12.5 at wing loadings of 26-30 N/m2 and Reynolds numbers of roughly 26,000-76,000, with cambered, tuned wings. A flat-plate glider at lower wing loading flies slower and at lower Reynolds number, so its L/D will be well below that.
7. **A sensible order of work:** fixed, passively stable shape first; then ground-set sweep as a trim experiment to measure how much sweep moves the trim speed and margin; only then an actuated sweep or tail.
