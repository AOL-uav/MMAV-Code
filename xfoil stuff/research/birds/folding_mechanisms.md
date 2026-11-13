# Folding wings for a 10-30 g drop glider: precedents and concepts

Compiled 2026-10-13. Web search and reading only. Nothing here was built, measured or modelled by us.
This is idea generation. Section 1 is sourced. Sections 2-5 are engineering reasoning and estimates, marked as such.

Companion notes (not repeated here): `research/bird_control/sources.md` (how birds and robots use sweep for control, actuator masses, bird glide data).

## How to read this

Tags on facts:

- **[full text]** - I read the paper text myself (open-access HTML or a PDF converted to text).
- **[page summary]** - the page (patent, magazine article, lab page) was opened, but read through a summarising fetch tool, so wording and small details could be off.
- **[abstract]** - abstract or repository record only.
- **[secondary]** - press, news or search-result snippet. Unconfirmed.
- **[estimate]** - my arithmetic, with the assumptions stated. Not data.
- **NOT FOUND** - looked for and not found in this search.

Could not open: Kovac 2011 (EPFL jumpglider, the folding-wing comparison) and Dufour 2016 (origami wing) - the EPFL repository refused downloads. Stowers & Lentink 2015, Chang 2020 (PigeonBot), Bahlman 2013, Desbiens 2014, Woodward & Sitti 2014 bodies were not read.

---

## 1. Precedents

### 1.1 Summary table

| # | Vehicle / source | Span, mass | How it folds | What deploys it | What holds it open | Once or reversible | Tag |
|---|---|---|---|---|---|---|---|
| 1 | Walker folding-wing toy glider (US 2,221,012, 1940; sold as the "Interceptor") | NOT FOUND (balsa hand toy) | Each wing swings back about a vertical axis at its root to lie along the body, then rolls 90 deg about a second axis | One rubber band through a tube in the body, hooked to both wings | U-shaped wire saddle over the body stops the wing at the glide position; a movable wedge trims incidence | Once per flight, reset by hand | [page summary] |
| 2 | Corbin folding-wing toy glider (US 4,836,817, 1989) | NOT FOUND (foam toy) | Two flat swept foam panels joined by tape hinges at the centreline; panels fold face to face | Rubber band between two hooks under the wing | The same rubber band, pulling the bevelled inner edges together | Once per flight | [page summary] |
| 3 | Schertle "Folding-Wing Glider", Make vol. 31 (2012) | NOT FOUND | Walker layout in balsa: wings fold back along the body with tips over the tail | Small rubber band through a tube, stapled to both wings | 1/8 in dowel stop and a wire saddle | Once per flight | [page summary] |
| 4 | Di Luca et al. 2017 feathered drone | 0.84 m open, 0.395 m folded; 330 g | Outer wing: 8 flat feathers on pin joints along a leading-edge bar that rotates in the wing plane; feathers stack when closed | Two tendons: a pre-stretched spring pulls the wing open, a servo pulls it closed | Spring preload (3 N) | Reversible in flight | [full text] |
| 5 | LisHawk (Ajanic et al. 2020) | span NOT FOUND; 284 g | Outer wing: 9 flat feathers fan out and overlap; outermost fixed to the skeleton, the rest pivot on pins and are linked by pre-stretched elastic | One 9 g servo per side; how the servo is linked to the wing was not found in the text read. (The tail uses a 0.2 mm Kevlar cable over printed pulleys.) | Servo | Reversible in flight | [full text, preprint] |
| 6 | PigeonBot (Chang et al. 2020) | NOT FOUND; about 280 g | Wrist and finger joints move 40 real feathers joined by elastic ligaments | 4 servos | Servos | Reversible | [abstract], mass [secondary] |
| 7 | Stowers & Lentink 2015 passive wrist | 400 mm span, 80 mm chord [secondary]; mass NOT FOUND | Unactuated hand wing on a wrist hinge; fold ratio 2-2.5 | Centrifugal force from flapping at 5-17 Hz; opens in 1-2 flaps | Centrifugal force | Reversible, passive | [abstract] |
| 8 | Baek et al. 2020 ladybird origami glider | 660 mm span; glider module 35 g without electronics | Each side: two stacked origami strips, each with two facets and one fold line, coupled to a pinned shoulder; folds to 1/8 of the open area | Elastic energy in curved facets; a trigger motor lets go; opens within 466 ms | The curved facet locks itself flat | Reversible: a winding motor pulls a wire to refold | [full text] |
| 9 | Dufour et al. 2016 origami wing | 26 g wing on an 82 g drone; 200 x 500 x 16 mm open, 115 x 215 x 40 mm folded | Origami tile pattern; stowed span 43% and area 26% of open | By the user's hand in about 0.5 s | NOT FOUND (body not read) | Ground folding only | [secondary] |
| 10 | EPFL jumpglider (Kovac et al. 2011; Vidyasagar et al. 2015) | 16.5 g robot; span NOT FOUND | Three bio-inspired folding mechanisms compared with a rigid wing; two built | NOT FOUND | NOT FOUND | NOT FOUND | [abstract] |
| 11 | EPFL bat-inspired wing folding mechanism (Kovac, Zufferey, Floreano 2009 chapter) | 4.5 g mechanism | NOT FOUND | Unfolds in 50 ms | NOT FOUND | NOT FOUND | [abstract] |
| 12 | Stanford jumpglider (Desbiens et al. 2014) | 70 cm wing, 30 g | Wing does not fold; it pivots freely in the climb | Passive | Magnets on wing and body lock it at the top | Once per jump | [secondary] |
| 13 | MultiMo-Bat (Woodward & Sitti 2011, 2014) | "three-inch robot, eight-inch flying legs" [secondary]; mass NOT FOUND | Membrane wing on the jumping four-bar legs | Leg extension | Legs | Each jump | [page summary] |
| 14 | Bat Bot B2 (Ramezani, Chung, Hutchinson 2017) | 93 g; built at fruit-bat size (the paper gives bats of 30-40 cm span) | Three-link arm (shoulder, elbow, wrist) constrained so one input drives all three | One motor-driven spindle per arm (push-pull) | Spindle | Reversible, every wingbeat | [full text] |
| 15 | Bahlman, Swartz, Breuer 2013 robotic bat wing | bench rig | 7 joints | 3 servos pulling cables that stand in for tendons; bones 3D printed | Servos | Reversible | [abstract], cables [secondary] |
| 16 | AquaMAV (Siddall, Ortega Ancel, Kovac 2017) | 592 mm span, 201.3 g; wings 41.0 g | Whole wings sweep back up to 90 deg about root pivots; figure caption mentions a gear linkage | Servo | Servo | Reversible in flight | [full text] |
| 17 | NRL CICADA | Mk5 65 g; an earlier figure of about 35 g; span NOT FOUND | Mk5 does not fold: flat circuit-board wings, stacked 32 to a sonobuoy tube. An earlier version is said to have had folding wings | Dropped | - | - | [secondary] |
| 18 | Tube-launched small UAV patents | not our scale | Wings stacked on one vertical pivot (scissor) | Tension or torsion spring; the tube itself holds the wings folded | Separate lock: over-centre links, or a spring-loaded tapered plunger | Once | [secondary, patent snippets only] |
| 19 | Wei, Xu, Lee 2023 origami micro glider (ASME IMECE2023-109491) | NOT FOUND | Origami wings, five shapes | Self-unfolds after release | Three self-locking bistable joints tried: elastic hinge, friction, peg | Once | [abstract] |

Already covered in `sources.md` and not repeated: RoboSwift (4 overlapping feathers per wing, press only), the earwig origami wing (11.4 g per wing, wind tunnel only).

**NOT FOUND:** a tensegrity wing, or a finger-like tendon-routed multi-segment wing, at anything near 10-30 g. Design details for Switchblade or Coyote wing pivots. Any published folding wing on an unpowered glider under about 30 g that states its mass and its repeatability. Any measured repeatability figure for a deployed folding wing, other than item 4.

### 1.2 The ones that matter most, and why

**Toy catapult gliders (items 1-3). Closest in mass and materials.**

- Walker: the wing swings open about a vertical axis at the root. The band's lever arm grows as the wing opens, so opening speeds up once it starts. A wire saddle stops the wing, and the saddle length sets the wing's angle of incidence; a wedge adjusts it "to compensate for manufacturing variation and warping" [page summary]. So even the 1940 toy needed a trim adjustment at the stop.
- Corbin: tape hinge plus one rubber band, in expanded polystyrene. The band is stressed in every position and holds the panels coplanar. The inner edges are cut at an angle and meet when open [page summary]. This is idea A reduced to one hinge.
- In both, the wings stay folded because air pressure at high launch speed holds them shut, and they open when the glider slows near the top [page summary]. **Our drop is the opposite case:** speed is lowest at release and rises afterwards. An air-held fold would never open. The wings have to open at release, or be opened by something else.
- Schertle's notes list the practical problems: wing clips rubbing the body slowed the opening; balsa wore at the pivot hole; the staples for the band "must be equally spaced" on both wings; glide trim was set by bending the stop wire [page summary].

**Di Luca 2017 (item 4). The best-documented tendon wing, and it is the reverse of idea B.**

- "The tendon for folding is directly driven by a servomotor ... while the one for deployment is pulled by a pre-stretched linear spring" [full text].
- "The pre-stretched spring limits the backlash of the mechanism allowing an angular accuracy between 0.3 deg and 0.5 deg" [full text]. This is the only repeatability number found anywhere in this search.
- The spring preload (3 N) was sized to hold 1.5 times the drag on the folding section at 20 m/s [full text]. So the design rule was: the hold-open force is sized against **drag**, with a margin.
- Full-stroke times: opening 137 ms at 0 g rising to 207 ms at 4 g; closing 157 to 187 ms. "Increasing the load factor increases friction in the mechanism" [full text].
- Feathers: 1.5 mm carbon shaft on a 0.2 mm FR4 frame covered in ripstop fabric. Folded area is 41% smaller [full text].

**LisHawk (item 5).**

- Feathers are linked by a pre-stretched elastic connector "to achieve regular feather spacing" and to "overcome the surface friction of the overlapping feathers when the wing is tucked" [full text]. Friction between stacked plates was a real design issue.
- The **tail** is the owner's version of idea B: "The tail extends when the cable is pulled, while the elastic connector tucks the feathers, when the servo tension is released" [full text].
- Feather skeletons (0.3 mm fibre sheet) were chosen for stiffness "to absorb the aerodynamic loads when the feathers are slightly overlapped" [full text].
- The tail cable is 0.2 mm Kevlar rated 20 kg, run over 3D-printed ABS pulleys to a servo at the front of the body [full text]. So a thin string over printed pulleys is proven at that scale.

**Baek 2020 (item 8). Spring deploys, wire refolds.**

- "Folding of the glider is actuated by a winding motor with a wire-driven mechanism ... Self-deployment is triggered by a trigger motor" [full text].
- The frame "can sustain six times the moment required to fold the glider" because it folds easily one way and is blocked the other way [full text]. Lesson: put the fold direction where the flight loads are small.
- The authors compare the locked facet to "the bending of a tape spring" [full text]. See concept C5.
- At 35 g for the wing module alone it is too heavy to copy.

**Bat Bot (item 14).** A three-link arm is made into a one-input mechanism by adding rigid constraint links, so one actuator drives shoulder, elbow and wrist together [full text]. This is the published form of "one input drives two joints".

**Bird anatomy caution.** The textbook idea that the bird's radius and ulna form a flat four-bar ("drawing parallels") that couples elbow and wrist was tested on pigeons: the bone paths are three-dimensional, the best flat four-bars lock up or need the bones to cross, and a six-bar with a sliding wrist bone fits [abstract, Stowers, Matloff, Lentink 2017]. A flat four-bar is still a perfectly good machine; it is just not exactly what the bird does.

**Stowers & Lentink 2015 (item 7).** The wrist opens by centrifugal force from flapping. A glider does not flap, so this does not transfer. The useful part is the other half: a free wrist hinge lets the hand wing give way on impact and recover [abstract].

### 1.3 Citations

- Walker NE. Toy glider. US Patent 2,221,012, filed 13 Oct 1939, granted 12 Nov 1940. https://patents.google.com/patent/US2221012A/en ; history page https://americanjuniorclassics.com/interceptor/interceptorpatent.htm (that page shows a garbled patent number; the Google Patents number is the one to use).
- Corbin SK. Folding wing toy glider. US Patent 4,836,817, filed 28 Apr 1988, granted 6 Jun 1989. https://patents.google.com/patent/US4836817
- Schertle R (2012). Folding-Wing Glider: Rockets Up, Glides Down. Make vol. 31, p. 88. https://makezine.com/projects/folding-wing-glider-rockets-up-glides-down/
- Di Luca M, Mintchev S, Heitz G, Noca F, Floreano D (2017). Bioinspired morphing wings for extended flight envelope and roll control of small drones. Interface Focus 7:20160092. doi:10.1098/rsfs.2016.0092. Read at https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5206609/
- Ajanic E, Feroskhan M, Mintchev S, Noca F, Floreano D (2020). Bioinspired wing and tail morphing extends drone flight capabilities. Sci Robot 5:eabc2897. doi:10.1126/scirobotics.abc2897. Preprint read: https://arxiv.org/abs/2002.02421
- Chang E, Matloff LY, Stowers AK, Lentink D (2020). Soft biohybrid morphing wings with feathers underactuated by wrist and finger motion. Sci Robot 5:eaay1246. doi:10.1126/scirobotics.aay1246
- Stowers AK, Lentink D (2015). Folding in and out: passive morphing in flapping wings. Bioinspir Biomim 10:025001. doi:10.1088/1748-3190/10/2/025001
- Stowers AK, Matloff LY, Lentink D (2017). How pigeons couple three-dimensional elbow and wrist motion to morph their wings. J R Soc Interface 14:20170224. (DOI not checked.)
- Baek S-M, Yim S, Chae S-H, Lee D-Y, Cho K-J (2020). Ladybird beetle-inspired compliant origami. Sci Robot 5:eaaz6262. doi:10.1126/scirobotics.aaz6262. Read from the lab copy at https://www.biorobotics.snu.ac.kr/glider
- Dufour L, Owen K, Mintchev S, Floreano D (2016). A drone with insect-inspired folding wings. IEEE/RSJ IROS 2016. https://infoscience.epfl.ch/record/221316 ; news https://actu.epfl.ch/news/a-drone-with-insect-inspired-folding-wings (DOI not checked.)
- Kovac M, Hraiz W, Fauria O, Zufferey J-C, Floreano D (2011). The EPFL jumpglider: a hybrid jumping and gliding robot with rigid or folding wings. IEEE ROBIO 2011. doi:10.1109/ROBIO.2011.6181502. https://infoscience.epfl.ch/record/170029
- Vidyasagar A, Zufferey J-C, Floreano D, Kovac M (2015). Performance analysis of jump-gliding locomotion for miniature robotics. Bioinspir Biomim 10. https://infoscience.epfl.ch/record/207112 (Article number and DOI not checked.)
- Kovac M, Zufferey J-C, Floreano D (2009). Book chapter describing a 4.5 g bat-inspired wing folding mechanism. https://infoscience.epfl.ch/record/130146 (Chapter title not checked.)
- Desbiens AL, Pope MT, Christensen DL, Hawkes EW, Cutkosky MR (2014). Design principles for efficient, repeated jumpgliding. Bioinspir Biomim 9:025009. doi:10.1088/1748-3182/9/2/025009. Secondary: https://newatlas.com/flying-fish-glider/27690/
- Woodward MA, Sitti M (2014). MultiMo-Bat: a biologically inspired integrated jumping-gliding robot. Int J Robot Res. https://is.mpg.de/publications/woodward2014multimo (Volume and pages not checked.)
- Ramezani A, Chung S-J, Hutchinson S (2017). A biomimetic robotic platform to study flight specializations of bats. Sci Robot 2:eaal2505. doi:10.1126/scirobotics.aal2505. Read from https://faculty.cc.gatech.edu/~seth/ResPages/pdfs/ScienceRobotics2017.pdf
- Bahlman JW, Swartz SM, Breuer KS (2013). Design and characterization of a multi-articulated robotic bat wing. Bioinspir Biomim 8. Press: https://engineering.brown.edu/news/2013-02-20/brown-researchers-build-robotic-bat-wing (Article number and DOI not checked.)
- Siddall R, Ortega Ancel A, Kovac M (2017). Wind and water tunnel testing of a morphing aquatic micro air vehicle. Interface Focus 7:20160085. Read at https://pmc.ncbi.nlm.nih.gov/articles/PMC5206604/ (DOI not checked.)
- CICADA: https://spectrum.ieee.org/naval-research-lab-tests-swarm-of-stackable-cicada-microdrones ; https://livescience.com/59966-tiny-stackable-gliding-drones.html
- Tube-launch patents seen as search snippets only: US 9,452,820 (spring resists folding, latch, tube holds the wing folded); US 7,732,741 (spring-loaded tapered plunger locks the open wing); CN108146613B (torsion spring opens, separate lock needed).
- Wei M, Xu J, Lee C (2023). ASME IMECE2023-109491. https://imece.secure-platform.com/a/solicitations/208/sessiongallery/14758/application/109491

---

## 2. Numbers used in the evaluation (all estimates)

Reference glider for the arithmetic: span 300 mm, mass 20 g, wing area about 0.02 m2 (mean chord about 67 mm). All numbers below are **[estimate]** and scale with whatever the real glider turns out to be.

**Loads on one half-wing**

| Case | Lift | Bending moment at root | Drag | In-plane (sweep-back) moment at root |
|---|---|---|---|---|
| Steady glide, 1 g | 0.10 N | about 6 mN.m | about 0.02 N (L/D 5 assumed) | about 1.2 mN.m |
| Pull-out, 4 g assumed | 0.39 N | about 25 mN.m | about 0.10 N (L/D 4 assumed) | about 6 mN.m |

Assumptions: lift acts at 42% of the half-span (63 mm from the centreline); drag acts at the same place. The 4 g pull-out and the L/D values are guesses. Nobody has measured our pull-out.

What this says:

- The lift moment is roughly four to five times the drag moment.
- A joint that only has to resist **drag** needs about 10 mN.m of hold-open torque with margin. With an 8 mm lever arm that is a string tension of about 1.3 N (about 130 gram-force).
- A joint that has to resist **lift** needs about 25 mN.m and the answer has to come from structure (a stop with a deep lever arm), not from a spring.
- A 1.7 g servo is listed at 0.075 kg-cm, which is about 7 mN.m [secondary, from `sources.md`]. So a 2 g-class servo cannot hold two wing roots open against the pull-out by brute force. Stops and springs have to carry the flight loads; a servo can only move the wing between stops.

**Part masses**

| Part | Estimate | Assumption |
|---|---|---|
| Printed pivot boss with stop tab | 0.3-0.5 g | 0.25-0.4 cm3 of PLA at 1.24 g/cm3 |
| Pin, 1 mm carbon rod, 15 mm long | 0.02 g | 1.6 g/cm3 |
| Tape hinge, 60 x 20 mm | 0.05-0.1 g | tape at 40-80 g/m2 |
| Braided fishing line, 0.1 mm, 0.5 m | under 0.01 g | 0.97 g/cm3 |
| Printed string guide | 0.05 g each | |
| Rubber band or elastic thread | 0.1-0.3 g | |
| Printed latch with pin | 0.3-0.5 g | |
| Printed sector gear pair at the roots | 0.6-1.0 g | |
| Servo, bare | 2.1 g | owner's part; wire, horn, receiver and cell are extra |

Wing material for scale: 0.6 mm PLA plate is about 0.74 kg/m2, so a 0.02 m2 wing is about 15 g before ribs. That alone is most of a 20 g glider. Foamboard areal mass is NOT KNOWN here (brand and thickness not weighed). The wings are a large share of the total mass, which matters for balance (section 3.5).

---

## 3. Evaluation of ideas A and B

### 3.1 Two fold families, and which way the loads act

```
In-plane fold (vertical hinge axis)        Out-of-plane fold (chordwise hinge line)
Top view:                                  Front view:

   body                                       hinge
    |\                                         v
    | \  wing swings back                 ====o====o====   open
    |  \ in its own plane
    o---\---->  open                      ====o
    |                                         |===o        folded (zigzag)
   pivot (pin points up)                          |====
```

- **In-plane fold (idea B, swing wing, feather fan).** The pin points up. Lift tries to rock the joint, and that is resisted by the pin and by the two plates lying flat on each other, not by the mechanism. The only thing that pushes the joint toward "folded" is drag. So the hold-open device sees the small load (about 10 mN.m at the root). This is what Di Luca did, and why a 3 N spring was enough on a 330 g drone.
- **Out-of-plane fold (idea A).** The hinge line runs front to back. Lift bends the wing directly about that line. Every hinge has to carry the full lift moment at its station.

This is the main reason B is the better family for this glider.

### 3.2 Idea A: accordion with hard stops

How it would work: wing cut into spanwise panels, tape or thread hinges along chordwise lines, panels fold on top of each other.

- **Loads.** Lift puts the top face in compression and the bottom face in tension. A hinge on the **bottom** face with panel edges butting on the **top** resists lift as a solid joint and folds freely downward. That is a good hinge. A hinge the other way up is held only by its latch or elastic.
- **The accordion problem.** A zigzag needs alternate hinges to fold opposite ways. Half the hinges are then the wrong way up for lift. They need a latch or a spring that beats the lift moment (about 25 mN.m at the root in a pull-out, less further out). **All panels folding the same way** (a roll-up, each panel folding under the previous one) avoids this but only works for two or three panels before the stack thickness gets in the way.
- **Stiffness and repeatability.** The stop lever arm is the sheet thickness. For 5 mm foamboard it is 5 mm: workable, and a butt joint in 5 mm foam is the Corbin toy. For 0.6 mm PLA it is 0.6 mm: useless without printed stop blocks standing 4-6 mm tall at each hinge. Foam edges crush with use, so the open angle (the dihedral at each hinge) drifts. Paper-faced foamboard also swells and warps with humidity.
- **Negative loads.** At release the glider is nose-down and may briefly see lift the other way. A bottom hinge then wants to fold. The elastic must hold against that.
- **Mass.** Three hinges per side, tape and elastic: 0.5-1 g in foamboard. With printed stop blocks for a PLA plate: 1.5-2.5 g. [estimate]
- **Folded shape.** A stack beside the body, one chord long. It does not tuck back along the body and the pieces do not shingle. Not bird-like.
- **Later control.** None. An accordion changes span, not sweep, and its mid-positions are floppy.

**Verdict on A (my judgement):** cheapest and lightest, and a one-hinge version is proven at toy scale. As a multi-panel accordion it puts hinges across the main load, and its repeatability depends on crushable foam edges. Keep it as a fallback, in the one-fold "Corbin" form.

### 3.3 Idea B: finger-joint wing with a leading-edge string

How it would work: three flat pieces per side joined by pin joints with vertical axes. A string is tied at the tip, runs along the leading edge over each joint, into the body. Pulling the string straightens the joints. Elastic on the other side folds them.

**What is right about it**

- It is an in-plane fold, so the mechanism only fights drag (section 3.1).
- The pieces must sit in different layers to pass each other, so they shingle when folded and brace each other when open. That is how real feathers and the LisHawk and Di Luca wings work.
- A string along the leading edge in front of the pivots is geometrically what a bird has: a tendon along the front of the wing that goes tight as the wing opens. (Anatomical comparison only; no source was checked for this.)
- Strings and printed pin joints suit a printer and sheet stock.

**What sets stiffness and free play**

- **Rocking (dihedral and twist) at each joint** is set by the bearing pad, not the pin. Play angle is about (vertical clearance) / (pad diameter). With 0.2 mm clearance on a 15 mm pad that is about 0.8 deg per joint [estimate]. Three joints in series add up, and the tip panel gets the sum. Twist error on the outer panel is the most harmful kind of error, because it acts like an aileron.
- Fixes: wide pads (15-20 mm), a light axial preload (a thread or elastic through the pin pulling the plates together), and stops placed far from the pin so they also clamp twist.
- **Sweep angle at each joint** is set by the stop, provided the string or spring holds the joint hard against it in all flight conditions. Then string length, stretch and creep do not change the shape at all. **Rule: geometry comes from stops, never from string length.**
- Sweep error is forgiving. One degree of sweep error at the root moves the lift centre about 1 mm fore-aft, roughly 1% of chord after the mass shift is counted [estimate]. Twist and dihedral play matter more than sweep accuracy.

**One string, three joints**

- This is under-actuated, like a tendon-driven finger. Between the end positions the shape is not defined: joints move in order of least resistance, set by elastic stiffness, friction and airload. Left and right sides will not follow the same path.
- At the end position it is defined only if the string tension holds **every** joint on its stop at once. The tension at each joint is the same string, but the moment arms differ, so the weakest joint sets the tension needed.
- Friction adds up at each guide the string bends around. The tip joint, at the far end of the string, gets the least force and is the one most likely to stop short. LisHawk needed pre-stretched elastic just to beat friction between stacked feathers, and Di Luca measured slower motion under load from friction [full text, both].

**Variant B1: elastic folds, string and servo (or counter-spring) deploy. The owner's version.**

- Buys: wings fold by themselves when tension is released, so the glider can fold again in flight or on landing. It is the natural start of a later sweep-control system. LisHawk's tail works exactly this way [full text].
- Costs: something must hold tension for the whole flight. If that is the servo, it sits loaded the entire glide, and by the estimate in section 2 a 2 g servo cannot hold both wings against a pull-out directly. If it is a counter-spring behind a latch, the servo is not needed at all, and the design has quietly become B2.
- Failure direction: a broken string, a slipped knot or a dead battery **folds the wings in flight**.
- String stretch and creep directly reduce the hold-open tension unless there is a spring in series.

**Variant B2: spring deploys, something holds it folded, released once at the drop.**

- Buys: no actuator, battery or radio on the glider. The spring holds every joint on its stop with a known preload, which is what gave Di Luca 0.3-0.5 deg accuracy [full text]. Failure direction is safe: anything that breaks leaves the wings open. It opens at release, which is the lowest-speed moment of the drop.
- The thing holding it folded does not have to be on the glider. **The drone's carrier can trap the folded wings** (a cradle or a loop), so that letting go of the glider is also the deploy trigger. Tube-launched UAVs do this with the launch tube [secondary]. Added glider mass for the latch: zero.
- Costs: it deploys once per flight and is reset by hand. It cannot fold again in the air. It becomes reversible later by adding a servo that pulls a fold string against the spring, which is the Di Luca arrangement.
- Stored spring: do not leave a printed PLA spring loaded. A hobby report says PLA held under load for a few hours does not spring back fully [secondary, anecdote]. Use rubber or steel for the spring, and store the glider unfolded. Rubber also relaxes; no number was found.

**Failure modes for B, both variants**

- **Asymmetric deployment.** One wing opens first, giving roll and yaw at the worst moment. Nothing in a string-and-elastic system forces symmetry. Mitigation: join left and right mechanically at the root (one yoke, or meshed sector gears as in concept C1) so only the outer joints can differ.
- **Tip joint stops short** from string friction. Mitigation: in B2 put a small spring at each joint instead of relying on one string for all three.
- **Flutter of a loose segment.** A joint that is not hard on its stop is a free flap. Mitigation: preload on every stop; fewer joints.
- **String snags** on the drone carrier or on a plate edge.
- **Layer stack.** Three shingled layers of 5 mm foamboard are 15 mm thick at the root, which is a big step in the surface. Layers of 0.6 mm plate are fine, but ribs on one plate collide with the next plate, so ribs must be on outside faces only or the overlap zones must be left flat.

**Verdict on B (my judgement):** the right family, and well supported by precedent, but the precedents chose the reverse arrangement. Three joints on one string is the risky part: the tip joint is the least certain and the most sensitive. Build it as B2 first (spring opens, carrier holds it folded), with two pieces per side before three, and with left and right tied together at the root. Add the servo and the owner's elastic-fold direction later, when sweep control is wanted.

### 3.4 How the pieces overlap

- In-plane folding needs neighbouring pieces in different layers. Order them like roof shingles seen from the front: each outer piece lies **on top of** the next inner piece at its leading edge, so the airflow presses the edges down instead of lifting them. (General practice with lapped skins; not sourced.)
- Open, the overlap should stay at least 10-15 mm wide so the plates brace each other in bending. LisHawk reports needing feather stiffness "when the feathers are slightly overlapped" [full text].
- Sliding surfaces need to be smooth. Printed PLA on PLA slides acceptably; paper-faced foamboard on itself does not. A strip of tape on the rubbing zone is the cheap fix.

### 3.5 Balance and pitch while the wing opens

All **[estimate]** or reasoning; none of it is tested.

- Folded, the wing area sits far back and the glider is an arrow: very stable in pitch, little lift. That is a good shape to be released nose-down in.
- As the wings swing forward, the lift centre moves forward by roughly the same distance the wing's area centre moves (about 60 mm for a 70 deg swing on the reference glider).
- The balance point also moves forward, because here the wings are heavy (possibly 40% or more of the mass, section 2). With a 40% wing share it moves about 24 mm.
- Net: the stability margin shrinks by roughly 35 mm, about half a chord, between folded and open. The glider goes from very stable to whatever margin was designed in. **The balance point must be set and checked with the wings open.**
- This is the opposite of the robots in `sources.md`, where the wings were light and the balance point barely moved (LisHawk 18.6 mm on a much bigger airframe). Do not carry their numbers over.
- A nose-up trim (reflex, or the body-blended tail) only does its job once the wing is open. Until then the glider keeps diving. Pull-out distance therefore depends on how fast and how evenly the wings open.
- Opening at release keeps the opening loads low and gives the most height for the pull-out.

---

## 4. Other candidate concepts

Mass figures are added mass for both sides, **[estimate]**, using the part masses in section 2. Sketches are top views, nose up.

### C1. Geared swing wing (single pivot per side)

Each wing is one rigid piece. It pivots on one pin at its root and swings back to lie along the body; the two wings stack one over the other on the back. Two meshed printed gear sectors at the roots force left and right to move together. One rubber band opens them against printed stop pegs. The drone's carrier holds them folded.

```
 folded                     open
    ^                         ^
   /A\                       /A\
   |o|                 ______|o|______
   |||                /      | |      \
   |||               /_______|_|_______\
   |||                       | |
   |||  <- wings stacked     \_/
   \_/     over the tail
```

- Precedent: Walker and Schertle toy gliders (same motion, without the second axis); AquaMAV swept whole wings 90 deg with a gear linkage [full text].
- Parts: 2 pivot bosses with gear sectors, 2 pins, 1 band, 2 stops. About 7 parts, 1.5-2.5 g.
- Print: bosses, gears, stops. Sheet: the wings, untouched.
- Folded width: about one chord plus the body, so span drops by roughly 70-75%. Folded length is the half-span, so the tips reach past a short tail, as a swift's do.
- Risk: one wing must sit a layer higher than the other where they cross, so the two wings are at slightly different heights (a small built-in asymmetry). Gear backlash becomes sweep play unless the band preloads both wings onto their own stops.

### C2. Elbow and wrist coupled by a link (two pieces per side, one input)

An inner "arm" plate pivots at the shoulder. An outer "hand" plate pivots on the arm at the wrist. A thin rod from the body to the hand plate, running alongside the arm, forms a four-bar: when the arm swings back, the rod makes the hand fold back over the arm. One input moves both joints along one defined path.

```
 folded                     open
    ^                         ^
   /A\                       /A\
   |o|=\                _____|o|_____
   | | \\  hand over   / arm | | arm \____
   | |  \\ arm        /______|_|______hand\
   | |   ||                  | |
   \_/                       \_/
```

- Precedent: Bat Bot's one-input three-link arm [full text]. The bird version of this coupling is disputed in detail (section 1.2) but the flat linkage works as a machine.
- Parts: 4 pivots, 2 link rods with 4 small pins, 1 band, stops at shoulder and wrist, optional root gears. About 14-16 parts, 3-4.5 g.
- Print: pivots, link ends. Rod: 1 mm carbon or wire. Sheet: four plates.
- Folded: half the length of C1 and about two chords wide. Span drops by roughly 60-70%.
- Risk: link length sets the hand angle, so pin-hole slop in the link becomes sweep and twist play in the hand plate, the most sensitive piece. A four-bar can jam near a straight-line position if the lengths are chosen badly; this has to be drawn out before printing.
- Compared with B: the path between folded and open is defined, so left and right track each other. It is B with a rod where B has a string.

### C3. Feather fan (shingled plates on a swinging leading-edge arm)

An inner wing piece is fixed. A leading-edge arm pivots at the wrist. Four to six flat "feather" plates pivot on pins along the arm and stack like a hand fan when the arm swings back. Short non-stretch threads between neighbouring feathers go tight at full spread and set each gap; a light elastic keeps them spaced on the way.

```
 folded                     open
    ^                         ^
   /A\                       /A\
  _|o|_                 _____|o|_____
 |_| |_|               |fixed| |fixed|\
  \\ //  feathers      |_____|_|_____|\\\   arm
  || ||  stacked             | |     \\\\\  with 5
   \_/                       \_/      \\\\\ feathers
```

- Precedent: RoboSwift (4 feathers, press only), Di Luca (8) and LisHawk (9), the last two with flat feathers on pins [full text]. All were 280-330 g aircraft with servos.
- Parts: 2 arms, 8-12 feathers, as many pins, limit threads, elastic, wrist pivots. 30 or more parts, 3-6 g.
- Print: this is the concept that suits the 0.6 mm printed plate best. Each feather is a flat plate with one printed rib as its shaft. Foamboard is too thick to stack.
- Folded: area drops about 40% in the precedents [full text]; span drops less than C1 or C2 unless the fixed inner piece is short.
- Risk: the lowest repeatability of the set. Feather spacing depends on thread lengths and friction; gaps leak air; thin feather tips can flutter. Most parts and most assembly time. Most bird-like.

### C4. One-piece pivoting wing (scissor or oblique)

The whole wing stays one rigid piece. It turns 90 deg about a single vertical pin on top of the body to lie along the back, like a pocket-knife blade. A band and one stop bring it square.

```
 folded                     open
    ^                         ^
   /A\                       /A\
   |#|                 ______|_|______
   |#|  wing lies     |______ o ______|
   |#|  along the            | |
   |#|  back                 | |
   \_/                       \_/
```

- Precedent: stacked single-pivot wings on tube-launched UAVs [secondary, patent snippets only].
- Parts: 1 pivot, 1 pin, 1 band, 1 stop. 4 parts, 0.5-1 g.
- Print: pivot and stop. Sheet: the wing, uncut.
- Folded width: one chord. Span drops by about 75-80%.
- Repeatability: the best possible, because the wing itself has no joints. Only its yaw alignment to the body depends on the stop.
- Risk: while it turns, one tip leads and the other trails, so it rolls and yaws during opening. It must open fast and at low speed. A swept or bird-outline wing looks odd folded. Not bird-like at all, and no path to sweep control.
- Use: as the repeatability baseline that the others are measured against.

### C5. Tape-spring hinges (self-opening, self-locking fold lines)

A hinge made from a short strip with a shallow curve across its width, like a slice of steel tape measure. Straight, it is stiff and stays straight; bent past a threshold it buckles flat and folds; let go, it snaps straight again. One part is hinge, spring and lock. Used on chordwise fold lines it would make idea A open itself and resist bending both ways.

```
 front view, one hinge:

  open:    =====(=====     curved strip bridges the joint, locked straight
  folded:  =====
               )           strip buckled flat, storing energy
           =====
```

- Precedent: Baek et al. compare their self-locking facet to a tape spring and get 6 times more holding moment than folding moment [full text]. No tape-spring wing at our scale was found.
- Parts: 1 strip per hinge, glued or clipped. For two hinges per side: 4 strips, under 1 g if cut from steel tape (about 0.15 g each for 6 x 30 x 0.1 mm) plus mounts.
- Print: mounts only. A printed PLA tape spring would creep if stored folded [secondary, anecdote], so use steel.
- Folded: as idea A.
- Risk: the locking moment of a strip this small is unknown and could be below the 25 mN.m pull-out moment at the root. A gust could snap it through in flight. Bonding steel to foam or thin PLA is a weak point. The least proven concept here.

---

## 5. Comparison and shortlist

Everything in this section is **my judgement**, built on the estimates above. No concept has been built.

### 5.1 Comparison

Scale: ++ best, + good, o middling, - poor, -- worst.

| Concept | Added mass (est.) | Parts (est.) | Repeatable open shape | Span reduction (est.) | Reversible in flight / path to sweep control | Ease with printer and foamboard | Bird-like |
|---|---|---|---|---|---|---|---|
| A. Accordion, hard stops | 0.5-2.5 g | 8-14 | - (crushable edges, hinges across the lift load) | 50-70% | -- | ++ in foamboard, - in thin plate | -- |
| B1. String deploys, elastic folds, servo | 3-4.5 g plus servo 2.1 g plus radio and cell | 20+ | - (depends on held tension; tip joint uncertain) | 60-70% | ++ | o | ++ |
| B2. Spring deploys, carrier holds folded | 3-4.5 g | 18+ | o (+ if cut to two pieces per side) | 60-70% | o (add servo and fold string later) | o | ++ |
| C1. Geared swing wing | 1.5-2.5 g | 7 | + | 70-75% | + (replace band with a servo: this is LisHawk-style sweep) | + | + |
| C2. Elbow and wrist with link | 3-4.5 g | 14-16 | o | 60-70% | + | o | ++ |
| C3. Feather fan | 3-6 g | 30+ | -- | 30-50% | + | - (printed plate only, fiddly) | ++ |
| C4. One-piece pivot | 0.5-1 g | 4 | ++ | 75-80% | -- | ++ | -- |
| C5. Tape-spring hinges | under 1 g plus mounts | 6-8 | - (unknown lock strength) | 50-70% | -- | o | -- |

On a 20 g glider, 3-4.5 g of mechanism is 15-22% of the mass. On a 10 g glider it is not affordable; only A, C1, C4 and C5 fit there.

### 5.2 Shortlist (my judgement)

**First: C1, geared swing wing, band-opened, carrier-held.**

- Fewest joints that still fold the way a bird does (wings swept back over the body).
- The fold only fights drag. The wing plates stay uncut, so the planform work in `planform_and_gliding.md` carries over unchanged.
- Its root pivot, stop and gear pair are the first joint of B2 and C2. Nothing built for it is wasted.
- It becomes the sweep-control joint later by swapping the band for a servo.

Cheapest bench test (no flight):

1. Print one root pivot pair with gear sectors and stops; mount two foamboard wings on a scrap body.
2. Fold and release 20 times over a printed grid, phone camera straight above. Read each wing's sweep angle off the photos. The spread is the repeatability. Compare with Di Luca's 0.3-0.5 deg.
3. With the wings open, hang a small known weight (a coin on a thread) at each tip, then lift the tip with the same force. The tip height difference is the rocking play. Do the same at the leading and trailing edge of the tip for twist play.
4. Pull each tip straight back with a thread over the table edge and add weight until the wing leaves its stop. Force times distance from the pivot is the hold-open torque. Target: above about 10 mN.m [estimate, section 2].
5. Film the opening at the phone's slow-motion setting. Check that both wings arrive together and read the opening time.

**Second: B2 cut down to two pieces per side (arm and hand), added onto the C1 root.**

- This is the owner's preferred idea in its lowest-risk form: finger-style pin joints, shingled overlap, a leading-edge string, but spring-opened with the carrier holding it folded, and one outer joint instead of two.
- If the wrist joint proves repeatable, a third piece and then a servo and the elastic-fold direction (B1) follow naturally.
- If string friction makes the hand piece unreliable, the same plates take the C2 link rod instead.

Cheapest bench test (no flight):

1. One half-wing only, pinned to a flat board: arm plate, hand plate, wrist pin with a 15-20 mm bearing pad, stop, opening spring or band, fold string.
2. Same 20-cycle photo test as above, reading the hand plate's angle. Note which joint moves first and whether the hand ever stops short of its stop.
3. Same tip-weight test for rocking and twist play at the wrist. This is the number that decides whether a jointed wing is acceptable at all.
4. Hang weights on the fold string to find the tension needed to fold, and compare with what a 2.1 g servo could pull.
5. Leave it folded for 24 hours, then repeat step 2, to see whether the spring, string or plastic has relaxed.

**Kept as a reference, not a candidate:** C4. It costs four parts and gives the best-case repeatability number to hold the others against.

**Not recommended to start with:** C3 (too many parts for a repeatability-first project), C5 (lock strength unknown), B1 with three joints and a servo as a first build (heaviest, fails folded, and the precedents that tried tendons chose the other direction).

### 5.3 What is known and what is guessed

Known from sources:

- In-plane folding wings with pinned overlapping flat plates work on 280-330 g powered drones.
- A pre-stretched spring holding a tendon-driven fold against its travel gave 0.3-0.5 deg accuracy on one of them.
- Rubber-band swing wings with wire or dowel stops work on balsa and foam toy gliders, and needed an adjustable stop for trim.
- Spring-deploy plus wire-refold has been built on a 35 g glider wing module.

Guessed here:

- Every load, torque, mass and percentage in sections 2-5.
- That the carrier can hold the wings folded and release cleanly.
- That printed pin joints at this size can keep twist play small enough. This is the largest open question and the bench tests are aimed at it.
- The pull-out behaviour while the wings open. Nothing found covers a nose-down drop with wings opening.
