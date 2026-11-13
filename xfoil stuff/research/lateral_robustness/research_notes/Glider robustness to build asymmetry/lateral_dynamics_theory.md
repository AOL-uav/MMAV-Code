# Lateral-directional dynamics theory and literature: making a small asymmetric glider fly straight

Evidence tags: **[M]** measured / peer-reviewed experiment or flight data; **[A]** model / analysis (textbook theory, simulation, or my own derivation from standard equations); **[H]** hobbyist practice; **[S]** speculation.
Scope: 0.15-0.5 m span, 8-25 g, Re ~10k-30k, flat thin wings, swept tapered flying wing with tip fins, or a bird-like planform with a short bent-up tail and no fin.
Method note: about 18 tool calls. Textbook equation forms come from standard references and my own knowledge, not from fetched text. I list the books with publisher or ISBN links, but I did not re-read the page numbers, so equation numbering is not given. Nothing was downloaded into `sources\`. One NASA PDF fetch came back as unreadable binary and was not used.

## Q1. Standard theory: spiral, dutch roll, roll subsidence; steady turn under a constant asymmetry; the derivative trades; what maximizes robustness

### Takeaway
The project's AVL formula is the textbook steady-state solution of the lateral moment equations. Rearranging it shows a structural asymmetry. Turn per unit **rolling** asymmetry (twist / incidence / differential TE) falls without bound as |Clβ|·|Cnr|/Cnβ grows, so dihedral effect is the cure. Turn per unit **yawing** asymmetry (fin toe) can never fall below about 1/|Cnr|, so yaw damping (fin area × arm²) is the cure. Dutch-roll damping limits how far dihedral can be pushed relative to Cnβ.

### Cited Findings
- **Mode structure [A].** The linearized lateral equations give a quartic with three modes. Roll subsidence is a fast real root, set mainly by Lp (Clp). The spiral is a slow real root. The dutch roll is an oscillatory pair, with frequency set mainly by Nβ (Cnβ) and damping by Nr (Cnr), and it degrades as the ratio Lβ/Nβ grows. — [Etkin & Reid, *Dynamics of Flight: Stability and Control*, 3rd ed., Wiley 1996](https://www.wiley.com/en-us/Dynamics+of+Flight%3A+Stability+and+Control%2C+3rd+Edition-p-9780471034186); [Nelson, *Flight Stability and Automatic Control*, 2nd ed., McGraw-Hill 1998, ISBN 0-07-046273-9](https://www.mheducation.com/highered/product/flight-stability-automatic-control-nelson/M9780070462731.html); [Cook, *Flight Dynamics Principles*, 3rd ed., Elsevier 2013](https://shop.elsevier.com/books/flight-dynamics-principles/cook/978-0-08-098242-7)
- **Spiral criterion [A].** The constant term E of the lateral characteristic polynomial must be positive. In level flight this reduces to the classical condition (Lβ·Nr − Nβ·Lr) > 0, in non-dimensional form Clβ·Cnr − Cnβ·Clr > 0, with the usual signs Clβ<0, Cnr<0, Cnβ>0, Clr>0. Glide or climb angle adds terms through Clp/Cnp, because in steady turning flight the body roll rate p = −Ω sinθ. Etkin & Reid and Phillips both give the full E with the θ terms. — [Etkin & Reid](https://www.wiley.com/en-us/Dynamics+of+Flight%3A+Stability+and+Control%2C+3rd+Edition-p-9780471034186); [Phillips, *Mechanics of Flight*, 2nd ed., Wiley 2010](https://www.wiley.com/en-us/Mechanics+of+Flight%2C+2nd+Edition-p-9780470539750)
  - In the project's notation, D = Clr·Cnβ − Clβ·Cnr, so a stable spiral means **D < 0**. The project used the opposite sign convention, so check the sign before comparing configurations.
- **Rule-of-thumb derivative estimates [A].**
  - Wing Clr ≈ CL/4 for a near-elliptic loading, so it grows with glide CL and erodes spiral margin at slow, high-CL trims.
  - Wing-alone Cnr is small, roughly −CD0/4 minus a CL² term, which is why tailless aircraft have weak yaw damping.
  - Fin contribution: Cnr,fin ≈ −2·η·a_v·V_v·(l_v/b), where V_v = S_v·l_v/(S·b). The fin's yaw damping therefore scales with S_v·l_v², while its Cnβ scales with S_v·l_v.
  - Clp ≈ −(a/12)(1+3λ)/(1+λ) for a tapered wing.
  - Sources: Etkin & Reid and Nelson, both standard textbook strip-theory results.
- **Steady response to a constant asymmetry [A].** In a steady turn, p ≈ 0 (level) and the moment equations are:
  - Clβ·β + Clr·r̂ + ΔCl = 0
  - Cnβ·β + Cnr·r̂ + ΔCn = 0
  - Solving gives r̂ = (Clβ·ΔCn − Cnβ·ΔCl)/(Clr·Cnβ − Clβ·Cnr). This is exactly the project's expression. The side-force equation then sets the bank angle (≈ r̂·2V²/(g b) for small angles).
  - This is the trim/"steady sideslip-turn" analysis found in Etkin & Reid and Stevens & Lewis. — [Stevens, Lewis & Johnson, *Aircraft Control and Simulation*, 3rd ed., Wiley 2015](https://www.wiley.com/en-us/Aircraft+Control+and+Simulation%3A+Dynamics%2C+Controls+Design%2C+and+Autonomous+Systems%2C+3rd+Edition-p-9781118870983)
- **Free-flight practice confirms the structural trade [H].**
  - Fin area of about 5-8 % of wing area and 5-15° dihedral are typical.
  - Too much fin relative to dihedral gives a spiral; too little gives dutch roll. "The larger the vertical tail is, the more wing dihedral is needed to balance it."
  - A spiral that trim changes cannot fix indicates true spiral instability, not mis-trim.
  - Indoor models are deliberately turned with wing offset of about 3-5 % of span or with warp.
  - Sources: [Model Aviation, "Making FF models fly"](https://www.modelaviation.com/node/2076); [Model Aviation 1992/04, "Vertical tail size"](https://library.modelaviation.com/article/vertical-tail-size-models-1); [McKiou glider trimming tips, NAR](https://nar.org/?p=2840)
  - McKiou found that 12° dihedral gave spiral stability but 15° was needed for an easy transition from a hand launch [H].

### Inferences
- **Decomposing the steady formula [A, my derivation from the equations above].** Write the signs explicitly and let K = |Clβ|·|Cnr|/Cnβ.
  - Turn per unit rolling asymmetry: r̂/ΔCl = −1/(K − Clr). This sensitivity goes to zero as dihedral effect |Clβ| or yaw damping |Cnr| grows, or as Cnβ shrinks. Weathercock stability actually *amplifies* twist-induced turning, because the aircraft yaws into the roll-induced sideslip.
  - Turn per unit yawing asymmetry: r̂/ΔCn = 1/(|Cnr| − Clr·Cnβ/|Clβ|). Its best possible value, at infinite dihedral, is 1/|Cnr|. Only yaw damping (fin area × arm²) bounds it.
  - This matches the AVL finding that a fin trim tab fixes yaw-type asymmetry but not twist. Twist is a ΔCl, and its cure is the |Clβ|·|Cnr|/Cnβ ratio, not the fin.
- **Robustness design rule [A].** Maximize |Clβ|·|Cnr| and keep Cnβ only as large as dutch-roll frequency and gust weathercocking require.
  - Long fin arm helps twice: in Cnr (∝ l²) and in K. Raising fin area at a fixed short arm raises Cnβ almost as fast as Cnr, so K barely changes.
  - Fly at moderate CL, because Clr ≈ CL/4 subtracts from K.
- **Ceiling [A].** Dihedral is limited by dutch roll: a large Clβ/Cnβ ratio lowers dutch-roll damping. This is the classic tradeoff and matches the AVL observation that centre dihedral erodes dutch-roll damping. The usable optimum is the most dihedral that keeps dutch-roll damping above roughly ζ ≈ 0.1-0.2 [S: no small-glider criterion was found; MIL-F-8785C-style minima are for manned aircraft].

### Gaps
- I did not retrieve the page or equation numbers for the glide-angle (θ) correction in Etkin & Reid or Phillips. Phillips Ch. 9-10 is the most likely location.
- I found no textbook that explicitly states the "turn per unit asymmetry" sensitivity as a design metric. The decomposition above is my derivation and should be checked against AVL numbers.

## Q2. Papers on robustness or sensitivity of small UAV / glider flight to manufacturing asymmetry or tolerance

### Takeaway
I found no peer-reviewed study that runs a Monte Carlo of geometric build asymmetry against lateral trim or turn rate for a small glider or UAV. The literature consists of general geometric-uncertainty methods and anecdotal reports that asymmetry causes non-zero trim on MAVs. The project's AVL sensitivity study appears to be ahead of what is published.

### Cited Findings
- **General geometric-UQ methods [A].** Manufacturing error and wear are the two main sources of geometric uncertainty. The uncertainty is usually small enough to justify moment (linearized) methods instead of full Monte Carlo, but linearization holds only for small σ. — [Ghate & Giles, "Inexpensive Monte Carlo uncertainty analysis," SAROD-2005](https://people.maths.ox.ac.uk/gilesm/files/sarod05.pdf)
- **Morphing MAV flight tests [M, anecdotal within paper].** Slight left/right asymmetries in cable tension caused control difficulties and a non-zero trim condition, and control response drifted over a series of flights as the linkages changed. — [Flight testing a MAV using morphing for aeroservoelastic control (Univ. Florida group), via ResearchGate](https://www.researchgate.net/publication/228541258_Flight_Testing_a_Micro_Air_Vehicle_Using_Morphing_for_Aeroservoelastic_Control)
- **Antisymmetric twist couples into yaw [A].** On straight tapered wings without symmetric twist, an antisymmetric-twist rolling moment of the same sign as the roll rate always produces an opposite (adverse) yawing moment. So a built-in twist asymmetry is not a pure ΔCl; it carries a ΔCn of predictable sign. — [Hunsaker, Montgomery & Joo, Proc. IMechE G, 2020, doi:10.1177/0954410019886939](https://doi.org/10.1177/0954410019886939)
- **Flapping MAV [M, qualitative].** Open-loop roll instability after takeoff was attributed to manufacturing process errors, among other causes. — reported in [search summary of an insect-scale flapping MAV paper, Research (SPJ), doi:10.34133/research.0787](https://spj.science.org/doi/10.34133/research.0787). Low confidence: the paper itself was not read.

### Inferences
- Because antisymmetric twist produces both ΔCl and an adverse ΔCn, the effective asymmetry vector lies partly in the "twist" direction and partly in the "yaw" direction. AVL captures this automatically [A].
- A linearized (moment-method) UQ is defensible here. Turn rate is linear in ΔCl and ΔCn by the formula above, so the variance of r̂ follows directly from the tolerance covariances and the two sensitivities. A full Monte Carlo is only needed near D ≈ 0 [A].

### Gaps
- No published Monte Carlo of stability derivatives under build tolerance for a fixed-wing MAV was found in this pass. Better search terms to try: "asymmetric warp," "rigging tolerance," "aircraft dispersion analysis lateral trim," and Google Scholar on IMAV proceedings.

## Q3. Tailless / flying-wing directional stability: sweep, tip fins, split drag, bell spanload and proverse yaw, low-Re MAV wings

### Takeaway
A swept flying wing gets its Cnβ from sweep (weak, CL-dependent) plus tip fins on a short arm. Its Cnr is intrinsically small, so by the Q1 decomposition it is poorly placed to reject yaw-type asymmetry. Bird-scale vehicles can tolerate very low static yaw stability because of size scaling (Sachs). At low Re and low aspect ratio, measured Clβ departs from vortex-lattice predictions, so AVL lateral derivatives are suspect.

### Cited Findings
- **Bell spanload / Prandtl-D [M (flight), A (theory)].**
  - Bowers et al. built the Prandtl-D unpowered research gliders on Prandtl's 1933 bell spanload, which gives minimum induced drag for a given structural weight and was claimed to eliminate adverse yaw ("proverse yaw" from induced thrust near the tips). No vertical tail. — [NASA/TP-2016-219072, NTRS 20160003578](https://ntrs.nasa.gov/citations/20160003578)
  - A follow-up memo validates the bell spanload in flight. — [Bowers et al., "Experimental Flight Validation of the Prandtl 1933 Bell Spanload," NASA TM, NTRS 20210014683](https://ntrs.nasa.gov/citations/20210014683)
  - A NASA stability-and-control presentation reports a positive Cn_δa from flight data as "quantifiable evidence of proverse yaw." — [NTRS 20180007802](https://ntrs.nasa.gov/citations/20180007802). Only the search summary was available; the PDF was unreadable to me.
  - Historical note: Jones (NACA TN 2249) independently derived a nearly identical bell load. — per the [NTRS record](https://ntrs.nasa.gov/citations/20160003578)
- **Tip fins: cant and size [M/A].** The AIAA J. Aircraft paper studies tip-fin cant and size relative to the wing, and finds that wing flexibility interacting with tip fins usually lowers flutter speed. That matters for thin flexible printed or foam wings. — ["Flutter and Directional Stability of Aircraft with Wing-Tip Fins: Conflicts and Compromises," J. Aircraft, doi:10.2514/1.C031978](https://arc.aiaa.org/doi/10.2514/1.C031978)
- **Tip fins on swept and delta wings [A].** Fins near the tips of swept and delta wings have been used to supply directional stability. — [NASA TN D-8260](https://ntrs.nasa.gov/api/citations/19760019075/downloads/19760019075.pdf)
- **Flying-wing model practice [H].**
  - Tip fins with some toe-in increase weathercocking; the author openly says the choice of toe-in is debatable. — [Apogee Rockets Newsletter #15, "Theory and Practice of Using Flying Wings"](https://www.apogeerockets.com/education/downloads/Newsletter15.pdf)
  - A flying-wing design paper notes that weak directional stability gives spiral stability. This is consistent with Q1: low Cnβ means a large K. — [Hamada, Athens J. Technology 2018](https://www.athensjournals.gr/technology/2018-5-3-2-Hamada.pdf)
- **Folding wingtips as fins [M].** Wind-tunnel tests of folding wingtips found that all configurations had positive Cnβ (directionally stable). — [Flight Dynamics and Control Using Folding Wingtips, Aerospace 4(2):19, MDPI](https://www.mdpi.com/2226-4310/4/2/19)
- **Low-Re, low-AR roll derivatives [M].**
  - Linehan & Mohseni built a theory for Clβ of thin low-AR flat plates at low Re that accounts for leading-edge reattachment and tip-vortex roll-up. They validated it on 13 flat-plate wings, with accuracy up to CL ≈ 0.7. Tip-vortex roll-up at low Re is *destabilizing* in Clβ. — [Linehan & Mohseni, AIAA J., doi:10.2514/1.J057160 (NSF PAR copy)](https://par.nsf.gov/servlets/purl/10097585)
  - The companion paper finds the classic Weissinger vortex arrangement inconsistent with the flow over low-AR wings with large tip chords, so lifting-line and VLM stability derivatives are inaccurate there. — [NSF PAR 10097590](https://par.nsf.gov/servlets/purl/10097590)
  - Shields & Mohseni found strong nonlinearities in sideslip loads, with the windward tip vortex keeping flow attached. — [APS DFD 2011 abstract](https://archive.aps.org/dfd/2011/l23/4); see also Shields & Mohseni, "Inherent stability modes of low-aspect-ratio wings," J. Aircraft 52 (2015) 141-155 (cited in [Proc. IMechE G review, doi:10.1177/0954410019878023](https://journals.sagepub.com/doi/ref/10.1177/0954410019878023))
  - Cross-coupling derivatives on MAVs are comparable in size to the primary ones and cannot be neglected. — [APS DFD 2010 abstract](https://archive.aps.org/dfd/2010/ex/6)
- **Flat-plate Clβ from tip vortices [M].** Flat uncambered plates show a Clβ caused by asymmetric tip-vortex development in yaw. — [APS DFD 2012 abstract](https://meetings-archive.aps.org/dfd/2012/e9/4/)
- **Bird and miniscale yaw stability without a fin [A, simulation].**
  - Small flyers can have adequate dynamic yaw response even with very small static yaw stability, because yaw inertia falls faster with size than the restoring moment does. Required yaw damping also falls with size.
  - The wing alone can supply the needed yawing moment if it has suitable features. Removing the fin affects the spiral only partially, and asymmetric flight can be managed without one.
  - Source: [Sachs, "Why birds and miniscale airplanes need no vertical tail," J. Aircraft 44(4):1159-1167, 2007, doi:10.2514/1.20175](https://portal.fis.tum.de/en/publications/why-birds-and-miniscale-airplanes-need-no-vertical-tail/); [Sachs, "Yaw stability in gliding birds," J. Ornithology 2005](https://portal.fis.tum.de/en/publications/yaw-stability-in-gliding-birds/)
  - Contested: other work argues that many fin-less bird planforms are dynamically unstable unless they use lateral tail tilt or twist for yaw control. — [Stanford thesis/record "Avian-inspired rudderless flight with morphing wings and tails"](https://purl.stanford.edu/gb160vd5738); this is per search summary only.
- **Rules for stable gliding animals [A].** Thomas & Taylor's rules of thumb: forward-swept wings in slow flight, tip wash-out, and extra surfaces at longitudinal dihedral. They also argue that pendulum stability and drag-based stability matter more for animals than for aircraft. — [Thomas & Taylor, "Animal flight dynamics I. Stability in gliding flight," J. Theor. Biol. 212(3), 2001, doi:10.1006/jtbi.2001.2387](https://ora.ox.ac.uk/objects/uuid:39c37f27-3793-45e2-9b0e-156495ace658)

### Inferences
- **Swept wing with tip fins [A].** The fin arm is roughly the sweep offset of the tip fins behind the CG, typically 0.3-0.5 of the semispan in x on a 25-30 cm wing. Cnr,fin ∝ l_v² is therefore small, which caps rejection of fin-toe asymmetry (floor ≈ 1/|Cnr|).
  - Practical consequence: hold fin toe tolerance tighter than wing twist tolerance on the flying wing, or move fins aft on booms.
  - Sweep also adds Clβ ∝ CL·sin2Λ, which helps reject twist asymmetry at high CL.
- **Bird planform without a fin [A].** It relies on wing-generated Cnβ and very small Cnr. By the Q1 decomposition, its sensitivity to yaw-type asymmetry is high. Its sensitivity to twist asymmetry can still be low if Cnβ is small and Clβ is large.
- **AVL validity at this Re [S].** Given Linehan & Mohseni, AVL's Clβ for flat thin wings at Re 10-30k may be off in magnitude at moderate CL; both the sign and size of the tip-vortex contribution are at issue. Treat the AVL ranking of designs as more reliable than the absolute turn rates.
- **Bell spanload [A/S].** Proverse yaw mainly addresses aileron yaw (Cn_δa), not Cnβ or Cnr. Its relevance here is that a differential-TE or twist asymmetry on a bell-loaded wing may produce a proverse ΔCn rather than an adverse one, changing the sign of the coupling found by Hunsaker et al. for straight wings [S].

### Gaps
- I did not fetch Nickel & Wohlfahrt, *Tailless Aircraft in Theory and Practice* (AIAA Education Series, 1994, ISBN 1-56347-094-2). Its fin-sizing and toe/split-drag guidance is not cited here.
- Not retrieved: Horten and Northrop primary sources; Kroo's tailless work; Torres & Mueller (AIAA J. 2004, low-AR wings at low Re), whose results are mainly longitudinal to my knowledge; Zimmerman-planform lateral data.
- No quantitative Cnβ-versus-toe/cant dataset for small flying wings was found.

## Q4. CG height / pendulum effect, polyhedral vs centre dihedral, canted tip fins, ventral fins

### Takeaway
A low CG under the wing does not create a "pendulum" restoring torque in free flight. The real effects are wing-fuselage sidewash (high wing adds effective dihedral) and fin-height terms. Canted fins and ventral or dorsal surfaces change Clβ through the fin's side-force moment arm about the roll axis.

### Cited Findings
- **High wing vs low wing [A, technical journalism].** The high-wing vs low-wing difference is due to wing-fuselage interference, not pendulum action. A high wing adds stable dihedral effect; a low wing subtracts it. — [Kitplanes "Wind Tunnel" (Barnaby Wainfan)](https://www.kitplanes.com/wind-tunnel-135/)
- **NACA tests [M].** Wing-fuselage interference changed the sign of effective dihedral at low α as the wing moved from low to high position. — [NACA report via NTRS 19930091107](https://ntrs.nasa.gov/api/citations/19930091107/downloads/19930091107.pdf)
- **Popular teaching [H].** The FAA PHAK and many hobby sources still list a "keel/pendulum" effect. — [RC Scale Builder dihedral tutorial](https://www.rcscalebuilder.com/Tutorials/dihedral/dihedral.pdf). The conflict with the sources above is noted explicitly.
- **Animals [A].** Thomas & Taylor argue that pendulum stability is more significant for gliding animals than for aircraft. — [Thomas & Taylor 2001](https://ora.ox.ac.uk/objects/uuid:39c37f27-3793-45e2-9b0e-156495ace658)
  - This partly conflicts with the "no pendulum" view. The resolution in textbook theory [A] is that a CG below the aerodynamic centre adds no gravity torque. It does make sideslip-generated side forces, such as those on a body, keel or fins, act with a vertical moment arm about the CG, which produces a Clβ contribution.

### Inferences
- **Fins and keel surfaces [A].** In Clβ, the sign of each fin's contribution is set by its centre of pressure height relative to the CG: above the CG is stabilizing, below is destabilizing.
  - Up-canted tip fins (above the wing) add stabilizing Clβ. Down-canted or ventral fins reduce it.
  - Tip-fin cant reduces effective Cnβ by about cos² of the cant angle while adding a Clβ term.
  - Ventral fins add Cnβ and Cnr while reducing Clβ, a useful knob if dutch roll, not spiral, is the limiting mode.
- **Polyhedral vs centre dihedral [A].** Per unit tip rise, dihedral nearer the tips (polyhedral) produces more Clβ, because the Clβ contribution of a panel scales roughly with its spanwise lift-moment arm. Centre dihedral gives the least Clβ per degree.
  - Both raise Clβ, and so raise the twist-rejection metric K.
  - Polyhedral also tends to raise Clp less [A, unverified quantitatively].

### Gaps
- I found no quantitative small-MAV measurement isolating CG height or fin cant effects on Clβ at Re < 50k.

## Q5. Vertical tail volume and fin-arm rules; yaw damping vs arm²

### Takeaway
Typical vertical tail volume V_v = S_v·l_v/(S·b) is about 0.02 for sailplanes and about 0.04 for light aircraft. Fin yaw damping scales as S_v·l_v², while Cnβ scales as S_v·l_v. A long arm with a modest fin therefore maximizes |Cnr| per unit Cnβ, which is exactly what both robustness sensitivities want.

### Cited Findings
- **Raymer's historical tail volumes [A, historical survey].** Sailplanes: c_VT ≈ 0.02. Homebuilts and general aviation: ≈ 0.04. — [Raymer, *Aircraft Design: A Conceptual Approach*, AIAA Education Series](https://arc.aiaa.org/doi/book/10.2514/4.104909) (table values from my knowledge of the text, not re-checked in this pass)
- **Fin derivatives [A].** Cnβ,fin ≈ η·a_v·(1 − dσ/dβ)·V_v and Cnr,fin ≈ −2·η·a_v·V_v·(l_v/b). — [Etkin & Reid](https://www.wiley.com/en-us/Dynamics+of+Flight%3A+Stability+and+Control%2C+3rd+Edition-p-9780471034186); [Nelson](https://www.mheducation.com/highered/product/flight-stability-automatic-control-nelson/M9780070462731.html)
- **Free-flight practice [H].** Fin area of 5-8 % of wing area for most free-flight models. — [Model Aviation](https://www.modelaviation.com/node/2076)
- **Scaling [A].** Sachs shows that yaw inertia falls faster than restoring moment with size, so small vehicles can accept lower V_v for the same dynamic yaw response. — [Sachs 2007](https://portal.fis.tum.de/en/publications/why-birds-and-miniscale-airplanes-need-no-vertical-tail/)

### Inferences
- **Ratio Cnr/Cnβ from the fin [A].** Cnr,fin/Cnβ,fin ≈ −2·l_v/b. Doubling the arm doubles the twist-rejection metric contribution K for the same Cnβ. It also halves the fin area needed for a given Cnβ, which saves mass and drag.
- **Tip fins vs boomed fins [A].** On a swept flying wing, l_v/b is roughly 0.1-0.2. A boomed fin or a bird tail with a vertical element can reach 0.3-0.5, roughly a 2-5× improvement in Cnr per unit Cnβ.

### Gaps
- No published V_v data was found specifically for 0.15-0.5 m gliders at Re < 30k. Fin effectiveness a_v at Re ~5-10k on a fin chord of a few cm will be low and nonlinear, and no data was found.

## Q6. Passive "self-correcting heading" mechanisms, including Kai's additions: aeroelastic and pendulum devices

### Takeaway
No passive mechanism in the literature returns an aircraft to a fixed *heading*. Aerodynamic stability only restores attitude and sideslip, so heading is neutrally stable: the yaw-angle root is zero, by symmetry of the equations of motion.
- The best passive outcome is minimizing steady turn rate (Q1) and ensuring the spiral is convergent.
- Flexible-wing MAVs (Florida "adaptive washout") give passive disturbance attenuation, but there is no evidence that flexibility cancels built-in asymmetry.
- Pendulum-linked controls are historical and hobby devices. Physically, a pendulum on the aircraft senses apparent gravity (sideslip/lateral acceleration), not bank or heading.

### Cited Findings
- **Neutral heading [A].** Heading does not appear in the aerodynamic forces, so the lateral equations carry a zero root for ψ. No aerodynamic configuration provides heading restoration without an external reference. This follows from standard lateral equation structure. — [Etkin & Reid](https://www.wiley.com/en-us/Dynamics+of+Flight%3A+Stability+and+Control%2C+3rd+Edition-p-9780471034186)
- **Florida flexible-wing MAVs [M (flight) / A].** Ifju, Waszak et al. flew flexible-wing MAVs from 18 in down to 5 in span. Passive "adaptive washout" comes from membrane extension and structural twisting as speed and attitude change. The authors credit it with reduced gust sensitivity and improved stability. — [Waszak & Jenkins, AIAA 2001-4005, NTRS 20010081319](https://ntrs.nasa.gov/api/citations/20010081319/downloads/20010081319.pdf); [Ifju et al., AIAA 2002-0705, NTRS 20020024082](https://ntrs.nasa.gov/api/citations/20020024082/downloads/20020024082.pdf); [Waszak, Davidson & Ifju, AIAA 2002-4875, NTRS 20030005489](https://ntrs.nasa.gov/api/citations/20030005489/downloads/20030005489.pdf)
  - The Stanford/Ifju/Albertani/Shyy review (Prog. Aerosp. Sci. 2008) notes that low-AR wings are susceptible to roll instabilities and that membrane aeroelasticity is a mitigation. — [HKUST record, "Fixed membrane wings for MAVs"](https://researchportal.hkust.edu.hk/en/publications/fixed-membrane-wings-for-micro-air-vehicles-experimental-characte/)
  - Battens strengthen local regions and improve gust rejection, though their flight-dynamics effects are less studied. — [Aeronautical Journal, "Aeroelastic effects of battens on the flight dynamics of a MAV"](https://www.cambridge.org/core/journals/aeronautical-journal/article/aeroelastic-effects-of-battens-on-the-flight-dynamics-of-a-mav/DE7A824445F189255EE11E57922CEE56)
  - I read only abstracts and summaries of these papers; no quantitative trim-asymmetry data was found.
- **Tip-fin flutter risk [M/A].** Tip fins on flexible wings usually reduce flutter speed. — [J. Aircraft doi:10.2514/1.C031978](https://arc.aiaa.org/doi/10.2514/1.C031978)
- **Pre-WWI automatic stability systems [A, historical review].** These used pendulum sensors, aerodynamic sensors, thrust-vector changes and gyros; work stopped with WWI and resumed in the 1920s. — [Kuzmin, "On the History of Early Aircraft Automatic Stability Control Systems," 2025](https://vestnik-pp.samgtu.ru/0205-9606/article/view/686671)
- **Pendulum control in free-flight models [H].** Free-flight scale models used pendulum control of rudder, ailerons or elevators before WWII; P. E. Norman used pendulums for aerobatics. — [Model Aviation 1998/10, "Free Flight Sport and Scale"](https://library.modelaviation.com/article/free-flight-sport-and-scale-30); ["Pits and Pendulum," Model Aviation (Haight)](https://prod-8babe12b-9af9aaae-vsp3hfy2.app.devpanel.com/article/pits-and-pendulum)

### Inferences
- **Does flexibility cancel built-in twist asymmetry? [A/S]**
  - If each wing half washes out in proportion to its own lift (swept-back bending-torsion coupling, or a compliant trailing edge that unloads under pressure), it acts like a reduced local lift-curve slope, a_eff = a/(1+k).
  - A built-in incidence difference Δi then produces ΔCl reduced by about 1/(1+k). The steady turn rate scales directly with ΔCl, since Clp does not enter the steady turn, so the turn is reduced by the same factor.
  - But Clβ from dihedral and sweep, which is lift-based, is also reduced by about 1/(1+k), and K falls with it. To first order, the turn per unit Δi is therefore roughly unchanged by uniform flexibility [S].
  - Flexibility can help only if it attenuates the asymmetric load more than the restoring Clβ. Example: a compliant trailing edge that relaxes a built-in *differential camber/TE deflection* (a pre-load), as opposed to an incidence error.
  - A flexible trailing edge would partly "self-fair" a bent TE tab under load [S, no data found]. A built-in TE warp that stays put under load gets no benefit [A].
- **Pendulum physics [A].**
  - In a coordinated turn or steady sideslip-free flight, a body-mounted pendulum hangs along the apparent gravity vector, which is aligned with the body z-axis (the same principle as the slip ball). It cannot sense bank angle or heading.
  - It deflects only under lateral specific force, mainly from sideslip-induced side force. A pendulum-linked rudder is therefore effectively a sideslip-to-rudder (Cnβ-augmenting) or sideslip-to-aileron (Clβ-augmenting) feedback with lag and inertia.
  - Any benefit at most equals that of more fin or more dihedral, and the pendulum mass and friction at 8-25 g make it unattractive [S].
  - A *hanging payload* below a parafoil gives genuine restoring torque only because the canopy and payload are separate bodies with distinct drag and lift points. That does not transfer to a rigid glider [A].

### Gaps
- No analysis or measurement was found showing that aeroelastic tailoring reduces trim sensitivity to build asymmetry. This is an open question, testable in AVL by scaling a and Clβ together.
- No modern quantitative study of pendulum-actuated controls on small gliders was found. The pre-WWI literature (e.g. *Flight* 1911) was not retrieved.
- No literature passive "heading-hold" mechanism was found. Absent an external reference such as a magnetic, optical or sun compass, this is likely impossible in principle [A].
