# Pure wing / reflex lip / small tail at 0.40 m span: comparison and modular article

2026-10-11. Model output, independently reviewed twice; the wording follows
the review. **Nothing here is validated by measurement, and no architecture
or planform is selected.** The purpose is to size one modular article that
can fly all three configurations so that flight decides.

Code: `pure_wing_v2_*.py`, `reflex_small_polars.py`, `arch_*.py`,
`arch_v2_*.py`. Outputs: `results/runs/rigid_glider/pure_wing_v2/`,
`reflex_small/polars/`, `arch_compare/` (first pass, CG scan censored at
+20% margin; superseded), `arch_compare_v2/` (uncensored, used below).
Figures: `results/runs/rigid_glider/figures.html`. Tests: 132 of 132 pass
(`arch_compare_v2/unittest_all_output.txt`).

## Terms

- **CG window:** the range of CG over which the model finds a statically
  stable trim with CL between 0.2 and 0.5 (an exploratory band). Scan step
  0.5% MAC. Unless stated, the window must hold for both XFOIL transition
  settings (Ncrit 5 and 9), on 160-panel polars.
- **Evidence tiers:** "any" counts every such trim. "Strict" counts only
  trims that do not sit on a polar interval where upper-surface transition
  is jumping. Under the strict tier the pure-wing window is zero, the NACA
  0006 tail window is zero to within one scan step (0.46 mm on P07 at
  16 N/m²), NACA 0008 tails are not zero (0.46–2.77 mm in most P07 and P09
  rows), and the 20% / 10 outboard lip has 1–7 mm in 15 of the 18 lip cells
  of the Section 4 table (zero only under form (c) at 8 N/m²).
  *Erratum 2026-10-11, after a third review: this sentence previously read
  "Almost every window in this document is zero under the strict tier, for
  all three architectures", which was wrong for the lip. Sources:
  `arch_compare_v2/tableD_corrected.txt`,
  `arch_compare_v2/item4i_tail_effectiveness.csv`. The strict tier is the
  one tier on which lip and tail differ; lip versus tail remains unresolved.*
- **Candidate wings**, all straight-tapered, untwisted, 0.40 m span:

  | Wing | AR | Taper | Quarter-chord sweep | Root / tip chord | Area | NACA 0006 thickness root / tip |
  |---|---:|---:|---:|---|---:|---|
  | P07 | 4.5 | 0.5 | 25 deg | 119 / 59 mm | 356 cm² | 7.1 / 3.6 mm |
  | P09 | 6 | 0.5 | 15 deg | 89 / 44 mm | 267 cm² | 5.3 / 2.7 mm |
  | P12 | 6 | 1.0 | 35 deg | 67 / 67 mm | 267 cm² | 4.0 / 4.0 mm |

  NACA 0008 is one third thicker.
- **Final deck:** all numbers in the tables below are from one AVL model:
  20 x 32 panels per half-wing, 3 deg dihedral, a centre fin of 6% of wing
  area at 1.5 root chords aft (for the tail case, the tail plus a 6% fin at
  the tail station).

## 1. Pure wing

Study: 888 cases (12 planforms, NACA 0006/0008/0010, spans 0.25–0.50 m,
wing loading 4–32 N/m²).

- 42% of cases have no CG window at all under both transition settings.
- Where one exists it is narrow: maximum 4.0 mm, median 0.75 mm. All of
  them sit on the transition feature; off it the widest is 0.5 mm.
- Windows shrink steadily as wing loading rises: median 2.0 mm at 4 N/m²,
  0.56 mm at 12, 0.28 mm at 32. Nothing exceeds 2 mm at 16 N/m² or above.
- A simple-sweep-theory bracket on the section lookup reduces the number of
  cases with a window from 107 to 64 (of 216 at 0.40 m) and the widest from
  4.0 to 3.1 mm.
- By section, a window exists in 84% of NACA 0006 cases, 60% of 0008, 26%
  of 0010. Smaller spans do slightly better (77% of cases at 0.25 m, 51% at
  0.40 m) but move toward the Re 10k floor of the polar data.

Pure-wing window on the three candidates, both transition settings:

| Wing | 6 N/m² | 8 N/m² | 12 N/m² | 16 N/m² |
|---|---:|---:|---:|---:|
| P07 | 2.3 mm | 1.8 mm | 0.5 mm | 0 mm |
| P09 | 3.1 mm | about 2 mm | about 1.4 mm | about 1 mm |
| P12 | 2.7 mm | about 2 mm | about 1.3 mm | about 1 mm |

On 240-panel polars the windows are equal or smaller (0 to 2.4 mm). These
differences between wings are below the model's resolution.

**Reading:** a pure-wing trim may exist on a light article, in a CG band of
one to three millimetres that depends on where a laminar separation closes.
Worth trying because it costs nothing on a modular article. Not expected to
be the repeatable configuration.

## 2. Reflex lip

XFOIL lip polars at small scale (`reflex_small/polars/`): six lips on NACA
0006 and 0008, Re 15k–100k, Ncrit 5 and 9, 160 and 240 panels.

- Fraction of the inviscid lip moment retained at Cl 0.25 ranges from 0.00
  to 0.45 below Re 40k; 29 of 84 cells are under 0.10. The 20% / 10 deg lip
  holds 0.21–0.45 at Re 30k–40k and collapses below about 30k. At Re
  60k–100k, the 15% / 8 and 20% / 10 lips retain 0.4–0.57; smaller lips
  less.
- NACA 0006 retains more than 0008 at low Re.
- Ncrit 9 changes the lip moment by −105% to +38% where it is large enough
  to matter; 240 panels raise it by 3–25%. Several Ncrit 9 polars at Re
  60k–100k are partial.

The coupled lip result is a **bracket**, because the model treats two parts
of the lip's effect differently. From the AVL decomposition on P07, 52–56%
of a lip's nose-up moment is the section couple and 38–48% is lift moved
inboard on the swept wing (the two estimates of the second part do not
close to better than 10%).

| Form | Section couple | Lift redistribution | Character |
|---|---|---|---|
| (a) | AVL couple plus viscous-minus-inviscid increment | full inviscid | oversubtracts the couple; redistribution optimistic |
| (b) | XFOIL viscous lip moment replaces the AVL couple | full inviscid | best treatment of the couple; redistribution optimistic |
| (c) | as (b) | scaled by the XFOIL retained fraction | heuristic pessimistic bound, single scale factor per case |

Outboard lips (40–100% of semispan) do better than full-span ones. Median
window over the six wing/section cases, outboard lip, 160 panels; a cell is
marked when at least four of six cases reach 3 mm:

| Lip (chord / angle) | Wing loading | (a) | (b) | (c) |
|---|---:|---|---|---|
| 10% / 3 | 6 | 3.1 mm | 3.7 mm, meets | 1.8 mm |
| 15% / 5 | 6 | 3.2 mm, meets | 4.2 mm, meets | 1.5 mm |
| 15% / 8 | 6 | 4.0 mm, meets | 6.0 mm, meets | 1.5 mm |
| 15% / 8 | 12 | 3.2 mm | 5.2 mm, meets | 0.7 mm |
| 20% / 10 | 6 | 7.1 mm, meets | 10.1 mm, meets | 2.4 mm |
| 20% / 10 | 12 | 7.4 mm, meets | 10.0 mm, meets | 1.7 mm |
| 20% / 10 | 24 | 6.9 mm, meets | 9.6 mm, meets | 2.5 mm |

Many cells are within 0.5 mm of the 3 mm line, so the threshold does not
separate lips cleanly. Under form (c) no lip meets it anywhere. The three
forms share a common CG band in only 9 of 30 cases, 0–4 mm wide; reducing
the redistribution moves the window 6–10 mm aft. Part of every lip window at
low wing loading is the pure wing's own section-moment hump.

Trim CL changes by 0.02–0.07 per degree of lip angle. The lip costs lift:
the AVL part of the alpha penalty at CL 0.3 on P07 is 0.6 deg for an
outboard 10% / 3 lip and 2.7 deg for 20% / 10.

## 3. Small tail

Tail of 8–10% of wing area, span 140 mm, quarter chord 2.0 root chords aft
of the root leading edge, incidence −3 deg.

- Its trim exists in the inviscid model independently of wing section
  moment. It depends instead on an unmodelled tail surface at Re 10k–14k
  (below all polar data) lying in the wing's wake.
- Median window over six wing/section cases, 10% tail: 10.3 mm at 6 N/m²
  falling to 5.6 mm at 24. With tail lift at 0.59 of the AVL value: 7.1 mm
  falling to 2.9 mm. With 0.81: 8.6 to 4.3 mm
  (`arch_compare_v2/item4i_tail_effectiveness.csv`).
- The tail-on neutral point is uncertain by about 4–5 mm on P07: 82.9 mm
  with the tail in the wing plane, 78.1–78.5 mm with it raised 5–10% of root
  chord. The window width does not change (9.7 mm in every variant); its
  position does.
- The coupled tail trims still sit on the wing's transition feature; the
  strict-tier window is zero for NACA 0006 to within one scan step, but not
  for NACA 0008 (erratum 2026-10-11; see Terms).
- The "0.59 lift" tail rows are the 5%-area tail model: its lift is
  0.58–0.60 of the 10% tail's across the three wings (0.603 on P07). Label
  clarified 2026-10-11; no numbers changed.
- Trim CL changes by 0.05–0.12 per degree of tail incidence.

## 4. Comparison

Both-setting CG window, NACA 0006, final deck, uncensored. Each cell: width
and CG range in mm aft of the root leading edge, at 8 / 16 N/m².

| | P07 | P09 | P12 |
|---|---|---|---|
| Pure | 2 (67–69) / 0 (70) | 2 (42–44) / 1 (43–44) | 2 (79–81) / 1 (80–81) |
| Lip 20% / 10 outboard, form (a) | 9 (52–60) / 8 (52–60) | 7 (32–38) / 7 (32–38) | 13 (57–70) / 13 (57–70) |
| same, form (b) | 12 (47–58) / 12 (47–58) | 8 (29–37) / 8 (29–37) | 16 (51–67) / 16 (51–67) |
| same, form (c) | 6 (58–63) / 7 (56–63) | 3 (37–40) / 4 (36–40) | 4 (70–74) / 4 (70–73) |
| Tail 10%, AVL lift | 11 (63–74) / 9 (65–75) | 10 (37–47) / 9 (39–48) | 5 (77–83) / 4 (79–83) |
| Tail 10%, 0.59 lift | 7 (64–72) / 6 (67–72) | 7 (39–46) / 6 (40–46) | 4 (78–82) / 3 (79–82) |

What can be said:

- **The pure wing is below both the lip and the tail on every metric.**
- **Lip versus tail is unresolved.** Over 30 cases, the 20% / 10 lip is
  wider than the full-effectiveness tail in 10 (form a), 15 (form b) or 1
  (form c); against the 0.59 tail in 17, 23 or 4. P12 favours the lip, P09
  the tail, P07 is mixed.
- The lip and tail rows compare the largest tested feature of each kind;
  smaller lips and tails have narrower windows.
- Each prediction rests on a different unvalidated quantity: pure on the lift
  coefficient at which a laminar separation closes; lip on low-Re lip
  authority and how much of the inviscid lift redistribution survives; tail
  on the lift of a 25 mm chord surface at Re 10k in a wake.

Lateral (inviscid): every configuration needs a fin. At CL 0.3 with 3 deg
dihedral, Cn_beta per rad:

| Wing | Bare | 4% tip fins | 6% centre fin at 1.5 root chords |
|---|---:|---:|---:|
| P07 | 0.0016 | 0.008 | 0.017 |
| P09 | −0.0001 | 0.005 | 0.017 |
| P12 | 0.0031 | 0.0085 | 0.0097 |

So the "pure wing" configuration that has useful yaw stiffness on P07 and
P09 carries a centre fin on a short boom; on P12 tip fins are nearly as
good. Three degrees of dihedral takes Cl_beta from about −0.03 to −0.06
and costs about 0.0015 of Cn_beta. Roll from a left/right wing incidence
mismatch is about 0.005 per degree; from a lip-angle mismatch about 0.0024
per degree.

## 5. Mass

No mass is known. An exploratory component budget
(`arch_compare_v2/item6_mass_cg_budget.csv`; shell or sparse-infill wing,
pod, boom, tail, fin, and nose ballast to reach each target CG) gives, for
P07 with NACA 0006:

| Case | Pure | Lip 20% / 10 | Tail 10% |
|---|---|---|---|
| Light | 24 g, 6.7 N/m², ballast 2.8 g | 28 g, 7.6 N/m², ballast 6.0 g | 28 g, 7.6 N/m², ballast 5.0 g |
| Heavy | 46 g, 12.8 N/m², ballast 5.3 g | 53 g, 14.5 N/m², ballast 11.4 g | 53 g, 14.7 N/m², ballast 9.8 g |

Across all three wings the budget spans about 6–18 N/m². Ballast is 11–23%
of all-up mass, carried a quarter of a root chord ahead of the root leading
edge; the lip needs the most. Solid PLA would be about 60 N/m², far outside
anything studied. The pure wing favours the light end; lips have more
authority at the heavy end.

## 6. Modular article

One wing, flown as pure wing, then with lip strips, then with the tail.

**Planform is Kai's choice.** P07 has the largest chords and the highest
Reynolds number and gives the widest lip and tail windows, but it is the
candidate where the pure wing does worst. P09 and P12 are slightly better
for the pure wing; P12 is constant-chord (simplest panel) and can use tip
fins. The model does not discriminate between them. Section is also a print
decision: NACA 0006 does better in the model; 0008 is a third thicker.

Common provisions, sized here for P07 (rescale by root chord for the
others):

| Item | Provision |
|---|---|
| Wing | two panels joined at the centre, 3 deg dihedral, no twist |
| CG rail | 45–78 mm aft of root LE, readable to 1 mm, nose pod extending at least a quarter root chord (30 mm) ahead of the LE |
| Trailing-edge strip | aft 20% of chord removable outboard of y = 80 mm (40% semispan); neutral strip plus 15% / 8 deg and 20% / 10 deg lips. Lip rise at inboard edge / tip: 2.0 / 1.2 mm and 3.3 / 2.1 mm |
| Tail | removable, 140 mm span, 25 mm chord (10%) with a 20 mm (8%) alternate, quarter chord 237 mm aft of root LE, incidence −2 to −4 deg, with provision to raise it 6–12 mm above the wing plane |
| Fin | 59 x 36 mm (6%), quarter chord 178 mm aft; at the tail station when the tail is fitted |

**CG starting points for P07, NACA 0006**, whole millimetres aft of the root
leading edge, final deck. These are places to begin, with model uncertainty
of several millimetres, not settings.

| Configuration | Window at 8 N/m² | at 16 N/m² | Start at |
|---|---|---|---|
| Pure | 67–69 | 70 | 64, then aft in 1 mm steps |
| Lip 20% / 10 outboard | 47–63 across the three forms; common band about 58 | 52–63; common band 56–58 | 54, then aft in 2 mm steps |
| Tail 10%, −3 deg | 63–74 | 65–75 | 62, then aft in 2 mm steps |

Start forward of the window, on the dive side, and step aft. A dive or a
fast flat glide means move the CG aft (or, with the tail, more negative
incidence); a pitch-up or stall means move it forward. Predicted trim CL is
0.2–0.35 and speed about 6–7.5 m/s at 8 N/m² and 9–11 m/s at 16 N/m².

Weigh the article and measure CG before flying. The tail's CG window
position is uncertain by 4–5 mm on top of this.

## 7. Limits

- Section data applied streamwise; a sweep bracket was run for the pure
  wing only.
- Form (c) uses one scale factor per case; it is a heuristic bound.
- Boom, pod, fin and tail drag are not modelled; L/D figures (about 10–12,
  profile drag only) are comparative.
- Tail in the wing plane is a degenerate vortex-lattice arrangement.
- No dynamics, no release transients, no structural flexibility.
- Mass models and all tolerance ranges are exploratory, not measured.
