# sheet_builder settings

Run: `py -B rigid_glider/sheet_builder.py rigid_glider/sheet_builder_configs/<name>.json [--out DIR] [--force] [--orca]`
Default output: `results/runs/sheet_builder/<name>/` (refuses to overwrite a non-empty folder; `--force` overwrites files in it). `--orca` slices every STL with the OrcaSlicer CLI afterwards (Bambu A1 0.4 nozzle, 0.12 mm Fine, generic PLA; the CLI does not resolve `inherits`, so the builder flattens the profile chain first).
Keys starting with `_` are comments. Every key has a default (`DEFAULTS` in `sheet_builder.py`); a config only lists what it changes. Units mm, deg, g. Tags: KAI = Kai's decision, ASSUMED = a value picked here and never measured.

## Top level
| key | default | meaning |
|---|---|---|
| `name` | unnamed | output folder name |
| `span_mm` | 400 | full span. Built-in outlines are generated at this span; DXF/SVG outlines are scaled to it (unless `source.scale_to_span` is false) |
| `dihedral_deg` | 0 | dihedral of each half, set by the tilt of the rail's saddle faces (KAI: printed, not hand set). 0 = as flown |
| `root_gap_mm` | 0.4 | gap between the two halves' root edges. Raised automatically when tail bend and dihedral are both set (root edges converge by 2 X sin(bend) sin(dihedral) over the tail length X) |

## `source` (outline)
| key | meaning |
|---|---|
| `type` | `builtin`, `dxf` or `svg` |
| `builtin` | `swiftA` (bird_outline_v4_swift GLIDE, incl. tail fan, hinge x and tail end) or `b45` (flat_test.wing_geometry(span, `ar`)) |
| `path` | DXF/SVG file (relative to the config). DXF: LINE, ARC, LWPOLYLINE (bulges), POLYLINE+VERTEX, SPLINE (control points + knots with de Boor, or fit points via an interpolating B-spline). SVG: path M L H V C Q Z (absolute and relative), polygon, polyline, `transform` translate/scale/rotate/matrix; units from width+viewBox when width has a unit, else 1 unit = 1 mm (or `mm_per_unit`) |
| `layers` | DXF layers to read (default all) |
| `kind` | `half` (right half, root edge on y = 0, closed), `full` (mirror-symmetric, split at y = 0, left/right asymmetry reported in mm), or `auto` |
| `symmetrize` | for `full`: `right` (default) or `left` half is the master |
| `flip_y` | DXF default false, SVG default true (SVG y points down) |
| `flip_x`, `rotate_deg`, `scale`, `scale_to_span`, `x_origin` (`root_le` shifts x so the root leading edge is 0), `join_tol_mm` (default 0.05, chain-joining tolerance) | usual |
| `hinge_x_mm` | tail hinge line x for imported outlines (a separate tail outline file is NOT implemented in v1) |

Axes: x aft, y right. The right half is the master; the left half is its exact mirror image.

## `tail`
`bend_deg` (tail bend; 0 = no tail features), `hinge_x_mm` (override), `hinge_zone_mm` (thin, un-thickened strip across the fold, default 4: the strip is interrupted there so the sheet can bend). The bend is printed as a rail face (the tail plates are rotated by `bend_deg` about the fold line).

## `sheet`
`layer_height_mm` (0.1), `n_layers` (>= 2, KAI). Membrane = layer_height x n_layers (smallest gliders: 2 x 0.1 = 0.2 mm; the shipped 400 mm configs use 4 x 0.1).

## `bead` (leading-edge spar)
`enabled`; `width_root_mm`, `width_tip_mm` (band between the leading edge and the leading edge moved inward along its normal); `height_root_mm`, `height_tip_mm` (extra height above the membrane, linear in y); `stop_frac` (fraction of the semi-span where it ends). Starts at the root strip's edge (the strip is taller and absorbs the bead). Rectangular section (the options study used half rounds). Root strip / joint strips are raised automatically to at least membrane + tallest bead.

## `root_strip` and `keys`
`root_strip.width_mm`, `thickness_mm`: thickened band along the root edge (total thickness; ASSUMED). `keys`: `head_r_mm`, `neck_w_mm`, `neck_len_mm` (jigsaw post outline: round head + straight neck, neck direction alternates +x/-x), `height_mm` (post height), `clearance_mm` (0.15 ASSUMED: socket outline = key moved outward by this on every edge; pocket is this much deeper than the post), `pitch_mm` (about this spacing along the strip), `margin_mm`, `n_min`.

## `split` (halves larger than the bed)
When a half does not fit the bed in any rotation (searched every `bed.rotation_step_deg`), it is cut at y = const by a greedy search from the root (largest piece that fits, tab allowance included; cuts stay outboard of the tail fan). Each cut gets a jigsaw seam (`key_head_r_mm`, `key_neck_w_mm`, `key_neck_len_mm`, `key_fracs` = positions as fractions of the chord, same clearance as `keys`) on thickened joint strips (`joint_width_mm` each side, `joint_thickness_mm`). `max_pieces`, `scan_step_mm`.

## `bed`
`size_mm` (250 design size; bed is 256), `rotation_step_deg` (0.5). Every part is checked and written already rotated and centred.

## `rail`
`web_thickness_mm`, `ballast_block_thickness_mm` (web is thicker where the lead window is), `plate_thickness_mm` (face plates; raised automatically to hold the pocket depth), `plate_overlap_mm` (web top inside the plates), `wall_mm`, `window_height_mm` (ballast through-window height), `plate_margin_mm`, `rib_mm` and `lightening` (holes between the sockets), `web_depth_min_mm`, `front_x_mm`, `nose_extension_mm`, `ballast_station_x_mm` (mm aft of the root LE; null = just behind the plate front), `fin_slot_len_mm`, `print_orientation` (`side` = default, dihedral faces become near-vertical perimeter walls; `upright`).

## `ballast`
`density_g_cm3` (11.3 lead, ASSUMED), `fill_margin` (0.25: pocket volume = lead volume x 1.25). Window length is solved with the ballast mass (fixed point).

## `fin`
`type`: `none`, `centre` (tongue into a slot at the rail's aft end, standing behind the rail), `tip` (two fins on tabs at the wing tips, `wing_tab_fracs` / `wing_tab_len_frac` place the tabs; they must lie ahead of the trim-tab zone). `height_mm`, `chord_mm`, `taper`, `thickness_mm`, `slot_clearance_mm`. Trim tab: `tab_chord_frac`, `tab_span_frac`, `tab_angles_deg` (printed 0, 2, 4; a negative angle = the same tab turned over), `tab_on` (`right`/`left`/`both`, tip fins), `tab_flat_mm`, `key_*`. The tab is a separate flat part with a jigsaw edge matching the fin body, tilted beyond a flat landing by a printed angle. If the tilted length is under 3 mm the tab family is not written and the run says so (FLAG).

## `mass`
`pla_density_g_cm3` (1.24 ASSUMED), `target_cg_mm` + `cg_reference` (`nose_tip` or `root_le`; Swift A 73.0 nose_tip, B45 66.0 root_le; null = no ballast), `glue_root_g`, `glue_per_split_g`, `glue_per_fin_g` (ASSUMED allowances).

## `foam`
`paper` (A4 or LETTER), `tile_overlap_mm`, `margin_mm`.
