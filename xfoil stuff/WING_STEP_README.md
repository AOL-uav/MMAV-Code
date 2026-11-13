# Airfoil DAT files to a solid STEP wing

Run `airfoil_wing_to_step.py` or `run_wing_step.cmd` **without arguments** to answer prompts for every wing parameter. Press Enter to keep a shown value. The answers are saved in `wing_settings.json` and offered as defaults next time, so you can reuse or edit them. STEP output defaults to `<HOME>\Downloads`. The first launcher run installs CadQuery into a dedicated environment at `%USERPROFILE%\.airfoil-wing-venv`. Pass a JSON path as the first argument to run directly without prompts. Absolute output paths go where specified; relative output paths in JSON are relative to that JSON file.

The example is a **400 mm full-span demonstration**, using the existing NACA 0006 DAT at both stations. Replace either `dat` path to blend different airfoils. Add more objects to `stations` for more sections. The first station must be at `span_mm: 0`, and later span positions must increase. `span_mm` is the projected distance from the centerline to that station; with `mirror: true`, the STEP contains **two separate solids**, one left wing and one right wing. They touch at the root but are not fused. With `mirror: false`, it contains just the listed right half-wing as one solid.

All lengths are in **mm**. The root leading edge starts at x = 0; x points aft, y points right along the span, and z points up. Positive `sweep_deg` moves the selected chord reference line aft with span. `sweep_reference_fraction` selects that line: 0 is the leading edge, 0.25 is quarter chord. Positive `dihedral_deg` raises both tips. `twist_deg` is set at each station; positive values raise the nose and negative values lower it. Twist rotates around `twist_axis_fraction` of the local chord, normally 0.25. The script linearly lofts between listed stations, so add a station wherever the wing shape must change direction.

DAT files must list the upper trailing edge, upper surface, nose, lower surface, and lower trailing edge in that order. Open trailing edges are closed with a straight edge. When mixing airfoils, use files with the same trailing-edge type (all open or all closed). The generator checks that the loft is one valid solid before writing STEP; imported STEP is a sized solid in Onshape, but the original station parameters remain in the JSON rather than Onshape's feature tree.

To run a custom configuration from PowerShell:

```powershell
& "<MAV-2026>\Codes\run_wing_step.cmd" "C:\path\to\my_wing.json"
```
