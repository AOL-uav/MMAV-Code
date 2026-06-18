# Drag-only fuselage-attached tail model

The fuselage-attached curled tail is modeled as a drag-producing sheet rather than a lifting surface.

For each tail panel `p`, the force is

```text
F_p^B = q * C_D_tail * A_projected,p * e_D^B
```

where:

```text
q = 0.5*rho*V^2
A_projected,p = A_p * |n_p^B dot e_D^B|
e_D^B = drag-force direction in the body/build frame
```

No `C_L`, no circulation, and no tail VLM solve are used in `drag_only` mode.

The pitching moment about the active CG is obtained through the normal component-wise aggregation rule:

```text
M_CG^B = sum_p (r_p^B - r_CG^B) x F_p^B
```

In longitudinal motion:

```text
M_y = (z_tail - z_CG)*F_x - (x_tail - x_CG)*F_z
```

Thus a drag-only tail creates nose-up moment mainly by placing the drag center above the CG. This is physically different from the earlier VLM-lifting-tail model, which created nose-up moment primarily through tail downforce.
