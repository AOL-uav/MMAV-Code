# Controlled trim notes

This package has two trim interpretations:

1. Passive trim: no controls. The solver checks whether the fixed geometry has a natural steady-glide equilibrium.
2. Controlled trim: wing incidence and tail deformation are allowed to change.

The new controlled-trim script is:

```matlab
RUN_02D_controlled_trim_at_speed
```

It uses a fixed speed and solves:

```matlab
x = [alpha_deg, gamma_deg, wing_collective_deg, tail_curl_delta_deg]
```

The residual equations are:

```matlab
L/W - cos(gamma) = 0
D/W - sin(gamma) = 0
Cm_y             = 0
```

Because there are two controls for one pitch-moment equation, the solver uses a small control regularization penalty. This chooses a relatively small-control trim solution among multiple possible solutions.

Wing incidence controls:

```matlab
wing_R_incidence = wing_collective + wing_differential
wing_L_incidence = wing_collective - wing_differential
```

For the current symmetric longitudinal trim, `wing_differential = 0`. A later full 6-DOF trim can use differential incidence to satisfy roll/yaw equations.

Tail deformation control:

```matlab
tail_curl_total = baseline_curl + tail_curl_delta
```

For the drag-only tail model, this changes the curled plate geometry and therefore the projected drag and pitch moment.
