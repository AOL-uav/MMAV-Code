# Velocity scan versus free EOM trim

`RUN_02B_eom_glide_trim.m` solves the passive steady-glide trim problem with

```matlab
z = [V_mps, alpha_deg, gamma_deg]
```

as unknowns. Therefore it does **not** sweep velocity; velocity is part of the
solution.

`RUN_02C_fixed_speed_velocity_scan.m` is a diagnostic branch-continuation tool.
For each fixed speed `V`, it solves only

```matlab
x = [alpha_deg, gamma_deg]
```

from the two force-balance equations:

```text
L(V, alpha) - W cos(gamma) = 0
D(V, alpha) - W sin(gamma) = 0
```

Then it reports the remaining pitching moment `Cm_y`. For a passive vehicle
without an elevator, thrust, or another trim control, an arbitrary speed cannot
usually satisfy all three equations. A passive trimmed speed is indicated where
`Cm_y` crosses zero on this force-balanced branch.

Use:

```matlab
RUN_02B_eom_glide_trim          % one free trim solve
RUN_02C_fixed_speed_velocity_scan % fixed-speed diagnostic sweep
```
