# EOM-Based Steady-Glide Trim Formulation

The previous fixed-speed routine swept angle of attack and then checked whether `Fz/W = 1` and `Cm_y = 0` happened at the same alpha.  That is useful as an aerodynamic diagnostic, but it is not a conventional trim solve.

The EOM-based trim routine solves the reduced symmetric glide trim problem directly.

## Unknowns

```text
z = [V_mps, alpha_deg, gamma_deg]
```

where `gamma_deg` is positive downward.

## Algebraic equations

For unpowered straight glide:

```text
L(V,alpha) = W cos(gamma)
D(V,alpha) = W sin(gamma)
M_y(V,alpha) = 0
```

The residual vector is normalized as:

```text
R1 = (L - W cos(gamma))/W
R2 = (D - W sin(gamma))/W
R3 = Cm_y
```

The VLM/nonlinear-polar solver provides the aerodynamic force and moment for each trial `V, alpha`.

## Conventional state interpretation

The reduced trim solution corresponds to the conventional longitudinal state:

```text
u_forward = V cos(alpha)
w_down    = V sin(alpha)
theta     = alpha - gamma
v = 0
p = q = r = 0
phi = 0
beta = 0
```

This keeps the implementation compact while preserving the main EOM trim structure needed for the passive glider problem.
