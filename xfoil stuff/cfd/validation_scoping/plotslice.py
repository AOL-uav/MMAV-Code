import numpy as np, sys
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
d = np.loadtxt(sys.argv[1]); out = sys.argv[2]
x, z = d[:,0], d[:,2]; U = np.hypot(d[:,3], d[:,5])
fig, ax = plt.subplots(1, 1, figsize=(9, 4.2))
m = (x > -0.6) & (x < 2.5) & (abs(z) < 0.6)
tc = ax.tricontourf(x[m], z[m], U[m], levels=np.linspace(0, 1.4, 29), cmap="viridis", extend="both")
fig.colorbar(tc, label="|U|/U_inf (in-plane)")
ax.set_aspect("equal"); ax.set_xlabel("x/c"); ax.set_ylabel("z/c")
ax.set_title("Coarse mesh, AR3, alpha=4, Re=60k, y/c=0.75, t=20", fontsize=10)
fig.tight_layout(); fig.savefig(out, dpi=100)
