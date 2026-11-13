from common import *
import glob
# gap-skipped brackets & non-monotonic Cl
for d in sorted(list((P/"reflex/xfoil").glob("NACA_00*_Re*_c10_a[03]"))+list((P/"reflex_ext/xfoil").glob("NACA_*_Re*"))):
    rows=polar(d)
    for t in (.2,.3,.4):
        skipped=[(a["alpha_deg"],b["alpha_deg"]) for a,b in zip(rows,rows[1:]) if b["alpha_deg"]-a["alpha_deg"]>0.51 and min(a["cl"],b["cl"])<=t<=max(a["cl"],b["cl"])]
        if skipped: print("GAP-SKIPPED",d.name,t,skipped)
    # alpha list gaps
    al=[r["alpha_deg"] for r in rows]
    if len(al)<23: 
        miss=sorted(set(x/2 for x in range(-6,17))-set(al)); print("NPTS",d.name,len(al),"missing alphas",miss, "Cl max",max(r["cl"] for r in rows))
