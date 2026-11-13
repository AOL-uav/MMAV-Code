from common import *
import csv
def cm_at(rows,t):
    b=brackets(rows,t); return b
pol={}
for Re in RES:
    pol[Re]=(polar(P/f"reflex/xfoil/NACA_0008_Re{Re}_c10_a0"),polar(P/f"reflex/xfoil/NACA_0008_Re{Re}_c10_a3"))
# table cm vs alpha
alphas=[i*0.5 for i in range(13)]
out=[];hdr=["alpha"]+[f"{Re//1000}k_clean_Cm" for Re in RES]+[f"{Re//1000}k_bent_Cm" for Re in RES]+[f"{Re//1000}k_clean_Cl" for Re in RES]+[f"{Re//1000}k_bent_Cl" for Re in RES]
def get(rows,a):
    for r in rows:
        if abs(r["alpha_deg"]-a)<1e-6: return r
    return None
tab=[]
for a in alphas:
    row=[a]
    for k in (0,1):
        for Re in RES:
            r=get(pol[Re][k],a); row.append("NA" if r is None else r["cm"])
    for k in (0,1):
        for Re in RES:
            r=get(pol[Re][k],a); row.append("NA" if r is None else r["cl"])
    tab.append(row)
with open(OUTD/"task4_cm_vs_alpha_0008.csv","w",newline="") as f:
    w=csv.writer(f);w.writerow(hdr);w.writerows(tab)
print("Cm vs alpha, NACA0008 clean (Cm x1e4) per Re",RES)
for r in tab: print(r[0],[("NA" if x=="NA" else round(x*1e4)) for x in r[1:7]],"| bent",[("NA" if x=="NA" else round(x*1e4)) for x in r[7:13]])
# slopes
def slope(rows,lo,hi):
    a=brackets(rows,lo);b=brackets(rows,hi)
    if not a or not b: return None,len(a),len(b)
    ss=[(y[1]-x[1])/(hi-lo) for x in a for y in b]
    return (b[0][1]-a[0][1])/(hi-lo),min(ss),max(ss),len(a),len(b)
srows=[]
for Re in RES:
    for k,nm in ((0,"clean"),(1,"bent10/3")):
        for lo,hi in ((0.1,0.3),(0.35,0.55)):
            s=slope(pol[Re][k],lo,hi)
            if s[0] is None: srows.append(dict(Re=Re,sect=nm,cl_range=f"{lo}-{hi}",dCm_dCl="MISSING"));continue
            srows.append(dict(Re=Re,sect=nm,cl_range=f"{lo}-{hi}",dCm_dCl=round(s[0],4),x_ac_over_c=round(0.25-s[0],4),ac_shift_from_c4=round(-s[0],4),min=round(s[1],4),max=round(s[2],4),n_lo=s[3],n_hi=s[4]))
w=csv.DictWriter(open(OUTD/"task4_ac_shift.csv","w",newline=""),fieldnames=list(srows[0])+["note"],extrasaction="ignore");w.writeheader();w.writerows(srows)
for r in srows: print(r)
# local slope vs Cl (consecutive alpha) and sign changes
print("local dCm/dCl between consecutive alpha points, sign changes")
loc=[]
for Re in RES:
    for k,nm in ((0,"clean"),(1,"bent")):
        rows=pol[Re][k]; prev=None
        for a,b in zip(rows,rows[1:]):
            if b["alpha_deg"]-a["alpha_deg"]>0.51 or abs(b["cl"]-a["cl"])<1e-6: continue
            s=(b["cm"]-a["cm"])/(b["cl"]-a["cl"]); cm_=(a["cl"]+b["cl"])/2
            loc.append((Re,nm,a["alpha_deg"],b["alpha_deg"],round(cm_,4),round(s,4)))
w=csv.writer(open(OUTD/"task4_local_slope.csv","w",newline=""));w.writerow(["Re","sect","a0","a1","Cl_mid","dCm_dCl"]);w.writerows(loc)
for Re in RES:
    for nm in ("clean","bent"):
        L=[x for x in loc if x[0]==Re and x[1]==nm and 0<=x[4]<=0.8]
        ch=[(L[i][4],L[i+1][4],L[i][5],L[i+1][5]) for i in range(len(L)-1) if L[i][5]*L[i+1][5]<0]
        print(Re,nm,"sign changes (Cl_mid pairs, slopes):",ch, "| slopes:",[(x[4],x[5]) for x in L])
