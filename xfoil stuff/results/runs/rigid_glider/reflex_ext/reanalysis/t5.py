from common import *
import csv
def fit(x,y):
    n=len(x);mx=sum(x)/n;my=sum(y)/n
    return sum((a-mx)*(b-my) for a,b in zip(x,y))/sum((a-mx)**2 for a in x)
out=[]
# reflex tolerances
T=list(csv.DictReader(open(P/"reflex/tolerances.csv")))
def series(rows,prefix):
    s=[(float(r["name"].split("_")[-1]),float(r["trim_cl_roll"])) for r in rows if r["name"].startswith(prefix)]
    return sorted(s)
for pre,lab in (("tol_mismatch_","lip L/R mismatch"),("tol_incidence_","incidence mismatch")):
    s=series(T,pre); x=[a for a,_ in s];y=[b for _,b in s]
    z=dict(s)[0.0]
    out.append(dict(source="reflex/tolerances.csv",case="nominal full 10/3",kind=lab,pts=str(s),LS_slope=round(fit(x,y),5),slope_pm1=round((dict(s)[1.0]-dict(s)[-1.0])/2,5),offset_at_zero=z))
# item4
I=list(csv.DictReader(open(P/"reflex_ext/item4_tolerances.csv")))
for g in ["nominal_full_c10_a3","outer_c10_a4","full_c05_a4","full_c15_a3","minlip_area_outer_c05_a3","minlip_fullspan_full_c05_a2"]:
    for kind,lab in (("mismatch_deg","lip L/R mismatch"),("incidence_mismatch_deg","incidence mismatch")):
        rr=[r for r in I if r["group"]==g+":oat" and r["tol_kind"]==kind]
        s=sorted((float(r["tol_error"]),float(r["trim_cl_roll"])) for r in rr)
        if len(s)<3: out.append(dict(source="item4",case=g,kind=lab,pts="MISSING"));continue
        x=[a for a,_ in s];y=[b for _,b in s]
        d=dict(s)
        out.append(dict(source="item4",case=g,kind=lab,pts=str(s),LS_slope=round(fit(x,y),5),slope_pm1=round((d[max(x)]-d[min(x)])/(max(x)-min(x)),5) ,offset_at_zero=d.get(0.0),errs=[r['error'] for r in rr if r['error']]))
# item3
M=list(csv.DictReader(open(P/"reflex_ext/item3_mismatch.csv")))
for pl in ("outer_c10_a4","full_c10_a3"):
    s=sorted((float(r["mismatch_deg"]),float(r["trim_cl_roll"])) for r in M if r["name"].startswith("mm_"+pl))
    out.append(dict(source="item3",case=pl,kind="lip L/R mismatch",pts=str(s),LS_slope=round(fit([a for a,_ in s],[b for _,b in s]),5),offset_at_zero=dict(s)[0.0]))
f=["source","case","kind","LS_slope","slope_pm1","offset_at_zero","pts","errs"]
w=csv.DictWriter(open(OUTD/"task5_roll_slopes.csv","w",newline=""),fieldnames=f,extrasaction="ignore");w.writeheader();w.writerows(out)
for o in out: print({k:o.get(k) for k in ["source","case","kind","LS_slope","slope_pm1","offset_at_zero"]},o.get("errs"))
print("ratios incid/lip (LS)")
for src in ("reflex/tolerances.csv","item4"):
    for c in sorted(set(o["case"] for o in out if o["source"]==src)):
        a=[o for o in out if o["source"]==src and o["case"]==c and o["kind"].startswith("incid") and "LS_slope" in o]
        b=[o for o in out if o["source"]==src and o["case"]==c and o["kind"].startswith("lip") and "LS_slope" in o]
        if a and b: print(src,c,a[0]["LS_slope"],b[0]["LS_slope"],round(a[0]["LS_slope"]/b[0]["LS_slope"],3))
