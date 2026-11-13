from common import *
import csv
I=list(csv.DictReader(open(P/"reflex_ext/item4_tolerances.csv")))
L=list(csv.DictReader(open(P/"reflex_ext/item4_limits.csv")))
print(L[0].keys())
for l in L: print(l["case"],l["margin_target"],l["cg_mm_neg_limit"],l["cg_mm_neg_binding"],l["cg_mm_pos_limit"],l["cg_mm_pos_binding"])
rows_out=[]
for g in ["minlip_fullspan_full_c05_a2","minlip_area_outer_c05_a3","nominal_full_c10_a3"]:
    s=sorted((float(r["tol_error"]),float(r["trim_cl"]),float(r["static_margin_mac"]),r["error"],r["target_margin"]) for r in I if r["group"]==g+":scan" and r["tol_kind"]=="cg_mm" and r["trim_cl"]!="")
    print(g,"n",len(s),"target_margin",s[0][4],"cg range",s[0][0],s[-1][0])
    print("  ",[(a,round(b,4),round(c,4)) for a,b,c,d,e in s])
    cross={}
    for t in (0.2,0.5):
        xs=[]
        for (a,b,_,_,_),(c,d,_,_,_) in zip(s,s[1:]):
            if min(b,d)<=t<=max(b,d) and b!=d: xs.append(a+(t-b)*(c-a)/(d-b))
        cross[t]=xs
    print("  crossings",cross)
    rows_out.append(dict(group=g,target_margin=s[0][4],cg_at_CL0p5=cross[0.5],cg_at_CL0p2=cross[0.2],scan_range=(s[0][0],s[-1][0]),n=len(s),errors=[x[3] for x in s if x[3]]))
w=csv.DictWriter(open(OUTD/"task7_cg_window.csv","w",newline=""),fieldnames=list(rows_out[0]));w.writeheader();w.writerows(rows_out)
