import csv,statistics as st
R=list(csv.DictReader(open("task2_ratio_table.csv")))
ks=["5/4_ratio","10/3_ratio","10/5_ratio","15/2_ratio","15/3_ratio"]
out=[]
for cl in ("0.2","0.3","0.4"):
  for lo,hi in ((40000,60000),(80000,160000)):
    rel=[];sp=[]
    for r in R:
        if r["Cl"]==cl and lo<=int(r["Re"])<=hi:
            v=[float(r[k]) for k in ks if r[k] not in ("","None")]
            if len(v)==5: sp.append(max(v)-min(v)); rel.append((max(v)-min(v))/st.mean(v))
    out.append((cl,f"Re {lo}-{hi}",len(sp),round(min(sp),3),round(st.median(sp),3),round(max(sp),3),"rel median %.2f max %.2f"%(st.median(rel),max(rel))))
    print(out[-1])
# mean ratio per lip per Cl (Re>=80k)
for cl in ("0.2","0.3","0.4"):
    print(cl,{k:round(st.mean([float(r[k]) for r in R if r["Cl"]==cl and int(r["Re"])>=80000 and r[k] not in("","None")]),3) for k in ks})
w=csv.writer(open("task2_spread_summary.csv","w",newline=""));w.writerow(["Cl","Re band","n","spread_min","spread_median","spread_max","rel"]);w.writerows(out)
