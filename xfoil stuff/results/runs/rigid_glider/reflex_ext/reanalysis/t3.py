from common import *
import csv
f=P/"reflex/raw/s25_full_c10_a3_trim/point_00.fs"
V=7.094;nu=1.5e-5
strips=[]
for ln in f.read_text().splitlines():
    p=ln.split()
    if len(p)==15:
        try: v=[float(x) for x in p]
        except: continue
        strips.append(dict(j=int(v[0]),y=v[2] if False else v[2],yle=v[2],chord=v[4],area=v[5],c_cl=v[6],cl_norm=v[8],cl=v[9]))
# columns: j Xle Yle Zle Chord Area c_cl ai cl_norm cl cd ...
strips=[]
for ln in f.read_text().splitlines():
    p=ln.split()
    if len(p)==15 and p[0].isdigit():
        v=[float(x) for x in p]
        strips.append(dict(j=int(v[0]),yle=v[2],chord=v[4],area=v[5],c_cl=v[6],cl_norm=v[8],cl=v[9]))
print(len(strips))
for s in strips:
    s["Re"]=V*s["chord"]/nu; s["cl_from_ccl"]=s["c_cl"]/s["chord"]
w=csv.DictWriter(open(OUTD/"task3_strips.csv","w",newline=""),fieldnames=list(strips[0]));w.writeheader();w.writerows(strips)
print("max |cl - c_cl/chord|",max(abs(s["cl"]-s["cl_from_ccl"]) for s in strips))
A=sum(s["area"] for s in strips)
mc=sum(s["cl"]*s["area"] for s in strips)/A; mr=sum(s["Re"]*s["area"] for s in strips)/A
print("area",A,"cl min/mean/max",min(s["cl"] for s in strips),mc,max(s["cl"] for s in strips),"Re",min(s["Re"] for s in strips),mr,max(s["Re"] for s in strips))
print("area-wt mean of cl_norm",sum(s["cl_norm"]*s["area"] for s in strips)/A, "cl_norm range",min(s["cl_norm"] for s in strips),max(s["cl_norm"] for s in strips))
# sanity: sum cl*area/Sref
print("sum cl*area/Sref",sum(s["cl"]*s["area"] for s in strips)/0.20571)
tc=[s for s in strips]
imin=min(strips,key=lambda s:s["cl"]);imax=max(strips,key=lambda s:s["cl"]);print("min cl at",imin["j"],imin["yle"],"max at",imax["j"],imax["yle"])
# bins by |y|
half=0.6;n=8
rows=[]
for k in range(n):
    lo,hi=half*k/n,half*(k+1)/n
    ss=[s for s in strips if lo<=abs(s["yle"])<hi or (k==n-1 and abs(s["yle"])==hi)]
    a=sum(s["area"] for s in ss)
    rows.append(dict(bin=f"{lo:.3f}-{hi:.3f}",eta=f"{(lo+hi)/2/half:.2f}",n=len(ss),chord=round(sum(s["chord"]*s["area"] for s in ss)/a,4),
      cl_mean=round(sum(s["cl"]*s["area"] for s in ss)/a,4),cl_min=min(s["cl"] for s in ss),cl_max=max(s["cl"] for s in ss),
      Re_mean=round(sum(s["Re"]*s["area"] for s in ss)/a),Re_min=round(min(s["Re"] for s in ss)),Re_max=round(max(s["Re"] for s in ss))))
w=csv.DictWriter(open(OUTD/"task3_span_stations.csv","w",newline=""),fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
for r in rows: print(r)
