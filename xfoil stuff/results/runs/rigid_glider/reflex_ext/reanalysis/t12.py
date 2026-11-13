from common import *
import csv
inv08=0.0291
invext={}
for r in csv.DictReader(open(P/"reflex_ext/xfoil/item5_inviscid_alpha0.csv")):
    invext[(r["section"],float(r["chord_fraction"]),int(r["angle_deg"]))]=float(r["inviscid_delta_cm_a0"])
# task1
rows=[];miss=[]
for sec in ("0006","0008","0010"):
  for Re in RES:
    cl=polar(P/f"reflex/xfoil/NACA_{sec}_Re{Re}_c10_a0"); bt=polar(P/f"reflex/xfoil/NACA_{sec}_Re{Re}_c10_a3")
    for t in (.2,.3,.4):
        c=cell(cl,bt,t)
        if c is None: miss.append((sec,Re,t,len(cl),len(bt))); rows.append(dict(section=sec,Re=Re,Cl=t,note=f"MISSING (n_clean={len(cl)}, n_bent={len(bt)}, clean_brk={len(brackets(cl,t))}, bent_brk={len(brackets(bt,t))})")); continue
        flag="" if sec=="0008" else "inviscid ref=0008 value (only 0008 10/3 inviscid exists)"
        rows.append(dict(section=sec,Re=Re,Cl=t,clean_cm=round(c["clean_cm"],4),bent_cm=round(c["bent_cm"],4),dCm_first=round(c["dcm"],4),
          pct_inviscid=round(100*c["dcm"]/inv08,1),n_clean_brk=c["nc"],n_bent_brk=c["nb"],dCm_min=round(c["lo"],4),dCm_max=round(c["hi"],4),
          clean_alpha=round(c["clean_alpha"],2),bent_alpha=round(c["bent_alpha"],2),note=flag+("; MULTI-BRACKET" if c["nc"]>1 or c["nb"]>1 else "")))
flds=["section","Re","Cl","clean_cm","bent_cm","dCm_first","pct_inviscid","n_clean_brk","n_bent_brk","dCm_min","dCm_max","clean_alpha","bent_alpha","note"]
w=csv.DictWriter(open(OUTD/"task1_matched_cl_10_3.csv","w",newline=""),fieldnames=flds);w.writeheader();w.writerows(rows)
print("TASK1");
for r in rows: print({k:r.get(k) for k in flds if r.get(k) not in (None,"")})
# check
print("CHECK 0008 vs reviewer")
rev={60000:(.0007,.0031,.0140),80000:(.0014,.0086,.0144),100000:(.0026,.0131,.0150)}
for Re,v in rev.items():
    mine=[r["dCm_first"] for r in rows if r["section"]=="0008" and r["Re"]==Re]
    print(Re,v,mine)
# task2
lips=[(0.05,4),(0.15,3),(0.15,2),(0.10,5)]
out2=[];ratio=[]
for sec in ("0006","0008"):
  for Re in RES:
    cl=polar(P/f"reflex/xfoil/NACA_{sec}_Re{Re}_c10_a0")
    for t in (.2,.3,.4):
      rr=dict(section=sec,Re=Re,Cl=t)
      # 10/3
      c=cell(cl,polar(P/f"reflex/xfoil/NACA_{sec}_Re{Re}_c10_a3"),t)
      lipsall=[(0.10,3,c)]
      for cf,an in lips:
        d=P/f"reflex_ext/xfoil/NACA_{sec}_Re{Re}_c{int(round(cf*100)):02d}_a{an}"
        lipsall.append((cf,an,cell(cl,polar(d),t)))
      for cf,an,c in lipsall:
        key=f"{int(round(cf*100))}/{an}"
        inv=inv08 if (cf,an)==(0.10,3) else invext[("NACA_"+sec,cf,an)]
        if (cf,an)==(0.10,3) and sec!="0008": invnote="0008 inviscid used (flag)"
        if c is None:
            out2.append(dict(section=sec,Re=Re,Cl=t,lip=key,dCm="MISSING")); rr[key+"_ratio"]=None; continue
        out2.append(dict(section=sec,Re=Re,Cl=t,lip=key,clean_cm=round(c["clean_cm"],4),bent_cm=round(c["bent_cm"],4),dCm=round(c["dcm"],4),
            inviscid_dCm=inv,pct_inviscid=round(100*c["dcm"]/inv,1),n_clean_brk=c["nc"],n_bent_brk=c["nb"],dCm_min=round(c["lo"],4),dCm_max=round(c["hi"],4),
            inv_flag="0008 inviscid used for 0006 10/3" if (cf,an)==(0.10,3) and sec!="0008" else ""))
        rr[key+"_ratio"]=round(c["dcm"]/inv,3)
      vals=[v for k,v in rr.items() if k.endswith("_ratio") and v is not None]
      rr["spread_max_min"]=round(max(vals)-min(vals),3) if vals else None
      rr["n_multi"]=sum(1 for _,_,c in lipsall if c and (c["nc"]>1 or c["nb"]>1))
      ratio.append(rr)
f2=["section","Re","Cl","lip","clean_cm","bent_cm","dCm","inviscid_dCm","pct_inviscid","n_clean_brk","n_bent_brk","dCm_min","dCm_max","inv_flag"]
w=csv.DictWriter(open(OUTD/"task2_ext_lips_long.csv","w",newline=""),fieldnames=f2);w.writeheader();w.writerows(out2)
f3=["section","Re","Cl","5/4_ratio","10/3_ratio","10/5_ratio","15/2_ratio","15/3_ratio","spread_max_min","n_multi"]
w=csv.DictWriter(open(OUTD/"task2_ratio_table.csv","w",newline=""),fieldnames=f3,extrasaction="ignore");w.writeheader();w.writerows(ratio)
print("TASK2 ratio (viscous dCm / inviscid a0 dCm)")
print(f3)
for r in ratio: print([r.get(k) for k in f3])
