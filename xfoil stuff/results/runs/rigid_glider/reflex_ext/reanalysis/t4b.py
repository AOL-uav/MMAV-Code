import csv
L=list(csv.DictReader(open("task4_local_slope.csv")))
out=[]
for Re in (40000,60000,80000,100000,130000,160000):
  for nm in ("clean","bent"):
    X=[(float(r["Cl_mid"]),float(r["dCm_dCl"])) for r in L if int(r["Re"])==Re and r["sect"]==nm]
    X=[x for x in X if 0<=x[0]<=0.8]
    cr=[]
    for (c0,s0),(c1,s1) in zip(X,X[1:]):
        if s0*s1<0: cr.append((round(c0+(0-s0)*(c1-c0)/(s1-s0),3),"-to+" if s0<0 else "+to-"))
    out.append(dict(Re=Re,sect=nm,crossings=cr))
    print(Re,nm,cr)
csv.DictWriter(open("task4_sign_changes.csv","w",newline=""),fieldnames=["Re","sect","crossings"]).writerows([{"Re":o["Re"],"sect":o["sect"],"crossings":str(o["crossings"])} for o in out])
