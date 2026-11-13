from common import *
import csv
S=list(csv.DictReader(open(OUTD/"task3_strips.csv")))
for s in S:
    for k in s: s[k]=float(s[k])
A=sum(s["area"] for s in S)
def frac(f): return sum(s["area"] for s in S if f(s))/A
print("area frac cl<0.2",frac(lambda s:s["cl"]<0.2),"0.2-0.3",frac(lambda s:0.2<=s["cl"]<=0.3),">0.3",frac(lambda s:s["cl"]>0.3),">0.4",frac(lambda s:s["cl"]>0.4))
print("area frac Re<60k",frac(lambda s:s["Re"]<60000),"60-80k",frac(lambda s:60000<=s["Re"]<80000),"80-100k",frac(lambda s:80000<=s["Re"]<=100000),">100k",frac(lambda s:s["Re"]>100000))
for side,name in ((lambda s:s["yle"]<0,"left"),(lambda s:s["yle"]>0,"right")):
    ss=[s for s in S if side(s)];a=sum(s["area"] for s in ss)
    print(name,len(ss),"area",a,"mean cl",sum(s["cl"]*s["area"] for s in ss)/a,"min cl",min(s["cl"] for s in ss),"|y| range",min(abs(s["yle"]) for s in ss),max(abs(s["yle"]) for s in ss))
# inboard (|y|<0.3) vs outboard
for nm,f in (("inboard |y|<0.3",lambda s:abs(s["yle"])<0.3),("outboard |y|>=0.3",lambda s:abs(s["yle"])>=0.3),("outer |y|>0.33 (55-100%)",lambda s:abs(s["yle"])>0.33)):
    ss=[s for s in S if f(s)];a=sum(s["area"] for s in ss)
    print(nm,"cl wmean %.4f min %.4f max %.4f"%(sum(s["cl"]*s["area"] for s in ss)/a,min(s["cl"] for s in ss),max(s["cl"] for s in ss)),"Re wmean %.0f min %.0f max %.0f"%(sum(s["Re"]*s["area"] for s in ss)/a,min(s["Re"] for s in ss),max(s["Re"] for s in ss)))
print("cl range for |y|<0.58:",min(s["cl"] for s in S if abs(s["yle"])<0.58))
