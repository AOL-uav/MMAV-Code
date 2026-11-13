import csv
X=r"<MAV-2026>\Codes\results\runs\rigid_glider\reflex\xfoil\comparison.csv"
R={}
for r in csv.DictReader(open(X)):
    R[(r['section'],int(r['reynolds']),float(r['angle_deg']),float(r['target_cl']))]=(float(r['cd']) if r['cd'] else None,float(r['cm']) if r['cm'] else None)
print('csv Re80k Cl0.3');
for s in('NACA_0004','NACA_0006','NACA_0008','NACA_0010'):
    print(s,R[(s,80000,0.0,.3)],R[(s,80000,3.0,.3)])
print('Cd 0006 vs 0008 (0006 lower? ) per Re, Cl, shape [from comparison.csv]')
for cl in(.2,.3,.4):
  for ang in(0.0,3.0):
    row=[]
    for re_ in(40000,60000,80000,100000,130000,160000):
        a=R[('NACA_0006',re_,ang,cl)][0];b=R[('NACA_0008',re_,ang,cl)][0]
        row.append('%dk:%s'%(re_//1000,'NA' if a is None or b is None else ('Y' if a<b else 'N')+f'({a:.4f}/{b:.4f})'))
    print('Cl',cl,'shape',ang,' '.join(row))
