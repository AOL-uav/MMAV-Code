import sys,json,math,subprocess,types,hashlib,os
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,r"<MAV-2026>\Codes\rigid_glider")
import reflex_xfoil as m
X=Path(r"<MAV-2026>\Codes\results\runs\rigid_glider\reflex\xfoil")
scr=Path(r"<HOME>\AppData\Local\Temp\claude\MAV-2026-Codes\905656ae-acf2-4a7d-bf56-cdf0f8f045c8\scratchpad\audit\regen")
scr.mkdir(exist_ok=True)
m.OUT=scr
calls=[]
def fake(*a,**k):
    calls.append(a); return types.SimpleNamespace(stdout="",returncode=0)
m.subprocess.run=fake
cases=[("NACA_0008",80000,0.10,3.0,False,False),("NACA_0004",40000,0.10,3.0,False,False),("NACA_0010",160000,0.10,0.0,False,False),
       ("NACA_0008",60000,0.10,3.0,False,True),("NACA_0008",80000,0.10,3.0,True,False)]
for sec,re_,c,a,inv,rev in cases:
    r=m.run(sec,re_,c,a,inv,rev)
    lab=Path(r['folder']).name
    for f in('foil.dat','commands.in'):
        old=(X/lab/f).read_bytes();new=(scr/lab/f).read_bytes()
        print(lab,f,'IDENTICAL' if old==new else 'DIFFERENT (%d vs %d bytes)'%(len(old),len(new)), 'identical modulo CRLF' if old.replace(b'\r\n',b'\n')==new.replace(b'\r\n',b'\n') else '')
print('subprocess stub calls',len(calls))
# key comparison
S=json.load(open(X/'summary.json'));print('stored summary keys',sorted(S[0].keys()))
r=m.run("NACA_0008",80000,.1,3.0);print('current run() keys',sorted(r.keys()))
for fn in('inviscid_check.json','reverse_check.json'):
    print(fn,sorted(json.load(open(X/fn))[0].keys()))
# geometry
pts=m.coordinates(.08,.1,3.0);print(len(pts),pts[0],pts[len(pts)//2-0],pts[-1])
up=pts[:161];lo=pts[161:]
print('first',pts[0],'last',pts[-1],'LE idx',pts[160])
print('expected TE y 0.1*sin3=',0.1*math.sin(math.radians(3)))
# stored foil.dat
L=(X/'NACA_0008_Re80000_c10_a3/foil.dat').read_text().split('\n')
q=[tuple(map(float,l.split())) for l in L[1:] if l.strip()]
print('stored n',len(q),'first',q[0],'last',q[-1],'LE',q[160])
# slope of upper surface before/after hinge
import numpy as np
x=np.array([p[0] for p in q[:161]]);y=np.array([p[1] for p in q[:161]])
# x monotone decreasing along upper?
print('upper x monotone dec',all(np.diff(x)<0),' lower x monotone inc',all(np.diff([p[0] for p in q[160:]])>0))
# camber line y at x=1 vs unrotated
c=m.coordinates(.08,.1,0.0)
print('unrotated TE',c[0],' max |dy| clean-vs-bent forward of hinge',max(abs(a[1]-b[1]) for a,b in zip(c,pts) if b[0]<0.9 and a[0]<0.9))
# local slope of aft segment of upper surface  (chordline: lower+upper mean)
aft=[(p[0],p[1]) for p in q[:161] if p[0]>0.9];print('aft upper pts',len(aft))
# signed area -> orientation
A=0.5*sum(q[i][0]*q[(i+1)%len(q)][1]-q[(i+1)%len(q)][0]*q[i][1] for i in range(len(q)));print('signed area',A,'(neg=clockwise)')
# panel spacing at hinge
dx=np.diff(x);i=np.argmin(abs(x-0.9));print('upper pt spacing near x=0.9',abs(dx[i-1]),abs(dx[i]))
# TE gap: upper vs lower end points
print('upper TE',q[0],'lower TE',q[-1],'duplicate TE point',q[0]==q[-1])
