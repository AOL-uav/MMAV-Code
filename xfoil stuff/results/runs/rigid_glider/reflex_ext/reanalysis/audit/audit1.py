import json,glob,os,re,collections
X=r"<MAV-2026>\Codes\results\runs\rigid_glider\reflex\xfoil"
def raw(d):
    p=os.path.join(X,d,'polar.dat')
    rows={}
    if not os.path.isfile(p): return rows
    for l in open(p,errors='replace'):
        t=l.split()
        if len(t)==7:
            try: v=list(map(float,t))
            except: continue
            rows[round(v[0],3)+0.0]=dict(a=v[0],cl=v[1],cd=v[2],cdp=v[3],cm=v[4],xt=v[5],xb=v[6])
    return rows
grid=[-3+0.5*i for i in range(23)]
secs=['0004','0006','0008','0010'];res=[40000,60000,80000,100000,130000,160000]
S=json.load(open(X+'/summary.json'))
sm={(r['section'],r['reynolds'],r['angle_deg']):r for r in S}
P={}
tot=0;miss=collections.Counter();missre=collections.Counter();missang=collections.Counter();agree=True
for s in secs:
  for re_ in res:
    for ang,suf in((0,'a0'),(3,'a3')):
      d=f'NACA_{s}_Re{re_}_c10_{suf}'
      r=raw(d);P[(s,re_,ang)]=r
      got=[g for g in grid if round(g,3)+0.0 in r or (g==0 and 0.0 in r)]
      n=len(got);tot+=n
      miss[s]+=23-n;missre[re_]+=23-n;missang[ang]+=23-n
      sn=sm[('NACA_'+s,re_,float(ang))]['converged_count']
      extra=[a for a in r if a not in grid]
      if sn!=n or extra: print('DISAGREE',d,'raw',n,'summary',sn,'extra',extra); agree=False
print('raw total',tot,'summary total',sum(r['converged_count'] for r in S if '0008'!=0),'agree',agree)
print('missing by section',dict(miss));print('by Re',dict(missre));print('by shape',dict(missang))
print('missing by (section,Re,shape)')
for k,v in sorted(P.items()):
    n=len([g for g in grid if round(g,3)+0.0 in v])
    if n<23: print(k,23-n,[g for g in grid if round(g,3)+0.0 not in v])
import pickle;pickle.dump(P,open('P.pkl','wb'))
# claim2
def atcl(rows,t):
    rs=sorted(rows.values(),key=lambda r:r['a'])
    for a,b in zip(rs,rs[1:]):
        if b['a']-a['a']>0.51: continue
        if min(a['cl'],b['cl'])<=t<=max(a['cl'],b['cl']) and abs(b['cl']-a['cl'])>1e-6:
            f=(t-a['cl'])/(b['cl']-a['cl']);return {k:a[k]+f*(b[k]-a[k]) for k in('a','cd','cm')}
print('\nCl=0.3 Re80k')
for s in secs:
    c=atcl(P[(s,80000,0)],.3);b=atcl(P[(s,80000,3)],.3)
    print(s,'clean',c and (round(c['cd'],5),round(c['cm'],4),round(c['a'],2)),'bent',b and (round(b['cd'],5),round(b['cm'],4),round(b['a'],2)))
    # also all brackets (non-monotone)
    for nm,r in(('clean',P[(s,80000,0)]),('bent',P[(s,80000,3)])):
        rs=sorted(r.values(),key=lambda x:x['a']);hits=[(a['a'],b['a']) for a,b in zip(rs,rs[1:]) if min(a['cl'],b['cl'])<=.3<=max(a['cl'],b['cl'])]
        if len(hits)!=1:print('  multiple/zero brackets',nm,hits)
# claim4
print('\nDelta table: dCm (bent-clean), dCL at alpha 0,2,4 ; clean/bent cl')
for s in secs:
  for re_ in res:
    out=[]
    for a in(0.0,2.0,4.0):
        c=P[(s,re_,0)].get(a);b=P[(s,re_,3)].get(a)
        out.append('a=%g: '%a+(f"dCm={b['cm']-c['cm']:+.4f} dCL={b['cl']-c['cl']:+.4f} (Cm {c['cm']:+.4f}->{b['cm']:+.4f}; CL {c['cl']:+.3f}->{b['cl']:+.3f})" if b and c else 'missing %s/%s'%(bool(c),bool(b))))
    print(s,re_,' | '.join(out))
# xtr 0008 80k
print('\nxtr 0008 80k')
for a in(0.0,2.0,4.0):
    for ang in(0,3):
        r=P[('0008',80000,ang)].get(a);print(a,ang,r and (r['xt'],r['xb'],r['cl'],r['cm']))
