import json,os,pickle,re,sys
sys.path.insert(0,r"<MAV-2026>\Codes\rigid_glider")
X=r"<MAV-2026>\Codes\results\runs\rigid_glider\reflex\xfoil"
P=pickle.load(open('P.pkl','rb'))
inv=json.load(open(X+'/inviscid_check.json'))
I={c['angle_deg']:{r['alpha_deg']:r for r in c['rows']} for c in inv}
print('inviscid files rows',{k:len(v) for k,v in I.items()})
for a in(0,2,4):
    b=I[3.0].get(float(a));c=I[0.0].get(float(a))
    print('inviscid alpha',a,'clean',c['cl'],c['cm'],'bent',b['cl'],b['cm'],'dCL',round(b['cl']-c['cl'],4),'dCm',round(b['cm']-c['cm'],4))
# reverse
rv=json.load(open(X+'/reverse_check.json'))
for c in rv:
    R={r['alpha_deg']:r for r in c['rows']};F=P_f=None
    re_=c['reynolds']
    # forward from summary
    fw={r['alpha_deg']:r for s in json.load(open(X+'/summary.json')) if s['section']=='NACA_0008' and s['reynolds']==re_ and s['angle_deg']==3.0 for r in s['rows']}
    print('rev',re_,len(R),'rows;',[(a,R[a]['cl'],fw[a]['cl'],R[a]['cm'],fw[a]['cm']) for a in(0.0,2.0,4.0)])
    # raw polar of reverse dir
    d=os.path.join(X,f'NACA_0008_Re{re_}_c10_a3_reverse','polar.dat')
    rr={}
    for l in open(d):
        t=l.split()
        if len(t)==7:
            try:rr[float(t[0])]=(float(t[1]),float(t[4]))
            except:pass
    print('  raw rev',[rr.get(a) for a in(0.0,2.0,4.0)],' rev rows',len(rr))
    mx=max(abs(R[a]['cl']-fw[a]['cl']) for a in R if a in fw);print('  max |dCL| over common alphas',round(mx,4),'n common',len([a for a in R if a in fw]))
    print('  per-alpha CL diffs>0.005:',[(a,R[a]['cl'],fw[a]['cl']) for a in sorted(R) if a in fw and abs(R[a]['cl']-fw[a]['cl'])>0.005])
