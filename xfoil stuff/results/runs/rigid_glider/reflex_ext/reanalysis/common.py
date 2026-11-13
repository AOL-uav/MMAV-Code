import sys, os
sys.dont_write_bytecode=True
sys.path.insert(0, r"<MAV-2026>\Codes\rigid_glider")
import reflex_xfoil as rx
from pathlib import Path
P=Path(r"<MAV-2026>\Codes\results\runs\rigid_glider")
OUTD=Path(__file__).parent
RES=[40000,60000,80000,100000,130000,160000]
def polar(path):
    return rx.read_polar(Path(path)/"polar.dat")
def brackets(rows,t):
    out=[]
    for a,b in zip(rows,rows[1:]):
        if b["alpha_deg"]-a["alpha_deg"]>0.51: continue
        if min(a["cl"],b["cl"])<=t<=max(a["cl"],b["cl"]) and abs(b["cl"]-a["cl"])>1e-6:
            f=(t-a["cl"])/(b["cl"]-a["cl"])
            out.append((a["alpha_deg"]+f*(b["alpha_deg"]-a["alpha_deg"]), a["cm"]+f*(b["cm"]-a["cm"])))
    ded={}
    for al,cm in out: ded[round(al,5)]=(al,cm)
    return list(ded.values())
def cell(clean,bent,t):
    c=brackets(clean,t); b=brackets(bent,t)
    if not c or not b: return None
    first=rx.at_cl(bent,t)["cm"]-rx.at_cl(clean,t)["cm"]
    ds=[bb[1]-cc[1] for cc in c for bb in b]
    return dict(clean_cm=c[0][1],bent_cm=b[0][1],dcm=first,nc=len(c),nb=len(b),lo=min(ds),hi=max(ds),
                clean_alpha=c[0][0],bent_alpha=b[0][0])
