#!/usr/bin/env python3
"""Hood 4-bar kinematics from Onshape CAD points (read 2026-10-31). See DEV_NOTES.md section 0, "Hood mechanism".
Shooter Assy frame, inches: y = rear(+)/forward(-), z = up. Run: python3 tools/hood_fourbar.py"""
import math
# Shooter Assy frame (in), from CAD 2026-10-31
O=(-0.64,-4.60)
S=(5.42,-2.71)
C=(4.41,-3.33)
P=(3.42,-2.42)
HOOD_EXIT_NOW=50.59
d=lambda a,b: math.hypot(a[0]-b[0],a[1]-b[1])
ang=lambda a,b: math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
g,cr,cp,rk=d(O,S),d(S,C),d(C,P),d(O,P)
print(f"ground {g:.3f} crank {cr:.3f} coupler {cp:.3f} rocker {rk:.3f}")
s,l=min(g,cr,cp,rk),max(g,cr,cp,rk); print("Grashof (s+l<=p+q):", s+l <= g+cr+cp+rk-s-l, f"{s+l:.3f} vs {g+cr+cp+rk-s-l:.3f}")
th0=ang(S,C); ph0=ang(O,P)
print(f"current crank angle {th0:.1f} deg, rocker angle {ph0:.1f} deg")
def solve(th, branch):
    c=(S[0]+cr*math.cos(math.radians(th)), S[1]+cr*math.sin(math.radians(th)))
    D=d(O,c)
    if D>cp+rk or D<abs(cp-rk): return None
    base=math.degrees(math.atan2(c[1]-O[1],c[0]-O[0]))
    a=math.degrees(math.acos((rk*rk+D*D-cp*cp)/(2*rk*D)))
    return base+branch*a
br=min((1,-1),key=lambda b: abs((solve(th0,b) or 1e9)-ph0)); print("branch",br, "check", round(solve(th0,br),2))
rows=[]
for k in range(-1800,1801):
    th=th0+k*0.1; ph=solve(th,br)
    if ph is not None: rows.append((th-th0,ph-ph0))
idx=[i for i,(a,_) in enumerate(rows) if abs(a)<1e-9][0]
lo=idx
while lo>0 and abs(rows[lo-1][0]-rows[lo][0]-(-0.1))<1e-6: lo-=1
hi=idx
while hi<len(rows)-1 and abs(rows[hi+1][0]-rows[hi][0]-0.1)<1e-6: hi+=1
seg=rows[lo:hi+1]
print(f"feasible servo travel from current: {seg[0][0]:+.1f} .. {seg[-1][0]:+.1f} deg (span {seg[-1][0]-seg[0][0]:.1f}) (ends = linkage toggle positions)")
dph=[r[1] for r in seg]; print(f"hood rotation range: {min(dph):+.1f} .. {max(dph):+.1f} deg (span {max(dph)-min(dph):.1f})")
el=lambda dp: 90-(HOOD_EXIT_NOW+dp)
print(f"launch elevation now {el(0):.1f}; range {el(max(dph)):.1f} .. {el(min(dph)):.1f} deg")
print("servo deg -> hood deg -> launch elev (every 20 deg)")
for a,p in seg[::200]: print(f"  {a:+7.1f}  {p:+6.1f}  {el(p):5.1f}")
def tr(th):
    c=(S[0]+cr*math.cos(math.radians(th)), S[1]+cr*math.sin(math.radians(th)))
    ph=solve(th,br); p=(O[0]+rk*math.cos(math.radians(ph)),O[1]+rk*math.sin(math.radians(ph)))
    v1=(c[0]-p[0],c[1]-p[1]); v2=(O[0]-p[0],O[1]-p[1])
    return math.degrees(math.acos((v1[0]*v2[0]+v1[1]*v2[1])/(math.hypot(*v1)*math.hypot(*v2))))
print(f"transmission angle now {tr(th0):.1f} deg (90 ideal)")
print("\nservo  hood   elev   transmission  d(elev)/d(servo)")
prev=None
for a,p in seg[::50]:
    t=tr(th0+a); s_=(f"{(el(p)-el(prev))/5:+.2f}" if prev is not None else "")
    print(f"{a:+7.1f} {p:+6.1f} {el(p):5.1f}  {t:6.1f}   {s_}"); prev=p
good=[(a,p) for a,p in seg if 30<=tr(th0+a)<=150]
imax=max(range(len(seg)),key=lambda i:seg[i][1])
mono=[(a,p) for a,p in seg[imax:] if 30<=tr(th0+a)<=150]
print(f"\nusable (monotonic, transmission 30-150): servo {mono[0][0]:+.1f}..{mono[-1][0]:+.1f} (span {mono[-1][0]-mono[0][0]:.0f} deg) -> elev {el(mono[0][1]):.1f}..{el(mono[-1][1]):.1f}")
