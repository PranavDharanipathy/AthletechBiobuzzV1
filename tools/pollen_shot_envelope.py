#!/usr/bin/env python3
"""Illustrative frontal pollen mouth-entry envelope; no robot commands or measured success rates.
FIRST TU07 figures 9-10/11; robot CAD in DEV_NOTES. Standard library only.
"""
import itertools
import json
import math

INCH=.0254
RADIUS=1.4*INCH
TARGET_Z=59.55*INCH
K=.5*1.2*math.pi*RADIUS**2/(.055*.45359237)

def release(angle):
    return (9.30+2.82*math.cos(math.radians(angle)))*INCH

def rhs(s,cd,cl):
    _,_,vx,vz=s
    v=math.hypot(vx,vz)
    return vx,vz,-K*v*(cd*vx+cl*vz),-9.80665+K*v*(-cd*vz+cl*vx)

def step(s,dt,cd,cl):
    a=rhs(s,cd,cl)
    b=rhs(tuple(s[i]+dt*a[i]/2 for i in range(4)),cd,cl)
    c=rhs(tuple(s[i]+dt*b[i]/2 for i in range(4)),cd,cl)
    d=rhs(tuple(s[i]+dt*c[i] for i in range(4)),cd,cl)
    return tuple(s[i]+dt*(a[i]+2*b[i]+2*c[i]+d[i])/6 for i in range(4))

def cross(distance,angle,speed,cd,cl,height_error=0,plane=True):
    a=math.radians(angle)
    s=(0,release(angle)+height_error,speed*math.cos(a),speed*math.sin(a))
    def signed(p):
        return (p[0]-distance)*math.sqrt(3)/2-(p[1]-TARGET_Z)*.5 if plane else p[0]-distance
    for _ in range(1600):
        n=step(s,.005,cd,cl)
        if signed(s)<=0<=signed(n):
            fraction=-signed(s)/(signed(n)-signed(s))
            return tuple(s[i]+fraction*(n[i]-s[i]) for i in range(4))
        if n[1]<0 or n[2]<=0:
            return None
        s=n
    return None

def solve(distance,angle):
    low,high=1,25
    for _ in range(32):
        speed=(low+high)/2
        q=cross(distance,angle,speed,.45,.1,plane=False)
        if q is None or q[1]<TARGET_Z:
            low=speed
        else:
            high=speed
    return (low+high)/2

POLYGON=[(-10,0),(10,0),(10,7.61),(0,14),(-10,7.61)]
def local_margin(distance,q,lateral):
    x,z,vx,vz=q
    u=(vx/math.hypot(vx,vz),0,vz/math.hypot(vx,vz))
    p=(x,lateral,z)
    vertices=[(distance+(s-7)*INCH*.5,y*INCH,TARGET_Z+(s-7)*INCH*math.sqrt(3)/2) for y,s in POLYGON]
    minimum=100
    for a,b in zip(vertices,vertices[1:]+vertices[:1]):
        av=[a[i]-p[i] for i in range(3)]
        ab=[b[i]-a[i] for i in range(3)]
        dot_av=sum(av[i]*u[i] for i in range(3));dot_ab=sum(ab[i]*u[i] for i in range(3))
        av=[av[i]-dot_av*u[i] for i in range(3)]; ab=[ab[i]-dot_ab*u[i] for i in range(3)]
        fraction=max(0,min(1,-sum(av[i]*ab[i] for i in range(3))/sum(v*v for v in ab)))
        minimum=min(minimum,math.sqrt(sum((av[i]+fraction*ab[i])**2 for i in range(3))))
    y=lateral/INCH; s=(z-TARGET_Z)/(INCH*math.sqrt(3)/2)+7
    inside=all((by-ay)*(s-ass)-(bs-ass)*(y-ay)>=0 for (ay,ass),(by,bs) in zip(POLYGON,POLYGON[1:]+POLYGON[:1]))
    return (minimum if inside else -minimum)-(RADIUS+.002+.005)

def main():
    for distance,angle in [(1.8288,54),(2.4384,50)]:
        a=math.radians(angle)
        speed=math.sqrt(9.80665*distance**2/(2*math.cos(a)**2*(distance*math.tan(a)-(TARGET_Z-release(angle)))))
        q=cross(distance,angle,speed,0,0,plane=False)
        assert abs(q[1]-TARGET_Z)<4e-5
    results=[]
    for feet,angle in itertools.product((6,8),(50,54)):
        distance=feet*.3048; speed=solve(distance,angle)
        margins=[]
        for da,dv,dd,dh,dy,cd,cl in itertools.product((-2,2),(-.05,.05),(-.05,.05),(-.0127,.0127),(-.025,.025),(.3,.6),(0,.2)):
            q=cross(distance+dd,angle+da,speed*(1+dv),cd,cl,dh)
            margins.append(-1 if q is None else local_margin(distance+dd,q,dy))
        results.append(dict(distance_ft=feet,nominal_angle_deg=angle,nominal_speed_mps=round(speed,3),
                            illustrative_rpm_eta045=round(60*speed/(.45*math.pi*.072)),
                            uncertainty_cases=len(margins),cases_without_local_rim_clearance=sum(m<=0 for m in margins),
                            worst_local_margin_in=round(min(margins)/INCH,2)))
    print(json.dumps(dict(warning='Hypothetical uncertainty corners; no retention/interior collision model or measured success probability. '
                         'Angle perturbations above estimated 54-degree hood range are hypothetical exit-direction errors, never servo commands.',
                         assumptions=dict(angle_error_deg=2,speed_error_fraction=.05,range_error_m=.05,release_height_error_m=.0127,
                                          lateral_error_m=.025,cd_bounds=[.3,.6],cl_bounds=[0,.2],extra_ball_radius_m=.002,extra_rim_margin_m=.005),results=results),indent=2))

if __name__=='__main__':
    main()
