import numpy as np

def geometric_curvature(xy, half_window=25, minimum_chord=.075):
    xy=np.asarray(xy,float);out=np.full(len(xy),np.nan)
    for i in range(half_window,len(xy)-half_window):
        a=xy[i]-xy[i-half_window];b=xy[i+half_window]-xy[i]
        na=np.linalg.norm(a);nb=np.linalg.norm(b)
        if min(na,nb)<minimum_chord:continue
        out[i]=np.arctan2(a[0]*b[1]-a[1]*b[0],np.dot(a,b))/((na+nb)/2)
    return out

def first_sustained(values, threshold, sign, start, count=3):
    for i in range(start,len(values)-count+1):
        if np.all(sign*np.asarray(values[i:i+count])>=threshold):return i
    return None

def reversal(records):
    initial=records[0]['active_drive_direction'];after=False;armed=0;streak=0;lastsign=0
    for i,r in enumerate(records):
        if r['source']=='tracking' and r['active_drive_direction']!=initial:after=True
        if not after or r['source']!='tracking' or r['world_xy_speed_m_s']<.15:
            streak=0;continue
        a=r['policy_protocol_auxiliary_target_rad'];sgn=1 if a>0 else -1
        if not armed:
            if abs(a)>=np.radians(20):
                streak=streak+1 if sgn==lastsign else 1
                lastsign=sgn
                if streak>=3:armed=sgn
            else:streak=0
        elif i+2<len(records):
            future=records[i:i+3]
            if all(x['source']=='tracking' and x['world_xy_speed_m_s']>=.15 and -armed*x['policy_protocol_auxiliary_target_rad']>=np.radians(10) for x in future):return i,-armed
    return None
