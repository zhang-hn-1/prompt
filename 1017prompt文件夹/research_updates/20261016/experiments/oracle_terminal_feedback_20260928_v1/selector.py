import math,numpy as np
from geometry import relocate
from gate import clearance

def select_sequence(hold,turn,start,yaw,goal,obstacle,remaining_s):
 candidates=[]
 for duration in np.arange(0,4.01,.5):
  n=int(round(duration/.02));hxy=hold['position_world_xy'][800:801+n];hyaw=hold['yaw_world_rad'][800:801+n]
  prefix=relocate(hxy,float(hyaw[0]),start,yaw)
  endyaw=float(hyaw[-1]-hyaw[0]+yaw)
  txy=relocate(turn['position_world_xy'][400:],float(turn['yaw_world_rad'][400]),prefix[-1],endyaw)
  finalyaw=float(turn['yaw_world_rad'][-1]-turn['yaw_world_rad'][400]+endyaw)
  xy=np.vstack([prefix[:-1],txy]);delta=np.asarray(goal)-xy[-1];distance=float(np.linalg.norm(delta));bearing=math.atan2(delta[1],delta[0]);angle=abs(math.atan2(math.sin(bearing-finalyaw),math.cos(bearing-finalyaw)))
  score=float(duration+4+distance/.4+angle/.25);margin=clearance(xy,obstacle,.65)
  candidates.append({'hold_s':float(duration),'compound_s':float(duration+4),'score_s_proxy':score,'goal_distance_m':distance,'heading_error_rad':angle,'clearance_m':margin,'eligible':bool(margin>0 and score<=remaining_s),'xy':xy,'end_yaw':finalyaw})
 valid=[c for c in candidates if c['eligible']]
 return (min(valid,key=lambda c:(c['score_s_proxy'],c['hold_s'])) if valid else None),candidates
