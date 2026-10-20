"""Retrospective net course change; invalid gaps are never motion evidence."""
import numpy as np

def net_course_arc(position,course,speed):
 position=np.asarray(position,float);course=np.asarray(course,float);speed=np.asarray(speed,float)
 valid=np.isfinite(course)&np.isfinite(speed)&(speed>=.05)&np.isfinite(position).all(axis=1)
 changes=[];arcs=[];samples=0;start=None
 for i in range(len(valid)+1):
  if i<len(valid) and valid[i]:
   if start is None:start=i
  elif start is not None:
   if i-start>=2:
    arc=float(np.linalg.norm(np.diff(position[start:i],axis=0),axis=1).sum())
    if arc>0:
     unwrapped=np.unwrap(course[start:i]);changes.append(float(unwrapped[-1]-unwrapped[0]));arcs.append(arc);samples+=i-start
   start=None
 total=float(np.linalg.norm(np.diff(position,axis=0),axis=1).sum());covered=sum(arcs);change=sum(changes)
 return dict(segments=len(arcs),covered_samples=samples,covered_arc_m=covered,excluded_arc_m=total-covered,arc_coverage=None if total==0 else covered/total,course_change_rad=change,net_curvature_1_m=None if covered==0 else change/covered)
