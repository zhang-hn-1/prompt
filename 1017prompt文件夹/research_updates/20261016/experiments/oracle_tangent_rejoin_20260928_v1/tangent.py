import math,numpy as np
from gate import clearance

def tangent_path(start,yaw,goal,obstacle):
 start=np.asarray(start,float);goal=np.asarray(goal,float);r=2.;candidates=[]
 if np.linalg.norm(start-obstacle)<=.65:return None
 forward=np.array([math.cos(yaw),math.sin(yaw)]);left=np.array([-forward[1],forward[0]])
 direct=goal-start
 if abs(math.atan2(forward[0]*direct[1]-forward[1]*direct[0],np.dot(forward,direct)))<1e-8 and clearance([start,goal],obstacle,.65)>0:
  return np.linspace(start,goal,max(2,int(np.ceil(np.linalg.norm(direct)/.1))+1))
 for sign in [-1,1]:
  c=start+sign*r*left;v=goal-c;d=np.linalg.norm(v)
  if d<=r:continue
  phi=math.atan2(v[1],v[0]);a0=math.atan2(start[1]-c[1],start[0]-c[0])
  for a in [phi-math.acos(r/d),phi+math.acos(r/d)]:
   t=c+r*np.array([math.cos(a),math.sin(a)]);heading=sign*np.array([-math.sin(a),math.cos(a)])
   if np.dot(heading,goal-t)<=0:continue
   sweep=(sign*(a-a0))%(2*math.pi);angles=a0+sign*np.linspace(0,sweep,max(2,int(np.ceil(r*sweep/.1))+1));arc=c+r*np.c_[np.cos(angles),np.sin(angles)]
   line=np.linspace(t,goal,max(2,int(np.ceil(np.linalg.norm(goal-t)/.1))+1));p=np.vstack([arc[:-1],line]);p[0]=start
   if clearance(p,obstacle,.65)>0:candidates.append((r*sweep+np.linalg.norm(goal-t),p))
 return min(candidates,key=lambda x:x[0])[1] if candidates else None
