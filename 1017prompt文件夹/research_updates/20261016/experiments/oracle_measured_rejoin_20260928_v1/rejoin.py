import numpy as np
from gate import clearance

def rejoin_path(start,goal,obstacle,forward):
 start=np.asarray(start);goal=np.asarray(goal);obstacle=np.asarray(obstacle);forward=np.asarray(forward)
 points=np.array([start,goal])
 if clearance(points,obstacle,.65)<=0:
  normal=np.array([-forward[1],forward[0]])
  side=1 if np.dot(start-obstacle,normal)>=0 else -1
  checkpoint=obstacle+.8*forward+side*normal
  points=np.array([start,checkpoint,goal])
 if clearance(points,obstacle,.65)<=0:return None
 segments=[np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.2))+1))[:-1] for a,b in zip(points[:-1],points[1:])]
 return np.vstack([*segments,goal[None,:]])
