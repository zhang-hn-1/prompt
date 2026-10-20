from pathlib import Path
import json,hashlib,numpy as np
D=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();rows=json.loads((D/'coverage.json').read_text())['rows'];C=D.parents[1]/'experiments/source_confidence_calibration_20261012_v1';out=D/'sgbm_predictions';out.mkdir(exist_ok=True);inputs={}
ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
for name in ['H','E0','E1']:
 R=D.parents[1]/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run' if name=='H' else C/f'data_{name.lower()}_run';pred=[]
 for p in sorted((R/'partial_observations').glob('*.npz')):
  a=np.load(p);kp=R/'known_poses'/p.name;b=np.load(kp);assert float(a['acquisition_time_s'])==float(b['time_s']);r=b['left_camera_usd_world'][:3,:3]@np.diag([1,-1,-1])@ROT.T;yaw=np.arctan2(r[1,0],r[0,0]);world=a['world'];xy=(world[:,:2]-b['root_position_world_m'][:2])@np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);inside=(xy[:,0]>=0)&(xy[:,0]<4)&(xy[:,1]>=-3)&(xy[:,1]<3);xy=xy[inside];z=world[inside,2];idx=np.floor((xy-[0,-3])/.1).astype(int);code=np.where(z<=.5,2,np.where(z<=1,3,4));m=np.zeros((40,60),np.uint8)
  if len(idx):np.maximum.at(m,tuple(idx.T),code)
  pred.append(m)
  for pp in [p,kp]:inputs[str(pp)]=sha(pp)
 np.savez_compressed(out/f'{name}.npz',labels=np.stack(pred))
(D/'sgbm_prediction_seal.json').write_text(json.dumps(dict(inputs=inputs,outputs={str(p):sha(p) for p in out.glob('*.npz')},source_sha=sha(Path(__file__)),truth_read=False,scope='current accepted partial surface points, any positive point forms cell; native ego registration; no ground/free output or historical risk'),indent=2)+'\n')
