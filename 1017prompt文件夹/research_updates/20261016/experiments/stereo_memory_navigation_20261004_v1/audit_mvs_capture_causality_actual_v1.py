from pathlib import Path
import json,datetime,numpy as np
from mvs_capture_causality import captured_request
from mvs_causal_packet_bridge import build_request
H=Path(__file__).resolve().parent;A=H/'mvsanywhere_source_preflight_v1';O=H/'qualified_hazard_navigation_run_v1';clocks={r['frame']:r['time_s'] for r in json.loads((O/'camera_clocks.json').read_text())};rows=[]
for i in [235,245,255]:
 for eye in ['left','right']:
  actual=captured_request(O,i,eye,clocks,np.load)
  def packet(j):
   im=np.load(O/'online_inputs'/f'frame_{j:03d}.npz');p=np.load(O/'known_poses'/f'frame_{j:03d}.npz');m=np.load(O/'masks'/f'frame_{j:03d}.npz');return dict(time_s=float(im['time_s']),rgb=im[eye+'_rgb'],pose=p[eye+'_camera_usd_world'],mask=m[eye])
  old=build_request(packet(i),[packet(i-10),packet(i-20)],clocks[i]);fields=0
  for key in old:
   if isinstance(old[key],list):assert all(np.array_equal(x,y) for x,y in zip(actual[key],old[key]))
   else:assert np.array_equal(actual[key],old[key])
   fields+=1
  guards=0
  for kind,frame in [('online_inputs',i),('known_poses',i-20),('masks',i-10)]:
   class Proxy:
    def __init__(self,p):self.file=np.load(p);self.poison=p.parent.name==kind and int(p.stem.split('_')[-1])==frame
    def __getitem__(self,key):
     if key not in ['time_s','frame']:raise AssertionError('payload read before poison rejection')
     value=self.file[key]
     return float(value)+.1 if self.poison and key=='time_s' else value
   try:captured_request(O,i,eye,clocks,lambda p:Proxy(p));raise AssertionError('poison unexpectedly accepted')
   except ValueError:guards+=1
  rows.append(dict(frame=i,eye=eye,identical_fields=fields,actual_clock_payload_guards=guards))
path=A/'capture_causality_actual_audit_v1.json';assert not path.exists();path.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,no_model_or_truth_read=True,predictor_only_current_and_two_past_payloads=True),indent=2)+'\n');print(rows)
