from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;D=H/'causal_four_view_new_region_v1';P=H/'causal_four_view_prefix_verification_v1'
s=(H/'predict_causal_four_view_new_region_v1.py').read_text();s=s.replace("D=H/'causal_four_view_new_region_v1'","D=H/'causal_four_view_prefix_verification_v1'");s=s.replace("sorted((O/'sensor_packets').glob('*.npz')):","sorted((O/'sensor_packets').glob('*.npz'))[:40]:")
exec(compile(s,str(H/'predict_causal_four_view_new_region_v1.py'),'exec'),{'__file__':str(H/'predict_causal_four_view_new_region_v1.py')})
fields=0;packets=0
for f in P.glob('frame_*.npz'):
 a=np.load(f);b=np.load(D/f.name);assert a.files==b.files
 for k in a.files:assert np.array_equal(a[k],b[k],equal_nan=True),(f,k);fields+=1
 packets+=1
from past_point_rgb_memory import PastPointRGBMemory
class Trap:
 def __getitem__(self,k):
  if k=='time_s':return 20.
  raise AssertionError('future payload accessed')
O=H/'qualified_hazard_navigation_run_v1';guards=0
for i in range(0,200,5):
 m=PastPointRGBMemory();src=np.load(O/'sensor_packets'/f'source_{i:03d}.npz');rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz');mask=np.load(O/'masks'/f'frame_{i:03d}.npz');t=float(src['time_s']);m.seed(src,rgb,t,qualified=True)
 for r,p in [(Trap(),pose),(rgb,Trap())]:
  try:m.query(r,p,mask,t)
  except ValueError:guards+=1
  else:raise AssertionError('future accepted')
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),prefix_updates=40,prefix_last_frame=195,exact_pair_packets=packets,exact_fields=fields,future_RGB_or_pose_guards=guards,no_labels_or_actions_read=True,archive_wide_future_poison_not_run=True,knownpose_privileged=True,source_only_prefix=True)
f=D/'prefix_verification_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n');print(result)
