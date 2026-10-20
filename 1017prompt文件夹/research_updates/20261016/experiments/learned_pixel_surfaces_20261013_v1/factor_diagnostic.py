"""Prespecified whole E1 factor diagnosis, offline GT replacements only.
No candidate change, fitting or navigation; unknown confidence/domain retained.
"""
from common import *
import numpy as np
from category_projection import point_packet,raster
from projection import root_from_camera
from rig_adapter import Rig,RigMask
from dynamic_self_mask import joints_from_proprio
old=(W/'experiments/learned_visible_obstacles_20261013_v1/score.py').read_text();ns={}
exec('import numpy as np\nfrom scipy.ndimage import maximum_filter,distance_transform_edt\n'+old[old.index('def metrics('):old.index('\nrows=[]')],ns);metrics=ns['metrics']
assert not json.loads((D/'result.json').read_text())['source_qualified']
verify_oracle();seal=json.loads((D/'prediction_seal.json').read_text())
for key in ['outputs_sha256','inputs_sha256','source_sha256']:verify_hashes(seal[key])
assert sha(D/'model_final.pt')==seal['model_sha256']
cov=json.loads((D/'coverage.json').read_text());verify_hashes(cov['input_sha256']);verify_hashes(cov['output_sha256'])
cad=RigMask(Rig('forward',.4,.3,.05));R=GROUPS['E1'];grids={'ideal_category_learned_range':[],'learned_category_ideal_range':[],'learned_both_same_gt_support':[]}
paths=sorted((D/'predictions/E1').glob('*.npz'));assert [int(p.stem.split('_')[1]) for p in paths]==list(range(0,901,5))
for j,p in enumerate(paths):
 a=np.load(p);f=np.load(R/'frames'/p.name);lab=np.load(Q/'labels/E1'/p.name);n=np.load(R/'known_poses'/p.name);h=np.load(R/'history'/p.name);mask=cad.project(joints_from_proprio(h['proprio_history']));nr,root=root_from_camera(n['left_camera_usd_world']);yaw=np.arctan2(nr[1,0],nr[0,0])
 assert float(a['time_s'])==float(f['time_s'])==float(lab['time_s'])==float(n['time_s'])
 for method in grids:
  ws=[];cs=[]
  for ei,eye in enumerate(['left','right']):
   if method=='ideal_category_learned_range':z=a['depth_m'][ei];cl=np.where(a['category'][ei]>0,lab[eye+'_category'],0)
   else:z=f[eye+'_depth_m'] if method=='learned_category_ideal_range' else a['depth_m'][ei];cl=np.where(lab[eye+'_eligible']&(lab[eye+'_category']>0),a['category'][ei],0)
   w,c,_=point_packet(z,cl,n[eye+'_camera_usd_world'],root,mask[eye]);ws.append(w);cs.append(c)
  grids[method].append(raster(np.concatenate(ws),np.concatenate(cs),root,yaw))
 if j%90==0:print(j+1,flush=True)
arr={k:np.stack(v) for k,v in grids.items()};np.savez_compressed(D/'E1_factor_maps.npz',**arr);target=np.load(Q/'E1_maps.npz')['target'];actual=np.load(D/'E1_maps.npz')['prediction'];met={k:metrics(target,v) for k,v in arr.items()};met['actual_learned']=metrics(target,actual)
save('factor_diagnostic.json',dict(group='E1',pairs=181,metrics=met,confidence_domain='learned positive/GT eligible support retained; oracle replacement does not fill unknown predictions; same-GT-support learned-both control distinguishes unsupported-domain removal',scope='Whole seen E1 diagnostic: authored-compatible category or renderer range substituted only offline; no source rescue/nav/second fit. Actual learned includes unsupported predictions; compare factor arms to same-support control, not attribute all change from actual to one factor.',map_sha256=sha(D/'E1_factor_maps.npz'),source_sha256=sha(D/'factor_diagnostic.py')))
