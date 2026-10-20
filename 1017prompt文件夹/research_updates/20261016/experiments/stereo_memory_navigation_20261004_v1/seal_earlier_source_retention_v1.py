"""Two early birth sources × four frozen old-B0 current exposures; no new depth."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from past_point_rgb_memory import PastPointRGBMemory
from memory_geometry_prompt_v2 import memory_geometry_prompt
H=Path(__file__).resolve().parent;D=H/'qualified_hazard_navigation_run_v1';S=H/'earlier_source_opportunity_v1';O=H/'earlier_source_retention_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();pair=json.loads((S/'records.json').read_text())['pair'];assert pair==[145,150];manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());paths=[Path(__file__),H/'past_point_rgb_memory.py',H/'memory_geometry_prompt_v2.py',H/'strict_bilinear_support.py',H.parent.parent/'docs/superpowers/plans/2026-10-07-earlier-qualified-source-opportunity.md',S/'prediction_seal.json'];rows=[];guards=0
class Bomb(dict):
 def __getitem__(self,key):
  if key=='time_s':return super().__getitem__(key)
  raise AssertionError('future payload read')
for sf in pair:
 sp=D/'sensor_packets'/f'source_{sf:03d}.npz';bp=D/'online_inputs'/f'frame_{sf:03d}.npz';paths.extend([sp,bp]);assert all(sha(p)==manifest['artifact_sha256'][str(p)] for p in [sp,bp]);source=np.load(sp);rgb=np.load(bp);birth=float(source['time_s']);m=PastPointRGBMemory();m.seed(source,rgb,birth,qualified=True)
 for frame in [175,185,195,205]:
  name=f'frame_{frame:03d}.npz';rp=D/'online_inputs'/name;pp=D/'known_poses'/name;mp=D/'masks'/name;paths.extend([rp,pp,mp]);assert all(sha(p)==manifest['artifact_sha256'][str(p)] for p in [rp,pp,mp]);rgb=np.load(rp);pose=np.load(pp);mask=np.load(mp);now=float(rgb['time_s']);proof=m.query(rgb,pose,mask,now);left=memory_geometry_prompt(proof,pose,mask,now,frame,'left');right=memory_geometry_prompt(proof,pose,mask,now,frame,'right');a=left['past_point_index'];b=right['past_point_index'];ids=np.intersect1d(a[a>=0],b[b>=0]);w=proof['past_world'][ids];bins=len(np.unique(np.floor(w[:,:2]/.1).astype(np.int64),axis=0)) if len(w) else 0;fname=f'birth_{sf:03d}_current_{frame:03d}.npz';np.savez_compressed(O/fname,old_world=w,left_uv=proof['left_uv'][ids],right_uv=proof['right_uv'][ids],past_query_id=proof['past_query_id'][ids],past_partner_id=proof['past_partner_id'][ids],geometry_time_s=birth,current_time_s=now,source_frame=sf,current_frame=frame,is_new_depth_observation=False,certified_free=False);rows.append(dict(file=fname,source_frame=sf,current_frame=frame,birth_time_s=birth,current_time_s=now,age_s=now-birth,n=len(w),XY_bins=bins,left_eligible=left['nonzero_prompts'],right_eligible=right['nonzero_prompts'],is_new_depth_observation=False))
  try:m.query(Bomb(time_s=now+.1),pose,mask,now)
  except ValueError:guards+=1
  else:raise AssertionError('future query allowed')
(O/'records.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'source_custody.json').write_text(json.dumps(dict(source_sha256={str(p):sha(p) for p in paths},GT_read=False,NN_calls=0,physics_calls=0,future_guards=guards,knownpose_privileged=True,old_B0_followup_not_new_control_labels=True,not_fresh=True),indent=2)+'\n');(O/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),output_sha256={str(p):sha(p) for p in O.iterdir() if p.is_file()},source_sha256={str(p):sha(p) for p in paths},GT_read=False,whole_research_complete=False),indent=2)+'\n');print(json.dumps(rows,indent=2))
