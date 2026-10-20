from pathlib import Path
import json,hashlib,numpy as np,cv2
h=Path(__file__).resolve().parent;out={};videos=[];rows=[]
for suite,expected in [('golden',3),('hard_0',7)]:
 root=h/('navigation_'+suite);ps=sorted(root.glob('episode_*/metrics.json'));assert len(ps)==expected,(suite,len(ps))
 summary=json.loads((root/'summary.json').read_text());items=[]
 for p in ps:
  m=json.loads(p.read_text());env=json.loads((p.parent/'environment.json').read_text());z=np.load(p.parent/'states.npz');item={k:m.get(k) for k in ['episode_id','difficulty','success','collision','timeout','runtime_error','termination_reason','elapsed_s','final_distance_m','path_length_m','spl','minimum_clearance_m']};item['saved_physical_contact_samples']=int(z['physical_contact'].sum());item['tracking_infeasible_states']=int((z['policy_braking_state_code']==5).sum());item['direction_braking_states']=int((z['policy_braking_state_code']==2).sum());item['budget_verified']=env['episode_budget_s']==35 and m['episode_budget_s']==35;assert item['budget_verified'];items.append(item);rows.append(dict(suite=suite,**item))
  for name in ['third_person.mp4','stereo_view.mp4']:
   v=p.parent/name;cap=cv2.VideoCapture(str(v));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=float(cap.get(cv2.CAP_PROP_FPS));first,im=cap.read();cap.set(cv2.CAP_PROP_POS_FRAMES,max(0,n-1));last,end=cap.read();cap.release();assert first and last and n>0 and fps>0,str(v);videos.append({'path':str(v),'frames':n,'fps':fps,'first_last_decodable':True})
 out[suite]={'summary':summary,'episodes':items}
identity=json.loads((h/'navigation_source_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items())
oldassets=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_golden_oracle_reproduced_20260922_v1/audit/asset_sha256.json');assets=json.loads(oldassets.read_text());verified=[dict(r,current_sha256=hashlib.sha256(Path(r['path']).read_bytes()).hexdigest()) for r in assets];assert all(r['sha256']==r['current_sha256'] for r in verified)
(h/'navigation_assessment.json').write_text(json.dumps(out,indent=2));(h/'video_verification.json').write_text(json.dumps(videos,indent=2));(h/'asset_verification.json').write_text(json.dumps(verified,indent=2));print(json.dumps(rows,indent=2));print('All source hashes unchanged; videos checked:',len(videos))
