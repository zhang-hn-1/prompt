"""Post-close execution/interface audit; no future teacher to actor, no source admission."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'E1_current_pixel_navigation_run_v1'
status=json.loads((H/'E1_current_pixel_navigation_status_v1.json').read_text());assert status['status']=='finished' and status['normal_close_confirmed']
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());bad=[f for f,s in manifest['artifact_sha256'].items() if hashlib.sha256(Path(f).read_bytes()).hexdigest()!=s];assert not bad
states=json.loads((D/'states.json').read_text());requests=np.load(D/'requests.npz')['requests'];identity=json.loads((D/'identity.json').read_text());termination=json.loads((D/'termination.json').read_text());assert len(states)==len(requests)+1==identity['captures'];assert len(requests)<=546
source_count=0;removed=0
for p in sorted((D/'partial_observations').glob('*.npz')):
 c=np.load(p);b=np.load(D/'baseline_partial_observations'/p.name);images=np.load(D/'source_inputs'/p.name)
 q=b['query_pixel_id'];r=b['partner_pixel_id'];left=images['left_rgb'];right=images['right_rgb'];keep=~np.all(left[q//640,q%640]==0,axis=1)&~np.all(right[r//640,r%640]==0,axis=1)
 for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:assert np.array_equal(c[k],b[k][keep]),(p,k)
 assert float(c['acquisition_time_s'])==float(b['acquisition_time_s'])
 removed+=int((~keep).sum());source_count+=1
videos={}
for p in D.glob('*.mp4'):
 cap=cv2.VideoCapture(str(p));count=0;fps=cap.get(cv2.CAP_PROP_FPS)
 while True:
  ok,img=cap.read()
  if not ok:break
  count+=1
 cap.release();assert count==len(states),(p,count);videos[p.name]={'decoded_frames':count,'fps':fps}
shadow=json.loads((D/'B0_shadow_until_fork.json').read_text());contacts=json.loads((D/'physics_contacts.json').read_text())
result={'scope':'ACTUAL_KNOWN_GTPOSE_TaskH_NOT_STRICT35','sealed_artifacts_verified':len(manifest['artifact_sha256']),'requests':len(requests),'states':len(states),'source_epochs':source_count,'exact_filtered_membership_all_epochs':True,'removed_exactblack_points':removed,'videos':videos,'termination':termination,'shadow_rows':len(shadow),'shadow_last_row':shadow[-1] if shadow else None,'physics_contact_samples':len(contacts),'full_research_complete':False,'source_admission':False,'formal35_NOTRUN':True}
if (D/'local54_6_gate.json').exists():result['local_gate']=json.loads((D/'local54_6_gate.json').read_text())
(D/'execution_interface_verification_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='shadow_last_row'}))
