"""Independent post-output CPU evidence validation; no model or checkpoint deserialization."""
from pathlib import Path
import numpy as np, cv2, json, hashlib, datetime, sys
H=Path(__file__).resolve().parents[1]; E=Path(__file__).resolve().parent
A=H/'mvsanywhere_source_preflight_v1'; O=H/'qualified_hazard_navigation_run_v1'; D=H/'mvs_hero_temporal_development_v2_api_scale_recovery'
def read(p): return json.loads(Path(p).read_text())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
 return h.hexdigest()
checks={}; issues=[]
def check(name, val):
 checks[name]=bool(val)
 if not val: issues.append(name)
seal=read(D/'prediction_seal.json'); custody=read(A/'whole_source_input_seal_v2.json'); author=read(D/'assessment_v1.json')
for group, mapping in [('outputs',seal['output_sha256']),('inputs',custody['input_sha256']),('code',custody['code_sha256']),('plans',custody['plan_sha256'])]:
 check(group+'_all_sha',all(sha(p)==v for p,v in mapping.items()))
checks['hash_counts']={k:len(v) for k,v in [('outputs',seal['output_sha256']),('inputs',custody['input_sha256']),('code',custody['code_sha256']),('plans',custody['plan_sha256'])]}
for key,p in [('whole_source_input_seal_sha256',A/'whole_source_input_seal_v2.json'),('artifact_identity_sha256',A/'artifact_identity_v1.json'),('source_identity_sha256',A/'source_identity.json')]:check(key,sha(p)==seal[key])
for key,p in [('independent_camera_clock_log_sha256',O/'camera_clocks.json'),('selected_clock_authority_sha256',A/'selected_clock_authority_v1.json'),('upstream_source_identity_sha256',A/'source_identity.json'),('DINOv2_source_identity_sha256',A/'DINOv2_source_identity_v1.json'),('artifact_identity_sha256',A/'artifact_identity_v1.json')]:check('custody_'+key,sha(p)==custody[key])
for name in ['source_identity.json','DINOv2_source_identity_v1.json']:
 ident=read(A/name); mapping=ident.get('files_sha256',ident.get('DINOv2_files_sha256',{})); assert mapping
 check(name+'_source_tree_sha',all(sha(A/p)==v for p,v in mapping.items())); checks[name+'_file_count']=len(mapping)
old=read(H/'mvs_hero_temporal_development_v1/runtime_identity_v1.json'); new=read(D/'runtime_identity_v1.json')
check('old_new_reported557_state_SHA_identical',old['strict_equal_state_tensors']==new['strict_equal_state_tensors']==557 and old['final_state_sha256']==new['final_state_sha256'])
checks['old_new_state_SHA']=new['final_state_sha256']
check('checkpoint_file_sha',sha(A/'weights/mvsanywhere_hero.ckpt')==new['Hero_sha256']==read(A/'artifact_identity_v1.json')['Hero_sha256'])
ts=lambda s:datetime.datetime.fromisoformat(s)
check('recorded_chronology',ts(custody['utc'])<ts(new['utc'])<ts(seal['utc'])<ts(author['utc']))
checks['chronology_utc']={k:v['utc'] for k,v in [('whole_input',custody),('runtime_before_first_model_call',new),('six_output_seal',seal),('assessment',author)]}
check('mtime_chronology', (A/'whole_source_input_seal_v2.json').stat().st_mtime < min((D/r['file']).stat().st_mtime for r in read(D/'records.json')) < (D/'prediction_seal.json').stat().st_mtime < (D/'assessment_v1.json').stat().st_mtime)
clock={int(k):v for k,v in read(A/'selected_clock_authority_v1.json').items()}; fx=584.8888888888889; K=np.array([[fx,0,319.5],[0,fx,239.5],[0,0,1.]])
def wpoints(uv,z,T):
 rays=np.column_stack(((uv[:,0]-319.5)/fx,(uv[:,1]-239.5)/fx,np.ones(len(uv))))
 return T[:3,3]+(rays*np.asarray(z)[:,None])@(T[:3,:3]@np.diag([1.,-1.,-1.])).T
def project(w,T):
 q=(w-T[:3,3])@(T[:3,:3]@np.diag([1.,-1.,-1.])); p=q@K.T
 return p[:,:2]/p[:,2,None],q[:,2]
seed=np.load(O/'sensor_packets/source_180.npz')['world'];seed=seed[(seed[:,2]>.05)&(seed[:,2]<=1.5)];sb=set(map(tuple,np.floor(seed[:,:2]/.1).astype(int)))
offset=np.array([(x,y) for y in range(-3,4) for x in range(-3,4)]); rows=[]
for r in read(D/'records.json'):
 frame,eye=r['frame'],r['eye']; f=np.load(D/r['file'],allow_pickle=False); uv=f['current_uv']; z=f['selected_depth_m']; w=f['world']; n=len(w); frames=[frame,frame-10,frame-20]
 poses=[np.load(O/'known_poses'/f'frame_{j:03d}.npz')[eye+'_camera_usd_world'] for j in frames]
 masks=[np.load(O/'masks'/f'frame_{j:03d}.npz')[eye] for j in frames]
 native=True
 for j in frames:
  for kind in ['online_inputs','known_poses','masks']:
   p=np.load(O/kind/f'frame_{j:03d}.npz',allow_pickle=False);native &= abs(float(p['time_s'])-clock[j])<=1e-9
   if kind=='online_inputs':native &= p[eye+'_rgb'].shape==(480,640,3) and p[eye+'_rgb'].dtype==np.uint8
   if kind=='masks':native &= int(p['frame'])==j
 check(f'{frame}_{eye}_native_causal_stamps',native and np.array_equal(f['source_frames'],frames[1:]) and np.allclose(f['source_time_s'],[clock[j] for j in frames[1:]],atol=1e-9,rtol=0) and float(f['current_time_s'])==clock[frame] and float(f['geometry_available_time_s'])==clock[frame] and all(0<clock[frame]-clock[j]<=2+1e-8 for j in frames[1:]))
 check(f'{frame}_{eye}_flags',not bool(f['certified_free']) and not bool(f['source_model_admitted']) and bool(f['learned_geometry_hypothesis_only']))
 check(f'{frame}_{eye}_query_identity',len(set(f['query_pixel_id'].tolist()))==n and np.array_equal(f['query_pixel_id'],uv[:,1]*640+uv[:,0]) and np.all(uv%2==0))
 check(f'{frame}_{eye}_raw_depth',f['depth_m'].shape==(480,640) and np.isfinite(z).all() and np.all((z>=.5)&(z<=3)) and np.array_equal(z,f['depth_m'][uv[:,1],uv[:,0]]))
 ww=wpoints(uv,z,poses[0]); wm=float(np.max(np.abs(ww-w))); pastmax=max(float(np.max(np.abs(project(ww,T)[0]-f['past_uv'][j]))) for j,T in enumerate(poses[1:]));check(f'{frame}_{eye}_world_pastUV',wm<1e-10 and pastmax<1e-9)
 root=poses[0][:3,3]-(poses[0][:3,:3]@np.diag([1.,-1.,-1.]))@np.array([[0,0,1],[0,-1,0],[1,0,0]]).T@np.array([.2,.3 if eye=='left' else -.3,.05])
 check(f'{frame}_{eye}_native_root_hazard',np.max(np.abs(root-f['derived_root_position_world_m']))<1e-10 and np.all((w[:,2]>.05)&(w[:,2]<=1.5)&(np.linalg.norm(w-root,axis=1)>.55)))
 failures=0
 for start in range(0,n,2048):
  tap=uv[start:start+2048,None,:]+offset; tx,ty=tap.reshape(-1,2).T
  good=(tx>=0)&(tx<640)&(ty>=0)&(ty<480);good &= ~masks[0][np.clip(ty,0,479),np.clip(tx,0,639)]
  tw=wpoints(tap.reshape(-1,2),np.repeat(z[start:start+2048],49),poses[0])
  for T,mask in zip(poses[1:],masks[1:]):
   xy,dz=project(tw,T);mx=xy[:,0].reshape(-1,49).astype(np.float32);my=xy[:,1].reshape(-1,49).astype(np.float32)
   inside=np.isfinite(mx)&np.isfinite(my)&(mx>=0)&(my>=0)&(mx<=639)&(my<=479)&(dz.reshape(-1,49)>0)
   support=cv2.remap((~mask).astype(np.float32),mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)==1
   good &= (inside&support).ravel()
  failures+=int((~good.reshape(-1,49).all(1)).sum())
 check(f'{frame}_{eye}_all49_CAD_bothpast_support',failures==0)
 errs=[]
 for j,xy in zip(frames,[uv,*f['past_uv']]):
  t=np.load(O/'offline_truth'/f'frame_{j:03d}.npz'); pixels=np.rint(xy).astype(int);zz=t[eye+'_depth_m'][pixels[:,1],pixels[:,0]]
  with np.errstate(invalid='ignore'):truth=wpoints(pixels,zz,t[eye+'_camera_usd_world']);errs.append(np.linalg.norm(truth-w,axis=1))
 errors=np.column_stack(errs);finite=np.isfinite(errors).all(1);ok=finite&(errors<=.1).all(1);bins=np.floor(w[:,:2]/.1).astype(int);nov=np.array([tuple(b) not in sb for b in bins]);cn=nov&ok
 rr=dict(frame=frame,eye=eye,time_s=clock[frame],n=n,correct=int(ok.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~ok).sum()),precision=float(ok.mean()),correct_novel_points=int(cn.sum()),correct_novel_bins=len(set(map(tuple,bins[cn]))),CAD_support_failures=failures,world_maxabs_m=wm,pastUV_maxabs_px=pastmax)
 rr['strict_coverage']=bool(clock[frame]>23.5 and rr['correct_novel_points']>=20 and rr['correct_novel_bins']>=2);rows.append(rr)
 ar=next(x for x in author['rows'] if x['frame']==frame and x['eye']==eye);check(f'{frame}_{eye}_author_score_agrees',all(rr[k]==ar[k] for k in ['n','correct','insufficient','finite_wrong','precision','correct_novel_points','correct_novel_bins','strict_coverage']))
 print(json.dumps(rr),flush=True)
gates=dict(every_nonempty_geometry_ge95=all(r['precision']>=.95 for r in rows),atleast_one_100_points=any(r['n']>=100 for r in rows),strict_both_eye_newregioncoverage=any(all(next(r for r in rows if r['frame']==j and r['eye']==e)['strict_coverage'] for e in ['left','right']) for j in [245,255]))
check('author_gates_agree',all(author[k]==v for k,v in gates.items()))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,issues=issues,rows=rows,gates=gates,decision='REJECT_SINGLE_HERO_SOURCE',totals={k:sum(r[k] for r in rows) for k in ['n','correct','insufficient','finite_wrong']},scope='CPU post-output validation; no model import, GPU, checkpoint deserialize, weight identity rerun, video or physics')
(E/'evidence.json').write_text(json.dumps(result,indent=2)+'\n');print('ISSUES',issues,flush=True)
