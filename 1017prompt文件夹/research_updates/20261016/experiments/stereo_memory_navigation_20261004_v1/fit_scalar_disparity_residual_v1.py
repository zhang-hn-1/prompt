from pathlib import Path
import json,hashlib,datetime,numpy as np
from stereo_geometry import restore_native
from scalar_disparity_residual import F,B
H=Path(__file__).resolve().parent;D=H/'native_reverse_bank_observation_run_v1';O=H/'scalar_disparity_residual_fit_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=D/'artifact_manifest_post_close.json'
for f,s in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==s,f
rows=json.loads((D/'source_rows.json').read_text());inputs={};res=[];counts=[]
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';fs=[D/'partial_observations'/name,D/'source_predictions'/name,D/'frames'/name];inputs.update({str(f):sha(f) for f in fs});s,p,gt=[np.load(f) for f in fs];y,x=np.divmod(s['query_pixel_id'],640);z=gt['left_depth_m'][y,x];d=restore_native(p['disparity_px'])[y,x];valid=np.isfinite(z)&(z>0)&np.isfinite(d)&(d>0);r=F*B/z[valid]-d[valid];res.append(r);counts.append(dict(frame=i,raw=len(d),finite_fit=int(valid.sum())))
r=np.concatenate(res);assert len(r)>0;delta=float(np.median(r));np.savez_compressed(O/'fit_residuals.npz',residual_px=r)
m=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),candidate='SINGLE_CONSTANT_DISPARITY_MODEL_ESTIMATE_V1',delta_disparity_px=delta,fit_rule='median(F*B/true_camera_Z - raw_disparity), all finite old reverse partial records',fit_sequence=str(D),fit_n=len(r),raw_n=sum(v['raw'] for v in counts),counts=counts,input_sha256=inputs,source_sha256={str(H/n):sha(H/n) for n in ['scalar_disparity_residual.py','fit_scalar_disparity_residual_v1.py']},no_validation_label_or_validation_input_reads=True,not_fresh=True,not_new_observed_correspondence=True,not_runtime_admitted=True)
(O/'frozen_model.json').write_text(json.dumps(m,indent=2)+'\n');print(json.dumps({k:v for k,v in m.items() if k not in ['counts','input_sha256','source_sha256']},indent=2))
