"""One scalar conservative perception uncertainty, fit labels never actor inputs."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;O=H/'partial_cue_ray_set_v1';D=H/'controlled_heading_probe_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();p=np.load(O/'possible_set_v1.npz');xy=np.unique(p['world_voxel_centres'][:,:2],axis=0);gt=np.load(D/'offline_occupancy_label.npz');truth=gt['origin']+np.argwhere(gt['labels']>0)*float(gt['resolution']);dist=cKDTree(xy).query(truth)[0];half=np.sqrt(2)*float(p['pitch_m'])/2;raw=max(0.,float(dist.max())-half);radius=float(np.ceil((raw-1e-12)/.1)*.1);ok=radius<=.2+1e-12
fit=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),method='max_fit_footprint_distance_minus_halfXYvoxel_rounded_up_010_v1',fit_sequence='controlled_heading_probe_run_v1_whole8s',fit_not_fresh=True,maximum_distance_m=float(dist.max()),halfXYvoxel_m=half,raw_extra_radius_m=raw,radius_m=radius,maximum_radius_m=.2,fit_pass=ok,old_raw_set_REJECT_preserved=True,not_exact_depth_or_measured_geometry=True,input_sha256={str(O/'possible_set_v1.npz'):sha(O/'possible_set_v1.npz'),str(D/'offline_occupancy_label.npz'):sha(D/'offline_occupancy_label.npz')},script_sha256=sha(__file__),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-07-calibrated-partial-set-fresh-validation.md'))
f=H/'partial_set_uncertainty_fit_v1.json';assert not f.exists();f.write_text(json.dumps(fit,indent=2)+'\n')
if ok:
 actor=dict(schema='partial_set_uncertainty_v1',radius_m=radius,maximum_radius_m=.2,method=fit['method'],fit_sha256=sha(f),is_model_hypothesis_not_observation=True)
 a=H/'partial_set_uncertainty_actor_v1.json';assert not a.exists();a.write_text(json.dumps(actor,indent=2)+'\n')
print(json.dumps(fit,indent=2))
