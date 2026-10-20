"""Readonly pending hypotheses, not certified sensor observations or free cells."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;S=H/'temporal_ray_sweep_diagnostic_v1';D=H/'temporal_geometry_persistence_v1';assert not D.exists();D.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seal=json.loads((S/'prediction_seal.json').read_text());assert all(sha(p)==s for p,s in seal['output_sha256'].items());plan=H.parent.parent/'docs/superpowers/plans/2026-10-07-temporal-geometry-persistence-diagnostic.md';rows=[];inputs={};hist={'left':[],'right':[]}
for r in json.loads((S/'records.json').read_text()):
 p=S/r['file'];inputs[str(p.resolve())]=sha(p);packet=np.load(p);t=float(packet['current_time_s']);assert np.isfinite(t);eye=r['eye'];previous=hist[eye];assert all(float(old['current_time_s'])<t and t-float(old['current_time_s'])<=2.+1e-9 for old in previous)
 w=packet['world'];dist=[]
 for old in previous:
  d=cKDTree(old['world']).query(w)[0] if len(old['world']) else np.full(len(w),np.inf);dist.append(d)
 accept=np.all(np.array(dist)<=.1,axis=0) if previous else np.zeros(len(w),bool);name=r['file'];np.savez_compressed(D/name,**{k:packet[k] for k in packet.files},persistence_corroborated=accept,prior_point_distances_m=np.array(dist),prior_proposal_frames=np.array([int(old['current_frame']) for old in previous]),proposal_epochs=len(previous)+1,underlying_images_shared=True,not_certified_geometry_or_free=True);rows.append(dict(**r,temporal_persistence_points=int(accept.sum()),number_previous_proposal_epochs=len(previous),underlying_unique_image_epochs=len(set([int(packet['current_frame']),*map(int,packet['source_frames']),*[int(o['current_frame']) for o in previous],*[int(j) for o in previous for j in o['source_frames']]]))))
 hist[eye].append({k:packet[k].copy() for k in packet.files})
(D/'records.json').write_text(json.dumps(rows,indent=2)+'\n');out=[p for p in D.iterdir() if p.is_file()];(D/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256={str(p.resolve()):sha(p) for p in out},plan_sha256=sha(plan),code_sha256=sha(__file__),this_predictor_labels_not_read=True,development_design_already_inspected_labels=True,no_depth_model_or_physics_rerun=True,no_geometry_source_time_refresh=True,not_sensor_observation_admission=True),indent=2)+'\n');print([(r['frame'],r['eye'],r['temporal_persistence_points']) for r in rows])
