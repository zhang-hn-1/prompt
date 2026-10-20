from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'live_partial_current_only_navigation_run_v1';O=H/'generic_depth_height_source_whole_validation_v1';O.mkdir(exist_ok=False);(O/'points').mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();inputs={};rows=[]
for r in json.loads((D/'source_rows.json').read_text()):
 i=r['frame'];f=D/'sensor_packets'/f'source_{i:03d}.npz';inputs[str(f)]=sha(f);a=np.load(f);t=float(a['time_s']);assert t==r['time_s'] and np.isfinite(t);w=a['world'];keep=np.isfinite(w).all(-1)&(w[:,2]>.05)&(w[:,2]<=1.5)
 np.savez_compressed(O/'points'/f'frame_{i:03d}.npz',world=w[keep],query_pixel_id=a['query_pixel_id'][keep],partner_pixel_id=a['partner_pixel_id'][keep],time_s=t,frame=i,flat_floor_height_is_prior=True,no_colour_selector=True,not_free_or_volume=True)
 rows.append(dict(frame=i,time_s=t,raw=len(w),selected=int(keep.sum())))
(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'prediction_seal_pretruth.json').write_text(json.dumps(dict(inputs=inputs,source_sha256=sha(__file__),outputs={str(f):sha(f) for f in O.rglob('*') if f.is_file()},whole_sequence_validation_frozen_before_new_generic_labels=True,same_known_fixture_NOT_scene_generalization=True,no_new_GT_label_reads=True),indent=2)+'\n');print('sealed',len(rows),sum(r['selected'] for r in rows))
