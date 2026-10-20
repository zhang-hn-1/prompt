"""All closed whole-sequence development outputs before their new-rule labels."""
from pathlib import Path
import json,hashlib,numpy as np
from paired_texture_surface import PairedTextureSurfaceMemory
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;O=H/'paired_texture_surface_development_v1';O.mkdir(exist_ok=False);inputs={};sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for case in ['live_partial_point_navigation_run_v1','live_partial_current_only_navigation_run_v1','live_generic_height_navigation_run_v1']:
 D=H/case;out=O/case;out.mkdir();m=PairedTextureSurfaceMemory();rows=[]
 for row in json.loads((D/'source_rows.json').read_text()):
  i=row['frame'];t=row['time_s'];f=f'frame_{i:03d}.npz';files=[D/'sensor_packets'/f'source_{i:03d}.npz',D/'source_inputs'/f,D/'source_predictions'/f]
  for p in files:inputs[str(p)]=sha(p)
  a=np.load(files[0]);rgb=np.load(files[1]);pred=np.load(files[2]);assert float(rgb['time_s'])==t;v=m.update(a,rgb['left_rgb'],rgb['right_rgb'],t,{str(p):sha(p) for p in files},restore_native(pred['disparity_px']));np.savez_compressed(out/f,**{k:v for k,v in v.items() if k!='input_hashes'});rows.append(dict(frame=i,time_s=t,points=len(v['world'])))
 (out/'rows.json').write_text(json.dumps(rows,indent=2)+'\n')
(O/'prediction_seal_before_new_rule_labels.json').write_text(json.dumps(dict(inputs=inputs,outputs={str(p):sha(p) for p in O.rglob('*') if p.is_file()},source_sha256={str(p):sha(p) for p in [Path(__file__).resolve(),H/'paired_texture_surface.py',H/'generic_height_surface.py',H/'audit_generic_height_binocular_support_v1.py']},closed_sequences_are_DEVELOPMENT_not_fresh=True,no_label_read_for_predictions=True,no_NCC_threshold_or_refinement=True),indent=2)+'\n');print('ALL THREE WHOLE DEVELOPMENTS SEALED')
