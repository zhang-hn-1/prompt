"""One factual already-sealed cause audit; never changes actor or gates."""
from pathlib import Path
import json,numpy as np
from scipy.ndimage import distance_transform_edt
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent;D=H/'front_view_handoff_navigation_run_v1'
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());ref=np.asarray(rows[371]['installed_reference_xy_after_act']);out=[]
for mf in [371,420,425]:
 m=np.load(D/'maps'/f'frame_{mf:03d}.npz');field=distance_transform_edt(np.pad(m['labels']!=1,1))[1:-1,1:-1]*float(m['resolution'])
 for pf in [420,425,426]:
  g=reference_geometry_status(states[pf]['position'][:2],ref,m['labels'],m['origin'],float(m['resolution']));parts={}
  for name in ['bridge_xy','suffix_xy']:
   p=np.array(g[name]);chunks=[p[:1]]
   for a,b in zip(p[:-1],p[1:]):chunks.append(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)))
   p=np.vstack(chunks);cells=np.rint((p-m['origin'])/float(m['resolution'])).astype(int);inside=(cells>=0).all(1)&(cells<np.array(field.shape)).all(1);assert inside.all();values=field[tuple(cells.T)];i=int(values.argmin());parts[name]=dict(minimum_raster_clearance_m=float(values[i]),argmin_world_xy=p[i].tolist(),argmin_cell=cells[i].tolist(),pass_existing_0p870711=bool(values.min()>=.8707106781186548-1e-8),pass_old_0p6=bool(values.min()>=.6-1e-8))
  out.append(dict(map_frame=mf,pose_frame=pf,projection_distance_m=g['projection_distance_m'],old_retention_necessary_pass=g['retention_necessary_screen_pass'],old_reject_reasons=g['reject_reasons'],remaining_arc_m=g['remaining_arc_m'],parts=parts))
result=dict(frozen_installed_reference=371,measurements=out,not_actor_change=True,not_gate_change=True,not_navigation_success=True,no_future_source_actor=True,no_new_physics=True,old_ref_pose_map_development_only=True)
(H/'front_view_handoff_bridge_suffix_audit_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(out[-3:],indent=2))
