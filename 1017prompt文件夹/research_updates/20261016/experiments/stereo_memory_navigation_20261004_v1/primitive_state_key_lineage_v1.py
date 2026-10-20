"""Saved samples only; no search/geometry integration or runtime action."""
from pathlib import Path
import json,hashlib
import numpy as np
from supported_four_point_planning import world_to_cell,yaw_to_bin

def summarize_saved_keys(root,checks,origin,resolution):
 root=np.asarray(root,float)
 if root.shape!=(3,) or not np.isfinite(root).all():raise ValueError('finite saved root')
 rootkey=(*world_to_cell(root[:2],origin_xy=origin,resolution_m=resolution),yaw_to_bin(root[2]));keys=[];rows=[]
 for c in checks:
  samples=np.asarray(c['samples'],float)
  if samples.ndim!=2 or samples.shape[1]!=3 or len(samples)<2 or not np.isfinite(samples).all():raise ValueError('finite saved primitive samples')
  q=samples[-1];k=(*world_to_cell(q[:2],origin_xy=origin,resolution_m=resolution),yaw_to_bin(q[2]),'forward');valid=bool(c['primitive_free'] and c.get('prefix_first_level_free',True))
  if valid:keys.append(k)
  rows.append(dict(aux_prefix=c.get('prefix_aux_rad'),endpoint_key=list(k),whole_prefix_free=valid))
 distinct=set(keys);roots={(*rootkey,'forward'),(*rootkey,'reverse')}
 return dict(rows=rows,root_latch_keys=2,distinct_free_prefix_keys=len(distinct),finite_state_count_including_roots=len(distinct|roots),duplicate_free_endpoint_keys=len(keys)-len(distinct),root_keys=[list(k) for k in sorted(roots)])

if __name__=='__main__':
 H=Path(__file__).resolve().parent;g=json.loads((H/'current_forward_root_geometry_assessment_v1.json').read_text());a=json.loads((H/'current_boundary_forward_only_assessment_v1.json').read_text());meta=a['forward_only']['meta'];D=H/'reverse_main_navigation_e1_b0_run_v1';m=np.load(D/'maps/frame_120.npz');r=summarize_saved_keys(g['root'],g['checks'],meta['crop_origin'],float(m['resolution']))
 from supported_forward_primitive_source import supported_curvature as old
 from execution_domain_four_point_source_v2 import supported_requests,supported_curvature as four
 forward=[c for c in supported_requests() if c[0]>0];comparison=[dict(request=c,forward_only_curvature=old(*c),four_point_forward_curvature=four(*c),equal=old(*c)==four(*c)) for c in forward];assert all(c['equal'] for c in comparison)
 globalkeys=summarize_saved_keys(g['root'],g['checks'],m['origin'],float(m['resolution']));offset=np.rint((np.asarray(meta['crop_origin'])-m['origin'])/float(m['resolution'])).astype(int);crop_global_equal=all(np.array_equal(np.array(x['endpoint_key'][:2])+offset,np.array(y['endpoint_key'][:2])) and x['endpoint_key'][2:]==y['endpoint_key'][2:] for x,y in zip(r['rows'],globalkeys['rows']));assert crop_global_equal
 from supported_forward_point_planning import world_to_cell as oldcell,yaw_to_bin as oldyaw
 assert all(oldcell(c['samples'][-1][:2],origin_xy=meta['crop_origin'],resolution_m=float(m['resolution']))==tuple(x['endpoint_key'][:2]) and oldyaw(c['samples'][-1][2])==x['endpoint_key'][2] for c,x in zip(g['checks'],r['rows']))
 p=H/'primitive_state_key_lineage_assessment_v1.json';assert not p.exists();inputs=[H/'current_forward_root_geometry_assessment_v1.json',H/'current_forward_root_geometry_precision_addendum_v1.json',H/'current_boundary_forward_only_assessment_v1.json',D/'maps/frame_120.npz'];sources=[H/x for x in ['primitive_state_key_lineage_scope_v1.json','primitive_state_key_lineage_v1.py','test_primitive_state_key_lineage_v1.py','supported_four_point_planning.py','supported_forward_point_planning.py','supported_forward_primitive_source.py','execution_domain_four_point_source_v2.py']]
 out=dict(**r,command_curvature_comparison=comparison,crop_global_key_consistency=crop_global_equal,saved_solver_expanded_nodes=meta['expanded_nodes'],count_compatible_with_saved_prefixes=r['finite_state_count_including_roots']==meta['expanded_nodes'],original_solver_result_preserved=meta['status'],root_precision_scope='rounded root saved prefix, not exact solver expansion trace; old precision addendum unchanged',no_new_solver=True,no_new_geometry_samples=True,no_physical=True,no_bug_proven=True,interpretation='Two initialized latches plus saved free unique forward prefix states are count-compatible. Free finite prefixes do not imply a complete route; absent expansion log prevents exact node attribution.',source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources},input_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in inputs},whole_mission_complete=False)
 p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ['rows','source_sha256','input_sha256']}))
