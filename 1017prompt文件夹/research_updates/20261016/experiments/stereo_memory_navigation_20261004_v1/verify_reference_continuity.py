"""Independent scalar geometry / brute-centre raster check; no extra searches."""
from pathlib import Path
import json,hashlib,datetime,math
import numpy as np
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;O=H/'depth_local_curve_run'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def distance_point_segment(p,a,b):
    d=b-a;n=float(np.dot(d,d));u=0. if n==0 else min(1.,max(0.,float(np.dot(p-a,d))/n));return float(np.linalg.norm(p-a-u*d)),u

def main():
 p=json.loads((H/'reference_continuity_protocol.json').read_text());a=json.loads((H/'reference_continuity_assessment.json').read_text());assert p['sealed_utc']<a['started_utc']<a['saved_utc']
 for d in [p['source_sha256'],p['input_sha256']]:
  for f,s in d.items():assert sha(f)==s,f
 records=json.loads((O/'policy_records.json').read_text());xy=np.load(O/'episode_arrays.npz')['positions_world_xy'];samples=0
 for row in a['rows']:
  if row['status']!='measured':continue
  i=row['frame'];path=np.load(O/'local_plans'/f'frame_{i-1:03d}.npz')['pose'][:,:2]
  for key,frame in [('previous_remaining',i),('at_installation',row['previous_install_frame'])]:
   result=row[key];g=np.load(O/'maps'/f'frame_{frame:03d}.npz');labels=g['labels'];origin=g['origin'];res=float(g['resolution']);point=xy[frame]
   pairs=[distance_point_segment(point,u,v) for u,v in zip(path[:-1],path[1:])];j=min(range(len(pairs)),key=lambda q:pairs[q][0]);arc=sum(float(np.linalg.norm(v-u)) for u,v in zip(path[:j],path[1:j+1]))+pairs[j][1]*float(np.linalg.norm(path[j+1]-path[j]))
   np.testing.assert_allclose([result['projection_distance_m'],result['projected_arc_m']],[pairs[j][0],arc],rtol=0,atol=1e-10)
   hazards=origin+np.argwhere(labels==1)*res
   for linekey,clearkey in [('suffix_xy','suffix_clearance_m'),('bridge_xy','bridge_clearance_m')]:
    line=np.asarray(result[linekey]);dist=min((distance_point_segment(h,u,v)[0] for h in hazards for u,v in zip(line[:-1],line[1:])),default=None)
    if len(line)==1 and len(hazards):dist=min(float(np.linalg.norm(h-line[0])) for h in hazards)
    if dist is None:assert result[clearkey] is None
    else:assert abs(dist-result[clearkey])<=1e-10
   points=[]
   for linekey in ['suffix_xy','bridge_xy']:
    line=np.asarray(result[linekey]);points.append(line[0])
    for u,vv in zip(line[:-1],line[1:]):
     steps=max(1,math.ceil(float(np.linalg.norm(vv-u))/.05));points.extend(u+(vv-u)*(k/steps) for k in range(1,steps+1))
   field=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*res;cells=np.rint((np.asarray(points)-origin)/res).astype(int);inside=((cells>=0)&(cells<np.array(labels.shape))).all(1);clear=[]
   for cell,inrange in zip(cells,inside):
    if not inrange:clear.append(-np.inf);continue
    # Independent nearest observed-centre + padded outside lattice distance.
    world=origin+cell*res;minimum=min(float(np.linalg.norm(world-h)) for h in hazards) if len(hazards) else np.inf
    boundary=min(cell[0]+1,cell[1]+1,labels.shape[0]-cell[0],labels.shape[1]-cell[1])*res;value=min(minimum,boundary);assert abs(value-field[tuple(cell)])<1e-10;clear.append(value)
   assert len(points)==result['raster_sample_count'];assert bool(np.all(np.asarray(clear)>=.6-1e-8))==result['raster_clearance_pass'];samples+=len(points)
   assert (g['hazard_time'][labels==1]<=float(g['time_s'])+1e-6).all()
 # Four diagonal actual map/pose routes exactly reproduce stored original outputs, no additional searches.
 for c in a['counterfactuals']:
  if c['map_frame']==c['position_frame']:
   rr=records[c['map_frame']]['local']['plan'];np.testing.assert_equal(c['route_xy'],rr['route_xy']);np.testing.assert_equal(c['target_xy'],rr['local_target'])
 # Descriptive witnesses on the old whole grid route under the new map; no retuning/intervention.
 witnesses=[]
 for event in [75,160]:
  old=event-5;route=np.asarray(next(c['route_xy'] for c in a['counterfactuals'] if c['event_frame']==event and c['map_frame']==old and c['position_frame']==old));g=np.load(O/'maps'/f'frame_{event:03d}.npz');field=distance_transform_edt(np.pad(g['labels']!=1,1))[1:-1,1:-1]*float(g['resolution']);cells=np.rint((route-g['origin'])/float(g['resolution'])).astype(int);values=field[cells[:,0],cells[:,1]];bad=np.flatnonzero(values<.6-1e-8)
  witnesses.append(dict(event_frame=event,old_route_raster_invalid_vertex_count=len(bad),old_route_minimum_current_raster_clearance_m=float(values.min()),invalid_old_route_world_xy=route[bad].tolist(),scope='whole old grid-route vertex rule only, beyond installed localprefix; no physical prediction'))
 assert a['grid_searches']==8 and len(a['rows'])==58 and not a['retention_proposal_necessary_screen_pass']
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),old_input_checks=len(p['input_sha256']),old_reference_geometry_checks=114,raster_samples_independently_checked=samples,actual_original_route_diagonals_exact=4,additional_grid_searches=0,whole_route_witnesses=witnesses,retention_proposal_rejected=True,whole_research_complete=False,assessment_sha256=sha(H/'reference_continuity_assessment.json'))
 (H/'reference_continuity_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
