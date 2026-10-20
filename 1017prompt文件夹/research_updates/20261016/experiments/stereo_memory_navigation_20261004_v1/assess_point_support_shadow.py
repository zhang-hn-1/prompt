"""Offline labels only after the entire historical shadow replay was saved."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
H=Path(__file__).resolve().parent;O=H/'point_support_runs';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'point_support_protocol.json').read_text());m=json.loads((O/'manifest.json').read_text());assert len(m['rows'])==181 and m['labels_not_loaded']
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 for r in m['rows']:
  for file,key in [(f"packet_{r['frame']:03d}.json",'packet_sha256'),(f"maps_{r['frame']:03d}.npz",'maps_sha256'),(f"frame_{r['frame']:03d}.json",'record_sha256')]:assert sha(O/file)==r[key]
 truth=np.load(H/'active_observation_run/offline_occupancy_label.npz');np.testing.assert_equal(truth['origin'],p['grid']['origin']);assert truth['labels'].shape==tuple(p['grid']['shape']);assert float(truth['resolution'])==p['grid']['resolution']
 old=json.loads((H/'active_feature_runs/assessment.json').read_text());querylabels={(r['frame'],r['index']):r for r in old['points'] if r['arm']=='causal'};rows=[];spatial={};sweep=np.load(O/'reference_and_sweep.npz');whole=set(map(tuple,sweep['whole_cells']));raw_count=inside_count=nominal_floor_inside=0
 for i in range(181):
  f=json.loads((O/f'frame_{i:03d}.json').read_text());maps=np.load(O/f'maps_{i:03d}.npz');packet=json.loads((O/f'packet_{i:03d}.json').read_text())
  for pt in packet['points']:
   raw_count+=1;cell=tuple(np.rint((np.array(pt['world'][:2])-p['grid']['origin'])/.1).astype(int));inside_count+=cell in whole;nominal_floor_inside+=cell in whole and abs(pt['world'][2])<=.05
  for arm in ['current','history']:
   a=f['arms'][arm];labels=maps[arm+'_labels'];prov=maps[arm+'_provenance'];floor=labels==0;hazard=labels>0;gt=truth['labels'];known=gt>=0
   valid=correct=wrong=unlabelled=0;bad_cells=[]
   for r in a['records']:
    q=r['query'];lab=querylabels[(q[0],q[2])];assert r['world']==lab['world'] and r['source']==lab['source'] and r['sigma_m']==lab['sigma_m']
    if lab['label_world'] is None:unlabelled+=1
    else:
     valid+=1;correct+=lab['correct'];wrong+=not lab['correct']
     if not lab['correct']:bad_cells.append(np.rint((np.array(r['world'][:2])-p['grid']['origin'])/.1).astype(int).tolist())
   falsefree=int((floor&known&(gt>0)).sum());unlabelledfloor=int((floor&~known).sum());confirmedhazard=int((hazard&(gt>0)).sum());falsehazard=int((hazard&known&(gt==0)).sum())
   row=dict(frame=i,time_s=f['time_s'],arm=arm,records=len(a['records']),labelled_records=valid,correct_records=int(correct),wrong_records=int(wrong),unlabelled_records=unlabelled,current_floor_cells=a['current_floor_cells'],historical_floor_cells=a['historical_floor_cells'],floor_cells=int(floor.sum()),hazard_cells=int(hazard.sum()),historical_hazard_cells=a['historical_hazard_cells'],unknown_cells=a['unknown_cells'],ignored_records=a['ignored_records'],conflict_cells=a['conflict_cells'],falsefree_occupied_overlap_cells=falsefree,unlabelled_floor_cells=unlabelledfloor,confirmed_occupied_proxy_cells=confirmedhazard,falsehazard_free_overlap_proxy_cells=falsehazard,wrong_endpoint_cell_indices=bad_cells,support={k:{j:v for j,v in s.items() if j not in ['unknown_indices','occupied_indices']} for k,s in a['support'].items()})
   rows.append(row)
 for arm in ['current','history']:
  rr=[r for r in rows if r['arm']==arm];spatial[arm]=dict(at6s=rr[60],at_end=rr[-1],peak_floor_cells=max(r['floor_cells'] for r in rr),peak_hazard_cells=max(r['hazard_cells'] for r in rr),peak_historical_floor_cells=max(r['historical_floor_cells'] for r in rr),peak_falsefree_cells=max(r['falsefree_occupied_overlap_cells'] for r in rr),peak_unlabelled_floor_cells=max(r['unlabelled_floor_cells'] for r in rr),peak_falsehazard_cells=max(r['falsehazard_free_overlap_proxy_cells'] for r in rr),all_phase_support_passes={phase:sum(r['support'][phase]['support_passed'] for r in rr) for phase in ['start','turn','stop','whole']},whole_min_unknown=min(r['support']['whole']['unknown_cells'] for r in rr),whole_max_occupied=max(r['support']['whole']['occupied_cells'] for r in rr))
 result=dict(assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),shadow_saved_utc=m['saved_utc'],rows=rows,arms=spatial,raw_endpoint_spatial_diagnostic=dict(all_causal_retained_points=raw_count,nominal_xy_inside_whole_sweep=inside_count,nominal_floor_inside_whole_sweep=nominal_floor_inside,scope='point centres only, BEFORE map-height/uncertainty filtering; not ray coverage or all-surfaces witness'),phase_sweep_cells={phase:len(sweep[phase+'_cells']) for phase in ['start','turn','stop','whole']},same_points_both_arms=True,source_gate_passed=False,consumer_support_gate_passed=any(r['support']['whole']['support_passed'] for r in rows),navigation_run=False,source_inputs_unchanged=True,scope='historical-development2.5D endpoint proxy; occupancy bodyband label only afteroutputs, no full3D/densefree/pixelarea/uniqueobjects certification')
 (O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
if __name__=='__main__':main()
