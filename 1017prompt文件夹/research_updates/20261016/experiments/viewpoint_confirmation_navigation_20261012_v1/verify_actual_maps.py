from pathlib import Path
import json,numpy as np
from viewpoint_confirmation import ViewpointConfirmationGrid

def verify_actual_maps(out):
 R=Path(out);native=json.loads((R/'native_predictions.json').read_text());poses={round(x['time_s'],7):x['position_world'] for x in native if x.get('available')};views={}
 for p in (R/'known_poses').glob('*.npz'):
  a=np.load(p);views[int(p.stem.split('_')[1])]=dict(time_s=float(a['time_s']),position_world=a['left_camera_usd_world'][:3,3])
 maps=sorted((R/'maps').glob('*.npz'));assert maps;first=np.load(maps[0]);g=ViewpointConfirmationGrid(first['origin'],10,static_scene_assumption=True);n=0
 for p in maps:
  a=np.load(p);t=float(a['time_s']);rs=json.loads((R/'raw_point_memory_snapshots'/(p.stem+'.json')).read_text())['records'];g.consume_records(rs,t,np.asarray(poses[round(t,7)]),views)
  np.testing.assert_array_equal(g.labels,a['labels']);np.testing.assert_array_equal(g.codes,a['codes']);n+=1
 return dict(actual_maps_exact=n,known_native_camera_views=len(views),recent10s_unchanged=True,viewpoint_separation_m=g.resolution,truth_read=False)
