from pathlib import Path
import json,numpy as np
from measurement import geometry
h=Path(__file__).resolve().parent;r=json.loads((h/'assessment.json').read_text());src=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_round4_drive_calibration_20260920_v1/traces');out=[]
for row in r['rows']:
 s=json.loads((src/(row['case_id']+'.json')).read_text())['samples'];xy=np.array([x['position_xy_m'] for x in s]);t=np.array([x['time_s'] for x in s]);a=xy[:-40];b=xy[20:-20];c=xy[40:];ab=b-a;bc=c-b;ac=c-a;den=np.linalg.norm(ab,axis=1)*np.linalg.norm(bc,axis=1)*np.linalg.norm(ac,axis=1);k=2*(ab[:,0]*ac[:,1]-ab[:,1]*ac[:,0])/np.maximum(den,1e-12);speed,_=geometry(xy);mask=(t[20:-20]>=row['window_s'][0])&(t[20:-20]<=row['window_s'][1])&(speed[20:-20]>=.08);out.append({'case':row['case_id'],'three_point_median':float(np.median(k[mask])),'SG_median':row['curvature_m_inv']})
(h/'curvature_crosscheck.json').write_text(json.dumps(out,indent=2));print([v for v in out if v['case'] in ['core_m0p50_ap30','core_m1p50_ap30','core_m1p50_an30']])
