"""One prespecified straight steady point and measured stopping; no model/navigation admission."""
from pathlib import Path
import sys,json,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'straight_pacing_probe_run_v1'
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import bank_from_quaternion
from straight_pacing_measurement_v1 import stop_qualification
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 status=json.loads((H/'straight_pacing_probe_status_v1.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
 sealed=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==v for p,v in sealed['artifact_sha256'].items());cfg=json.loads((H/'straight_pacing_probe_protocol_v1.json').read_text());assert all(sha(p)==v for p,v in cfg['source_sha256'].items());assert all(sha(p)==v for p,v in cfg['input_sha256'].items())
 ident=json.loads((D/'identity.json').read_text());s=json.loads((D/'states.json').read_text());c=json.loads((D/'physics_contacts.json').read_text());a=np.load(D/'requests.npz')['requests'];planned=np.load(D/'planned_requests.npz')['requests'];p=np.load(D/'physics.npz');times=p['simulation_time_s'];vel=p['linear_velocity_world_m_s'];speed=np.linalg.norm(vel[:,:2],axis=1);bank=np.array([bank_from_quaternion(q) for q in p['root_quaternion_world_wxyz']])*180/np.pi
 windows={}
 for name,(lo,hi) in cfg['analysis_windows_s'].items():
  m=(times>=lo-1e-5)&(times<=hi+1e-5)
  windows[name]=dict(samples=int(m.sum()),mean_XY_speed_m_s=float(speed[m].mean()),bank_abs_p95_deg=float(np.quantile(np.abs(bank[m]),.95)),actual_joint1_velocity_mean_rad_s=float(p['joint_velocity_rad_s'][m,0].mean()))
 t=np.array([x['time_s'] for x in s]);v=np.array([x['velocity'] for x in s]);xy=np.array([x['position'][:2] for x in s]);qual=stop_qualification(t,v,22.);atstop=int(np.argmin(abs(t-22.)));tail=xy[atstop:]-xy[atstop];taildist=np.linalg.norm(tail,axis=1);gates=dict(complete_one30s_sequence=ident['complete'] and len(a)==300 and len(s)==301 and len(times)==1501 and np.array_equal(a,planned),available_no_contact=bool(c) and all(x['available'] for x in c) and not any(x['contact'] for x in c),high_mean_speed_at_least_0p30=windows['high']['mean_XY_speed_m_s']>=.30,high_bank_abs_p95_at_most3deg=windows['high']['bank_abs_p95_deg']<=3.,five_actual_low_speed_samples_by30=qual['first_qualified_s'] is not None and qual['first_qualified_s']<=30.+1e-5 and qual['last_count']>=5)
 videos={}
 for name in ('stereo_view','third_person'):
  cap=cv2.VideoCapture(str(D/(name+'.mp4')));fps=cap.get(cv2.CAP_PROP_FPS);count=0
  while True:
   ok,_=cap.read()
   if not ok:break
   count+=1
  cap.release();assert count==len(s) and abs(fps-10)<1e-5;videos[name]=dict(decoded_frames=count,fps=fps)
 r=dict(gates=gates,one_straight_operating_point_screen_pass=all(gates.values()),windows=windows,stop_qualification=qual,stop_tail_radial_peak_m=float(taildist.max()),stop_tail_final_m=float(taildist[-1]),same_sequence_contrast_not_same_state_causal_AB=True,no_high_speed_turn_or_geometry_model_admission=True,no_certified_stopping_envelope=True,formal_navigation_result=None,whole_mission_complete=False,raw_sealed_items_checked=len(sealed['artifact_sha256']),videos=videos)
 (D/'assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n');(D/'artifact_manifest_post_assessment_v1.json').write_text(json.dumps(dict(artifact_sha256={str(x):sha(x) for x in D.rglob('*') if x.is_file()}),indent=2)+'\n');print(json.dumps(r,indent=2))
if __name__=='__main__':main()
