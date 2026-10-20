from pathlib import Path
import json,hashlib,numpy as np,cv2
from high_main_turn_measurement_v1 import measured_arm
from high_main_turn_schedule_v1 import schedule
from local_reserve_execution_classification_v1 import shared_execution_prefix
H=Path(__file__).resolve().parent

def verify(mapping):
 for f,h in mapping.items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h,f

def main():
 cfg=json.loads((H/'high_main_turn_response_protocol_v1.json').read_text());verify(cfg['source_sha256']);verify(cfg['input_sha256']);arms={};raw={}
 for arm in ('low','high'):
  D=H/('high_main_turn_'+arm+'_run_v1');status=json.loads((H/('high_main_turn_'+arm+'_status_v1.json')).read_text());manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());verify(manifest['artifact_sha256'])
  identity=json.loads((D/'identity.json').read_text());assert json.loads((D/'response_contract.json').read_text())==dict(arm=arm,protocol=cfg)
  s=json.loads((D/'states.json').read_text());c=json.loads((D/'physics_contacts.json').read_text());p=dict(np.load(D/'physics.npz'));q=np.load(D/'requests.npz')['requests'];n=len(q)
  assert np.array_equal(q,schedule(arm)[:n]);m=measured_arm(status,identity,s,c,p,n)
  if m['classification'].startswith('unassessed'):raise ValueError(str(m))
  videos={}
  for name in ('stereo_view','third_person'):
   v=cv2.VideoCapture(str(D/(name+'.mp4')));fps=v.get(cv2.CAP_PROP_FPS);count=0
   while v.read()[0]:count+=1
   v.release();assert count==len(s) and fps==10.;videos[name]=dict(decoded_frames=count,fps=fps)
  assert identity['reset_count']==1 and identity['internal_reset'] is False
  initial=json.loads((D/'initial_execution_after_reset.json').read_text());assert initial['pid_config']==cfg['pid_config'] and initial['inner_config']==cfg['inner_config']
  clocks=json.loads((D/'camera_clocks.json').read_text());assert len(clocks)==len(s)
  for x in clocks:assert abs(x['eyes']['left']-x['raw_time_s'])<1e-7 and abs(x['eyes']['right']-x['raw_time_s'])<1e-7
  m.update(sealed_items=len(manifest['artifact_sha256']),videos=videos,counts=dict(requests=n,captures=len(s),physics=len(p['simulation_time_s']),contacts=len(c)))
  arms[arm]=m;raw[arm]=dict(physics=p,requests=q,initial=initial)
 lo,hi=arms['low'],arms['high'];a,b=raw['low'],raw['high'];prefix=False;execution_prefix=False
 if min(len(a['requests']),len(b['requests']))>=120:
  am=a['physics']['simulation_time_s']<=12+1e-5;bm=b['physics']['simulation_time_s']<=12+1e-5
  keys=[k for k in a['physics'] if not k.startswith('raw_simulation_time') and k!='raw_physics_step_index']
  prefix=bool(np.array_equal(a['requests'][:120],b['requests'][:120]) and a['initial']==b['initial'] and all(np.array_equal(a['physics'][k][am],b['physics'][k][bm]) for k in keys))
 execution_prefix=shared_execution_prefix(H/'high_main_turn_low_run_v1',H/'high_main_turn_high_run_v1',120,len(a['requests']),len(b['requests']))
 gates=dict(complete_both30=lo['complete'] and hi['complete'],prefix_logged_requests_physics_initial_exact=prefix,execution_snapshot_prefix_exact=execution_prefix)
 if gates['complete_both30']:
  gates.update(steady_both=lo['window']['accepted'] and hi['window']['accepted'],high_speed=hi['actual_mean_speed_m_s']>=.3,speed_gain=hi['actual_mean_speed_m_s']>=1.3*lo['actual_mean_speed_m_s'],actual_bank_retained=abs(hi['actual_mean_bank_deg']-lo['actual_mean_bank_deg'])<=3,terminal_stop_both=lo['terminal_stop']['last_count']>=5 and hi['terminal_stop']['last_count']>=5)
 result=dict(arms=arms,gates=gates,point_response_accept=all(gates.values()),classification='point_response_pass' if all(gates.values()) else 'measured_point_response_reject',no_general_inverse_map=True,no_navigation_admission=True,execution_snapshots_compared_through_s=12.,no_source_precision_admission=True,whole_mission_complete=False)
 out=H/'high_main_turn_response_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':
 try:main()
 except (ValueError,AssertionError,OSError,KeyError) as e:
  r=dict(classification='unassessed_execution_or_evidence',reason=repr(e),point_response_accept=False,scientific_rejection=False,whole_mission_complete=False)
  out=H/'high_main_turn_response_unassessed_v1.json';assert not out.exists();out.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True)
