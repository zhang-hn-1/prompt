from pathlib import Path
import json,hashlib,datetime,argparse,numpy as np
from bank_response_measurement import summarize
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
a=argparse.ArgumentParser();a.add_argument('--arm',required=True,choices=['development','validation']);arm=a.parse_args().arm
D=H/('reverse_straight_'+arm+'_run_v1');p=json.loads((H/'reverse_straight_protocol_v1.json').read_text());manifest=json.loads((D/'post_close_manifest.json').read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in manifest.items());assert all(sha(f)==s for k in ['source_sha256','input_sha256'] for f,s in p[k].items());assert json.loads((D/'identity.json').read_text())['complete'];z=np.load(D/'physics.npz');c=summarize(z['simulation_time_s'],z['position_world_xy'],z['root_quaternion_world_wxyz'],z['linear_velocity_world_m_s'][:,:2],z['yaw_world_rad'],-.3,21,25.5)
checks=dict(steady=c['accepted'],contact_free=not any(s['contact'] for s in json.loads((D/'physics_contacts.json').read_text())))
if arm=='validation':
 point=json.loads((H/'reverse_straight_fitted_point_v1.json').read_text());base=point['measured'];checks.update(speed_consistency=abs(c['actual_speed_m_s']-base['actual_speed_m_s'])<=.03,curvature_consistency=abs(c['curvature_body_1_m']-base['curvature_body_1_m'])<=max(.2,.3*abs(base['curvature_body_1_m'])))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arm=arm,measured=c,checks=checks,decision='PASS_POINT_ONLY_PENDING_INTERFACE_VERIFY' if all(checks.values()) else 'REJECT_UNIQUE_REVERSE_POINT',not_dynamic_prediction=True,not_navigation=True,whole_research_complete=False,protocol_sha256=sha(H/'reverse_straight_protocol_v1.json'),raw_manifest_sha256=sha(D/'post_close_manifest.json'))
f=H/('reverse_straight_'+arm+'_assessment_v1.json');assert not f.exists();f.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
