from pathlib import Path
import json,hashlib,datetime,argparse,numpy as np
from bank_response_measurement import summarize
from reverse_half_point_gates_v1 import point_checks
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--arm',required=True,choices=['development','validation']);arm=parser.parse_args().arm
    D=H/('reverse_half_'+arm+'_run_v1');p=json.loads((H/'reverse_half_protocol_v1.json').read_text())
    status=json.loads((H/('reverse_half_'+arm+'_status_v1.json')).read_text())
    assert status['status']=='finished' and status['child_returncode']==0
    manifest=json.loads((D/'post_close_manifest.json').read_text())['artifact_sha256']
    for file,s in manifest.items():assert sha(file)==s,file
    for group in ['source_sha256','input_sha256']:
        for file,s in p[group].items():assert sha(file)==s,file
    identity=json.loads((D/'identity.json').read_text());assert identity['complete'] and identity['actions']==320 and identity['captures']==321
    z=np.load(D/'physics.npz');assert len(z['simulation_time_s'])==1601
    contacts=json.loads((D/'physics_contacts.json').read_text());assert len(contacts)==1921 and all(s['available'] for s in contacts)
    c=summarize(z['simulation_time_s'],z['position_world_xy'],z['root_quaternion_world_wxyz'],z['linear_velocity_world_m_s'][:,:2],z['yaw_world_rad'],-.5,21,25.5)
    dev=None
    if arm=='validation':dev=json.loads((H/'reverse_half_fitted_point_v1.json').read_text())['measured']
    checks=point_checks(c,contacts=sum(s['contact'] for s in contacts),development=dev)
    result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arm=arm,measured=c,checks=checks,decision='PASS_POINT_ONLY_PENDING_EXECUTION_VERIFY' if all(checks.values()) else 'REJECT_UNIQUE_REVERSE_HALF_POINT',not_dynamic_prediction=True,not_navigation=True,whole_research_complete=False,protocol_sha256=sha(H/'reverse_half_protocol_v1.json'),raw_manifest_sha256=sha(D/'post_close_manifest.json'))
    out=H/('reverse_half_'+arm+'_assessment_v1.json');assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
