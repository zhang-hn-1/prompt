from pathlib import Path
import json,hashlib,datetime
from imu_event_witness import assess
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();p=json.loads((H/'imu_event_witness_protocol.json').read_text());s=json.loads((H/'imu_event_witness_status.json').read_text());assert s['status']=='finished' and s['exit_code']==0;out=H/'imu_event_witness_run';m=json.loads((out/'artifact_manifest_post_close.json').read_text());checks=0
for obj,key in [(p,'source_sha256'),(m,'artifact_sha256')]:
 for f,v in obj[key].items():assert sha(f)==v,f;checks+=1
rows=json.loads((out/'rows.json').read_text());result=assess(rows);assert all(not r['native_orientation_read'] for r in rows)
result.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),actual_status=s,source_count=len(p['source_sha256']),integrity_checks=checks,actual_physics_steps=len(rows),robot_episodes=0,predictor_calls=0,old_robot_interface_rejection_preserved=True,old_model_scores_null=True,whole_research_complete=False)
(H/'imu_event_witness_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
