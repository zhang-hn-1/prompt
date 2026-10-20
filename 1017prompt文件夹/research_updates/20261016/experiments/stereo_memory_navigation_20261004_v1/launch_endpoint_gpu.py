from pathlib import Path
import json,hashlib,datetime,subprocess,sys
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
p=json.loads((H/'endpoint_navigation_protocol.json').read_text());s=json.loads((H/'endpoint_navigation_status.json').read_text())
assert s['exit_code']==-11 and s['status']=='finished'
out=H/'endpoint_navigation_run';assert not any(out.rglob('*.npz')) and not any(out.rglob('*.json')) and not (out/'metrics.json').exists()
for f,h in p['source_sha256'].items():assert sha(f)==h,f
assert not (H/'endpoint_navigation_gpu_launch.json').exists()
for old,new in [('endpoint_navigation_run','endpoint_navigation_preinit_failure'),('endpoint_navigation_status.json','endpoint_navigation_preinit_status.json'),('endpoint_navigation_run.log','endpoint_navigation_preinit_failure.log')]:
 assert not (H/new).exists();(H/old).rename(H/new)
addendum=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),original_protocol_sha256=sha(H/'endpoint_navigation_protocol.json'),original_gpu_unavailable_attempt=s,failed_attempt_physics_and_sensor_artifacts=0,preserved_failure_log_sha256=sha(H/'endpoint_navigation_preinit_failure.log'),reason='restricted sandbox reported no GPU/Vulkan device, unavailable CUDA, DerivedDataCache lock then SIGSEGV before acquisition',environment_only_retry=True,maximum_completed_physical_sequences=1,maximum_gpu_environment_retries=1,source_or_numeric_gate_change=False,launcher_sha256=sha(__file__))
(H/'endpoint_navigation_gpu_launch.json').write_text(json.dumps(addendum,indent=2)+'\n')
print('PRESERVED startup failure; preregistered single GPU-environment retry',flush=True)
r=subprocess.run([sys.executable,str(H/'run_endpoint_navigation_stage.py')]);raise SystemExit(r.returncode)
