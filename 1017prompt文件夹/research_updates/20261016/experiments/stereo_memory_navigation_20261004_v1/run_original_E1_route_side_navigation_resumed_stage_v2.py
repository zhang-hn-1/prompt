from pathlib import Path
import signal, os, sys, json, hashlib, datetime, subprocess, time
import numpy as np
from probe_cleanup import seal_existing
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
    protocol=json.loads((H/'original_E1_route_side_navigation_resumed_protocol_v2.json').read_text())
    for g in ['source_sha256','input_sha256']:
        for f,s in protocol[g].items():assert sha(f)==s,f
    geometry=json.loads((H/'simple_E1_partial_source_probe_assessment_v1/assessment.json').read_text());assert all(geometry['checks'].values())
    member=json.loads((H/'simple_E1_source_membership_and_error_layer_v1/assessment.json').read_text());assert member['semantic_subset95']
    cpu=json.loads((H/'E1_route_side_continuation_geometry_v1/assessment.json').read_text());assert cpu['decision']=='PASS_ONE_NOMINAL_SIDE_CONTINUATION_SCREEN_ONLY'
    interface=json.loads((H/'E1_route_side_shadow_CPU_v1/assessment.json').read_text());assert interface['decision']=='PASS_SAME_INPUT_ROUTE_CONSUMER_SHADOW_INTERFACE_CPU_ONLY'
    review=json.loads((H/'original_E1_route_side_navigation_resumed_author_review_closed_v2.json').read_text())
    assert review['unresolved_critical']==review['unresolved_important']==0
    assert review['driver_sha256']==sha(H/'run_original_E1_route_side_navigation_resumed_v2.py')
    # User-authorized resource-change resumption, never repeat failed v1 stage.
    old=H/'original_E1_route_side_navigation_run_v1'
    previous=json.loads((H/'original_E1_route_side_navigation_stage_status_v1.json').read_text())
    assert previous['stage']=='FAILED_NO_RETRY' and previous['post_close_manifest_sha256']==sha(old/'artifact_manifest_post_close.json')
    for f,digest in json.loads((old/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==digest,f
    assert np.load(old/'requests.npz')['requests'].shape==(0,10)
    gpu=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid,process_name,used_memory','--format=csv,noheader'],text=True)
    free=subprocess.check_output(['nvidia-smi','--query-gpu=memory.free','--format=csv,noheader,nounits'],text=True)
    resource=dict(utc=now(),compute_apps=gpu,free_MiB=int(free.strip()),no_external_compute_apps=not gpu.strip())
    assert resource['no_external_compute_apps'] and resource['free_MiB']>=10000,'external GPU resource block persists; no child launch'
    (H/'original_E1_route_side_resumption_resource_preflight_v2.json').write_text(json.dumps(resource,indent=2)+'\n')
    status=H/'original_E1_route_side_navigation_resumed_stage_status_v2.json';assert not status.exists()
    row=dict(start_utc=now(),pid=os.getpid(),stage='RUNNING',maximum_actual_sequences=1,not_original35=True,whole_research_complete=False)
    def save():status.write_text(json.dumps(row,indent=2)+'\n')
    save()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_partial_surface',TORCH_COMPILE_DISABLE='1')
    with (H/'original_E1_route_side_navigation_resumed_actual_v2.log').open('x') as log:
        child=subprocess.Popen([sys.executable,str(H/'run_original_E1_route_side_navigation_resumed_v2.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        row['child_pid']=child.pid;save();begin=time.monotonic();timeout=False;forced=False
        try:code=child.wait(timeout=1200)
        except subprocess.TimeoutExpired:
            timeout=True;os.killpg(child.pid,signal.SIGTERM)
            try:code=child.wait(timeout=30)
            except subprocess.TimeoutExpired:forced=True;os.killpg(child.pid,signal.SIGKILL);code=child.wait()
    row.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 and not timeout else 'FAILED_NO_RETRY',end_utc=now(),exit_code=code,wall_s=time.monotonic()-begin,wall_timeout=timeout,forced_kill=forced)
    out=H/'original_E1_route_side_navigation_resumed_run_v2'
    if out.exists():row['post_close_manifest_sha256']=sha(seal_existing(out,dict(complete=code==0 and not timeout,forced_kill=forced,wall_timeout=timeout)))
    save();print(json.dumps(row,indent=2));raise SystemExit(code if not timeout else 1)
if __name__=='__main__':main()
