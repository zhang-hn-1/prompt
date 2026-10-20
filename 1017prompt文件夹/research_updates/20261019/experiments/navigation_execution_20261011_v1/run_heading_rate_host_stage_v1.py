"""At most one launch of each arm, guarded by an independent admission."""
from pathlib import Path
import sys,os,json,hashlib,datetime
HERE=Path(__file__).resolve().parent
LEGACY=HERE.parent/'stereo_memory_navigation_20261004_v1'
sys.path.insert(0,str(LEGACY))
from durable_stage_supervision_v1 import run_durable

if __name__=='__main__':
    if len(sys.argv)!=2 or sys.argv[1] not in ('b0','candidate'):
        raise ValueError('one explicit arm required')
    arm=sys.argv[1]
    admission=json.loads((HERE/'heading_rate_host_prelaunch_admission_v1.json').read_text())
    assert admission['admitted'] and admission['max_actual_arms']==2 and admission['no_retry']
    for file,digest in admission['protocol_sha256'].items():
        assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    if arm=='candidate':
        baseline=json.loads((HERE/'heading_rate_e1_b0_status_v2.json').read_text())
        assert baseline['status']=='finished' and baseline['child_returncode']==0
    environment=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_heading_rate')
    status=run_durable([sys.executable,str(HERE/'run_heading_rate_e1_v2.py'),'--arm',arm],
        HERE/f'heading_rate_e1_{arm}_status_v2.json',HERE/f'heading_rate_e1_{arm}_run_v2.log',
        timeout=1200,grace=45,env=environment)
    run=HERE/f'heading_rate_e1_{arm}_run_v2'
    if run.exists():
        name='artifact_manifest_post_close.json' if status['status']=='finished' else 'artifact_manifest_after_runtime_failure.json'
        (run/name).write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status=status,
            artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in run.rglob('*') if p.is_file()}),indent=2)+'\n')
    print(json.dumps(status),flush=True)
    sys.exit(0 if status['status']=='finished' else 1)
