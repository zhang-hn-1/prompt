"""Sequential GPU use; persist bounded matrix status and check real artifacts."""
import datetime,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
status=[]
def run(label,cmd,verify):
    print('START',label,flush=True);start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    with (HERE/(label+'.log')).open('w') as log:
        result=subprocess.run(cmd,env=dict(os.environ,PYTHONPATH=str(PROJECT),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_new_rig'),stdout=log,stderr=subprocess.STDOUT)
    good=result.returncode==0 and verify()
    status.append(dict(label=label,command=cmd,start_utc=start,exit_code=result.returncode,artifacts_complete=bool(good)))
    (HERE/'stage_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISH',label,result.returncode,'verified',good,flush=True)
    if not good:raise RuntimeError('stage stopped at '+label)

def main():
    protocol=json.loads((HERE/'capture_protocol.json').read_text());samples=[]
    for seq in protocol['sequences']:
        name=seq['sequence'];root=HERE/'capture'/name
        run('capture_'+name,[sys.executable,str(HERE/'capture_replay.py'),'--sequence',name],lambda root=root:(root/'capture_summary.json').is_file() and not (root/'failure.txt').exists())
        summary=json.loads((root/'capture_summary.json').read_text());assert summary['selected_indices']==seq['selected_indices']
        import hashlib
        for i in seq['selected_indices']:
            rgb=root/'online_inputs'/f'frame_{i:03d}.npz';truth=root/'offline_truth'/rgb.name
            assert rgb.is_file() and truth.is_file()
            samples.append(dict(sequence=name,rgb_file=str(rgb),truth_file=str(truth),rgb_sha256=hashlib.sha256(rgb.read_bytes()).hexdigest(),truth_sha256=hashlib.sha256(truth.read_bytes()).hexdigest()))
    assert len(samples)==66
    (HERE/'offline_manifest.json').write_text(json.dumps(dict(role='new actual camera captures of familiar replayed development trajectories; not held-out',rig_id='ball_stereo_front020_up005_v2',frame_stride=10,samples=samples),indent=2)+'\n')
    for model in ('da3_metric','unidepth_v2'):
        run(model+'_inference',[sys.executable,str(HERE/'model_inference.py'),'--model',model],lambda model=model:len(list((HERE/'predictions'/model).glob('*/*.npz')))==66 and len(json.loads((HERE/(model+'_inference_records.json')).read_text()))==66)
    run('depth_assessment',[sys.executable,str(HERE/'evaluate_depth.py')],lambda:(HERE/'depth_assessment.json').exists())
    run('confidence_assessment',[sys.executable,str(HERE/'confidence_audit.py')],lambda:(HERE/'confidence_assessment.json').exists())
    # O-map/O-visible are unchanged diagnostic conditions, not an accepted RGB controller.
    run('online_matrix',[sys.executable,str(HERE/'run_remaining.py')],lambda:len(list((HERE/'runs').glob('*/*/episode_*/metrics.json')))==8 and len(json.loads((HERE/'run_status.json').read_text()))==8 and all(r['exit_code']==0 for r in json.loads((HERE/'run_status.json').read_text())))
    run('run_assessment',[sys.executable,str(HERE/'analyze_runs.py')],lambda:(HERE/'run_assessment.json').exists())
    print('DATA_AND_RUNS_COMPLETE',flush=True)

if __name__=='__main__':main()
