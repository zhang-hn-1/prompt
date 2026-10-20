"""Frozen latency development check then full470-input output-byte equivalence; no GT."""
from pathlib import Path
import json,hashlib,sys,os,time,datetime
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1]
load=lambda p:json.loads(Path(p).read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def verify(pred,path):
    from parallel_source import legacy
    with np.load(path) as old:
        for n in legacy.NAMES:
            for k in ['depth','valid','disparity','CAD']:
                actual=pred[n][k];reference=old[n+'_'+k]
                assert actual.dtype==reference.dtype and actual.shape==reference.shape and actual.tobytes()==reference.tobytes(),(path,n,k)
    return 16

def read(row):
    from parallel_source import legacy
    assert sha(row['input_rgb_frame'])==row['input_sha256'];assert sha(row['prediction_path'])==row['prediction_sha256']
    with np.load(row['input_rgb_frame']) as a:return {n:a[n+'_rgb'] for n in legacy.NAMES},a['joints_current']

def worker(args):
    slot,rows=args
    from parallel_source import ParallelFourSGBM
    model=ParallelFourSGBM();records=[]
    put(D/f'worker_{slot}_status.json',dict(status='running',pid=os.getpid(),parent_pid=os.getppid(),processed=0,total=len(rows),start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    try:
        for i,row in enumerate(rows):
            rgb,j=read(row);pred,seconds=model.predict(rgb,j);arrays=verify(pred,row['prediction_path']);records.append(dict(sequence=row['sequence'],frame=row['frame'],source_wall_s=seconds,arrays_exact=arrays,raw_sha256=row['input_sha256'],reference_sha256=row['prediction_sha256']))
            if (i+1)%10==0:put(D/f'worker_{slot}_status.json',dict(status='running',pid=os.getpid(),parent_pid=os.getppid(),processed=i+1,total=len(rows),last_sequence=row['sequence'],last_frame=row['frame'],last_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
        put(D/f'worker_{slot}_result.json',dict(complete=True,rows=records,pid=os.getpid(),source_invocations=len(records)))
        put(D/f'worker_{slot}_status.json',dict(status='finished',pid=os.getpid(),processed=len(records),total=len(rows),end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
        return records
    finally:model.close()

def main():
    cfg=load(D/'protocol.json')
    for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
    for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
    from parallel_source import ParallelFourSGBM,legacy
    feature=load(cfg['feature_seal']);rows=feature['rows'];selection=[next(r for r in rows if r['sequence']==s and r['frame']==f) for s,f in cfg['latency_frames']]
    old=legacy.FourSGBM();new=ParallelFourSGBM();bench=[]
    try:
        for i,row in enumerate(selection):
            rgb,j=read(row);values={}
            for kind,model in ([('serial',old),('parallel',new)] if i%2==0 else [('parallel',new),('serial',old)]):
                pred,seconds=model.predict(rgb,j);verify(pred,row['prediction_path']);values[kind]=seconds
            bench.append(dict(sequence=row['sequence'],frame=row['frame'],**values,ratio=values['parallel']/values['serial'],all16_arrays_byte_exact=True));print('LATENCY',bench[-1],flush=True)
    finally:new.close()
    gates=dict(all4_output_exact=True,median_ratio_le80pct=float(np.median([r['ratio'] for r in bench]))<=.8,all4_ratio_le110pct=max(r['ratio'] for r in bench)<=1.1)
    put(D/'latency_result.json',dict(complete=True,rows=bench,gates=gates,pass_local_development=all(gates.values()),source_invocations_serial=4,source_invocations_parallel=4,scope='Four prospectively frozen known development captures, alternating serial/parallel order once each; no statistical generalization or complete onlinechain latency claim.'))
    if not all(gates.values()):
        put(D/'result.json',dict(complete=True,decision='STOP_LOCAL_PERFORMANCE_CHECK',gates=gates,source_invocations=8,fit=0,physics=0,navigation=0));return
    import multiprocessing as mp
    ctx=mp.get_context('spawn');children=[ctx.Process(target=worker,args=((i,rows[i::4]),)) for i in range(4)]
    try:
        for i,p in enumerate(children):
            p.start();put(D/f'worker_{i}_launch.json',dict(pid=p.pid,parent_pid=os.getpid(),source_frames=len(rows[i::4])))
        while any(p.is_alive() for p in children):
            if any(p.exitcode not in (None,0) for p in children):raise RuntimeError('Worker failure: stop remaining workers, do not continue source batch')
            time.sleep(.1)
        for p in children:p.join()
        assert all(p.exitcode==0 for p in children)
    finally:
        for p in children:
            if p.pid and p.is_alive():p.terminate()
        for p in children:
            if p.pid:
                p.join(timeout=5)
                if p.is_alive():p.kill();p.join(timeout=5)
    full=sorted([r for i in range(4) for r in load(D/f'worker_{i}_result.json')['rows']],key=lambda r:(r['sequence'],r['frame']))
    assert len(full)==470 and len({(r['sequence'],r['frame']) for r in full})==470 and sum(r['arrays_exact'] for r in full)==7520
    put(D/'result.json',dict(complete=True,decision='FULL470_OUTPUT_EQUIVALENCE_PASSED_LOCAL_LATENCY_CHECK',rows=full,gates=gates,full_exposures=470,output_arrays_byte_exact=7520,source_invocations_serial=4,source_invocations_parallel=474,total_source_invocations=478,fit=0,learned_inference=0,physics=0,navigation=0,online_interface_not_yet_integrated=True,no_new_accuracy_evaluation=True,scope='4-frame controlled source-only latency check; full470 oldreference arrays byte-identical. Offline4frameworkers x4independent SGBMcompute threads for equality throughput; batch timings under load are NOT deployed realtime/endtoend latency. Original source/frozen outputs unchanged.'))
if __name__=='__main__':main()
