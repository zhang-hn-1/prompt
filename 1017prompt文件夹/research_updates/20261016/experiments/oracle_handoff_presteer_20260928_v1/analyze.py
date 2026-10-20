from pathlib import Path
import json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scripts.analyze_round8_tracking_audit import project_trace_on_segment
from probe import write,OLD

HERE=Path(__file__).resolve().parent
def metrics(path):
    status=json.loads((path/'status.json').read_text());e=status['event'];a=status['anchor']
    z=np.load(path/'trace.npz');ref=z['reference_xy'];directions=z['reference_direction'];lo=a;hi=a
    while lo>0 and directions[lo-1]==directions[a]:lo-=1
    while hi+1<len(ref) and directions[hi+1]==directions[a]:hi+=1
    xy=z['canonical_position'][e:e+121]
    p=project_trace_on_segment(xy,ref,segment_start=lo,segment_end=hi,initial_segment_index=a-lo)
    err=p['absolute_cross_track_m'];progress=p['progress_s_m']
    tracking=json.loads((path/'tracking.json').read_text())[e:e+120]
    entry=json.loads((path/'entry.json').read_text())
    yaw=math.atan2(2*(entry['before']['quaternion'][0]*entry['before']['quaternion'][3]+entry['before']['quaternion'][1]*entry['before']['quaternion'][2]),1-2*(entry['before']['quaternion'][2]**2+entry['before']['quaternion'][3]**2))
    out=dict(event_block=e,anchor=a,segment=[lo,hi],rms_m=float(np.sqrt(np.mean(err**2))),peak_m=float(err.max()),entry_error_m=float(err[0]),exit_error_m=float(err[-1]),progress_m=float(progress[-1]-progress[0]),start_s_m=float(progress[0]),end_s_m=float(progress[-1]),reference_segment_length_m=float(np.linalg.norm(np.diff(ref[lo:hi+1],axis=0),axis=1).sum()),entry_root_heading_error_rad=float((yaw-z['reference_yaw'][a]-entry['transform']['angle']+math.pi)%(2*math.pi)-math.pi),protocol_clip_count=sum(bool(r.get('protocol_clipped')) for r in tracking),checkpoint_1m_reached=bool(np.any((progress-progress[0]>=1)&(err<=.25))),checkpoint_2m_reached=bool(np.any((progress-progress[0]>=2)&(err<=.25))))
    return out,z,p

def main():
    results={};parity={};fig,axes=plt.subplots(2,2,figsize=(12,9))
    for row,case in enumerate(['0002','0004']):
        real=np.load(HERE/f'{case}_real/trace.npz');aligned=np.load(HERE/f'{case}_aligned/trace.npz');e=json.loads((HERE/f'{case}_real/status.json').read_text())['event']
        parity[case]={k:bool(np.array_equal(real[k][:e+(k!='action')],aligned[k][:e+(k!='action')])) for k in ['position','proprio','quaternion','action']}
        rb=json.loads((HERE/f'{case}_real/entry.json').read_text())['before'];ab=json.loads((HERE/f'{case}_aligned/entry.json').read_text());parity[case]['entry_full_snapshot_equal']=rb==ab['before'];parity[case]['physical_and_policy_state_untouched_by_alignment']=ab['before']==ab['after']
        old=np.load(OLD/f'episode_hard_0_e1_{case}'/'states.npz')
        parity[case]['historical_B0_position_max_delta_m']=float(np.max(np.abs(real['position']-old['position_world_xy'][:len(real['position'])])))
        for condition in ['real','aligned']:
            m,z,p=metrics(HERE/f'{case}_{condition}');results[f'{case}_{condition}']=m
            axes[row,0].plot(z['canonical_position'][e:e+121,0],z['canonical_position'][e:e+121,1],label=condition)
            axes[row,1].plot(np.arange(len(p['absolute_cross_track_m']))*.1,p['absolute_cross_track_m'],label=condition)
        a,b=results[f'{case}_real']['segment'];ref=real['reference_xy'];axes[row,0].plot(ref[a:b+1,0],ref[a:b+1,1],'k--',label='frozen reference');axes[row,0].axis('equal');axes[row,0].set_title(f'{case}: canonical reference coordinates');axes[row,1].set_title(f'{case}: lateral error');axes[row,1].set_xlabel('Time after event (s)');axes[row,1].set_ylabel('m')
        for ax in axes[row]:ax.legend();ax.grid(alpha=.3)
    fig.suptitle('Diagnostic pose alignment only; NOT navigation improvement');fig.tight_layout();fig.savefig(HERE/'entry_diagnostic.png',dpi=170)
    write(HERE/'diagnostic_results.json',dict(metrics=results,parity=parity,interpretation='Alignment removes entry XY/root-yaw error only. Dynamics, joints and smoother are unchanged. Does not establish an ideal dynamically settled state.'))
    print(json.dumps(dict(metrics=results,parity=parity),indent=2))

if __name__=='__main__':main()
