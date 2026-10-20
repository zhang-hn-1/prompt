from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib.pyplot as plt
from analyze import metrics,HERE
from probe import write

def main():
    results={};fig,axes=plt.subplots(2,2,figsize=(12,9))
    for row,case in enumerate(['0002','0004']):
        b,bz,bp=metrics(HERE/f'{case}_real');c,cz,cp=metrics(HERE/f'{case}_candidate_source_map');e=b['event_block']
        parity={k:bool(np.array_equal(bz[k][:e+(k!='action')],cz[k][:e+(k!='action')])) for k in ['position','proprio','quaternion','action']}
        entry_b=json.loads((HERE/f'{case}_real/entry.json').read_text())['before'];entry_c=json.loads((HERE/f'{case}_candidate_source_map/entry.json').read_text())['before']
        parity['shared_policy_scalars']=all(v==entry_c['policy'][k] for k,v in entry_b['policy'].items())
        parity['smoother']=entry_b['smoother']==entry_c['smoother']
        parity['all_physical_entry_fields']=all(entry_b[k]==entry_c[k] for k in entry_b if k!='policy')
        changed=int(np.sum(np.any(bz['action'][e:e+80]!=cz['action'][e:e+80],axis=1)))
        applied_change=int(np.sum(np.any(bz['proprio'][e+1:e+81,-2:]!=cz['proprio'][e+1:e+81,-2:],axis=1)))
        provenance=json.loads((HERE/f'{case}_candidate_source_map/map_provenance.json').read_text())
        expected='547dd73c52b45f9cc1aa8dca561c779349391f6ecef06ef59026921284b467da'
        map_ok=provenance['sha256']==expected and hashlib.sha256(Path(provenance['path']).read_bytes()).hexdigest()==expected
        ref_ok=all(np.array_equal(bz[k],cz[k]) for k in ['reference_xy','reference_yaw','reference_direction','reference_command'])
        event_ok=c['event_block']==e and c['anchor']==b['anchor']
        gates=dict(prefix_identical=all(parity.values()),original_map_verified=map_ok,original_reference_verified=ref_ok,event_verified=event_ok,rms=c['rms_m']<=.8*b['rms_m'],peak=c['peak_m']<=b['peak_m']+.05,progress=c['progress_m']>=b['progress_m']-.05,applied_changed=applied_change>0)
        rp=json.loads((HERE/f'{case}_candidate_source_map/replan.json').read_text())
        goal=bz['reference_xy'][-1]
        records=json.loads((HERE/f'{case}_candidate_source_map/tracking.json').read_text())[e:e+80]
        results[case]=dict(baseline=b,candidate=c,prefix=parity,changed_request_blocks=changed,changed_applied_sample_count=applied_change,gates=gates,accepted=all(gates.values()),replan_status=rp['result']['status'],replan_seconds=rp['seconds'],expanded_nodes=rp['result']['expanded_nodes'],replanned_initial_direction=rp['result']['waypoints'][0]['drive_direction'] if rp['result']['waypoints'] else None,braking_blocks=sum(r['source']=='direction_change_braking' for r in records),goal_distance_at_event_m=float(np.linalg.norm(cz['position'][e]-goal)),goal_distance_after_window_m=float(np.linalg.norm(cz['position'][e+80]-goal)))
        normal=[r for r in records if r['source']=='tracking']
        results[case]['normal_tracking_blocks']=len(normal)
        results[case]['clipped_normal_tracking_blocks']=sum(bool(r['protocol_clipped']) for r in normal)
        ref=bz['reference_xy'];lo,hi=b['segment'];axes[row,0].plot(ref[lo:hi+1,0],ref[lo:hi+1,1],'k--',label='original frozen reference')
        new=np.asarray([(w['x_m'],w['y_m']) for w in rp['result']['waypoints']])
        if len(new):axes[row,0].plot(new[:,0],new[:,1],':',color='gray',label='candidate plan (not evaluator)')
        for label,z,p in [('B0',bz,bp),('candidate',cz,cp)]:
            axes[row,0].plot(z['position'][e:e+81,0],z['position'][e:e+81,1],label=label)
            axes[row,1].plot(np.arange(len(p['absolute_cross_track_m']))*.1,p['absolute_cross_track_m'],label=label)
        axes[row,0].axis('equal');axes[row,0].set_title(case+' actual XY');axes[row,1].set_title(case+' error to ORIGINAL reference');axes[row,1].set_xlabel('s after live event');axes[row,1].set_ylabel('m')
        for ax in axes[row]:ax.grid(alpha=.3);ax.legend()
    accepted=all(v['accepted'] for v in results.values())
    write(HERE/'candidate_assessment.json',dict(results=results,accepted=accepted,promote_to_full_reference=accepted,formal_benchmark_triggered=False,old_reference_is_evaluation_truth=True))
    fig.tight_layout();fig.savefig(HERE/'candidate_comparison.png',dpi=170)
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
