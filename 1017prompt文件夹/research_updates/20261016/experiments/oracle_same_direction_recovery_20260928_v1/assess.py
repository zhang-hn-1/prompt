from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze import metrics
from probe import write
HERE=Path(__file__).resolve().parent
results={};fig,axes=plt.subplots(2,2,figsize=(11,8))
for row,case in enumerate(['0002','0004']):
    b,bz,bp=metrics(HERE/f'{case}_real');c,cz,cp=metrics(HERE/f'{case}_recovery');e=b['event_block']
    eb=json.loads((HERE/f'{case}_real/entry.json').read_text())['before'];ec=json.loads((HERE/f'{case}_recovery/entry.json').read_text())['before']
    parity={k:bool(np.array_equal(bz[k][:e+(k!='action')],cz[k][:e+(k!='action')])) for k in ['position','quaternion','proprio','action']}
    parity['physical_entry']=all(eb[k]==ec[k] for k in eb if k!='policy')
    parity['policy_entry']=all(v==ec['policy'][k] for k,v in eb['policy'].items())
    parity['reference']=all(np.array_equal(bz[k],cz[k]) for k in ['reference_xy','reference_yaw','reference_direction','reference_command'])
    parity['event']=e==c['event_block'] and b['anchor']==c['anchor']
    changed=int(np.any(bz['action'][e:]!=cz['action'][e:],axis=1).sum())
    applied=int(np.any(bz['proprio'][e+1:,-2:]!=cz['proprio'][e+1:,-2:],axis=1).sum())
    gates=dict(identical_prefix_reference_event=all(parity.values()),rms=c['rms_m']<=.8*b['rms_m'],peak=c['peak_m']<=b['peak_m']+.05,progress=c['progress_m']>=b['progress_m']-.05,changed_applied=applied>0)
    results[case]=dict(baseline=b,candidate=c,parity=parity,gates=gates,accepted=all(gates.values()),changed_request_blocks=changed,changed_applied_samples=applied)
    lo,hi=b['segment'];ref=bz['reference_xy'];axes[row,0].plot(ref[lo:hi+1,0],ref[lo:hi+1,1],'k--',label='frozen reference')
    for label,z,p in [('B0',bz,bp),('recovery',cz,cp)]:
        axes[row,0].plot(z['position'][e:e+81,0],z['position'][e:e+81,1],label=label)
        axes[row,1].plot(np.arange(81)*.1,p['absolute_cross_track_m'],label=label)
    axes[row,0].set_title(case+' actual XY');axes[row,0].axis('equal');axes[row,1].set_title(case+' original-reference error');axes[row,1].set_xlabel('s after event');axes[row,1].set_ylabel('m')
    for ax in axes[row]:ax.legend();ax.grid(alpha=.3)
write(HERE/'assessment.json',dict(results=results,accepted=all(x['accepted'] for x in results.values()),formal_benchmark=False))
fig.tight_layout();fig.savefig(HERE/'comparison.png',dpi=170)
print(json.dumps(results,indent=2))
