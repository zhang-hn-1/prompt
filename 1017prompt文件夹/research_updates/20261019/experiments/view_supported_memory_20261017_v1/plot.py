from pathlib import Path
import json
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;r=json.loads((D/'result.json').read_text());rows=r['rows'];t=[v['time_s'] for v in rows]
fig,axs=plt.subplots(1,3,figsize=(13,3.5))
for k in 'ABC':
 axs[0].plot(t,[v[k]['false'] for v in rows],label=k);axs[1].plot(t,[v[k]['blind_missing']/v['blind_supported_targets'] if v['blind_supported_targets'] else float('nan') for v in rows],label=k)
axs[0].set_ylabel('false memory cells vsGT bodyband');axs[1].set_ylabel('missing previously observed blind targets');axs[1].set_ylim(0,1)
for ax in axs[:2]:ax.legend();ax.set_xlabel('E0 development seconds, all110epochs')
keys=list('ABC');axs[2].bar(keys,[r['aggregate'][k]['false_cell_seconds'] for k in keys]);axs[2].set_ylabel('false-cell seconds')
fig.suptitle('Same sealed 4RGB-source E0 replay: A current, B repeated epochs, C view novelty. C rejected; no actual three-arm navigation.');fig.tight_layout();fig.savefig(D/'replay_tradeoff.png',dpi=130)
