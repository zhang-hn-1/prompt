import json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from clock_prefix import prefix_matches
h=Path(__file__).resolve().parent;out={};fig,axes=plt.subplots(1,2,figsize=(12,5))
scenes=json.loads((h/'scene_manifest.json').read_text())
for ax,kind in zip(axes,['LR','RL']):
 p=h/f'{kind}_blocked_ungated';z=np.load(p/'physics.npz');g=np.load(h/f'{kind}_blocked/physics.npz');contact=np.load(p/'contact.npz')['contact'];hits=np.flatnonzero(contact[:,0]);same=all(prefix_matches(z,{k:g[k][:801] for k in g.files},801).values());pred=json.loads((p/'prediction_status.json').read_text())
 out[kind]={'prefix_matches_gated':same,'valid_contact_monitor':bool(contact[:,1:].all()),'contact_samples_above_1N':int(len(hits)),'first_contact_s':float(z['simulation_time_s'][hits[0]]) if len(hits) else None,'gate_would_reject':not pred['selected_turn'],'positive_control_passed':bool(same and contact[:,1:].all() and len(hits)>0)}
 for label,x in [('gate enabled: brake',g['position_world_xy'][800:]),('gate disabled: turn',z['position_world_xy'][800:])]:ax.plot(*x.T,label=label)
 c=np.array(scenes[f'{kind}_blocked']['center_xy']);ax.add_patch(plt.Circle(c,.15,color='black',label='physical obstacle'));ax.add_patch(plt.Circle(c,.55,fill=False,color='red',ls=':',label='body+obstacle'));ax.axis('equal');ax.grid(alpha=.3);ax.legend(fontsize=8);ax.set_title(kind+' identical blocked scene')
manifest=json.loads((h/'contact_control_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in manifest.items());(h/'contact_control_assessment.json').write_text(json.dumps(out,indent=2)+'\n');fig.tight_layout();fig.savefig(h/'gate_on_off.png',dpi=150);print(json.dumps(out,indent=2))
