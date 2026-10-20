from pathlib import Path
import json,numpy as np
from measurement import bank,geometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent;out={};fig,axes=plt.subplots(4,3,figsize=(14,12))
for row,p in enumerate(sorted(p for p in h.glob('hold_m*_a*') if p.is_dir())):
 if not p.is_dir():continue
 z=np.load(p/'physics.npz');t=np.arange(len(z['position_world_xy']))*.02;b=np.degrees(bank(z['root_quaternion_world_wxyz']));v,k=geometry(z['position_world_xy']);summ=[]
 for start,end in [(10,15),(20,25),(30,34)]:
  w=(t>=start)&(t<=end);valid=w&np.isfinite(k);median=float(np.nanmedian(k[w]));iqr=float(np.nanquantile(k[w],.75)-np.nanquantile(k[w],.25));bs=float(np.quantile(b[w],.95)-np.quantile(b[w],.05));vs=float(np.quantile(v[w],.95)-np.quantile(v[w],.05));speed=float(np.median(v[w]));summ.append({'window':[start,end],'bank_deg':float(np.median(b[w])),'bank_span_deg':bs,'speed_m_s':speed,'speed_span_m_s':vs,'curvature_m_inv':median,'curvature_iqr':iqr,'valid_fraction':float(valid.sum()/w.sum()),'stable':bool(bs<=3 and vs<=.2*speed and iqr<=max(.15,.25*abs(median)) and valid.sum()>=.9*w.sum()),'actual_aux_deg':float(np.median(np.degrees(z['joint_position_rad'][w,1])))})
 out[p.name]=summ
 for col,(values,label) in enumerate([(b,'Bank deg'),(v,'Speed m/s'),(k,'Curvature 1/m')]):
  ax=axes[row,col];ax.plot(t,values);ax.set_title(p.name+' '+label);ax.grid(alpha=.3);ax.axvline(5,color='gray',ls='--')
(h/'hold_assessment.json').write_text(json.dumps(out,indent=2));fig.tight_layout();fig.savefig(h/'long_hold.png',dpi=150);print(json.dumps(out,indent=2))
