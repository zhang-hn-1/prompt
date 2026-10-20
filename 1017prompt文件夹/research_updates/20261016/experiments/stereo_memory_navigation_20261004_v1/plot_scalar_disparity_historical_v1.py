from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;a=json.loads((H/'scalar_disparity_residual_historical_v1/assessment.json').read_text());r=a['frames'];t=np.array([v['frame']/10 for v in r]);fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
for key,label in [('raw_metric','raw depth'),('metric','frozen scalar MODEL depth')]:
 ax[0].plot(t,[np.nan if v[key]['within_0p10m'] is None else 100*v[key]['within_0p10m'] for v in r],label=label);ax[1].plot(t,[np.nan if v[key]['unconditional_p90_m'] is None else v[key]['unconditional_p90_m'] for v in r],label=label)
ax[0].axhline(95,color='gray',ls='--');ax[1].axhline(.1,color='gray',ls='--');ax[0].set_ylabel('Queries within 10 cm (%)');ax[1].set_ylabel('Unconditional p90 error (m)');ax[1].set_xlabel('Historical actual episode time (s); no selected queries after 26 s');ax[1].set_xlim(0,60);ax[0].legend();fig.suptitle('Frozen +3.455 px scalar, whole historical native-R1 sequence\nDevelopment only: not fresh / source admission / navigation');fig.tight_layout();fig.savefig(H/'scalar_disparity_historical_v1.png',dpi=160)
