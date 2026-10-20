from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_request_chain_20260929_v6');fig,axs=plt.subplots(3,2,figsize=(13,11))
for col,c in enumerate(['0002','0004']):
 rr=json.loads((h/f'chain_{c}.json').read_text());r=[x for x in rr if x['valid']];t=[x['time'] for x in r]
 for key,label in [('requested_curvature','Actual requested curvature'),('ideal_geometry_curvature','At reference pose, same target'),('position_increment','Position increment'),('heading_increment','Heading increment')]:axs[0,col].plot(t,[x[key] for x in r],label=label)
 for key,label in [('desired_bank_deg','Requested bank'),('bank_deg','Measured bank')]:axs[1,col].plot(t,[x[key] for x in r],label=label)
 for key,label in [('aux_mapping_deg','Mapped feedforward'),('pid_aux_deg','PID output'),('applied_aux_deg','Applied target'),('actual_aux_deg','Measured joint')]:axs[2,col].plot(t,[x[key] for x in r],label=label)
 axs[0,col].set_title(c+' baseline command chain');axs[0,col].set_ylabel('curvature [1/m]');axs[1,col].set_ylabel('bank [deg]');axs[2,col].set_ylabel('auxiliary angle [deg]')
 for ax in axs[:,col]:
  for x in rr:
   if not x['valid']:ax.axvspan(x['time'],x['time']+.1,color='.8',alpha=.3)
  ax.grid();ax.legend(fontsize=8);ax.set_xlabel('simulation time [s]')
fig.tight_layout();fig.savefig(h/'request_chain.png',dpi=160)
