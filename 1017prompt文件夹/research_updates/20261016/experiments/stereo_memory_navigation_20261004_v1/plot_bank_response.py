from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from bank_response_measurement import _measurement
H=Path(__file__).resolve().parent;z=np.load(H/'bank_response_run/physics.npz');a=json.loads((H/'bank_response_assessment.json').read_text());t=z['simulation_time_s'];bank=np.degrees(_measurement.bank(z['root_quaternion_world_wxyz']));speed,k=_measurement.geometry(z['position_world_xy']);target=np.zeros(len(t));target[(t>=6)&(t<26)]=-22.5
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
axes[0].plot(t,bank,label='Measured body-Y bank');axes[0].plot(t,target,'--',label='Known requested bank');axes[0].set_ylabel('Bank / deg')
axes[1].plot(t,speed,label='XY measured SG31 speed');axes[1].axhline(a['v1_nominal']['speed_m_s'],ls=':',label='V1 nominal');axes[1].axhline(a['bank_map_frozen_point']['speed_point_m_s'],ls='--',label='Old BankMap case');axes[1].set_ylabel('Speed / m/s')
axes[2].plot(t,k,label='Offline XY geometric curvature');axes[2].axhline(a['v1_nominal']['curvature_1_m'],ls=':',label='V1 nominal');axes[2].axhline(a['bank_map_frozen_point']['body_curvature_1_m'],ls='--',label='Old BankMap point');axes[2].set_ylabel('Curvature / 1/m');axes[2].set_ylim(-5,5);axes[2].set_xlabel('Episode time / s')
for ax in axes:ax.axvspan(21,25.5,color='green',alpha=.12,label='Presealed steady window');ax.axvline(6,color='gray',alpha=.4);ax.axvline(26,color='gray',alpha=.4);ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8)
fig.suptitle('ONE current-controller open-plane response / measured data, not navigation');fig.tight_layout();fig.savefig(H/'bank_response_measurement.png',dpi=150)
