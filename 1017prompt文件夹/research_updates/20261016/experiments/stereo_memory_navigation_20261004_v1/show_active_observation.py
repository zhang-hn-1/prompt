from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image,ImageDraw
H=Path(__file__).resolve().parent;r=H/'active_observation_run';o=H/'active_feature_runs';fig=H/'figures';a=json.loads((o/'assessment.json').read_text());p=np.load(r/'physics.npz');d=json.loads((r/'decision.json').read_text());t=p['simulation_time_s']
ref=np.array(d['world_xy']);actual=p['position_world_xy'][300:];q=p['root_quaternion_world_wxyz'];bank=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3])/np.sum(q*q,axis=1),-1,1)));v=np.linalg.norm(p['linear_velocity_world_m_s'][:,:2],axis=1)
f,axes=plt.subplots(1,3,figsize=(13,4));axes[0].plot(ref[:,0],ref[:,1],label='frozen measured reference');axes[0].plot(actual[:,0],actual[:,1],ls='--',label='actual E1');axes[0].set_aspect('equal');axes[0].set_xlabel('world X (m)');axes[0].set_ylabel('world Y (m)');axes[0].legend()
axes[1].plot(t,v,label='actual XY speed');axes[1].axhline(.05,c='r',ls='--');axes[1].axvline(12,c='grey',ls=':',label='zero request');axes[1].axvline(14.2,c='g',ls=':',label='five actual low samples');axes[1].set_ylabel('m/s');axes[1].set_xlabel('episode seconds');axes[1].legend(fontsize=8)
axes[2].plot(t,bank,label='actual bank');axes[2].set_ylabel('degrees');axes[2].set_xlabel('episode seconds');axes[2].legend();f.suptitle('ONE actual18s original-scene observation; no navigation success claim');f.tight_layout();f.savefig(fig/'active_observation_motion.png',dpi=150);plt.close(f)
frames=[0,60,75,120,142,180];canvas=Image.new('RGB',(1280,6*510+25),'white')
for n,i in enumerate(frames):
 raw=np.load(r/'online_inputs'/f'frame_{i:03d}.npz');rec=json.loads((o/f'prediction_{i:03d}.json').read_text());points={pt['index']:pt for pt in a['points'] if pt['frame']==i and pt['arm']=='causal'}
 for col,eye in enumerate(['left','right']):
  im=Image.fromarray(raw[eye+'_rgb']);draw=ImageDraw.Draw(im)
  if eye=='left':
   for j,(x,y) in enumerate(rec['causal']['query_xy']):
    pt=points.get(j);color=(150,150,150) if pt is None else ((0,255,70) if pt['correct'] else ((255,50,50) if pt['error_m'] is not None else (255,190,0)));rad=2 if pt is None else 4;draw.ellipse((x-rad,y-rad,x+rad,y+rad),outline=color,width=2)
  canvas.paste(im,(col*640,n*510+30));ImageDraw.Draw(canvas).text((col*640+5,n*510+8),f"{i:03d} {eye} | t={rec['time_s']:.1f}s | left queries {rec['causal']['query_count']}",fill='black')
ImageDraw.Draw(canvas).text((5,6*510+5),'LEFT overlay: grey missing; green <=10cm; red >10cm; amber no renderer label. RIGHT native RGB.',fill='black');canvas.save(fig/'active_observation_stereo.png')
focus=Image.new('RGB',(1280,1020),'white');focus.paste(canvas.crop((0,510,1280,1020)),(0,0));focus.paste(canvas.crop((0,1020,1280,1530)),(0,510));focus.save(fig/'active_observation_motion_rgb.png')
f,ax=plt.subplots(1,2,figsize=(11,4));
for arm in ['current','causal']:
 rows=[rr for rr in a['rows'] if rr['arm']==arm];tt=np.array([rr['time_s'] for rr in rows]);correct=np.cumsum([rr['correct'] for rr in rows]);queries=np.cumsum([rr['queries'] for rr in rows]);ax[0].plot(tt,100*correct/np.maximum(queries,1),label=arm)
ax[0].axhline(90,c='red',ls='--',label='unchanged source gate');ax[0].set_ylim(0,100);ax[0].set_ylabel('Cumulative correct / ALL queries (%)');ax[0].set_xlabel('episode seconds');ax[0].legend()
mm=a['memory_rows'];ax[1].plot([rr['frame']/10 for rr in mm],[rr['records'] for rr in mm],label='all sparse records');ax[1].plot([rr['frame']/10 for rr in mm],[rr['correct_records'] for rr in mm],label='offline labelled <=10cm');ax[1].set_ylabel('record counts, not spatial area');ax[1].set_xlabel('episode seconds');ax[1].legend();f.suptitle('Full181-pair source regression: knownpose, source gate REJECTED');f.tight_layout();f.savefig(fig/'active_observation_sources.png',dpi=150);plt.close(f)
