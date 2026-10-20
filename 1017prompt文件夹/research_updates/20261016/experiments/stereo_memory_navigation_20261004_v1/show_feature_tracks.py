"""Actual archived RGB overlays and all-query diagnostic plots."""
from pathlib import Path
import json
import numpy as np
from PIL import Image,ImageDraw
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;o=H/'feature_tracks_runs';p=json.loads((H/'feature_tracks_protocol.json').read_text());a=json.loads((o/'assessment.json').read_text());fig=H/'figures'
panels=[];focus=[]
for seq,samples in p['sequences'].items():
 for s in samples:
  i=s['frame'];r=json.loads((o/seq/f'prediction_{i:03d}.json').read_text());im=Image.fromarray(np.load(s['rgb_file'])['left_rgb']);d=ImageDraw.Draw(im)
  labelled={v['index']:v for v in a['points'] if v['sequence']==seq and v['frame']==i and v['arm']=='causal'}
  for j,(x,y) in enumerate(r['causal']['query_xy']):
   v=labelled.get(j);c=(150,150,150) if v is None else ((0,255,70) if v['correct'] else ((255,50,50) if v['error_m'] is not None else (255,190,0)))
   radius=2 if v is None else 4;d.ellipse((x-radius,y-radius,x+radius,y+radius),outline=c,width=2)
  panel=Image.new('RGB',(640,510),'white');panel.paste(im,(0,30));ImageDraw.Draw(panel).text((5,7),f"{seq} {i:03d}: queries {r['causal']['query_count']}, kept {len(r['causal']['points'])}",fill='black');panels.append(panel)
  if (seq,i) in [('matte_board',70),('e1',100),('hard_0_e1_0004',0),('hard_0_e1_0004',20)]:focus.append(panel)
allim=Image.new('RGB',(2560,7*510+25),'white')
for n,im in enumerate(panels):allim.paste(im,((n%4)*640,(n//4)*510))
ImageDraw.Draw(allim).text((5,7*510+5),'grey missing; green <=10cm; red >10cm; amber no label. Feature queries only, NOT area/free space.',fill='black');allim.save(fig/'feature_tracks_all26.png')
focusim=Image.new('RGB',(1280,1045),'white')
for n,im in enumerate(focus):focusim.paste(im,((n%2)*640,(n//2)*510))
ImageDraw.Draw(focusim).text((5,1025),'grey missing; green <=10cm; red >10cm; amber no label. Native RGB with offline label overlay.',fill='black');focusim.save(fig/'feature_tracks_focus.png')
f,ax=plt.subplots(1,2,figsize=(11,4.5));seqs=list(a['sequences']);x=np.arange(3)
for offset,arm in [(-.18,'current'),(.18,'causal')]:
 ax[0].bar(x+offset,[100*a['sequences'][s][arm]['unconditional_fraction'] for s in seqs],width=.36,label=arm)
 ax[1].bar(x+offset,[a['sequences'][s][arm]['retained'] for s in seqs],width=.36,label=arm)
ax[0].axhline(90,color='red',linestyle='--',label='frozen source gate');ax[0].set_ylim(0,100);ax[0].set_ylabel('Correct / ALL feature queries (%)');ax[1].set_ylabel('Retained sparse records')
for aa in ax:aa.set_xticks(x,seqs,rotation=12);aa.legend();aa.grid(axis='y',alpha=.2)
f.suptitle('Historical known-pose input ablation: no navigation/free-space claim');f.tight_layout();f.savefig(fig/'feature_tracks_metrics.png',dpi=150);plt.close(f)
# Single actual falsely matched current stereo feature: no true partner is supplied.
seq='hard_0_e1_0004';i=0;s=p['sequences'][seq][0];raw=np.load(s['rgb_file']);r=json.loads((o/seq/'prediction_000.json').read_text());pt=r['causal']['points'][0];feat=np.load(o/seq/'features_000.npz');im=Image.new('RGB',(1280,520),'white');draw=ImageDraw.Draw(im)
for eye,col,idx in [('left',0,pt['query'][2]),('right',1,pt['source'][2])]:
 im.paste(Image.fromarray(raw[eye+'_rgb']),(col*640,40));x,y=feat[eye+'_xy'][idx];draw.ellipse((col*640+x-8,40+y-8,col*640+x+8,40+y+8),outline='red',width=3)
draw.text((8,8),'Actual ORB match: accepted sensitivity 0.0167m, reprojection 0.500px; offline error 11.003m.',fill='black');draw.text((8,24),'Left query vs chosen right source (not the true partner). Geometric consistency cannot certify correspondence.',fill='black');im.save(fig/'feature_tracks_false_pair.png')
