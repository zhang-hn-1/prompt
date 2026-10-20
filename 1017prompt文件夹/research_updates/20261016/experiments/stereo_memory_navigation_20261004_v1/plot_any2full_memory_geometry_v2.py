"""Show ALL actual nonempty predictions only after postseal assessment."""
from pathlib import Path
import json,hashlib,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;I=H/'any2full_memory_inputs_v1';O=H/'any2full_memory_predictions_v2';D=H/'qualified_hazard_navigation_run_v1';A=H/'any2full_memory_geometry_assessment_v2.json';assert A.exists();out=H/'any2full_predicted_geometry_actual_panels_v2.png';assert not out.exists();s=json.loads((O/'prediction_seal.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();assert all(sha(p)==v for p,v in s['output_sha256'].items());fig,axes=plt.subplots(3,4,figsize=(16,10),layout='constrained')
for row,frame in enumerate([235,245,255]):
 i=np.load(I/f'frame_{frame:03d}_right.npz');p=np.load(O/f'frame_{frame:03d}_right.npz');g=np.load(D/'offline_truth'/f'frame_{frame:03d}.npz');rgb=np.rot90(i['current_rgb_world_up'],-1);z=p['depth_m'];prompt=i['prompt_depth_native_m'];gt=g['right_depth_m'];yy,xx=np.nonzero(prompt);axes[row,0].imshow(rgb);axes[row,0].scatter(xx,yy,s=4,c='red');axes[row,0].set_title(f'{frame}: actual right RGB + old anchors ({len(xx)})');axes[row,1].imshow(np.where(prompt>0,prompt,np.nan),vmin=.5,vmax=3,cmap='viridis');axes[row,1].set_title('Input old axial depth (sparse)');im=axes[row,2].imshow(z,vmin=.5,vmax=3,cmap='viridis');axes[row,2].set_title('Actual NN predicted depth, NOT observed');rayfactor=np.sqrt(1+((np.arange(640)-319.5)/584.8889)[None,:]**2+((np.arange(480)-239.5)/584.8889)[:,None]**2);error=np.abs(z-gt)*rayfactor;axes[row,3].imshow(np.where(np.isfinite(gt),error,np.nan),vmin=0,vmax=.2,cmap='magma');axes[row,3].set_title('Postseal GT 3D endpoint error (m)')
 for ax in axes[row]:ax.axis('off')
fig.colorbar(im,ax=axes[:,2],label='axial depth m');fig.suptitle('Known-pose development only. Left eyes EMPTY / NOT EXERCISED. No navigation admission.');fig.savefig(out,dpi=140);plt.close(fig);print(out)
