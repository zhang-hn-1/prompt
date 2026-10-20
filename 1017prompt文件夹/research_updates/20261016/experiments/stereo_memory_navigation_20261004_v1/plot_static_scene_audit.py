"""Frozen frame010 silhouette overlay and all-eight-eye descriptive measurements."""
from pathlib import Path
import json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent

def main():
    a=np.load(H/'static_scene_ray_measurements.npz'); m=np.load(H/'static_scene_mesh_measurements.npz')
    rgb=np.load(H/'capture_transaction_development/hard_0_e1_0004/online_inputs/frame_010.npz')
    ideal=json.loads((H/'static_scene_label_audit.json').read_text()); mesh=json.loads((H/'static_scene_mesh_decomposition.json').read_text())
    shapes=json.loads((H/'static_scene_geometry.json').read_text())['shapes']
    fig,axes=plt.subplots(2,3,figsize=(15,8),constrained_layout=True)
    for i,eye in enumerate(['left','right']):
        prefix=eye+'_010'; core=a[prefix+'_sphere_core_selection']; ids=a[prefix+'_expected_scene_id']
        axes[i,0].imshow(rgb[eye+'_rgb'])
        for identity in np.unique(ids[core]):
            selected=ids==identity
            axes[i,0].contour(np.arange(0,640,2),np.arange(0,480,2),selected,levels=[.5],colors=['lime'],linewidths=.8)
        axes[i,0].set_title(eye+' RGB + independent ideal silhouette\nvisual overlay only; no semantic gate')
        for j,(label,err) in enumerate([('Ideal sphere',a[prefix+'_current_ray_error_m']),('Raw authored mesh',m[prefix+'_current_raw_mesh_error_m'])],1):
            image=axes[i,j].imshow(np.ma.array(err,mask=~core),vmin=0,vmax=.025,cmap='magma')
            axes[i,j].set_title(label+' ray error (m)\nsame full independent sphere core')
        for ax in axes[i]: ax.axis('off')
    fig.colorbar(image,ax=list(axes[:,1:].ravel()),shrink=.8,label='Euclidean ray discrepancy (m), capped at .025')
    fig.suptitle('Archived hard0004 frame010: source geometry vs current transaction labels')
    fig.savefig(H/'figures/static_scene_fixed_frame010.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,4.8),constrained_layout=True)
    x=np.arange(8); labels=[str(r['frame'])+' '+r['eye'][0].upper() for r in ideal['rows']]
    def vals(rows,field):
        return [np.nan if r['comparisons'][field]['within_0p01m'] is None else 100*r['comparisons'][field]['within_0p01m'] for r in rows]
    iv=[100*r['comparisons']['current_pose_transaction']['sphere_core']['within_0p01m'] if r['independent_core_n'] else np.nan for r in ideal['rows']]
    axes[0].bar(x-.22,iv,width=.22,label='Ideal current')
    axes[0].bar(x,vals(mesh['rows'],'current'),width=.22,label='Raw mesh current')
    axes[0].bar(x+.22,vals(mesh['rows'],'pretransaction'),width=.22,label='Raw mesh pretransaction')
    axes[0].axhline(90,color='gray',ls='--',label='Original ideal gate (not mesh admission)')
    axes[0].set_ylim(0,105);axes[0].set_ylabel('% of all independently selected core rays <= .01m')
    axes[1].bar(x,[r['independent_core_n'] for r in ideal['rows']],color='#3477aa');axes[1].set_ylabel('Independent core rays, stride 2')
    for ax in axes: ax.set_xticks(x,labels);ax.set_xlabel('Archived frame / eye; 20R & 23R have no core support')
    axes[0].legend(fontsize=8,loc='upper center',bbox_to_anchor=(.5,-.23),ncol=2);fig.suptitle('Eight eye-frames; original ideal gate failed; mesh descriptive only')
    fig.savefig(H/'figures/static_scene_all_eye_frames.png',dpi=140);plt.close(fig)
    print('SAVED 2 predeclared diagnostic figures')
if __name__=='__main__':main()
