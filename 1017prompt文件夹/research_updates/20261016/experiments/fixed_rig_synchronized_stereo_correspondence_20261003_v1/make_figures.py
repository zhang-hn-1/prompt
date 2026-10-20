"""Offline measurement illustrations; none of these truth overlays feed control."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from measurement import H,DATA,actual_points,body_coordinates,project,masks,uv


def pixel_sets(frame,eye):
    other='right' if eye=='left' else 'left'
    points,valid,_=actual_points(frame,eye)
    root=frame['root_position_world_m'];w,x,y,z=frame['root_quaternion_world_wxyz']
    yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    body=body_coordinates(points,root[:2],yaw)
    scene=valid&(np.linalg.norm(points-root,axis=1)>=.55)&~masks[eye][::4,::4].ravel()
    front=scene&(points[:,2]>=.25)&(points[:,2]<=1.5)&(body[:,0]>=.35)&(body[:,0]<=3.5)&(np.abs(body[:,1])<=.6)
    pose=frame[other+'_camera_usd_world']
    rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
    mapped,expected=project(points,(pose[:3,3],rotation))
    finite=np.nan_to_num(mapped,nan=-1,posinf=-1,neginf=-1)
    u,v=np.rint(finite).astype(int).T
    inside=(expected>.45)&(u>=0)&(u<640)&(v>=0)&(v<480)
    u=np.clip(u,0,639);v=np.clip(v,0,479)
    observed=frame[other+'_depth_m'][v,u]
    same=inside&~masks[other][v,u]&np.isfinite(observed)&(observed>.45)&(np.abs(observed-expected)<=np.maximum(.1,.05*expected))
    return front,front&same


choices=[('direct_e1',45,'E1 direct approach, 4.5 s'),('direct_e1',57,'E1 direct approach, 5.7 s'),
         ('hard_0_e1_0003',75,'hard 0003, 7.5 s'),('hard_0_e1_0004',1,'hard 0004, 0.1 s')]
fig,axes=plt.subplots(4,2,figsize=(13,17))
for row,(label,i,title) in enumerate(choices):
    root=H/'direct_e1' if label=='direct_e1' else DATA/label
    name=f'frame_{i:03d}.npz'
    with np.load(root/'online_inputs'/name) as f,np.load(root/'offline_truth'/name) as truth,np.load(H/'replay'/label/name) as replay:
        for col,eye in enumerate(('left','right')):
            ax=axes[row,col];ax.imshow(f[eye+'_rgb'])
            front,same=pixel_sets(truth,eye)
            ax.contour(masks[eye],levels=[.5],colors=['#ff6070'],linewidths=.8)
            ax.scatter(uv[front,0],uv[front,1],s=3,color='#ffaa22',alpha=.7,label='Measured front obstacle')
            ax.scatter(uv[same,0],uv[same,1],s=6,color='#60ff40',alpha=.9,label='Same surface in both eyes')
            kept=replay[eye+'_uv'];band=replay[eye+'_bodyband']
            ax.scatter(kept[band,0],kept[band,1],s=18,facecolors='none',edgecolors='#00e9ff',linewidths=.6,label='Candidate retained body-height points')
            ax.set_title(f'{title} | {eye}\nfront samples {front.sum()}, same-surface {same.sum()}, retained body band {band.sum()}',fontsize=11)
            ax.set_xlim(0,639);ax.set_ylim(479,0);ax.set_xticks([]);ax.set_yticks([])
handles,labels=axes[0,0].get_legend_handles_labels()
fig.legend(handles,labels,loc='lower center',ncol=1,fontsize=12)
fig.suptitle('Frozen camera view: stereo obstruction and retained correspondence\nOrange/green are OFFLINE renderer-depth measurements; red is known base_link CAD',fontsize=14)
fig.tight_layout(rect=(0,.06,1,.95));fig.savefig(H/'stereo_visibility_examples.png',dpi=150);plt.close(fig)

whole=json.loads((H/'overlap_whole_sequences.json').read_text())
fig,ax=plt.subplots(figsize=(12,7));names=[];values=[]
for label in ('e1','hard_0_e1_0003','hard_0_e1_0004','direct_e1'):
    for eye in ('left','right'):
        c=whole[label]['totals'][eye]['front'];n=c['source']
        names.append(label+' / '+eye+f' (n={n:,})')
        values.append(np.array([n-c['projected_inside'],c['projected_inside']-c['other_cad_visible'],c['other_cad_visible']-c['same_surface'],c['same_surface']])/max(n,1)*100)
values=np.array(values);offset=np.zeros(len(names));colors=['#b7c3d1','#65738c','#f5ba5b','#4ea467']
for j,title in enumerate(('Outside other image','Inside image, base_link CAD blocked','Other visible, different/invalid surface','Same surface in both eyes')):
    ax.barh(np.arange(len(names)),values[:,j],left=offset,label=title,color=colors[j]);offset+=values[:,j]
ax.set_yticks(np.arange(len(names)),names);ax.invert_yaxis();ax.set_xlim(0,100);ax.set_xlabel('Fraction of measured source-eye front obstacle pixels (%)')
ax.set_title('Complete synchronized sequences, including terminal capture\nSampling every 4 pixels; repeated views are pixel-time observations, not independent obstacles')
ax.legend(loc='lower center',bbox_to_anchor=(.5,-.28),ncol=2);fig.tight_layout();fig.savefig(H/'common_view_breakdown.png',dpi=150);plt.close(fig)

audits=json.loads((H/'audit.json').read_text());audits.update(json.loads((H/'audit_direct.json').read_text()))
fig,axes=plt.subplots(1,2,figsize=(13,5));names=list(audits);raw=[];kept=[];counts=[]
for name,item in audits.items():
    raw.append(100*item['raw_bodyband_error']['within_0p3m'])
    value=item['validated_bodyband_error']['within_0p3m'];kept.append(np.nan if value is None else 100*value)
    counts.append(item['validated_bodyband_error']['n'])
x=np.arange(len(names));axes[0].bar(x-.16,raw,width=.32,label='Old range-only check',color='#c18880');axes[0].bar(x+.16,kept,width=.32,label='New endpoint checks',color='#5385aa')
axes[0].axhline(90,ls='--',color='black',label='Frozen 90% gate');axes[0].set_ylim(0,110);axes[0].set_ylabel('3D error <= 0.3 m (%)');axes[0].legend(fontsize=8)
for i,(v,n) in enumerate(zip(kept,counts)):
    if np.isnan(v):axes[0].text(i,5,'no points',ha='center',rotation=90,fontsize=8)
    else:axes[0].text(i+.16,v+2,f'n={n}',ha='center',fontsize=8)
axes[0].set_title('Invalid truth counts as error; sparse 1-point result is not coverage')
axes[1].bar(x,[audits[n]['max_front_cells'] for n in names],color='#5385aa');axes[1].axhline(6,ls='--',color='black');axes[1].set_ylabel('Max distinct front cells, 0.1 m grid');axes[1].set_title('No sequence met the two-frame obstacle-event rule')
for ax in axes:ax.set_xticks(x,names,rotation=30,ha='right')
fig.tight_layout();fig.savefig(H/'correspondence_gate.png',dpi=150);plt.close(fig)
