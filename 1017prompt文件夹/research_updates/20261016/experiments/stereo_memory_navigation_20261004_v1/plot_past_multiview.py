"""All four preregistered source-query frames, not selected successful examples."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
H=Path(__file__).resolve().parent

def main():
    result=json.loads((H/'past_multiview_audit.json').read_text());data=np.load(H/'past_multiview_measurements.npz')
    fig,axes=plt.subplots(4,3,figsize=(13,13.5),constrained_layout=True)
    visibility_colors=['#eee','#777','#a777bf','#222','#efbb4b','#d86241','#38a681']
    support_colors=['#eee','#38a681','#efaa39','#428ccc','#9965aa']
    for i,row in enumerate(result['rows']):
        frame=row['frame'];p=f'frame_{frame:03d}__';xy=data[p+'source_pixels_yx'];y,x=xy.T
        rgb=np.load(H/f'capture_transaction_development/hard_0_e1_0004/online_inputs/frame_{frame:03d}.npz')['left_rgb']
        vr=np.zeros((480,640),np.uint8);vr[y,x]=data[p+'current_right_status']+1
        support=np.zeros((480,640),np.uint8)
        current=data[p+'current_stereo'];past=data[p+'past_stereo'];single=data[p+'past_any_eye']
        label=np.where(current,1,np.where(past,2,np.where(single,3,4)))
        support[y,x]=label
        axes[i,0].imshow(rgb);axes[i,1].imshow(vr,cmap=ListedColormap(visibility_colors),vmin=0,vmax=6)
        axes[i,2].imshow(support,cmap=ListedColormap(support_colors),vmin=0,vmax=4)
        axes[i,0].set_title(f'Frame {frame:03d}, t={row["time_s"]:.1f}s: actual left RGB',fontsize=10)
        axes[i,1].set_title('Current right visibility, mapped to left queries',fontsize=10)
        axes[i,2].set_title(f'Only earlier frames {row["loaded_history_frames"]}\nreliable frozen RGB sources: 0',fontsize=10)
        for ax in axes[i]:ax.axis('off')
    handles=[Patch(color=support_colors[i],label=l) for i,l in enumerate(['not queried','current stereo visible','past stereo only','past single-eye only','no earlier visible source'])]
    fig.legend(handles=handles,loc='outside lower center',ncol=3,fontsize=9)
    fig.suptitle('Offline labels on same full source-sphere queries; past visibility is not reliable RGB memory')
    fig.savefig(H/'figures/past_multiview_all_frames.png',dpi=130);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5.5),constrained_layout=True)
    rows=result['rows'];x=np.arange(4);n=np.array([r['metrics']['n'] for r in rows])
    current=np.array([r['metrics']['current_stereo_n'] for r in rows]);gain=np.array([r['metrics']['stereo_past_gain_n'] for r in rows])
    axes[0].bar(x,current/n*100,label='current stereo',color='#38a681')
    axes[0].bar(x,gain/n*100,bottom=current/n*100,label='strict earlier stereo gain',color='#efaa39')
    axes[0].bar(x,(n-current-gain)/n*100,bottom=(current+gain)/n*100,label='no stereo source',color='#bbb')
    axes[0].set_ylim(0,105);axes[0].set_xticks(x,[str(r['frame']) for r in rows]);axes[0].set_xlabel('Archived query frame (no future images)')
    axes[0].set_ylabel('% of full independent source queries');axes[0].legend(fontsize=8,loc='upper left')
    status=np.concatenate([data[f'frame_{r["frame"]:03d}__current_right_status'] for r in rows])
    unavailable=np.concatenate([~data[f'frame_{r["frame"]:03d}__current_stereo'] for r in rows])
    history=np.concatenate([data[f'frame_{r["frame"]:03d}__past_stereo'] for r in rows])
    selected=[unavailable&(status==k) for k in [1,2,4,5]]
    total=np.array([s.sum() for s in selected]);recovered=np.array([(s&history).sum() for s in selected]);x=np.arange(4)
    axes[1].bar(x,recovered,label='earlier stereo source',color='#efaa39')
    axes[1].bar(x,total-recovered,bottom=recovered,label='not recovered by earlier stereo',color='#bbb')
    axes[1].set_xticks(x,['right outside','right body','right inconsistent','right visible /\nleft unavailable'],rotation=10)
    axes[1].set_ylabel('Current-stereo-unavailable query count');axes[1].legend(fontsize=8)
    for k,v in enumerate(total):axes[1].text(k,v+1000,str(v),ha='center',fontsize=9)
    fig.suptitle('Current stereo 8,379 → causal union 24,298 / 73,383; reliable frozen RGB union remains 0')
    fig.savefig(H/'figures/past_multiview_support_counts.png',dpi=140);plt.close(fig)
    print('SAVED all4 queryframes and causal support-count figure')

if __name__=='__main__':main()
