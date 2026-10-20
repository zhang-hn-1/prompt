"""Independent sampled ray/CAD checks and direct RGB/depth/BEV presentation."""
from pathlib import Path
import json, hashlib, time
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from qualify import D,W,INPUT,NAMES,AX,RobotSelfMask,triangles_body

def intersect(tri, direction, near=.01):
    """Moller-Trumbore in optical coordinates; returned parameter is optical Z."""
    v0=tri[:,0];e1=tri[:,1]-v0;e2=tri[:,2]-v0
    p=np.cross(np.broadcast_to(direction,e2.shape),e2);det=np.einsum('ij,ij->i',e1,p)
    valid=np.abs(det)>1e-12;inv=np.zeros_like(det);inv[valid]=1/det[valid]
    u=np.einsum('ij,ij->i',-v0,p)*inv
    q=np.cross(-v0,e1);v=q@direction*inv;t=np.einsum('ij,ij->i',e2,q)*inv
    valid&=(u>=0)&(v>=0)&(u+v<=1)&(t>=near)
    return float(t[valid].min()) if valid.any() else None

def main():
    start=time.monotonic();r=json.loads((D/'geometry_result.json').read_text())
    seal=json.loads((INPUT/'recovery_run/capture_seal.json').read_text())
    local=[np.array(v['local_matrix']) for v in seal['initial_local_identity']]
    F=seal['initial_local_identity'][0]['optics'][0]/18*640
    cad=RobotSelfMask();checks=[];aggregate={k:0 for k in r['aggregate_grid_instances']}
    for row in r['frames']:
        with np.load(INPUT/'recovery_run/frames'/row['frame']) as a, np.load(D/(Path(row['frame']).stem+'_audit.npz')) as audit:
            tri=triangles_body(cad,a['joints_current']);sets=[]
            for j,name in enumerate(NAMES):
                m=audit[name+'_CAD'];assert int(m.sum())==row['arms'][j]['CAD_pixels']
                sets.append(set(map(tuple,audit[name+'_cells'].tolist())))
                optical=(tri-local[j][:3,3])@(local[j][:3,:3]@AX)
                for expected,region in [(True,m),(False,~m)]:
                    inner=cv2.erode(region.astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool)
                    positions=np.argwhere(inner)
                    chosen=positions[np.linspace(0,len(positions)-1,4,dtype=int)]
                    for y,x in chosen:
                        hit=intersect(optical,np.array([(x-319.5)/F,(y-239.5)/F,1.]))
                        z=float(a[name+'_depth_label'][y,x]);finite=np.isfinite(z) and z>0
                        checks.append(dict(frame=row['frame'],camera=name,uv=[int(x),int(y)],
                            raster_masked=expected,ray_hit=hit is not None,
                            CAD_depth_m=hit,rendered_depth_m=z if finite else None,
                            difference_m=abs(hit-z) if hit is not None and finite else None))
            front=sets[0]|sets[1];rear=sets[2]|sets[3];union=front|rear
            current=dict(front_union=len(front),rear_union=len(rear),overlap=len(front&rear),
                all_union=len(union),rear_new=len(rear-front),
                union_near_reference=row['union_near_reference'],union_far_reference=row['union_far_reference'])
            for k,v in current.items():assert v==row[k];aggregate[k]+=v
            assert set(map(tuple,audit['all_union'].tolist()))==union
            assert int(audit['ring_visible'].sum())==row['ring_after_CAD']
    assert aggregate==r['aggregate_grid_instances']
    disagree=[c for c in checks if c['raster_masked']!=c['ray_hit']]
    errors=[c['difference_m'] for c in checks if c['raster_masked'] and c['difference_m'] is not None]
    result=dict(sample_count=len(checks),ray_raster_disagreements=disagree,
        masked_render_depth_count=len(errors),masked_render_depth_error_p90_m=float(np.quantile(errors,.9)),
        masked_render_depth_error_max_m=max(errors),
        all_eight_saved_set_counts_exact=True,wall_s=time.monotonic()-start,samples=checks,
        scope='Four safely interior CAD and four non-CAD pixels per camera per saved pose; selected by CAD only. Not exhaustive ray or renderer certification.')
    (D/'independent_check.json').write_text(json.dumps(result,indent=2)+'\n')
    print({k:v for k,v in result.items() if k not in ['samples','ray_raster_disagreements']},flush=True)
    print('ray disagreements',len(disagree),flush=True)
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False})
    fig,axs=plt.subplots(3,4,figsize=(14,12),constrained_layout=True)
    a=np.load(INPUT/'recovery_run/frames/frame_000.npz');audit=np.load(D/'frame_000_audit.npz')
    names=['前左','前右','后左','后右']
    for i,(name,label) in enumerate(zip(NAMES,names)):
        rgb=np.rot90(a[name+'_rgb']);mask=np.rot90(audit[name+'_CAD'])
        axs[0,i].imshow(rgb);axs[0,i].set_title(label+'：实际 RGB')
        axs[1,i].imshow(rgb);overlay=np.zeros((*mask.shape,4));overlay[mask]=[0,1,1,.65]
        axs[1,i].imshow(overlay);axs[1,i].set_title(f'青色＝CAD遮挡 {100*mask.mean():.2f}%')
        z=np.rot90(a[name+'_depth_label']);depth=np.where(np.isfinite(z)&(z>0),z,np.nan)
        axs[2,i].imshow(depth,cmap='turbo',vmin=0,vmax=3);axs[2,i].set_title('渲染真值深度：0–3 m')
        for j in range(3):axs[j,i].axis('off')
    fig.suptitle('固定首帧 0.5 s：实际画面、机体遮挡与渲染深度',fontsize=19)
    fig.colorbar(plt.cm.ScalarMappable(norm=plt.Normalize(0,3),cmap='turbo'),ax=axs[2,:],orientation='horizontal',shrink=.65,label='渲染深度（m）',pad=.02)
    fig.supxlabel('上排保留完整画面；中排仅叠加青色CAD遮挡；底排白色为无有限标签，超过3 m按色阶上限显示。\n本轮没有模型预测、预测误差或长期地图，相关指标 N/A。',fontsize=11)
    fig.savefig(D/'rgb_CAD_depth.png',dpi=135);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,6),constrained_layout=True)
    origin=audit['origin'];root=audit['root'];reference=audit['reference_xy']-root[:2]
    ax=axs[0];near=np.linalg.norm(reference,axis=1)<=3.6
    ax.scatter(reference[near,1]*-1,reference[near,0],s=8,c='#bbbbbb',label='bodyband真实参照')
    for key,color,label in [('front_union','#1265b0','前方两路 GT支持'),('rear_union','#dc6d1e','后方两路 GT支持')]:
        c=audit[key]*.1+origin-root[:2]
        ax.scatter(-c[:,1],c[:,0],s=28,c=color,label=label)
    ax.scatter(0,0,c='k',marker='^',s=50);ax.set_xlim(-3.5,3.5);ax.set_ylim(-3.5,3.5)
    ax.set_title('首帧：真实可见表面投影（GT-only）');ax.set_xlabel('机体左 ← 横向 −Y（m）→ 右');ax.set_ylabel('前后 X（m）');ax.legend(fontsize=9);ax.set_aspect('equal');ax.grid(alpha=.2)
    ax=axs[1];ring=audit['ring_body'];visible=audit['ring_visible']
    ax.scatter(-ring[~visible,1],ring[~visible,0],s=5,c='#bdbdbd',label='不满足几何可用条件')
    ax.scatter(-ring[visible,1],ring[visible,0],s=5,c='#078787',label='视域/距离/CAD可用')
    ax.scatter(0,0,c='k',marker='^',s=50);ax.set_title(f'2 m环：{visible.sum()}/3600 = {visible.mean()*100:.2f}%')
    ax.set_xlabel('机体左 ← 横向 −Y（m）→ 右');ax.set_ylabel('前后 X（m）');ax.legend(fontsize=9);ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle('当前配置的可见支持与侧向盲区',fontsize=19)
    fig.supxlabel('左：完整有限0.5–3 m、CAD未遮挡、世界高度0.05–1.5 m支持；右：纯几何，未扣场景遮挡。\n771/771为8组格实例距真实参照≤0.15 m，不能当作预测精度或全场景召回。',fontsize=11)
    fig.savefig(D/'BEV_and_ring.png',dpi=150)

if __name__=='__main__':main()
