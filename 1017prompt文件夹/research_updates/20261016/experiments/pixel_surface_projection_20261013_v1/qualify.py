"""OFFLINE oracle/category consumer eligibility; all authored/renderer input here only."""
from common import *
import numpy as np
from scipy.spatial.transform import Rotation
from projection import backproject,project_world,root_from_camera
from category_projection import raster,point_packet
from surface_identity import categories
from category_memory import CategoryMemory
from scipy.spatial import cKDTree
from rig_adapter import Rig,RigMask

# Reuse evaluation definitions, not rejected model/checkpoint or its targets.
old=(W/'experiments/learned_visible_obstacles_20261013_v1/score.py').read_text();ns={}
exec('import numpy as np\nfrom scipy.ndimage import maximum_filter,distance_transform_edt\n'+old[old.index('def metrics('):old.index('\nrows=[]')],ns);metrics=ns['metrics']

out=D/'labels';out.mkdir(exist_ok=False);cad=RigMask(Rig('forward',.4,.3,.05));inputs={};outputs={};results=[]
for name,(R,cp) in GROUPS.items():
    scene=json.loads((D/'scenes'/f'{name}.json').read_text());cfg=json.loads(cp.read_text());states=json.loads((R/'states.json').read_text());paths=sorted((R/'frames').glob('*.npz'))
    assert [int(p.stem.split('_')[1]) for p in paths]==list(range(0,901,5))
    init=np.load(R/'maps/frame_000.npz');memory=CategoryMemory(init['origin'],init['labels'].shape,float(init['resolution']),name!='H');snapshots={};cfgocc=np.load(cfg['occupancy']);reference=cfgocc['global_origin_xy']+np.argwhere(cfgocc['grid']>0)*float(cfgocc['resolution_m']);reftree=cKDTree(reference)
    frames=[];maps={k:[] for k in ['target','native_oracle','verified_binocular','sgbm_with_oracle_category']};point_errors=[];coverage=[];counts=np.zeros(5,np.int64)
    gd=out/name;gd.mkdir(exist_ok=False)
    for j,p in enumerate(paths):
        i=int(p.stem.split('_')[1]);f=np.load(p);nativepath=R/'known_poses'/p.name;n=np.load(nativepath);sp=R/'source_inputs'/p.name;current=np.load(sp);predpath=R/'source_predictions'/p.name;pred=np.load(predpath)
        t=float(f['time_s']);assert t==float(n['time_s'])==float(current['time_s']) and abs(t-float(states[i]['time_s']))<1e-6
        for eye in ['left','right']:assert np.array_equal(f[eye+'_rgb'],current[eye+'_rgb'])
        root=f['root_position_world_m'];q=f['root_quaternion_world_wxyz'];rr=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();yaw=np.arctan2(rr[1,0],rr[0,0]);nr,nroot=root_from_camera(n['left_camera_usd_world']);nyaw=np.arctan2(nr[1,0],nr[0,0]);mask=cad.project(np.array(states[i]['joints']))
        clouds={k:[] for k in maps};clds={k:[] for k in maps};labs={};cov=dict(eligible=0,labelled=0,ground=0,obstacle=0,ambiguous=0,unsupported=0,verified_obstacle_pixels=0,monocular_obstacle_pixels=0)
        for eye,other in [('left','right'),('right','left')]:
            z=f[eye+'_depth_m'];v=np.isfinite(z)&(z>=.5)&(z<=3)&~mask[eye];vy,vx=np.where(v);truth=backproject(vx,vy,z[vy,vx],f[eye+'_camera_usd_world']);outside=np.linalg.norm(truth-root,axis=1)>=.55
            vx=vx[outside];vy=vy[outside];truth=truth[outside];lab,cc=categories(truth,scene);pix=np.zeros((480,640),np.uint8);pix[vy,vx]=lab;eligible=np.zeros_like(pix,bool);eligible[vy,vx]=True
            cov['eligible']+=len(truth);cov['labelled']+=int((lab>0).sum())
            for k,value in cc.items():cov[k]+=value
            labs[eye+'_category']=pix;labs[eye+'_eligible']=eligible
            clouds['target'].append(truth);clds['target'].append(lab)
            nw,cl,idx=point_packet(z,pix,n[eye+'_camera_usd_world'],nroot,mask[eye]);clouds['native_oracle'].append(nw);clds['native_oracle'].append(cl)
            ty,tx=idx//640,idx%640;tw=backproject(tx,ty,z.ravel()[idx],f[eye+'_camera_usd_world']);point_errors.append(np.linalg.norm(nw-tw,axis=1))
            u,vo,zo=project_world(truth,f[other+'_camera_usd_world']);inside=np.isfinite(u)&np.isfinite(vo)&(u>=0)&(u<=639)&(vo>=0)&(vo<=479)&(zo>=.5)&(zo<=3)
            ix=np.clip(np.rint(np.nan_to_num(u)).astype(int),0,639);iy=np.clip(np.rint(np.nan_to_num(vo)).astype(int),0,479);od=f[other+'_depth_m'][iy,ix];agree=inside&~mask[other][iy,ix]&np.isfinite(od)&(np.abs(od-zo)<=.03)
            clouds['verified_binocular'].append(truth[agree]);clds['verified_binocular'].append(lab[agree]);cov['verified_obstacle_pixels']+=int((agree&(lab==2)).sum());cov['monocular_obstacle_pixels']+=int((~agree&(lab==2)).sum())
            if eye=='left':
                sz=np.where(pred['confidence'],pred['depth_m'],np.nan);pw,pc,_=point_packet(sz,pix,n[eye+'_camera_usd_world'],nroot,mask[eye]);clouds['sgbm_with_oracle_category'].append(pw);clds['sgbm_with_oracle_category'].append(pc)
        for k in maps:
            pose_root,pose_yaw=(nroot,nyaw) if k in ['native_oracle','sgbm_with_oracle_category'] else (root,yaw)
            maps[k].append(raster(np.concatenate(clouds[k]),np.concatenate(clds[k]),pose_root,pose_yaw))
        memory.update(np.concatenate(clouds['native_oracle']),np.concatenate(clds['native_oracle']),i,t,n['left_camera_usd_world'][:3,3])
        if i in [450,495,500]:
            sf=D/f'{name}_memory_{i:03d}.npz';np.savez_compressed(sf,**memory.snapshot(t));outputs[str(sf)]=sha(sf)
        if i==895:
            sf=D/f'{name}_memory_899.npz';np.savez_compressed(sf,**memory.snapshot(float(np.load(R/'maps/frame_899.npz')['time_s'])));outputs[str(sf)]=sha(sf)
        coverage.append(cov);counts+=np.array([cov['eligible'],cov['labelled'],cov['ground'],cov['obstacle'],cov['ambiguous']+cov['unsupported']])
        lf=gd/p.name;np.savez_compressed(lf,**labs,time_s=t,frame=i);outputs[str(lf)]=sha(lf)
        frames.append(dict(frame=i,time_s=t,coverage=cov))
        for pp in [p,nativepath,sp,predpath]:inputs[str(pp)]=sha(pp)
        if j%45==0:print(name,j+1,'/181 labelled',flush=True)
    arr={k:np.stack(v) for k,v in maps.items()};mf=D/f'{name}_maps.npz';np.savez_compressed(mf,**arr);outputs[str(mf)]=sha(mf)
    pe=np.concatenate(point_errors);met={k:metrics(arr['target'],a) for k,a in arr.items() if k!='target'}
    covfrac=int(counts[1])/max(1,int(counts[0]));badframes=[i*5 for i,c in enumerate(coverage) if c['eligible']>=1000 and c['labelled']<.8*c['eligible']]
    gate=dict(label_coverage=covfrac>=.95,per_frame_coverage=not badframes,oracle_projection=met['native_oracle']['pass_gate'])
    memory_metrics=[]
    for frame in [450,495,500,899]:
        snap=np.load(D/f'{name}_memory_{frame:03d}.npz');ids=np.argwhere(snap['labels']==1);xy=snap['origin']+ids*float(snap['resolution']);dist=reftree.query(xy)[0];age=float(snap['time_s'])-snap['clock'][tuple(ids.T)];memory_metrics.append(dict(frame=frame,risk=len(ids),far015=int((dist>.15).sum()),older10s=int((age>10).sum()),old_far015=int(((age>10)&(dist>.15)).sum())))
    row=dict(group=name,memory_metrics=memory_metrics,epochs=181,counts=dict(eligible=int(counts[0]),labelled=int(counts[1]),ground=int(counts[2]),obstacle=int(counts[3]),unavailable=int(counts[4])),label_coverage=covfrac,label_coverage_failure_frames=badframes,world_point_error_p50_p90_p95=np.quantile(pe,[.5,.9,.95]).tolist() if len(pe) else [None]*3,metrics=met,gates=gate,qualification_pass=all(gate.values()),frames=frames);results.append(row)
    for pp in [R/'states.json',R/'termination.json',R/'task_episode.json',cp,D/'scenes'/f'{name}.json',Path(scene['scene'])]:inputs[str(pp)]=sha(pp)
    print('RESULT',name,covfrac,met['native_oracle'],flush=True)
save('qualification.json',dict(rows=results,qualification_pass=all(r['qualification_pass'] for r in results),fit=0,physical=0,input_sha256=inputs,output_sha256=outputs,source_sha256={str(p):sha(p) for p in [D/'protocol.json',D/'projection.py',D/'category_projection.py',D/'surface_identity.py',D/'category_memory.py',D/'qualify.py']},scope='ideal authored-compatible pixel classes and ideal ranges, offline only; no new predictor/nav; pixel categories independent of reconstructed height'))
