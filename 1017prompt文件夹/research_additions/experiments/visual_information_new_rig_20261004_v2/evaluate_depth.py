"""Score predictions only after they have been persisted; GT is never a model input."""
import json
from pathlib import Path
import numpy as np
from measurement import point_errors,stats,evaluation_masks,subgroup_pass
H=Path(__file__).resolve().parent;E=H.parent
FOCAL=16.45/18*640
y,v=np.mgrid[0:480:4,0:640:4];rays=np.column_stack(((v.ravel()-319.5)/FOCAL,(y.ravel()-239.5)/FOCAL,np.ones(v.size)))
mask=np.load(H/'base_link_masks.npz')
manifest=json.loads((H/'offline_manifest.json').read_text())
results={}
for model in ('da3_metric','unidepth_v2'):
    collected={};missing=[];frames=[]
    for row in manifest['samples']:
        f=H/'predictions'/model/row['sequence']/Path(row['rgb_file']).name
        if not f.exists():missing.append(str(f));continue
        with np.load(f) as pred,np.load(row['truth_file']) as truth:
            seq=collected.setdefault(row['sequence'],{})
            frame_record={'sequence':row['sequence'],'frame':f.name,'eyes':{}}
            for eye in ('left','right'):
                pose=truth[eye+'_camera_usd_world'];R=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
                depth=truth[eye+'_depth_m'][::4,::4].ravel();points=pose[:3,3]+rays*depth[:,None]@R.T
                true_valid,body=evaluation_masks(depth,points,truth['root_position_world_m'],mask[eye][::4,::4].ravel())
                estimated=pred[eye+'_depth_m'][::4,::4].ravel();error=point_errors(estimated,depth,rays)
                for name,select in [('scene',true_valid),('obstacle',body),('near_obstacle',body&(depth<=1.5)),('far_obstacle',body&(depth>1.5))]:
                    seq.setdefault(name,[]).extend(error[select].tolist())
                frame_record['eyes'][eye]=dict(obstacle=stats(error[body]),near_obstacle=stats(error[body&(depth<=1.5)]))
            frames.append(frame_record)
    result={'model':model,'sample_pairs_completed':len(manifest['samples'])-len(missing),'sample_pairs_expected':len(manifest['samples']),'missing':missing,
            'sequences':{seq:{k:stats(v) for k,v in groups.items()} for seq,groups in collected.items()}}
    result['critical_gates']={seq:subgroup_pass(result['sequences'].get(seq,{}).get('obstacle',stats([]))) for seq in ['e1','hard_0_e1_0003','hard_0_e1_0004','direct_e1']}
    result['accepted']=not missing and all(result['critical_gates'].values())
    if (H/(model+'_inference_records.json')).exists():
        times=json.loads((H/(model+'_inference_records.json')).read_text());a=np.array([v['pair_seconds'] for v in times])
        # First pair includes warm-up and is separately reported.
        b=a[1:] if len(a)>1 else a
        result['latency']=dict(first_pair_s=float(a[0]),pair_p50_s=float(np.median(b)),pair_p95_s=float(np.percentile(b,95)),n=len(a),ten_hz_p95_met=bool(np.percentile(b,95)<=.1))
    (H/(model+'_assessment.json')).write_text(json.dumps(result,indent=2)+'\n');results[model]=result
(H/'depth_assessment.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps({m:{'pairs':r['sample_pairs_completed'],'accepted':r['accepted'],'gate':r['critical_gates'],'latency':r.get('latency'),'obstacles':{s:g['obstacle'] for s,g in r['sequences'].items()}} for m,r in results.items()},indent=2))
