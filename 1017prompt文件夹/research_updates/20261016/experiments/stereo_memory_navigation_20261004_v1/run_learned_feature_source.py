"""One presealed learned-source diagnostic; predictions finish before labels."""
from pathlib import Path
import datetime,json,hashlib,time
import numpy as np
import torch
from learned_feature_models import FrozenLearnedFeatures
from learned_correspondence import predict_correspondences
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
def main():
    protocol=json.loads((H/'learned_feature_protocol.json').read_text())
    for section in ['source_sha256','input_sha256']:
        for p,v in protocol[section].items():assert sha(p)==v,p
    out=H/'learned_feature_runs';out.mkdir(exist_ok=False);torch.manual_seed(0)
    model=FrozenLearnedFeatures('cuda');masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    config=dict(extractor=vars(model.extractor.conf),matcher=vars(model.matcher.conf),device=model.device,resize=None,mask_radius_native_px=72)
    # Exact pretrained parameter identity, not just a strict=False loader claim.
    metadata=json.loads((H/'learned_feature_weights.json').read_text());parameter_checks={}
    for name,network,weight in [('superpoint',model.extractor,'superpoint_v1'),('lightglue',model.matcher,'superpoint_lightglue')]:
        state=torch.load(metadata['weights'][weight]['path'],map_location='cpu',weights_only=True)
        if name=='lightglue':
            for i in range(9):
                state={k.replace(f'self_attn.{i}',f'transformers.{i}.self_attn').replace(f'cross_attn.{i}',f'transformers.{i}.cross_attn'):v for k,v in state.items()}
        n=0
        for k,p in network.named_parameters():assert k in state and torch.equal(p.cpu(),state[k]),k;n+=1
        parameter_checks[name]=n
    (out/'model_configuration.json').write_text(json.dumps(dict(config=config,pretrained_parameters_exact=parameter_checks),indent=2)+'\n')
    rows=[]
    for sequence,group in protocol['groups'].items():
        directory=out/sequence;directory.mkdir();cache={};match_cache={}
        def read_pair(frame):
            if frame in cache:return cache[frame]
            prefix=H/'learned_feature_inputs'/sequence
            with np.load(prefix/f'rgb_{frame:03d}.npz') as rgb,np.load(prefix/f'known_pose_{frame:03d}.npz') as known:
                assert set(rgb.files)=={'left_rgb','right_rgb','time_s'} and set(known.files)=={'left_camera_usd_world','right_camera_usd_world','time_s'}
                timestamp=float(rgb['time_s']);assert timestamp==float(known['time_s']);pair=[];saved={}
                for eye in ['left','right']:
                    feature=model.extract(rgb[eye+'_rgb'],masks[eye]);view=dict(feature,pose=known[eye+'_camera_usd_world'].copy(),frame=frame,time_s=timestamp,eye=eye);pair.append(view)
                    for k in ['xy','desc','scores','original_indices','pose','detected_before_mask']:saved[eye+'_'+k]=view[k]
            np.savez_compressed(directory/f'features_{frame:03d}.npz',**saved,time_s=timestamp);cache[frame]=pair;return pair
        def provider(a,b):
            key=(a['frame'],a['eye'],b['frame'],b['eye'])
            if key not in match_cache:match_cache[key]=model.pairs(a,b)
            return match_cache[key].copy()
        for sample in group['samples']:
            start=time.perf_counter();frame=sample['frame'];pair=read_pair(frame);history=[]
            for old in sample['past_frames']:history.extend(read_pair(old))
            current=predict_correspondences(pair[0],[pair[1]],K,provider);causal=predict_correspondences(pair[0],[pair[1]]+history,K,provider)
            record=dict(sequence=sequence,frame=frame,time_s=pair[0]['time_s'],current=current,causal=causal,seconds=time.perf_counter()-start,labels_not_loaded=True,certified_free=False,pose_layer='known_pose',control_consumed=False)
            path=directory/f'prediction_{frame:03d}.json';path.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
            rows.append(dict(sequence=sequence,frame=frame,path=str(path),sha256=sha(path),queries=current['query_count'],current_points=len(current['points']),causal_points=len(causal['points']),seconds=record['seconds']));print(sequence,frame,current['query_count'],len(current['points']),len(causal['points']),flush=True)
    assert len(rows)==8
    (out/'manifest.json').write_text(json.dumps(dict(saved_utc=now(),labels_not_loaded=True,rows=rows,source_sha256=protocol['source_sha256'],input_sha256=protocol['input_sha256'],whole_research_complete=False),indent=2)+'\n');print('LEARNED_FEATURE_PREDICTIONS_COMPLETE',now(),flush=True)
if __name__=='__main__':main()
