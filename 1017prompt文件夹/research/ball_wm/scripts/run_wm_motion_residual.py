"""One frozen-feature residual training run, whole-sequence historical evaluation."""
from pathlib import Path
import argparse, hashlib, json, random, time, traceback
import numpy as np
import torch
from training.ball_wm_dataset import BallWMDataset,_load_episode,_load_image,_normalize_image,_proprio17
from training.joint_jepa import JointModelConfig,build_joint_model
from training.ablation_protocol import scene_indices
from training.motion_residual import CVResidualPredictor,build_window,motion_loss
from scripts.evaluate_wm_diverse_prediction import pose_errors,summarize_rows


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def summarize(rows):
    result=summarize_rows(rows)
    diag=result['action_diagnostics']
    diag['prefix_pose_max_abs_delta']=diag.pop('prefix_latent_max_abs_delta')
    return result


def tags(window):
    req=window['requests'].numpy().reshape(-1,2)
    last=window['past_requests'][-1,-2:].numpy()
    changes=np.abs(np.diff(np.vstack([last,req]),axis=0))
    changed=bool(np.any(changes[:,0]>=.1-1e-6) or np.any(changes[:,1]>=np.deg2rad(5)-1e-6))
    velocity=float(window['prior'][0,0])/.1
    braking=bool(abs(velocity)>.05 and np.any((req[:,0]*velocity<0)|(np.abs(req[:,0])<.05)))
    return {'request_change':changed,'steady_request':not changed,'brake_or_reverse_request':braking,
            'steering_request':bool(np.any(np.abs(req[:,1])>=np.deg2rad(10)))}


@torch.inference_mode()
def evaluate(model,tensors,indices,metadata,batch_size=64):
    model.eval(); rows=[]
    for offset in range(0,len(indices),batch_size):
        selected=indices[offset:offset+batch_size]
        b={k:v[selected] for k,v in tensors.items()}
        p=model(b); alt={k:v for k,v in b.items()};alt['requests']=-b['requests']; neg=model(alt)
        alt['requests']=b['requests'].clone();alt['requests'][:,3:]*=-1; suffix=model(alt)
        prefix=(p[:,:3]-suffix[:,:3]).abs().flatten(1).amax(1).cpu().numpy()
        p,neg,target,prior=p.cpu().numpy(),neg.cpu().numpy(),b['target'].cpu().numpy(),b['prior'].cpu().numpy()
        for j,index in enumerate(selected):
            zero=np.zeros((5,4));zero[:,3]=1
            scored={}
            for step in [1,3,5]:
                scored[f'{step*.1:.1f}']={name:pose_errors(arr[step-1],target[j,step-1]) for name,arr in [('model',p[j]),('constant_velocity',prior[j]),('zero_motion',zero)]}
                scored[f'{step*.1:.1f}']['recursive_occupied_bev']=None
            differences=[pose_errors(neg[j,k],p[j,k]) for k in [0,2,4]]
            rows.append({**metadata[index],'per_horizon':scored,'action_diagnostics':{
                'prefix_latent_max_abs_delta':float(prefix[j]),
                'negated_request_pose_xy_change_m':float(np.mean([x['displacement_error_m'] for x in differences])),
                'negated_request_pose_yaw_change_rad':float(np.mean([x['yaw_abs_error_rad'] for x in differences])),
                'request_change_l1_mean':float((2*b['requests'][j]).abs().mean().cpu())}})
    per_episode=[]
    for ep in sorted({r['episode'] for r in rows}):
        subset=[r for r in rows if r['episode']==ep]
        per_episode.append({'episode':ep,'scene':subset[0]['scene'],'window_start_frames':[r['start_frame'] for r in subset],**summarize(subset)})
    categories={}
    for category in ['request_change','steady_request','brake_or_reverse_request','steering_request']:
        subset=[r for r in rows if r['tags'][category]]
        categories[category]=summarize(subset) if subset else None
    return {'aggregate':summarize(rows),'per_episode':per_episode,'categories':categories,'rows':rows,
        'bev':'No new BEV predictor; frozen visual backbone supplies observed features only.',
        'categories_note':'Overlapping request-defined groups, not proof of actual stop/reversal/turn completion.'}


def run(args):
    out=args.run_dir.resolve(); protocol=json.loads((out/'protocol.json').read_text())
    assert not (out/'status.json').exists(), 'Refuse to overwrite prior attempt'
    status={'status':'running','phase':'integrity','started_epoch':time.time()};dump(out/'status.json',status)
    def phase(name):status['phase']=name;dump(out/'status.json',status);print(name,flush=True)
    try:
        root=Path(protocol['root']);old=Path(protocol['baseline_run']);data=Path(protocol['data_root'])
        for p,h in protocol['frozen_sources'].items():assert sha(root/p)==h,p
        assert sha(protocol['feature_checkpoint'])==protocol['feature_checkpoint_sha256']
        fingerprints=json.loads((old/'dataset_sha256.json').read_text())['files']
        changed=[p for p,h in fingerprints.items() if sha(data/p)!=h];assert not changed
        dump(out/'data_integrity.json',{'files':len(fingerprints),'changed':changed})
        source=BallWMDataset(data,history_frames=5,rollout_horizon=5,image_size=(224,224),action_source='commanded_action',proprio_velocity_frame='base')
        split=json.loads(Path(protocol['scene_split']).read_text());partitions=scene_indices(source.sample_scenes,split)
        device=torch.device(protocol['device'])
        checkpoint=torch.load(protocol['feature_checkpoint'],map_location='cpu',weights_only=False)
        encoder=build_joint_model(JointModelConfig(**checkpoint['config'])).to(device).eval()
        encoder.load_state_dict(checkpoint['model']);encoder.requires_grad_(False)
        phase('cache observed-frame features with frozen encoder')
        episode_embeddings=[]
        for ep in source.episode_paths:
            loaded=_load_episode(str(ep));states=loaded['states'];vectors=[]
            with torch.inference_mode():
                for start in range(0,loaded['num_frames'],16):
                    frames=list(range(start,min(start+16,loaded['num_frames'])))
                    inputs={f'pixels_{side}':torch.stack([_normalize_image(_load_image(ep/f'{side}_{frame:06d}.png',(224,224))) for frame in frames]).unsqueeze(0).to(device) for side in ['left','right']}
                    ids=[5*(f+1)-1 for f in frames]
                    inputs['proprio']=torch.from_numpy(np.stack([_proprio17(states,i,velocity_frame='base') for i in ids])).unsqueeze(0).to(device)
                    inputs['action']=torch.from_numpy(np.stack([states['commanded_action'][i-4:i+1].reshape(10) for i in ids]).astype(np.float32)).unsqueeze(0).to(device)
                    vectors.append(encoder.encode(inputs)['emb'][0].cpu().numpy())
            episode_embeddings.append(np.concatenate(vectors))
            print('CACHED',str(ep),len(episode_embeddings[-1]),flush=True)
        del encoder;torch.cuda.empty_cache()
        phase('build causal windows and verify zero-residual baseline')
        windows=[];metadata=[]
        for ep_index,start in source.samples:
            ep=source.episode_paths[ep_index];loaded=_load_episode(str(ep))
            w=build_window(loaded['states'],episode_embeddings[ep_index],start);windows.append(w)
            metadata.append({'episode':str(ep.resolve()),'scene':loaded['metadata']['scene'],'start_frame':int(start),'tags':tags(w)})
        tensors={k:torch.stack([w[k] for w in windows]).to(device) for k in windows[0]}
        torch.save({'tensors':{k:v.cpu() for k,v in tensors.items()},'metadata':metadata,'partitions':partitions,'feature_checkpoint_sha256':protocol['feature_checkpoint_sha256']},out/'feature_windows.pt')
        random.seed(protocol['seed']);np.random.seed(protocol['seed']);torch.manual_seed(protocol['seed']);torch.cuda.manual_seed_all(protocol['seed'])
        model=CVResidualPredictor().to(device)
        with torch.no_grad():initial_delta=max(float((model({k:v[i:i+64] for k,v in tensors.items()})-tensors['prior'][i:i+64]).abs().max()) for i in range(0,len(windows),64))
        assert initial_delta==0
        initial={k:v.detach().cpu().clone() for k,v in model.state_dict().items()};torch.save(initial,out/'initial_weights.pt')
        dump(out/'initialization_check.json',{'max_pose_delta_from_cv':initial_delta,'windows':len(windows),'split_windows':list(map(len,partitions)),'trainable_parameters':sum(p.numel() for p in model.parameters()),'initial_weights_sha256':sha(out/'initial_weights.pt')})
        optimizer=torch.optim.AdamW(model.parameters(),lr=protocol['lr'],weight_decay=protocol['weight_decay'])
        phase('train one residual candidate for fixed 10 epochs')
        started=time.time();history=[];steps=0
        for epoch in range(protocol['epochs']):
            model.train();order=np.random.permutation(partitions[0]);total=0
            for offset in range(0,len(order),protocol['batch_size']):
                ids=order[offset:offset+protocol['batch_size']].tolist();b={k:v[ids] for k,v in tensors.items()}
                optimizer.zero_grad();loss=motion_loss(model(b),b['target']);assert torch.isfinite(loss)
                loss.backward();assert all(p.grad is None or torch.isfinite(p.grad).all() for p in model.parameters())
                optimizer.step();steps+=1;total+=float(loss.detach())*len(ids)
            model.eval();validation=0
            with torch.no_grad():
                for offset in range(0,len(partitions[1]),64):
                    ids=partitions[1][offset:offset+64];b={k:v[ids] for k,v in tensors.items()}
                    validation+=float(motion_loss(model(b),b['target']))*len(ids)
            row={'epoch':epoch,'train_motion_loss':total/len(partitions[0]),'val_motion_loss':validation/len(partitions[1]),'optimizer_steps':steps}
            history.append(row);torch.save({'model':model.state_dict(),'optimizer':optimizer.state_dict(),'epoch':epoch,'metrics':row,'protocol':protocol},out/f'residual_epoch{epoch:02d}.pt')
            dump(out/'training_history.json',history);print(json.dumps(row),flush=True)
        selected=min(history,key=lambda x:(x['val_motion_loss'],x['epoch']))
        path=out/f"residual_epoch{selected['epoch']:02d}.pt";saved=torch.load(path,map_location=device,weights_only=False);model.load_state_dict(saved['model'])
        assert any(not torch.equal(v.cpu(),initial[k]) for k,v in model.state_dict().items())
        dump(out/'checkpoint_selection.json',{'rule':protocol['selection'],'selected':selected,'checkpoint':str(path),'sha256':sha(path),'weights_changed':True,'test_used_for_selection':False,'training_seconds':time.time()-started})
        phase('evaluate selected residual on historical whole sequences')
        for name,ids in zip(['val','test'],partitions[1:]):
            selected_ids=[i for i in ids if source.samples[i][1]%5==0]
            report=evaluate(model,tensors,selected_ids,metadata)
            report.update(split=name,checkpoint=str(path),checkpoint_sha256=sha(path),historical_regression=True)
            previous=json.loads((Path(protocol['coordinate_run'])/f'{name}_prediction.json').read_text())
            for horizon in ['0.1','0.3','0.5']:
                for baseline_name in ['constant_velocity','zero_motion']:
                    assert report['aggregate']['per_horizon'][horizon][baseline_name]==previous['aggregate']['per_horizon'][horizon][baseline_name]
            assert [e['window_start_frames'] for e in report['per_episode']]==[e['window_start_frames'] for e in previous['per_episode']]
            report['baselines_and_windows_unchanged']=True;dump(out/f'{name}_prediction.json',report)
        assert sha(protocol['feature_checkpoint'])==protocol['feature_checkpoint_sha256']
        status.update(status='complete',phase='complete',returncode=0,ended_epoch=time.time());dump(out/'status.json',status)
    except BaseException as e:
        status.update(status='failed',error=f'{type(e).__name__}: {e}',ended_epoch=time.time());dump(out/'status.json',status);raise

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run-dir',type=Path,required=True);run(parser.parse_args())
