"""Immutable-field observed-data stationarity check; no inference-source admission."""
from pathlib import Path
import sys,json,hashlib,time
import numpy as np
import torch
import torch.nn.functional as F
D=Path(__file__).resolve().parent;W=D.parents[1]
c=json.loads((D/'protocol.json').read_text());P=W/c['parent']
sys.path.insert(0,str(P));import compaction as fixed
base=fixed.base
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def render(model,oo,dd,delta):
    offset=torch.stack([delta,torch.zeros_like(delta),torch.zeros_like(delta)])
    pts=oo[:,None,:]+dd[:,None,:]*model.z[None,:,None]+offset
    normalized=(pts-model.centre)/model.half_extent
    values=F.grid_sample(torch.cat([model.density,model.color],1),normalized[None,None],align_corners=True,padding_mode='zeros').squeeze(0).squeeze(1).permute(1,2,0)
    sigma=F.softplus(values[...,0]+model.shift)/model.voxel*(normalized.abs()<=1).all(-1)
    tau=sigma*model.dz*torch.linalg.vector_norm(dd,dim=-1)[:,None]
    weights=torch.exp(-(torch.cumsum(tau,-1)-tau))*(-torch.expm1(-tau))
    T=torch.exp(-tau.sum(-1))
    return (weights[...,None]*torch.sigmoid(values[...,1:])).sum(1)+T[:,None]*model.background
def main():
    start=time.monotonic();identity=json.loads((D/'dispatch_identity.json').read_text())
    for p,h in identity['source_sha256'].items():assert sha(p)==h,p
    for p,h in c['checkpoint_sha256'].items():assert sha(W/p)==h,p
    prior_path=W/c['prior_objective'];assert sha(prior_path)==c['prior_objective_sha256']
    prior=json.loads(prior_path.read_text());assert sha(prior_path.parent/'objective_arrays.npz')==c['prior_objective_arrays_sha256']
    with np.load(prior_path.parent/'objective_arrays.npz') as a:q=a['q'].copy()
    yy,xx=np.indices((480,640));assert np.array_equal(q,np.flatnonzero((yy%16==0)&(xx%16==0)))
    data=json.loads((P/'data_seal.json').read_text());cfg=json.loads((P/'prediction_seal.json').read_text())['parent_effective_configuration']
    records=[(r,base.load(r)) for r in data['rows'] if r['split']=='train'];assert len(records)==8
    ids=torch.as_tensor(q,device='cuda');torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    arrays={'q':q};arms={}
    for name,path in c['checkpoints'].items():
        model=fixed.CompactGrid(cfg,[0.,0.,.75],base.c['data']['background_RGB'])
        state=torch.load(W/path,map_location='cuda',weights_only=True);model.load_state_dict(state)
        for v in model.parameters():v.requires_grad_(False)
        rows=[]
        for i,(row,record) in enumerate(records):
            oo,dd=base.rays(record,ids,data['focal']);observed=record['rgb'][ids].float()/255.
            key=name+'_'+row['name'][:-4];losses={};rgbs={}
            delta=torch.zeros((),device='cuda',requires_grad=True)
            rgb=render(model,oo,dd,delta)
            loss=(rgb.double()-observed.double()).square().sum(-1).mean()
            derivative=torch.autograd.grad(loss,delta)[0]
            assert torch.isfinite(derivative)
            rgbs['0']=rgb.detach().cpu().numpy();losses['0']=float(loss)
            assert abs(losses['0']/3-prior['arms'][name]['per_view'][i]['RGB_MSE'])<2e-7
            with torch.no_grad():
                for value,label in [(-.001,'minus'),(.001,'plus')]:
                    rv=render(model,oo,dd,torch.tensor(value,device='cuda'))
                    rgbs[label]=rv.cpu().numpy()
                    losses[label]=float((rv.double()-observed.double()).square().sum(-1).mean())
            rows.append(dict(input=row['name'],rays=1200,RGB_SSE=losses,derivative_RGB_SSE_per_m=float(derivative),central_difference_per_m=(losses['plus']-losses['minus'])/.002))
            for label,value in rgbs.items():arrays[key+'_'+label]=value
            arrays[key+'_observed']=observed.cpu().numpy()
        aggregates={}
        for aggregation,weights in [('equal_view',np.full(8,1/8)),('training_expectation',np.array(c['aggregations']['training_expectation']))]:
            g=float(np.dot(weights,[r['derivative_RGB_SSE_per_m'] for r in rows]))
            losses={k:float(np.dot(weights,[r['RGB_SSE'][k] for r in rows])) for k in ['minus','0','plus']}
            direction='minus' if g>0 else 'plus'
            decrease=losses['0']-losses[direction];threshold=max(2e-8,1e-5*losses['0'])
            aggregates[aggregation]=dict(RGB_SSE=losses,derivative_RGB_SSE_per_m=g,
                central_difference_per_m=(losses['plus']-losses['minus'])/.002,
                fixed_descent_direction=direction,observed_loss_decrease=decrease,
                corroboration_threshold=threshold,nonstationary_gradient=abs(g)>1e-6,
                descent_corroborates=decrease>threshold,
                stationary_centre_contradicted=bool(abs(g)>1e-6 and decrease>threshold))
        assert not model.optimizer.state
        for k,v in model.state_dict().items():assert torch.equal(v,state[k]),k
        arms[name]=dict(rows=rows,aggregates=aggregates,checkpoint_sha256=sha(W/path));del model
    np.savez_compressed(D/'RGB_arrays.npz',**arrays)
    for p,h in identity['source_sha256'].items():assert sha(p)==h,p
    base.write(D/'result.json',dict(complete=True,arms=arms,source_sha256=identity['source_sha256'],arrays_sha256=sha(D/'RGB_arrays.npz'),
        fit_calls=0,optimizer_steps=0,labels_read=False,parameters_unchanged_exact=True,
        centre_failure_count=sum(x['stationary_centre_contradicted'] for a in arms.values() for x in a['aggregates'].values()),
        interpretation='Observed-RGB stationary-centre condition in one valid spatial direction. Not fullBayesRays or a test of its uncertainty-ranking performance; no Hessian/variance/error interval computed. Nonzero gradient violates stationarity even with a zero-centred quadratic deformationprior. A decrease in RGB loss is not a proven decrease in depth error.',
        wall_s=time.monotonic()-start))
    print(json.dumps({name:arm['aggregates'] for name,arm in arms.items()}),flush=True)
if __name__=='__main__':main()
