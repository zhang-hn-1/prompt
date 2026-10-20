"""GT-only complete BEV scoring. Called only after prediction seal validation."""
from common import *
import numpy as np
from scipy.spatial import cKDTree

def cells(world,origin,res=.1):
 c=np.floor((world[:,:2]-origin)/res+.5).astype(np.int64)
 return np.unique(c,axis=0)
def match(pred,true,res=.1):
 # Same fixed .15m cell-centre tolerance both directions; empty prediction retains FN.
 if len(pred) and len(true):
  pd=cKDTree(true*res).query(pred*res)[0];td=cKDTree(pred*res).query(true*res)[0]
 else:pd=np.full(len(pred),np.inf);td=np.full(len(true),np.inf)
 return dict(predicted_cells=len(pred),true_visible_cells=len(true),correct_predicted_cells=int((pd<=.15+1e-10).sum()),covered_true_cells=int((td<=.15+1e-10).sum()),position_errors_m=pd[np.isfinite(pd)].tolist())
def truth_frame(f,mask,origin):
 z=f['left_depth_m'];y,x=np.indices(z.shape);valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask.astype(bool);T=f['left_camera_usd_world'];rays=np.stack([(x[valid]-319.5)/584.8888888888889,(y[valid]-239.5)/584.8888888888889,np.ones(valid.sum())],1);world=T[:3,3]+(rays*z[valid,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 # Target is visible GT body-band surfaces, not free volume and not predicted semantics.
 hazard=(world[:,2]>.05)&(world[:,2]<=1.5);ground=np.abs(world[:,2])<=.03
 return cells(world[hazard],origin),cells(world[ground],origin),dict(full_image_pixels=z.size,finite_left_range_unmasked_pixels=int(valid.sum()),visible_hazard_pixel_instances=int(hazard.sum()),visible_ground_pixel_instances=int(ground.sum()))
def main():
 seal=json.loads((D/'prediction_seal.json').read_text());assert not seal['truth_read']
 for part in ['inputs','outputs','sources']:
  assert all(sha(p)==h for p,h in seal[part].items())
 result={}
 for name,(R,cp) in GROUPS.items():
  cfg=json.loads(cp.read_text());mask=np.load(cfg['static_masks'])['left'];origin=np.load(R/'maps/frame_000.npz')['origin'];totals=[np.zeros(5,np.int64),np.zeros(5,np.int64)];errs=[[],[]];frames=[]
  for p in sorted((R/'source_predictions').glob('*.npz')):
   f=np.load(R/'frames'/p.name);target,ground,coverage=truth_frame(f,mask,origin);arms=[]
   for j,sp in enumerate([R/'partial_observations'/p.name,D/'predictions'/name/('surface_'+p.stem.split('_')[1]+'.npz')]):
    a=np.load(sp);pred=cells(a['world'],origin);v=match(pred,target);ground_false=int(((cKDTree(ground*.1).query(pred*.1)[0]<=.15+1e-10)&((cKDTree(target*.1).query(pred*.1)[0]>.15+1e-10) if len(target) else np.ones(len(pred),bool))).sum()) if len(ground) and len(pred) else 0;totals[j]+=np.array([v['predicted_cells'],v['true_visible_cells'],v['correct_predicted_cells'],v['covered_true_cells'],ground_false]);errs[j].extend(v.pop('position_errors_m'));v['ground_target_false_cells']=ground_false;arms.append(v)
   frames.append(dict(frame=p.name,time_s=float(f['time_s']),coverage=coverage,arms=arms))
  arms=[]
  for a,e in zip(totals,errs):
   n,t,tp,covered,ground=map(int,a);arms.append(dict(predicted_cell_instances=n,true_visible_cell_instances=t,correct_predicted=tp,covered_true=covered,precision=tp/max(1,n),recall=covered/max(1,t),position_p90_m=float(np.quantile(e,.9)) if e else None,ground_false_cells=ground,ground_false_prediction_fraction=ground/max(1,n)))
  c=arms[1];gates=dict(precision=c['precision']>=.95,recall=c['recall']>=.8,location=c['position_p90_m'] is not None and c['position_p90_m']<=.15,ground_false=c['ground_false_prediction_fraction']<=.01);result[name]=dict(arms=arms,gates=gates,frames=frames);print(name,gates,flush=True)
 save('bev_result.json',dict(groups=result,pass_gate=all(all(x['gates'].values()) for x in result.values()),scope='All current left-visible GT finite .5–3m nonstatic-CAD bodyband surfaces; cell instance counts, tolerance .15m, not full scene recall or observed free. GT height target only, fixed original prediction consumer unchanged. Ground-only false cells exclude cells within .15m of visible hazard target; this is spatial ground-only shadow, not pixel semantic ground false rate. Exact source point ground false rate remains separate in result.json. All groups development only.'))
if __name__=='__main__':main()
