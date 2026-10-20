from pathlib import Path
import sys,json,hashlib,datetime
import numpy as np
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;W=D.parents[1];sys.path.insert(0,str(W/'experiments/four_camera_weak_baseline_20261017_v1'))
from source import NAMES,RAYS,AX
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
put=lambda p,v:Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def quant(a):return dict(zip(['P50','P90','P95','max'],map(float,np.quantile(a,[.5,.9,.95,1])))) if len(a) else None
def cells(xy,origin):return np.unique(np.rint((xy-origin)/.1).astype(np.int64),axis=0)
def metric(pred,target):
 a=cKDTree(target*.1).query(pred*.1)[0] if len(pred) and len(target) else np.full(len(pred),np.inf)
 b=cKDTree(pred*.1).query(target*.1)[0] if len(pred) and len(target) else np.full(len(target),np.inf)
 return dict(predicted_cells=len(pred),correct_cells=int((a<=.15).sum()),false_visible_cells=int((a>.15).sum()),target_cells=len(target),covered_cells=int((b<=.15).sum()),precision=float((a<=.15).mean()) if len(a) else None,recall=float((b<=.15).mean()) if len(b) else None)

def main():
 cfg=json.loads((D/'protocol.json').read_text());seal=json.loads((D/'prediction_seal.json').read_text());assert seal['complete'] and len(seal['rows'])==8;assert seal['protocol_sha256']==sha(D/'protocol.json')
 for sec in ['input_sha256','code_sha256']:
  for p,h in cfg[sec].items():assert sha(p)==h,p
 for row in seal['rows']:assert sha(row['path'])==row['sha256']
 lookup={(x['sequence'],x['frame'],x['arm']):x for x in seal['rows']};results=[];agg={k:dict(predicted_cells=0,correct_cells=0,false_visible_cells=0,target_cells=0,covered_cells=0,target_pixels=0,valid_target_pixels=0,axial=[],radial=[]) for k in ['original','baseline','self_supported']}
 for row in cfg['rows']:
  paths={'original':row['prediction_path'],**{k:lookup[row['sequence'],row['frame'],k]['path'] for k in ['baseline','self_supported']}}
  with np.load(paths['self_supported']) as a:origin=a['origin']
  with np.load(row['prediction_path']) as a:CAD={n:a[n+'_CAD'] for n in NAMES}
  targets={};truthdepth={};gtcounts={}
  # First numeric renderer/GT access occurs only after complete predictionseal above.
  with np.load(row['input_rgb_frame']) as a:
   for n in NAMES:
    z=a[n+'_depth_label'];T=a[n+'_camera_usd_world_label'];root=a['root_position_physics_label']
    with np.errstate(invalid='ignore'):
     w=(RAYS*z[...,None])@(T[:3,:3]@AX).T+T[:3,3]
    k=np.isfinite(z)&(z>=.5)&(z<=3)&~CAD[n]&(np.linalg.norm(w-root,axis=2)>=.55)&(w[...,2]>.05)&(w[...,2]<=1.5)
    targets[n]=cells(w[k,:2],origin);truthdepth[n]=(z,k);gtcounts[n]=dict(full_pixels=int(z.size),finite_positive_GT=int((np.isfinite(z)&(z>0)).sum()),invalid_GT_UNKNOWN=int((~(np.isfinite(z)&(z>0))).sum()),target_pixels=int(k.sum()))
  union=np.unique(np.concatenate(list(targets.values())),axis=0);frame=dict(sequence=row['sequence'],frame=row['frame'],time_s=row['time_s'],GT_pixel_counts=gtcounts,arms={});sets={}
  for arm,path in paths.items():
   predxy=[];pe={};ea=[];er=[];tp=vp=0
   with np.load(path) as a:
    for n in NAMES:
     z,k=truthdepth[n];v=a[n+'_valid']&np.isfinite(a[n+'_depth'])&(a[n+'_depth']>=.5)&(a[n+'_depth']<=3);err=abs(z[k&v].astype(float)-a[n+'_depth'][k&v].astype(float));rr=err*np.linalg.norm(RAYS[k&v],axis=1);q=cells(a[n+'_world'][:,:2],origin);m=metric(q,targets[n]);m.update(target_pixels=int(k.sum()),valid_target_pixels=int((k&v).sum()),conditional_axial_error_m=quant(err),conditional_radial_error_m=quant(rr),risk_points=len(a[n+'_world']));pe[n]=m;predxy.append(a[n+'_world'][:,:2]);ea.extend(err.tolist());er.extend(rr.tolist());tp+=int(k.sum());vp+=int((k&v).sum())
   q=cells(np.concatenate(predxy),origin);sets[arm]=set(map(tuple,q.tolist()));m=metric(q,union);m.update(target_pixels=tp,valid_target_pixels=vp,conditional_axial_error_m=quant(ea),conditional_radial_error_m=quant(er));frame['arms'][arm]=dict(union=m,per_eye=pe)
   for key in ['predicted_cells','correct_cells','false_visible_cells','target_cells','covered_cells','target_pixels','valid_target_pixels']:agg[arm][key]+=m[key]
   agg[arm]['axial'].extend(ea);agg[arm]['radial'].extend(er)
  lost=np.array(sorted(sets['baseline']-sets['self_supported']),int).reshape(-1,2);added=np.array(sorted(sets['self_supported']-sets['baseline']),int).reshape(-1,2)
  frame['fixed_difference_attribution']=dict(baseline_cells_lost=metric(lost,union),self_supported_cells_added=metric(added,union),scope='Newreference arm difference vsvisibleGT target, not staticphysical false/true source witness; not v117padding sensitivity teacher retry.')
  results.append(frame);print('SCORED',row['sequence'],row['frame'],flush=True)
 for arm,a in agg.items():
  a['precision']=a['correct_cells']/a['predicted_cells'] if a['predicted_cells'] else None;a['recall']=a['covered_cells']/a['target_cells'] if a['target_cells'] else None;a['conditional_axial_error_m']=quant(a.pop('axial'));a['conditional_radial_error_m']=quant(a.pop('radial'))
 u,m=agg['baseline'],agg['self_supported'];reduction=1-m['false_visible_cells']/u['false_visible_cells'] if u['false_visible_cells'] else None;rdrop=m['recall']-u['recall'] if u['recall'] is not None and m['recall'] is not None else None
 gates=dict(false_visible_cells_reduction_ge20pct=reduction is not None and reduction>=.2,pooled_precision_loss_le1pp=m['precision'] is not None and u['precision'] is not None and m['precision']>=u['precision']-.01,pooled_recall_loss_le5pp=rdrop is not None and rdrop>=-.05,all_frames_recall_loss_le5pp=all(x['arms']['self_supported']['union']['recall'] is not None and x['arms']['self_supported']['union']['recall']>=x['arms']['baseline']['union']['recall']-.05 for x in results),nonempty_all_target_frames=all(x['arms']['self_supported']['union']['predicted_cells']>0 for x in results if x['arms']['self_supported']['union']['target_cells']>0))
 for sec in ['input_sha256','code_sha256']:
  for p,h in cfg[sec].items():assert sha(p)==h,p
 put(D/'result.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),complete=True,decision='LOCAL_FOUR_FRAME_SELF_SUPPORT_GATES_PASSED_NOT_FULL_SOURCE' if all(gates.values()) else 'REJECT_FIXED_SELF_SUPPORT_REPAIR_BEFORE_FULL_SEQUENCE_OR_NAV',gates=gates,aggregate=agg,rows=results,false_visible_cell_reduction=reduction,recall_change=rdrop,prediction_seal_sha256=sha(D/'prediction_seal.json'),GT_scope='Samefullfinitevisibleheight-risk target/domain as current weaknavigation scorer. FiniteGTcondition remains, invalidGTUNKNOWN; no ground/RGBsemantics, staticoccupancytruth, independence, actualmap/memory/nav or3Dsafety certificate.',old_strict_gate_pass=False,actual_input_frames=4,per_eye_fullpixels=307200,total_fullpixels=4915200,fit=0,physics=0,navigation=0))
if __name__=='__main__':main()
