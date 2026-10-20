from pathlib import Path
import numpy as np,json
D=Path(__file__).resolve().parent
for arm in ['baseline_e1','candidate_e1']:
 R=D/(arm+'_run')
 if not (R/'termination.json').exists():continue
 errors=[];predn=0;finite=0;total=0;nohit=0
 for p in sorted((R/'source_predictions').glob('*.npz')):
  a=np.load(p);f=np.load(R/'frames'/p.name);z=a['depth_m'];gt=f['left_depth_m'];keep=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3);ok=keep&np.isfinite(gt)&(gt>0);predn+=int(keep.sum());finite+=int(ok.sum());total+=z.size;nohit+=int((keep&np.isposinf(gt)).sum());errors.append(np.abs(z[ok]-gt[ok]))
 e=np.concatenate(errors);r=dict(total_source_image_pixels=total,predicted_confident_range_pixels=predn,finite_scored_pixels=finite,conditional_scoring_coverage=finite/max(1,predn),full_image_scored_fraction=finite/max(1,total),predicted_on_renderer_nohit_pixels=nohit,p50=float(np.quantile(e,.5)),p90=float(np.quantile(e,.9)),p95=float(np.quantile(e,.95)),scope='181 source images; confident0.5-3m predictions; renderer nohit not physical free; instance counts not unique coverage or recall')
 (D/(arm+'_depth_coverage.json')).write_text(json.dumps(r,indent=2)+'\n');print(arm,r)
