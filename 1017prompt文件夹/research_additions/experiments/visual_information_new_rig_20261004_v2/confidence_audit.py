"""Exploratory per-image uncertainty ranks; no acceptance denominator filtering."""
import json
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr
from measurement import evaluation_masks,point_errors,stats
H=Path(__file__).resolve().parent
manifest=json.loads((H/'offline_manifest.json').read_text())
mask=np.load(H/'base_link_masks.npz')
y,x=np.mgrid[0:480:4,0:640:4];rays=np.c_[(x.ravel()-319.5)/(16.45/18*640),(y.ravel()-239.5)/(16.45/18*640),np.ones(x.size)]
collected={};correlations=[]
for row in manifest['samples']:
    pred=np.load(H/'predictions/unidepth_v2'/row['sequence']/Path(row['rgb_file']).name);true=np.load(row['truth_file'])
    seq=collected.setdefault(row['sequence'],{'lowest_reported_error_quartile':[],'other_three_quartiles':[],'all_obstacles':[]})
    for eye in ('left','right'):
        d=true[eye+'_depth_m'][::4,::4].ravel();pose=true[eye+'_camera_usd_world'];R=np.c_[pose[:3,0],-pose[:3,1],-pose[:3,2]]
        points=pose[:3,3]+(rays*np.where(np.isfinite(d),d,0)[:,None])@R.T
        _,body=evaluation_masks(d,points,true['root_position_world_m'],mask[eye][::4,::4].ravel())
        error=point_errors(pred[eye+'_depth_m'][::4,::4].ravel(),d,rays)[body]
        score=pred[eye+'_confidence'][::4,::4].ravel()[body]
        seq['all_obstacles'].extend(error.tolist())
        finite=np.isfinite(score);e=error[finite];s=score[finite]
        if len(e)<4:continue
        order=np.argsort(s,kind='stable');n=max(1,len(order)//4)
        seq['lowest_reported_error_quartile'].extend(e[order[:n]].tolist());seq['other_three_quartiles'].extend(e[order[n:]].tolist())
        if len(e)>=100 and np.isfinite(e).all() and np.std(s)>0:correlations.append(float(spearmanr(s,e).statistic))
result=dict(role='exploratory historical development; rank within current image, not calibrated risk probability',selection='lowest reported-error quartile within each eye and frame; no inference or scale fitting',
    sequences={k:{n:stats(v) for n,v in seq.items()} for k,seq in collected.items()},per_image_spearman_n=len(correlations),per_image_spearman_median=None if not correlations else float(np.median(correlations)))
(H/'confidence_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
