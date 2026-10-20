"""One saved-array failure attribution, no model invocation or thresholds search."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
load=lambda p:json.loads(Path(p).read_text())
def put(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
if __name__=='__main__':
    cfg=load(D/'protocol.json')
    for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
    assert sha(__file__)==cfg['code_sha256'][str(Path(__file__))]
    f=load(cfg['feature_seal']);a=load(cfg['action_seal']);p=load(cfg['prediction_seal']);groups={};index=[]
    for ff,aa,pp in zip(f['rows'],a['rows'],p['rows']):
        assert (ff['sequence'],ff['frame'])==(aa['sequence'],aa['frame'])==(pp['sequence'],pp['frame'])
        for path,h in [(ff['feature_path'],ff['feature_sha256']),(aa['path'],aa['sha256']),(pp['path'],pp['sha256'])]:assert sha(path)==h,path
        with np.load(ff['feature_path']) as z,np.load(aa['path']) as y,np.load(pp['path']) as s:
            assert np.array_equal(z['cells'],y['cells']) and np.array_equal(y['cells'],s['cells'])
            groups.setdefault(ff['sequence'],[]).append((z['features'].copy(),y['physical_teacher_class'].copy(),s['scores'].copy()))
        index.append(dict(sequence=ff['sequence'],frame=ff['frame'],feature_sha256=ff['feature_sha256'],action_sha256=aa['sha256'],prediction_sha256=pp['sha256']))
    arrays={k:tuple(np.concatenate([r[j] for r in rows]) for j in range(3)) for k,rows in groups.items()}
    trainX,trainY,_=arrays['E0'];globalmin=trainX.min(0);globalmax=trainX.max(0);negmin=trainX[trainY==0].min(0);negmax=trainX[trainY==0].max(0)
    names=f['feature_names'];out={}
    for seq,(X,Y,S) in arrays.items():
        classes={}
        for label,name in [(1,'physical_positive'),(0,'physical_negative'),(-1,'physical_UNKNOWN')]:
            m=Y==label;xx=X[m];ss=S[m];outside=(xx<globalmin)|(xx>globalmax);negoutside=(xx<negmin)|(xx>negmax)
            classes[name]=dict(rows=int(m.sum()),fixed_threshold=.95,rejected=int((ss>=.95).sum()),score_quantiles=np.quantile(ss,[0,.1,.5,.9,.95,1]).tolist(),
                outside_E0_anyclass_feature_envelope_rows=int(outside.any(1).sum()),outside_E0_negative_feature_envelope_rows=int(negoutside.any(1).sum()),
                features=[dict(name=nm,feature_quantiles=np.quantile(xx[:,j],[0,.1,.5,.9,1]).tolist(),outside_E0_allclass_rows=int(outside[:,j].sum()),outside_E0_negative_rows=int(negoutside[:,j].sum()),E0_all_min=float(globalmin[j]),E0_all_max=float(globalmax[j]),E0_negative_min=float(negmin[j]),E0_negative_max=float(negmax[j])) for j,nm in enumerate(names)])
        out[seq]=classes
    assert len(index)==470 and sum(len(v[1]) for v in arrays.values())==40052
    put(D/'result.json',dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=40052,exposures=470,classes=out,index=index,
      interpretation='Fixed rejection score often below .95 on E1 negatives. Feature-envelope differences are descriptive support shift, not proof of root causality/irreducible ambiguity/nonlinear cure; no score/feature normalization rule changed.',
      feature_extraction=0,model_fit=0,model_score_calls=0,threshold_variants=0,source_calls=0,physics=0,navigation=0))
    print('SAVED_ARRAY_DIAGNOSIS_COMPLETE',flush=True)
