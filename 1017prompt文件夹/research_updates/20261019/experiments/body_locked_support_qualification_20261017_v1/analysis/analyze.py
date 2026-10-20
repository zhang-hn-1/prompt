"""Saved fixed-condition counts only, not alternate policy or nearest queries."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
load=lambda p:json.loads(Path(p).read_text())
if __name__=='__main__':
    cfg=load(D/'protocol.json')
    for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
    assert sha(__file__)==cfg['code_sha256'][str(Path(__file__))]
    fs=load(cfg['feature_seal']);ps=load(cfg['prediction_seal']);ys=load(cfg['action_seal']);out={};index=[];points=0;cells=0
    for f,p,y in zip(fs['rows'],ps['rows'],ys['rows']):
        assert (f['sequence'],f['frame'])==(p['sequence'],p['frame'])==(y['sequence'],y['frame'])
        for path,h in [(f['feature_path'],f['feature_sha256']),(p['path'],p['sha256']),(y['path'],y['sha256'])]:assert sha(path)==h,path
        with np.load(f['feature_path']) as a,np.load(p['path']) as z,np.load(y['path']) as teacher:
            assert np.array_equal(a['cells'],z['cells']) and np.array_equal(z['cells'],teacher['cells'])
            inverse=a['source_cell_row'];n=len(a['cells']);dw=z['distance_world'];db=z['distance_body'];v=np.isfinite(dw)&np.isfinite(db)
            assert len(inverse)==len(dw)==len(db)
            count=np.bincount(inverse,minlength=n)
            allvalid=(count>0)&(np.bincount(inverse,weights=v,minlength=n)==count)
            allbody=allvalid&(np.bincount(inverse,weights=v&(db<=.1),minlength=n)==count)
            allworld=allvalid&(np.bincount(inverse,weights=v&(dw>=.2),minlength=n)==count)
            allboth=allbody&allworld
            assert np.array_equal(allboth,z['scores']==1) and np.array_equal(z['point_witness'],v&(db<=.1)&(dw>=.2))
            stats=out.setdefault(f['sequence'],dict(frames=0,frames_with_anchor=0,sourcepoints=0,point_witness=0,classes={}))
            stats['frames']+=1;stats['frames_with_anchor']+=int(p['anchor_frame']>=0);stats['sourcepoints']+=len(inverse);stats['point_witness']+=int(z['point_witness'].sum())
            yy=teacher['physical_teacher_class']
            for value,name in [(1,'positive'),(0,'negative'),(-1,'UNKNOWN')]:
                m=yy==value;s=stats['classes'].setdefault(name,dict(total_cells=0,allpoints_anchor_available=0,missing_anchor_somepoint=0,all_body_close=0,all_world_far=0,both_all_withheld=0,some_world_compatible=0,body_close_world_compatible=0))
                for key,mask in [('total_cells',m),('allpoints_anchor_available',m&allvalid),('missing_anchor_somepoint',m&~allvalid),('all_body_close',m&allbody),('all_world_far',m&allworld),('both_all_withheld',m&allboth),('some_world_compatible',m&allvalid&~allworld),('body_close_world_compatible',m&allbody&~allworld)]:s[key]+=int(mask.sum())
            points+=len(inverse);cells+=n
            index.append(dict(sequence=f['sequence'],frame=f['frame'],feature_sha256=f['feature_sha256'],prediction_sha256=p['sha256'],action_sha256=y['sha256']))
    assert len(index)==470 and points==2985832 and cells==40052
    result=dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),exposures=470,sourcepoint_rows=points,cell_rows=cells,sequences=out,index=index,
        scope='Conditional diagnostics at original fixed.10/.20/10s/.20translation only. No alternative gate/policy/performance evaluated. Body/world nearest sets do not identify correspondence or groundtruth.',
        geometry_nearest_queries=0,model_calls=0,source_calls=0,fit=0,new_scoring=0,physics=0,navigation=0)
    (D/'result.json').write_text(json.dumps(result,indent=2)+'\n');print('ALL_FIXED_CONDITION_COUNTS_COMPLETE',flush=True)
