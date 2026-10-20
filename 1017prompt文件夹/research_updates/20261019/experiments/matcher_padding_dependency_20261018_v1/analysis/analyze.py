from pathlib import Path
import json,hashlib,datetime
import numpy as np
D=Path(__file__).resolve().parent;P=D.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
put=lambda p,v:Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
N=['front_left','front_right','rear_left','rear_right']
def main():
 cfg=json.loads((D/'protocol.json').read_text())
 for p,h in cfg['inputs'].items():assert sha(p)==h,p
 for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
 rows=[];byclass={k:dict(cells=0,fully_affected_cells=0,points=0,affected_points=0) for k in ['positive','negative','UNKNOWN']}
 for row in cfg['rows']:
  with np.load(row['action']) as a:cells=a['cells'];cls=a['physical_teacher_class'];origin=a['origin']
  with np.load(row['feature']) as a:assert np.array_equal(cells,a['cells']) and np.array_equal(origin,a['origin'])
  lookup={tuple(c):i for i,c in enumerate(cells.tolist())};num=np.zeros(len(cells),int);bad=np.zeros(len(cells),int);eyestats=[]
  with np.load(row['constant']) as a,np.load(row['replicate']) as b:
   for n in N:
    q=a[n+'_query_pixel'];xy=a[n+'_world'][:,:2];grid=np.floor((xy-origin)/.1).astype(int)
    ci=np.array([lookup[tuple(c)] for c in grid],int);z=a[n+'_depth'].ravel()[q];z2=b[n+'_depth'].ravel()[q];v2=b[n+'_valid'].ravel()[q]
    affected=~v2|(abs(z.astype(float)-z2.astype(float))>.15)
    np.add.at(num,ci,1);np.add.at(bad,ci,affected.astype(int));labels=cls[ci]
    eyestats.append(dict(eye=n,points=int(len(q)),affected=int(affected.sum()),classes={k:dict(points=int((labels==v).sum()),affected=int((affected&(labels==v)).sum())) for k,v in [('positive',1),('negative',0),('UNKNOWN',-1)]}))
  assert np.all(num>0);full=bad==num
  cs={}
  for k,v in [('positive',1),('negative',0),('UNKNOWN',-1)]:
   mask=cls==v;cs[k]=dict(cells=int(mask.sum()),fully_affected_cells=int((mask&full).sum()),points=int(num[mask].sum()),affected_points=int(bad[mask].sum()))
   for key,value in cs[k].items():byclass[k][key]+=value
  rows.append(dict(sequence=row['sequence'],frame=row['frame'],cells=len(cells),classes=cs,eyes=eyestats))
 den=byclass['negative']['cells']; frac=byclass['negative']['fully_affected_cells']/den if den else None
 decision='STOP_PADDING_STABILITY_ADMISSION_BEFORE_FULL_RUN' if frac is None or frac<.2 else 'LOCAL_NECESSARY_EVIDENCE_ONLY_NOT_FULL_SOURCE_QUALIFICATION'
 for p,h in cfg['inputs'].items():assert sha(p)==h,p
 put(D/'result.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),complete=True,rows=rows,totals=byclass,negative_cell_fully_affected_fraction=frac,local20pct_necessary_gate=frac is not None and frac>=.2,decision=decision,scope='All originalsource points/cells in four prospectivelyfixed development frames, labels from unchanged archived static XY bodyband proxy (positive<=.10m/negative>=.20m/marginUNKNOWN). Pointclass is cellproxy, NOT true point/ray depth correctness. No GTrawdepth/scene/pose loaded, no new sourcecall or maps. Fullyaffected is a narrow withholding possibility for this fixed original set, NOT a bound on all padding repairs or all newsource output. No navigation/safety/generic uncertainty claim.',new_matcher_calls=0,new_map_calls=0,fit=0,physics=0,navigation=0))
if __name__=='__main__':main()
