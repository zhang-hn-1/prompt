"""Frozen historical causal prefixes; location agreement is not source confirmation."""
from pathlib import Path
import hashlib,json,time
import numpy as np
import builtins,re
_original_builtin_open=builtins.open
def guarded_builtin_open(file,*args,**kwargs):
 hit=re.match(r"(?:frame|source)_(\d+)\.npz$",Path(file).name) if isinstance(file,(str,Path)) else None
 if hit and int(hit[1])>100 and (not args or "r" in args[0]):raise AssertionError("future payload forbidden")
 return _original_builtin_open(file,*args,**kwargs)
builtins.open=guarded_builtin_open
from paired_current_evidence_v1 import qualify_packet
from paired_prior_memory_v1 import PairedPriorMemory
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';O=H/'persistent_consumer_provenance_prefix100_v1';O.mkdir()

_original_open=Path.open
def guarded_open(self,*args,**kwargs):
 import re
 hit=re.match(r"(?:frame|source)_(\d+)\.npz$",self.name)
 if hit and int(hit[1])>100 and (not args or 'r' in args[0]):
  raise AssertionError('future source/map forbidden')
 return _original_open(self,*args,**kwargs)
Path.open=guarded_open

sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();inputs={};rows=[];m=PairedPriorMemory();birth=np.load(D/'immutable_possible_set_birth.npz');bt=float(birth['time_s']);started=time.perf_counter();source_count=0
for f in range(101):
 mp=D/'maps'/f'frame_{f:03d}.npz';z=np.load(mp);inputs[str(mp)]=sha(mp);t=float(z['time_s']);origin=z['origin'];res=float(z['resolution']);labels=np.full(z['labels'].shape,-1,np.int8);codes=np.zeros_like(z['provenance'])
 if t>=bt and t-bt<=60.+1e-9:
  cells=np.rint((birth['possible_xy']-origin)/res).astype(int);assert ((cells>=0)&(cells<np.asarray(labels.shape))).all();idx=tuple(cells.T);labels[idx]=1;codes[idx]=6 if t-bt<=.5+1e-9 else 7
 np.testing.assert_array_equal(labels,z['labels']);np.testing.assert_array_equal(codes,z['provenance']);assert not np.any(labels==0)
 if f%5==0:
  files=[D/'sensor_packets'/f'source_{f:03d}.npz',D/'online_inputs'/f'frame_{f:03d}.npz',D/'masks'/f'frame_{f:03d}.npz'];inputs.update({str(p):sha(p) for p in files});p=dict(np.load(files[0]));rgb=np.load(files[1]);mask=np.load(files[2]);q=qualify_packet(p,rgb['left_rgb'],rgb['right_rgb'],mask['left'],mask['right'],float(rgb['time_s']),t,f);a=q['eligible_packet'];b=q['unknown_packet'];a['input_sha256']=b['input_sha256']={str(v):sha(v) for v in files};m.update(a,b,t);source_count+=1
 if f not in [20,100,200,300,400]:continue
 snap=m.at(t);sets={};counts={}
 for layer in ['provisional','unknown']:
  for temporal in ['current','history']:
   rs=[r for r in snap[layer]['records'] if (r['time_s']==t)==(temporal=='current')];w=np.concatenate([r['world'] for r in rs],axis=0) if rs else np.empty((0,3));haz=w[(w[:,2]>.05)&(w[:,2]<=1.5)];bins=np.rint((haz[:,:2]-origin)/res).astype(int);inside=((bins>=0)&(bins<np.asarray(labels.shape))).all(1);bins=bins[inside];keys=set(map(tuple,bins.tolist()));name=layer+'_'+temporal;sets[name]=keys;counts[name]=dict(raw_points=len(w),hazard_points=len(haz),hazard_XY_bins=len(keys),prior_negative_points=sum(int(r['prior_inconsistent'].sum()) for r in rs))
 model=set(map(tuple,np.argwhere(labels==1).tolist()));agreement={k:len(v&model) for k,v in sets.items()}
 rows.append(dict(frame=f,time_s=t,active_MODEL_cells=len(model),static_map_reconstruction_exact=True,counts=counts,MODEL_location_agreement_cells=agreement,MODEL_cells_without_provisional_location_agreement=len(model-(sets['provisional_current']|sets['provisional_history'])),measured_free_cells=0,current_or_history_surface_points_consumed_by_real_map=False,source_admitted=False,agreement_is_independent_confirmation=False,absence_is_free=False))
 print('PREFIX',f,'MODEL',len(model),'agreement',agreement,flush=True)
inputs[str(D/'immutable_possible_set_birth.npz')]=sha(D/'immutable_possible_set_birth.npz')
r=dict(scope='HISTORICAL_PROVENANCE_NOT_NEW_ONLINE_OR_NAVIGATION',map_frames_exact=101,source_epochs=source_count,rows=rows,inputs=inputs,code_sha256=sha(__file__),protocol_sha256=sha(H/'persistent_consumer_provenance_audit_protocol_v1.json'),wall_s=time.perf_counter()-started,no_teacher_read=True,no_actor_change=True,old_source_reject_preserved=True)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');(O/'seal.json').write_text(json.dumps(dict(outputs={str(O/'assessment.json'):sha(O/'assessment.json')},inputs=inputs,dependencies={str(H/x):sha(H/x) for x in ['paired_current_evidence_v1.py','paired_prior_memory_v1.py','paired_evidence_memory_v1.py','tentative_point_memory.py','ground_prior_consistency_v1.py']}),indent=2)+'\n')
