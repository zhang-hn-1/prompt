"""Read-only qualification audit and one isolated interface repair; no source/label regeneration."""
from common import *
from scipy.spatial import cKDTree
from datetime import datetime,timezone
import importlib.util

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    cfg=load(D/'protocol.json');fs=load(D/'feature_seal.json');result=load(D/'qualification_result.json')
    trees={};summary={};byseq={};last={};maxdiff=0.;total=0
    for group,path in cfg['teacher_occupancy'].items():
        with np.load(path) as a:trees[group]=cKDTree(np.argwhere(a['grid']>0)*float(a['resolution_m'])+a['global_origin_xy'])
    for row,labrow in zip(fs['rows'],result['rows']):
        assert sha(row['feature_path'])==row['feature_sha256'] and sha(labrow['label_file'])==labrow['sha256']
        seq=row['sequence'];t=row['time_s'];assert seq not in last or t>last[seq];last[seq]=t
        assert row['max_feature_source_time']<=t and not row['teacher_read']
        with np.load(row['feature_path']) as f,np.load(labrow['label_file']) as a:
            assert f['features'].shape==(len(a['label']),24) and np.isfinite(f['features']).all()
            assert len(f['source_eye'])==row['in_bounds_points']==row['source_points']
            assert np.array_equal(f['promotion_eligible'],a['promotion_eligible'])
            xy=f['cells']*.1+f['origin'];d=trees[row['group']].query(xy)[0] if len(xy) else np.empty(0)
            err=float(np.max(np.abs(d-a['teacher_distance_m']))) if len(d) else 0.;maxdiff=max(maxdiff,err);assert err<1e-12
            e=f['promotion_eligible'];y=a['label'];inside=a['teacher_inside_domain'];expected=np.full(len(y),-1,np.int8)
            expected[inside&(d<=.10+1e-12)]=1;expected[inside&(d>=.20-1e-12)]=0;assert np.array_equal(y,expected)
            counts=dict(rows=len(y),eligible=int(e.sum()),eligible_positive=int(((y==1)&e).sum()),eligible_negative=int(((y==0)&e).sum()),eligible_unknown=int(((y==-1)&e).sum()),unknown_inside_margin=int(((y==-1)&e&inside).sum()),unknown_outside_teacher=int(((y==-1)&e&~inside).sum()),negative_not_yet_eligible=int(((y==0)&~e).sum()))
            for name,table in [(row['group'],summary),(seq,byseq)]:
                target=table.setdefault(name,{k:0 for k in counts})
                for k,v in counts.items():target[k]+=v
            total+=row['source_points']
    assert total==result['original_source_points']==2985832
    for g,s in summary.items():
        for k in ['rows','eligible','eligible_positive','eligible_negative','eligible_unknown']:assert s[k]==result['groups'][g][k]
    immutable={}
    for key in ['code_sha256','feature_metadata_sha256','teacher_input_sha256','old_reference_sha256']:
        for path,h in cfg[key].items():assert sha(path)==h;immutable[path]=h
    audit=dict(complete=True,utc=datetime.now(timezone.utc).isoformat(),files=470,feature_label_SHA_pairs=470,finite24=True,strict_increasing_observed_timestamps=True,recomputed_teacher_nearest_distances_max_error_m=maxdiff,source_points=total,by_group=summary,by_sequence=byseq,original_result_SHA=sha(D/'qualification_result.json'),immutable_protocol_inputs_verified=len(immutable),fit=0,source_regeneration=0,physics=0)
    write(D/'author_audit.json',audit)
    original=(D/'consumer.py').read_text();old='or (frame==self.frame and source_time!=self.source_time):raise';new='or (frame==self.frame and source_time!=self.source_time) or (frame>self.frame and source_time<=self.source_time):raise'
    assert original.count(old)==1
    repaired=D/'consumer_repaired.py';repaired.write_text(original.replace(old,new));compile(repaired.read_text(),str(repaired),'exec')
    repair_contract=dict(utc=datetime.now(timezone.utc).isoformat(),reason='Reject reindexed same physical exposure before payload/state changes; observed470 strictly increasing so immutable result unchanged.',original_sha=sha(D/'consumer.py'),repaired_sha=sha(repaired),single_edit=dict(old=old,new=new),recovery_cap=1,recovery_used_before=0,fixtures_only_cap=1,source_regeneration=0,label_regeneration=0,fit=0,physics=0,stop='Any fixture failure ends this recovery; do not repeat full dataset or change thresholds.')
    write(D/'interface_repair_protocol.json',repair_contract)
    oldm=module('original_entry',D/'consumer.py');newm=module('repaired_entry',repaired)
    c=np.array([[1,1]],int);g=oldm.EntryState((4,4));g.update(0,0.,0.,c,[1.],0,0.);g.update(1,0.,0.,c,[1.],1,0.);assert g.epochs[1,1]==2 and np.isfinite(g.persistent[1,1])
    checks=newm.interface_checks();g=newm.EntryState((4,4));g.update(0,0.,0.,c,[1.],0,0.);before=[x.copy() for x in [g.last,g.persistent,g.epochs]]
    class Poison:
        def __array__(self,*a,**kw):raise AssertionError('duplicate exposure payload touched')
    try:g.update(1,0.,0.,Poison(),Poison(),1,0.)
    except ValueError:pass
    else:raise AssertionError('duplicate physical exposure accepted')
    for oldstate,newstate in zip(before,[g.last,g.persistent,g.epochs]):assert np.array_equal(oldstate,newstate)
    checks.update(reindexed_same_timestamp_rejected_before_payload=True,duplicate_rejection_state_exact=True,original_loophole_reproduced=True,observed_dataset_unchanged=True,no_learned_output=True)
    write(D/'interface_repair_result.json',dict(complete=True,checks=checks,repair_protocol_sha256=sha(D/'interface_repair_protocol.json'),result_sha256=sha(D/'qualification_result.json'),feature_seal_sha256=sha(D/'feature_seal.json'),fit=0,source_regeneration=0,label_regeneration=0,physics=0))
    print(json.dumps(dict(groups=summary,sequences=byseq,checks=checks),indent=2))
if __name__=='__main__':main()
