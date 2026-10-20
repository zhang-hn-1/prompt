from paired_current_gate_v1 import check_gates

def row(n,p,h):return dict(eligible=n,precision=p,eligible_hazard=h)
def test_empty_unknown_epoch_not_extra_unfrozen_rejection():
 r=check_gates([row(100,.99,100),row(0,None,0),row(100,.99,100)],100,100,13,13,3)
 assert all(r.values());assert 'every_eligible_epoch_nonempty' not in r
def test_no_vacuous_pass_without_actual_two_hazard_epochs():
 assert not all(check_gates([row(0,None,0)],0,0,0,0,1).values())
def test_low_precision_and_retention_still_fail_frozen95():
 r=check_gates([row(100,.94,100),row(100,.99,100)],100,94,13,12,2)
 assert not r['each_nonempty_epoch_ge95'] and not r['original_GTvalid_retention_ge95'] and not r['original_GTvalid_XY10cm_bin_retention_ge95']
