from coverage_gate import assess

def episodes():
 return [dict(group=f'{split}-{i}',split=split,planned=70,attempted=70,scorable=70,programs=['stop','forward','left','right'],state_bins=['low','moving'],support_from='complete_scorable_only',support_cells=[(a,b) for a in ('stop','forward','left','right') for b in ('low','moving')]) for split in ('development','calibration','evaluation') for i in range(10)]
def test_full_policy():assert assess(episodes())['data_gate_pass']
def test_correlated_pilot_rejected():assert not assess([dict(group='pilot',split='development',planned=70,attempted=70,scorable=70,programs=['stop'],state_bins=['low'])])['data_gate_pass']
def test_group_leak():
 d=episodes();d[-1]['group']=d[0]['group'];assert 'group_reused' in assess(d)['reasons']
def test_censor_denominator():
 d=episodes();d[0]['scorable']=0;assert 'insufficient_complete_fraction' in assess(d)['reasons']
def test_hidden_denominator():
 d=episodes();d[0]['attempted']=0;assert 'invalid_denominator' in assess(d)['reasons']
def test_support_per_group_not_pooled():
 d=episodes();d[0]['programs']=['stop'];assert 'missing_group_support' in assess(d)['reasons']

def test_unattempted_stays_in_denominator():
 d=episodes();d[0].update(attempted=20,scorable=19);assert "insufficient_complete_fraction" in assess(d)["reasons"]

def test_censored_support_rejected():
 d=episodes();d[0]["support_from"]="attempted";assert "invalid_support_origin" in assess(d)["reasons"]
def test_missing_joint_cell():
 d=episodes();d[0]["support_cells"]=d[0]["support_cells"][:-1];assert "missing_joint_support" in assess(d)["reasons"]
