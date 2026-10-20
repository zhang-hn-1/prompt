from selection_guard import choose

def test_unknown_state_cannot_use_any_motion():
 assert choose({'turn':{'clearance_m':1,'goal_distance_m':1}},False,True)=='brake'

def test_blocked_turn_selects_known_hold():
 d={'turn':{'clearance_m':-.65,'goal_distance_m':1},'hold':{'clearance_m':.2,'goal_distance_m':2}}
 assert choose(d,True,True)=='hold'
 assert choose(d,True,False)=='brake'
