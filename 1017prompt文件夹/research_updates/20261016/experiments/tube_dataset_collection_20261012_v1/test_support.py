from support import derive_support

def row(name='forward',speed=.1,scorable=True):return dict(scorable=scorable,program=[[.3,0.]]*25,estimated_velocity=[speed,0.])
def test_real_values():assert derive_support([row()])['support_cells']==[['forward','moving']]
def test_censored_does_not_support():assert derive_support([row(scorable=False)])['support_cells']==[]
def test_unknown_program():
 r=row();r['program'][0]=[.4,0.];assert derive_support([r])['support_cells']==[]
def test_outside_state():assert derive_support([row(speed=1.)])['support_cells']==[]
def test_nonfinite_state():assert derive_support([row(speed=float('nan'))])['support_cells']==[]
