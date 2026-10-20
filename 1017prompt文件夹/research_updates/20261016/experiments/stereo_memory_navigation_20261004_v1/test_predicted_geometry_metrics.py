import numpy as np
from predicted_geometry_metrics import evaluate_geometry

def fixture():
 z=np.full((480,640),1.,dtype=np.float32);t=np.eye(4);t[:3,:3]=np.diag([1,-1,-1]);t[2,3]=.4;m=np.zeros(z.shape,bool);p=np.zeros_like(z);p[240,320]=1
 return z,t,m,p,np.array([0,0,0])
def test_exact_geometry_is_predicted_and_covers_outside_anchor():
 z,t,m,p,root=fixture();r=evaluate_geometry(z,z,t,m,p,root);assert r['precision']==1 and r['coverage']==1 and r['outside_halo_correct']>=20 and r['predicted_is_observation'] is False

def test_far_prediction_cannot_hide_visible_hazard():
 z,t,m,p,root=fixture();r=evaluate_geometry(z*10,z,t,m,p,root);assert r['predicted_eligible']==0 and r['coverage']==0 and r['visible_domain']>=100 and r['gate'] is False

def test_copy_only_has_no_completion_admission():
 z,t,m,p,root=fixture();r=evaluate_geometry(np.where(p>0,z,np.nan),z,t,m,p,root);assert r['outside_halo_correct']==0 and r['gate'] is False

def test_insufficient_truth_stays_in_precision_denominator():
 z,t,m,p,root=fixture();g=z.copy();g[:,320:]=np.nan;r=evaluate_geometry(z,g,t,m,p,root);assert r['predicted_eligible']>r['correct'] and r['precision']<.95 and r['insufficient_truth']>0

def test_bad_odd_pixels_cannot_escape_whole_visible_domain():
 z,t,m,p,root=fixture();pred=z*10;pred[::2,::2]=z[::2,::2];r=evaluate_geometry(pred,z,t,m,p,root);assert r['coverage']<.9 and r['gate'] is False
