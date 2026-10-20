import json
from pathlib import Path
import numpy as np
from audit_first_reference_shape_attribution_v1 import project,current_state
H=Path(__file__).resolve().parent
def test_actual_sealed_state_schema():
 row=json.loads((H/'bank_qualified_navigation_e1_b0_run_v1/states.json').read_text())[60]
 xy,v,q=current_state(row)
 assert np.array_equal(xy,np.asarray(row['position'])[:2])
 assert np.array_equal(v,np.asarray(row['velocity'])[:2])
 assert np.array_equal(q,np.asarray(row['quaternion']))
def test_continuous_projection_independent_of_waypoint_cursor():
 q=np.array([[0.,0.],[1.,0.],[1.,1.]])
 arc,dist,tangent,k=project(np.array([.75,.25]),q)
 assert np.isclose(arc,.75) and np.isclose(dist,.25) and k==0 and tangent==0
 assert np.isclose(project(np.array([1.2,.7]),q)[0],1.7)
