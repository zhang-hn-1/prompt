from fixed_reference_runtime_setup_v2 import prepare_fixed_runtime

def test_open_plane_runtime_requires_reference_before_first_reset():
 calls=[]
 class R:
  def set_reference_visualization_path(self,xy):calls.append(('reference',xy));self.injected=True
  def reset(self,episode):
   assert getattr(self,'injected',False),'diagnostic_open_plane requires an injected reference path'
   calls.append(('reset',episode))
 frozen={'waypoints':[{'x_m':1,'y_m':2},{'x_m':3,'y_m':4}]}
 prepare_fixed_runtime(R(),frozen,'episode')
 assert calls==[('reference',[[1,2],[3,4]]),('reset','episode')]
