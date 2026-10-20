import numpy as np
from test_memory_geometry_prompt import fixture
from memory_geometry_prompt_v2 import memory_geometry_prompt

def test_empty_historical_geometry_is_unknown_without_reading_camera_payload():
 f,p,m=fixture()
 for k in ['past_world','past_query_id','past_partner_id','left_uv','right_uv','left_z','right_z','patch_NCC','patch_std']:
  if k in f:f[k]=np.asarray(f[k])[:0]
 class Trap:
  def __array__(self,*a,**k):raise AssertionError('empty geometry unnecessarily reads camera')
 p['right_camera_usd_world']=Trap();r=memory_geometry_prompt(f,p,m,23.5,235,'right');assert r['nonzero_prompts']==0 and r['status']=='NOT_EXERCISED_EMPTY_PROMPT' and not r['is_current_depth_observation']
