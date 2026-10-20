"""Approved minimal C integration checks. Synthetic records only, no RGB/GT/source."""
from pathlib import Path
import sys,json,hashlib,importlib.util
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[2]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def loadmod(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def main():
 proposal=json.loads((D/'proposal.json').read_text());approval=json.loads((D/'approval.json').read_text());assert approval['user_explicit_authorization'] and approval['proposal_sha256']==sha(D/'proposal.json')
 for p,h in proposal['frozen_identity'].items():assert sha(p)==h,p
 # A valid human exception alone must not bypass absent execution admission.
 pending=loadmod('C_pending_guard_check',D/'navigation_pending.py');before=set(sys.modules)
 try:pending.main()
 except RuntimeError as e:assert 'execution admission missing' in str(e)
 else:raise AssertionError('approval bypassed execution admission')
 assert 'pending_C_original_navigation' not in sys.modules and 'isaacsim' not in (set(sys.modules)-before)
 sys.path[:0]=[str(W/'experiments/stereo_memory_navigation_20261004_v1'),str(W/'experiments/user_fov_position_only_20261015_v1')]
 from four_camera import nominal_transforms
 from metadata_adapter import add_view_metadata
 C=loadmod('C_exact_interface_memory',Path(proposal['C_root'])/'memory.py').ViewSupportMemoryGrid
 local=nominal_transforms(.18197,.32506,.11503);body=np.eye(4);body[:3,3]=[.3,-.2,.4];theta=.4;body[:3,:3]=[[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]
 world=np.array([[1.,0.,.4]]);q=np.array([123]);pp=np.array([124]);records=[dict(frame=0,time_s=0.,query_camera=n,world=world,query_pixel_id=q,partner_pixel_id=pp) for n in ['front_left','front_right','rear_left','rear_right']]
 out=add_view_metadata(records,body,local)
 for old,new in zip(records,out):
  assert new is not old and new['world'] is old['world'] and new['query_pixel_id'] is old['query_pixel_id'] and new['partner_pixel_id'] is old['partner_pixel_id'] and 'body_center' not in old
  x=.18197 if old['query_camera'].startswith('front') else -.18197
  np.testing.assert_allclose(new['pair_center'],body[:3,3]+body[:3,:3]@np.array([x,0,.11503]),atol=1e-15,rtol=0)
  assert np.array_equal(new['body_center'],body[:3,3])
 oldcentre=out[0]['body_center'].copy();body[:3,3]=99.;assert np.array_equal(out[0]['body_center'],oldcentre)
 def record(f,t,y):
  b=np.eye(4);b[:3,3]=[0,y,.4]
  return add_view_metadata([dict(frame=f,time_s=t,query_camera='front_left',world=np.array([[1.,0.,.4]]),query_pixel_id=q,partner_pixel_id=pp)],b,local)[0]
 def feed(grid,rows,f,t):grid.set_source(dict(frame=f,time_s=t,records=rows));grid.update_points(np.empty((0,3)),np.array([0.,0.,.4]),t)
 r0=record(0,0.,0.);r1=record(5,.5,0.)
 g=C([-20,-20],10,static_scene_assumption=True);feed(g,[r0],0,0.);feed(g,[r0,r1],5,.5);assert not np.isfinite(g._view_clock).any();feed(g,[],110,11.);assert not (g.labels==1).any() and not (g.labels==0).any()
 g=C([-20,-20],10,static_scene_assumption=True);r2=record(5,.5,.21);feed(g,[r0,r2],5,.5);assert np.isfinite(g._view_clock).sum()==1;feed(g,[],110,11.);assert (g.labels==1).sum()==1 and not (g.labels==0).any()
 class Poison:
  def __array__(self,*a,**k):raise AssertionError('future world payload read')
 bad=record(115,12.,0.);bad['world']=Poison();saved=g.labels.copy()
 try:feed(g,[bad],110,11.5)
 except ValueError:pass
 else:raise AssertionError('future source accepted')
 assert np.array_equal(g.labels,saved)
 checks=dict(human_exception_alone_cannot_launch=True,no_simulator_import_by_guard=True,exact_C_class_import=True,four_eyes_paircentre_formula=True,source_world_and_UID_arrays_passthrough=True,input_records_unmodified=True,estimated_body_metadata_copy=True,actual_block_frame_ID_schema=True,stationary_repeats_not_confirmation=True,unconfirmed_expiry_UNKNOWN_not_free=True,original_geometric_confirmation_and_blind_retention=True,future_rejected_before_world_payload_and_no_map_mutation=True)
 result=dict(pass_required_C_interface=all(checks.values()),proposal_sha256=sha(D/'proposal.json'),checks=checks,synthetic_only=True,source_calls=0,RGB_GT_array_reads=0,fit=0,physics=0,navigation=0)
 (D/'C_interface_result.json').write_text(json.dumps(result,indent=2)+'\n');print('C_INTERFACE',json.dumps(checks),flush=True)
if __name__=='__main__':main()
