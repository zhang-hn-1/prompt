import pytest,json
from bridge import Bridge

def setup(tmp_path):
 b=Bridge(tmp_path/'log','s');b.anchor(1.,left_rgb=b'left',right_rgb=b'right',estimated_xy=(0.,0.),estimated_velocity=(0.,0.),estimated_heading=0.,source='native_imu_rolling',initial_gauge=True,flat_ground_prior=True);b.commit(tuple((.3,.1) for _ in range(25)),initial_supervision_xy=(0.,0.));return b

def run(b,actual=(.3,.1)):
 for i in range(25):b.step(lambda p,i=i:dict(request_time=1.+i*.02,actual_raw_pair=actual,end_time=1.+(i+1)*.02,end_world_xy=(i*.01,0.)))
 return b.qualify()
def test_complete(tmp_path):assert run(setup(tmp_path))['scorable']
def test_actual_pair_not_requested(tmp_path):assert run(setup(tmp_path),(.3,.2))['reason']=='program_mismatch'
def test_rgb_bound_before_execution(tmp_path):
 b=setup(tmp_path);r=json.loads((tmp_path/'log').read_text().splitlines()[0]);snap=json.loads(bytes.fromhex(r['snapshot_hex']));assert snap['left_rgb_hex']=='6c656674' and snap['sequence_id']=='s'
def test_callback_failure(tmp_path):
 b=setup(tmp_path)
 def fail(p):raise RuntimeError('interrupted')
 with pytest.raises(RuntimeError):b.step(fail)
 assert b.qualify()['reason']=='execution_interrupted'
def test_source_not_silently_gt(tmp_path):
 b=Bridge(tmp_path/'log','s')
 with pytest.raises(ValueError):b.anchor(1.,left_rgb=b'l',right_rgb=b'r',estimated_xy=(0.,0.),estimated_velocity=(0.,0.),estimated_heading=0.,source='simulation_truth',initial_gauge=True,flat_ground_prior=True)
