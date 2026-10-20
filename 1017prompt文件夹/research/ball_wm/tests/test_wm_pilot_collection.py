"""Current-protocol collection must execute the same feedback-limited requests."""
import json
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import collect_data_dubins as collector


def test_requested_initial_heading_uses_degrees_and_wxyz_order():
    args,_=collector._parse_args(['--initial_yaw_deg','90'])
    np.testing.assert_allclose(collector._initial_orientation_wxyz(args.initial_yaw_deg),
        [2**-.5,0,0,2**-.5],atol=1e-12)
    defaults,_=collector._parse_args([])
    np.testing.assert_array_equal(collector._initial_orientation_wxyz(defaults.initial_yaw_deg),[1,0,0,0])


@pytest.mark.parametrize('value',['nan','inf','-inf'])
def test_nonfinite_initial_heading_rejected_before_simulation(value):
    with pytest.raises(SystemExit):
        collector._parse_args(['--initial_yaw_deg='+value])


def test_collection_supervisor_uses_current_measured_joint_state():
    args,contract=collector._parse_args(['--auxiliary_velocity_limit_rad_s','0.30'])
    smoother=collector._make_collection_smoother(args,contract)
    action=collector._collection_applied_action(smoother,[1.0,0.5],contract,[0,0.2,0],[0,0.31,0])
    np.testing.assert_allclose(action,[0.01,0.2],atol=1e-7)
    # Releasing the measured overspeed must resume from the last applied target.
    action=collector._collection_applied_action(smoother,[1.0,0.5],contract,[0,0.2,0],[0,0.29,0])
    np.testing.assert_allclose(action,[0.02,0.2+np.pi/300],atol=1e-7)


def test_sequence_keeps_full_request_blocks_and_requires_exact_duration(tmp_path):
    _,contract=collector._parse_args([])
    path=tmp_path/'sequence.json'
    path.write_text(json.dumps({'segments':[{'steps':5,'request':[1.0,0.5]},{'steps':5,'request':[0.0,-0.5]}]}))
    sequence=collector._load_command_sequence(path,contract,10)
    np.testing.assert_array_equal(sequence[:5],np.tile(np.array([1,.5],dtype=np.float32),(5,1)))
    np.testing.assert_array_equal(sequence[5:],np.tile(np.array([0,-.5],dtype=np.float32),(5,1)))
    with pytest.raises(ValueError,match='duration'):
        collector._load_command_sequence(path,contract,15)


@pytest.mark.parametrize('segment',[
    {'steps':4,'request':[1,0]},
    {'steps':5,'request':[1,0.6]},
    {'steps':5,'request':[float('nan'),0]},
    {'steps':True,'request':[1,0]},
])
def test_sequence_rejects_misaligned_or_unexecutable_requests(tmp_path,segment):
    _,contract=collector._parse_args([])
    path=tmp_path/'sequence.json';path.write_text(json.dumps({'segments':[segment]}))
    with pytest.raises(ValueError):collector._load_command_sequence(path,contract,5)


def test_stereo_render_does_not_advance_extra_physics_ticks():
    class World:
        ticks=0
        renders=0
        def step(self,render):
            # With a slower rendering cadence, Isaac step(render=True)
            # advances the rendering interval rather than a single physics tick.
            self.ticks += 5 if render else 1
        def render(self):
            self.renders += 1
    world=World()
    for index in range(10):
        collector._advance_collection_step(world,is_stereo=(index+1)%5==0)
    assert world.ticks==10
    assert world.renders==2


def test_capture_guard_rejects_observed_half_rate_camera_cache():
    # Values from the rejected Isaac sequence: the 10 Hz SDK gate compared
    # 0.0999999978 against 0.1 and served the preceding image again.
    frames=[{'rendering_time':0.6599999852478504,'rendering_frame':91}]*2
    with pytest.raises(RuntimeError,match='stale'):
        collector._validate_stereo_capture(frames,0.7599999830126762,[91,91])


def test_capture_guard_accepts_current_frames_and_rejects_nonadvancing_ids():
    frames=[{'rendering_time':0.7599999830126762,'rendering_frame':92}]*2
    collector._validate_stereo_capture(frames,0.7599999830126762,[91,91])
    with pytest.raises(RuntimeError,match='advance'):
        collector._validate_stereo_capture(frames,0.7599999830126762,[92,92])


def test_priming_requires_current_time_not_just_nonempty_depth():
    class Camera:
        prim_path='/World/camera'
        frame={'distance_to_image_plane':np.ones((2,3)), 'rendering_time':0., 'rendering_frame':1}
        def get_current_frame(self):return self.frame
    cameras=[Camera(),Camera()]
    class World:
        current_time=0.
        steps=0
        def step(self,render):
            self.steps+=1
            self.current_time=self.steps*.1
            if self.steps>=3:
                for camera in cameras:
                    camera.frame=dict(camera.frame,rendering_time=self.current_time,rendering_frame=self.steps)
        def render(self):pass
    world=World()
    collector._prime_stereo_cameras(world,*cameras,max_steps=5)
    assert world.steps==3


def test_capture_waits_for_renderer_without_advancing_physics():
    class Camera:
        def get_current_frame(self):
            return {'rendering_time':.18 if world.renders>=3 else .06,
                    'rendering_frame':98 if world.renders>=3 else 97}
    class World:
        current_time=.18
        renders=0
        def render(self):self.renders+=1
        def step(self,**kwargs):raise AssertionError('must not advance physics')
    world=World()
    frames,extra=collector._read_current_stereo(world,[Camera(),Camera()],[97,97])
    assert extra==3
    assert frames[0]['rendering_time']==.18
    with pytest.raises(RuntimeError,match='stereo'):
        collector._read_current_stereo(world,[Camera(),Camera()],[98,98],max_extra_renders=2)
