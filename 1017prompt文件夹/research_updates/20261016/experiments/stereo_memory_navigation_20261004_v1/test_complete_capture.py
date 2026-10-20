"""Breaks: reading pre-capture images, capturing twice, or advancing physics."""
from dataclasses import dataclass
import numpy as np
import pytest
from complete_capture import complete_capture

@dataclass(frozen=True)
class Observation:
    stereo_left_rgb: object
    stereo_right_rgb: object
    history: object

@dataclass(frozen=True)
class Step:
    observation: object
    frames: object
    simulation_time_s: float
    physics_step_index: int


def test_single_capture_then_fresh_images_preserves_history_and_clock():
    state={'generation':0,'renders':0,'clock':2.,'history':[]}
    zero=np.zeros((2,3,3),np.uint8)
    def capture():
        state['generation']+=1;state['history'].append(7)
        return Step(Observation(zero,zero,state['history']),{},2.,100)
    def render():
        state['renders']+=1
    def images():
        assert state['renders']==3 and state['generation']==1
        return tuple(np.full_like(zero,n) for n in (11,22,33))
    step=complete_capture(capture,render,lambda:state['clock'],lambda:[np.array([1.])],images)
    assert state['history']==[7]
    assert step.observation.history is state['history']
    assert step.simulation_time_s==2. and step.physics_step_index==100
    assert np.all(step.observation.stereo_left_rgb==11)
    assert np.all(step.observation.stereo_right_rgb==22)
    assert np.all(step.frames['third_person']==33)


def test_transaction_rejects_render_physics_tick_before_reading_images():
    state={'clock':2.,'read':False}
    def render():state['clock']+=.02
    def images():state['read']=True;return None
    with pytest.raises(RuntimeError,match='render advanced physics clock'):
        complete_capture(lambda:None,render,lambda:state['clock'],lambda:[np.zeros(1)],images)
    assert not state['read']
