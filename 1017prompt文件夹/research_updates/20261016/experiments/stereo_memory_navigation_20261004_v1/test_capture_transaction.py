"""A corrected image transaction must not duplicate physical/history captures."""
from dataclasses import dataclass
import numpy as np
from capture_transaction import replace_step_images

@dataclass(frozen=True)
class Obs:
    stereo_left_rgb:object
    stereo_right_rgb:object
    history:object

@dataclass(frozen=True)
class Step:
    observation:object
    frames:object
    simulation_time_s:float
    physics_step_index:int

def test_replace_images_preserves_state_history_time_and_previous_step():
    old=np.zeros((2,3,3),np.uint8);history=object();step=Step(Obs(old,old,history),{'left_rgb':old,'right_rgb':old,'stereo_view':np.concatenate((old,old),axis=1),'third_person':old},1.,50)
    left=np.ones_like(old);right=left*2;third=left*3;new=replace_step_images(step,left,right,third)
    assert new.simulation_time_s==1. and new.physics_step_index==50
    assert new.observation.history is history and step.observation.stereo_left_rgb is old
    assert np.array_equal(new.frames['stereo_view'],np.concatenate((left,right),axis=1))
    assert new.observation.stereo_left_rgb is left and new.observation.stereo_right_rgb is right
    assert new.frames['third_person'] is third
