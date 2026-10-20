"""Replace image outputs after the single base capture and render-only barrier."""
from dataclasses import replace
import numpy as np

def replace_step_images(step,left,right,third):
    observation=replace(step.observation,stereo_left_rgb=left,stereo_right_rgb=right)
    frames=dict(step.frames,left_rgb=left,right_rgb=right,stereo_view=np.concatenate((left,right),axis=1),third_person=third)
    return replace(step,observation=observation,frames=frames)
