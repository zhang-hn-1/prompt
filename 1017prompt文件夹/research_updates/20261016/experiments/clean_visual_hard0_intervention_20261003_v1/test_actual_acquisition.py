"""These exercise saved actual runtime traces, not simulator mocks."""
from pathlib import Path
import json,numpy as np
import pytest
H=Path(__file__).parent

def test_original_capture_trace_exposes_stale_frame_instead_of_claiming_10hz():
 v=json.loads((H/'acquisition_pair/original/camera_clocks.json').read_text())
 assert len(v)==81
 assert sum(x['eyes']['left']['age_s']>.05 for x in v)==40
 assert any(v[i]['eyes']['left']['rendering_frame']==v[i-1]['eyes']['left']['rendering_frame'] for i in range(1,len(v)))

@pytest.mark.parametrize('label,episode',[('e0','golden_e0_0001'),('e1','golden_e1_0001'),('e2','golden_e2_0001'),('hard_0_e1_0003','hard_0_e1_0003'),('hard_0_e1_0004','hard_0_e1_0004')])
def test_actual_synchronized_images_match_state_without_extra_physics(label,episode):
 root=H/'synchronized_v2'/label;v=json.loads((root/'camera_clocks.json').read_text());physical=np.load(root/'physics.npz');metrics=json.loads((root/f'episode_{episode}/metrics.json').read_text())
 assert all(abs(x['eyes'][eye]['age_s'])<=1e-5 for x in v for eye in ('left','right'))
 assert all(v[i]['eyes'][eye]['rendering_frame']>v[i-1]['eyes'][eye]['rendering_frame'] for i in range(1,len(v)) for eye in ('left','right'))
 assert len(v)==metrics['action_blocks']+1
 assert len(physical['simulation_time_s'])==metrics['action_blocks']*5+1
 assert np.array_equal(physical['physics_step_index'],np.arange(len(physical['simulation_time_s'])))
 assert np.isclose(physical['simulation_time_s'][-1],metrics['elapsed_s'],atol=1e-5)
 assert json.loads((root/'initial_render_invariants.json').read_text())=={'clock_unchanged':True,'physical_state_unchanged':True}
