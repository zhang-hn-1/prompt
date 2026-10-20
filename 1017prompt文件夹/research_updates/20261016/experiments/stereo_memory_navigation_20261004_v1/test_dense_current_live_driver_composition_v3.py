"""Execute the actual driver's source-to-surface statements on a CPU fixture.

No render/CAD or physics is run. The extracted statements are the real capture
owner's contiguous block, not a reconstructed equivalent pipeline.
"""
import ast
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from test_dense_current_source_packet_v1 import fixture
from test_dense_current_live_transport_v1 import CAD

@pytest.mark.parametrize('arm,expected',[('b0',1),('candidate',2)])
def test_real_capture_owner_transmits_selected_packet_to_surface_memory(tmp_path,arm,expected):
 import hashlib
 from dense_source_points import point_packet
 from dense_current_live_transport_v1 import paired_sources as dense
 from dynamic_self_mask import joints_from_proprio
 from paired_texture_surface import PairedTextureSurfaceMemory
 from tentative_point_memory import TentativePointMemory
 from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
 driver=Path(__file__).with_name('run_dense_current_live_e1_v3.py')
 tree=ast.parse(driver.read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
 select=next(n for n in main.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='paired_sources' for t in n.targets))
 capture=next(n for n in ast.walk(main) if isinstance(n,ast.FunctionDef) and n.name=='_capture')
 branch=next(n for n in capture.body if isinstance(n,ast.If) and 'i % 5' in ast.unparse(n.test))
 start=next(i for i,n in enumerate(branch.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='proprio' for t in n.targets))
 end=next(i for i,n in enumerate(branch.body) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute) and isinstance(n.value.func.value,ast.Name) and n.value.func.value.id=='raw_point_memory')
 # Original B0 acquisition function: sparse current packet and unchanged frozen
 # mask shadow, using the same original point_packet. No new source redefinition.
 def sparse(prediction,pose,proprio,cad,now):
  m=cad.project(joints_from_proprio(proprio))
  return point_packet(prediction,pose,cad.frozen['left'],cad.frozen['right']),point_packet(prediction,pose,m['left'],m['right']),m
 # Ensure selection owns the original module binding; execute imported function
 # when available rather than the fixture equivalent.
 from masked_source_pair import paired_sources as sparse
 prediction,pose,_,_=fixture();pose['left_camera_usd_world'][2,3]=1.5;pose['right_camera_usd_world'][2,3]=1.5
 rgb=np.random.default_rng(11).integers(0,256,(480,640,3),dtype=np.uint8)
 out=tmp_path
 for name in ['sensor_packets','source_inputs','known_poses','source_predictions','partial_observations','baseline_partial_observations','current_masks']:(out/name).mkdir()
 for name in ['source_inputs','known_poses','source_predictions']:(out/name/'frame_005.npz').write_bytes(b'fixture_source_identity')
 cad=CAD();cad.last_project_wall_s=0.
 space=dict(np=np,hashlib=hashlib,a=SimpleNamespace(arm=arm),sparse_paired_sources=sparse,dense_paired_sources=dense,
  step=SimpleNamespace(observation=SimpleNamespace(proprio_history=np.zeros((5,17)))),i=5,t=.5,
  pred=dict(depth_m=prediction['depth_m'],confidence=prediction['confidence'],disparity_px=prediction['disparity_native_px']),
  restore_native=lambda x:x,self_masker=cad,source_poses={k:pose[k] for k in ['left_camera_usd_world','right_camera_usd_world']},
  rootT=np.eye(4),out=out,rgb={'left_rgb':rgb,'right_rgb':rgb},surface_memory=PairedTextureSurfaceMemory(),baseline_surface_memory=PairedTextureSurfaceMemory(),
  baseline_point_memory=TentativePointMemory(),raw_point_memory=TentativePointMemory(),source_rows=[],residuals={})
 space['rootT'][:3,3]=pose['root_position_world_m']
 code=ast.fix_missing_locations(ast.Module(body=[select]+branch.body[start:end+1],type_ignores=[]))
 exec(compile(code,str(driver),'exec'),space)
 saved=dict(np.load(out/'sensor_packets/source_005.npz'));surface=space['surface_memory'].at(.5)[0]
 assert len(saved['world'])==len(surface['world'])==expected and cad.calls==1
 row=space['source_rows'][0]
 assert row['source_packet_points']==expected
 assert row['live_packet_sampler']==('without_stride2' if arm=='candidate' else 'original_stride2')
 record=space['raw_point_memory'].at(.5)['records'][0]
 assert record['frame']==5 and record['time_s']==.5
 assert np.array_equal(record['world'],surface['world'])
 grid=RepeatedExposureMemoryGrid([-20,-20],10,static_scene_assumption=True)
 grid.consume_records([record],.5,pose['root_position_world_m'])
 assert (grid.labels==1).sum()>0 and not (grid.labels==0).any()
 assert grid.diagnostics()['measured_free']==0
