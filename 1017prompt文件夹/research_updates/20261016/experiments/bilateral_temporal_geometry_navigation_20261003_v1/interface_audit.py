"""Audit actual RGB consumption. Counterfactual requests are not trajectories."""
import builtins
from contextlib import contextmanager
import json
import os
from pathlib import Path
import cv2
import numpy as np
from benchmark.policies import EpisodeContext,PolicyObservation
from training.action_protocol import ActionProtocol
from policy import BilateralTemporalPolicy

HERE=Path(os.environ.get('BILATERAL_ASSESS_ROOT',Path(__file__).resolve().parent))
cv2.setNumThreads(2)


@contextmanager
def no_file_inputs():
    original=builtins.open;original_np=np.load
    def blocked(*args,**kwargs):raise AssertionError('predictor requested undeclared file/log/truth')
    builtins.open=blocked;np.load=blocked
    try:yield
    finally:builtins.open=original;np.load=original_np


def replay(frames,blank_eye=None):
    protocol=ActionProtocol('ball_action_v2_aux30',(-2.,-np.pi/6),(2.,np.pi/6),5)
    policy=BilateralTemporalPolicy(protocol)
    policy.reset(EpisodeContext('metadata_not_a_scene_input',False,None,protocol.protocol_id))
    actions=[]
    with no_file_inputs():
        for f in frames:
            left=f['left_rgb'] if blank_eye!='left' else np.zeros_like(f['left_rgb'])
            right=f['right_rgb'] if blank_eye!='right' else np.zeros_like(f['right_rgb'])
            o=PolicyObservation(stereo_left_rgb=left,stereo_right_rgb=right,
                 proprio_history=f['proprio_history'],action_history=f['action_history'],
                 position_world_xy=f['position_world_xy'],yaw_world_rad=float(f['yaw_world_rad']),
                 quaternion_world_wxyz_history=f['quaternion_world_wxyz_history'])
            actions.append(policy.act(o,f['pointgoal_base_xy']))
    return np.stack(actions),policy.records


def load_frames(folder):
    keys=('left_rgb','right_rgb','proprio_history','action_history','position_world_xy',
          'yaw_world_rad','quaternion_world_wxyz_history','pointgoal_base_xy','request')
    frames=[]
    for path in sorted(folder.glob('frame_*.npz')):
        with np.load(path) as f:frames.append({key:f[key].copy() for key in keys})
    return frames


def main():
    results={}
    for difficulty in ('e2','e1'):
        root=HERE/f'candidate_{difficulty}';frames=load_frames(root/'online_inputs')
        actual=np.stack([f['request'] for f in frames])
        predicted,records=replay(frames)
        exact=bool(np.array_equal(actual,predicted))
        created=next((i for i,r in enumerate(records) if r['reference_created']),None)
        n=min(len(frames),max(40,(created or 0)+10))
        prefix=frames[:n];baseline=predicted[:n]
        left,_=replay(prefix,'left');right,_=replay(prefix,'right')
        truncated,_=replay(prefix)
        suffix=[]
        for f in frames[n:n+10]:
            altered={k:v.copy() for k,v in f.items()}
            altered['left_rgb'][:]=255;altered['right_rgb'][:]=0
            altered['position_world_xy']+=1000;altered['pointgoal_base_xy']*=-1
            suffix.append(altered)
        replaced,_=replay(prefix+suffix)
        passed=exact and np.array_equal(truncated,baseline) and np.array_equal(replaced[:n],baseline)
        r=dict(full_actual_request_replay_exact=exact,full_online_blocks=len(frames),prefix_blocks=n,
          future_truncation_prefix_exact=bool(np.array_equal(truncated,baseline)),
          future_replacement_prefix_exact=bool(np.array_equal(replaced[:n],baseline)),
          undeclared_file_and_depth_access_blocked_during_act=True,
          left_eye_blank_changed_prefix_requests=int(np.any(left!=baseline,axis=1).sum()),
          right_eye_blank_changed_prefix_requests=int(np.any(right!=baseline,axis=1).sum()),
          first_actual_visual_reference_block=created,passed=bool(passed),
          counterfactual_scope='same recorded measured observations; no alternate true trajectory or success score')
        assert passed,r
        results[difficulty]=r;print(difficulty,r,flush=True)
    (HERE/'interface_audit.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
