"""Precondition for consuming a curve, independent of the old argmax."""
import sys
from pathlib import Path
import numpy as np
sys.path.append(str(Path(__file__).parent.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from control import make_detour


def qualify_curves(points,position,goal_world,known_clear_swept_region=False):
    points=np.asarray(points).reshape(-1,2);position=np.asarray(position);goal_world=np.asarray(goal_world)
    curves={}
    for side in (-1,1):
        path=make_detour(position,goal_world,side)
        if path is None or not len(points):
            curves[side]=dict(clearance_m=None,observed_clearance_pass=False,execution_eligible=False,reason='no_observed_space')
            continue
        forward=(goal_world-position)/np.linalg.norm(goal_world-position)
        near=path[(path-position)@forward<=3]
        distance=float(np.min(np.linalg.norm(near[:,None,:]-points[None,:,:],axis=2)))
        observed_pass=distance>=.5
        eligible=observed_pass and bool(known_clear_swept_region)
        reason='qualified' if eligible else 'insufficient_clearance' if not observed_pass else 'unverified_swept_space'
        curves[side]=dict(clearance_m=distance,observed_clearance_pass=observed_pass,execution_eligible=eligible,reason=reason)
    eligible=[s for s in (-1,1) if curves[s]['execution_eligible']]
    side=max(eligible,key=lambda s:curves[s]['clearance_m']) if eligible else None
    return dict(selected_side=side,curves=curves,required_clearance_m=.5,
                scope='next three forward metres only; downstream remainder still unverified')
