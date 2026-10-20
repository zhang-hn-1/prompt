"""Stereo VO TDD scaffold; accepts feature arrays, fixed calibration only."""
import numpy as np
def temporal_matches(q,qd,s,sd):return []
def pose_from_matches(points,pixels,K):return dict(accepted=False,reason='scaffold',transform=None)
class PoseChain:
    def __init__(self):self.pose=np.eye(4)
    def update(self,transform):return self.pose.copy()
