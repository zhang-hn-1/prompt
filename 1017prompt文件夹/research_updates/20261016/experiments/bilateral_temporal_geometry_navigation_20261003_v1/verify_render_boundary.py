"""Verify the recorded actual USD visibility at every camera capture."""
import json
import os
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt


def main():
    here = Path(os.environ['BILATERAL_ASSESS_ROOT'])
    expected = {'/World/BenchmarkMarkers/'+s for s in ('Start','Goal','OraclePath','ActualPath')}
    cases = {}
    for label in ('e2','e1'):
        for mode in ('b0','candidate'):
            case = f'{mode}_{label}'; root = here/case
            creation = json.loads((root/'marker_visibility_at_creation.json').read_text())
            frames = json.loads((root/'marker_visibility_each_capture.json').read_text())
            assert set(creation) == expected
            assert frames
            for snapshot in [creation]+[f['markers'] for f in frames]:
                assert set(snapshot) == expected
                assert all(row['computed_visibility']=='invisible' for row in snapshot.values())
            cases[case] = dict(captures_checked=len(frames), all_four_actual_markers_invisible=True,
                               at_creation_before_first_render=True)
    (here/'render_boundary_verification.json').write_text(json.dumps(cases, indent=2)+'\n')
    # Actual camera/video observations, no fabricated pixels or scene rendering.
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    for col, version in enumerate(('clean_render_v1','clean_render_v2')):
        folder = here.parent/version
        with np.load(folder/'candidate_e2/online_inputs/frame_015.npz') as f:
            axes[0,col].imshow(f['left_rgb']);axes[0,col].axis('off')
            axes[0,col].set_title(f'{version}: actual E2 left RGB at 1.5 s')
        cap = cv2.VideoCapture(str(folder/'b0_e2/episode_golden_e2_0001/third_person.mp4'))
        ok, frame = cap.read();cap.release();assert ok
        axes[1,col].imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));axes[1,col].axis('off')
        axes[1,col].set_title(f'{version}: actual third-person first video frame')
    fig.savefig(here/'render_boundary_actual_frames.png', dpi=150);plt.close(fig)
    print(cases)


if __name__ == '__main__':
    main()
