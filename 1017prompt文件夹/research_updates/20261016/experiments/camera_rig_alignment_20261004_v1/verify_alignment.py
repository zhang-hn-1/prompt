"""Verify live source identities, actual simulator measurements, and new masks."""
import ast,difflib,hashlib,json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
ROOTS={'worktree':Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm'),
       'primary':Path('/home/an/project/ball_wm_project_full/research/ball_wm')}
rows=json.loads((HERE/'source_changes.json').read_text());patch=[]
for row in rows:
    path=Path(row['path']);current=path.read_bytes()
    assert hashlib.sha256(current).hexdigest()==row['after_sha256'],path
    ast.parse(current)
    rel=path.relative_to(ROOTS[row['checkout']]);before=(HERE/'before'/row['checkout']/rel).read_text()
    assert hashlib.sha256(before.encode()).hexdigest()==row['before_sha256']
    patch.extend(difflib.unified_diff(before.splitlines(True),current.decode().splitlines(True),fromfile=f'{row["checkout"]}/before/{rel}',tofile=f'{row["checkout"]}/after/{rel}'))
(HERE/'camera_source_patch.diff').write_text(''.join(patch))
helper_hashes=[];assets={}
for label,root in ROOTS.items():
    helper_hashes.append(hashlib.sha256((root/'scripts/camera_rig.py').read_bytes()).hexdigest())
    assert (root/'tests/test_camera_rig.py').read_bytes()==(HERE/'test_camera_rig.py').read_bytes()
    assets[label]={}
    for relative in ('base_link_masks.npz','mask_identity.json','README.md'):
        path=root/'configs/camera_rigs/ball_stereo_front020_up005_v2'/relative
        assets[label][relative]=hashlib.sha256(path.read_bytes()).hexdigest()
    assert assets[label]['base_link_masks.npz']==hashlib.sha256((HERE/'base_link_masks.npz').read_bytes()).hexdigest()
assert len(set(helper_hashes))==1
mask=np.load(HERE/'base_link_masks.npz');assert set(mask.files)=={'left','right'}
for eye in mask.files:assert mask[eye].shape==(480,640) and mask[eye].dtype==bool
measurements=[]
for episode in ('golden_e1_0001','golden_e2_0001'):
    dest=HERE/'render'/episode
    assert not (dest/'failure.txt').exists()
    m=json.loads((dest/'measurement.json').read_text());assert m['render_preserved_physics']
    assert m['nominal_rig']['rig_id']=='ball_stereo_front020_up005_v2'
    for eye,y in [('left',.3),('right',-.3)]:
        np.testing.assert_allclose(m['actual'][eye]['local_translation_m'],[.2,y,.05],atol=1e-6,rtol=0)
        t=np.asarray(m['actual'][eye]['optical_to_base_link']);np.testing.assert_allclose(t[:3,3],[.2,y,.05],atol=1e-6)
        assert abs(m['actual'][eye]['rendering_time_s']-m['actual'][eye]['raw_physics_time_s'])<=1e-5
        assert m['actual'][eye]['rgb_shape']==[480,640,3]
        assert (dest/f'{eye}_rgb.png').stat().st_size>1000
    measurements.append(m)
result=dict(source_files_verified=len(rows),shared_helper_identical=True,shared_helper_sha256=helper_hashes[0],
    installed_assets_sha256=assets,actual_e1_e2_measurements=measurements,
    historical_records_relabelled=False,navigation_episodes_evaluated=0)
(HERE/'final_verification.json').write_text(json.dumps(result,indent=2)+'\n')
print('Verified 12 changed existing source files, shared helper/tests, both installed masks, and actual E1/E2 stereo captures.')
