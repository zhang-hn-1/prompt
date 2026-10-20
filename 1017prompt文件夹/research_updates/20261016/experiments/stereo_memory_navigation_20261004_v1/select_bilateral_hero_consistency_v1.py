"""Select once from sealed same-exposure hypotheses, without truth or inference."""
from pathlib import Path
import datetime,hashlib,json,time
import numpy as np
from bilateral_hypothesis_consistency import bilateral_hypothesis_pairs

H=Path(__file__).resolve().parent
S=H/'mvs_hero_temporal_development_v2_api_scale_recovery'
O=H/'qualified_hazard_navigation_run_v1'
D=H/'bilateral_hero_consistency_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not D.exists();D.mkdir()
original=json.loads((S/'prediction_seal.json').read_text())
assert all(sha(p)==v for p,v in original['output_sha256'].items())
plan=H.parent.parent/'docs/superpowers/plans/2026-10-07-bilateral-hero-hypothesis-consistency.md'
inputs=[S/'prediction_seal.json',S/'records.json',plan,Path(__file__),H/'bilateral_hypothesis_consistency.py']
inputs += [S/f'frame_{i:03d}_{eye}.npz' for i in (235,245,255) for eye in ('left','right')]
inputs += [O/'known_poses'/f'frame_{i:03d}.npz' for i in (235,245,255)]
(D/'input_custody_v1.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256={str(p):sha(p) for p in inputs},no_new_model_or_truth_input=True,development_not_fresh=True),indent=2)+'\n')
records=[]
for i in (235,245,255):
    left=np.load(S/f'frame_{i:03d}_left.npz');right=np.load(S/f'frame_{i:03d}_right.npz')
    now=float(left['current_time_s']);pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz')
    assert abs(float(pose['time_s'])-now)<=1e-6
    begin=time.perf_counter()
    pairs,gaps=bilateral_hypothesis_pairs(left,right,pose['left_camera_usd_world'],pose['right_camera_usd_world'],now)
    elapsed=time.perf_counter()-begin
    for e,packet in enumerate((left,right)):
        selected=pairs[:,e];eye=('left','right')[e];name=f'frame_{i:03d}_{eye}.npz'
        np.savez_compressed(D/name,world=packet['world'][selected],current_uv=packet['current_uv'][selected],
                            past_uv=packet['past_uv'][:,selected],source_frames=packet['source_frames'],
                            current_time_s=now,source_time_s=packet['source_time_s'],
                            query_pixel_id=packet['query_pixel_id'][selected],original_point_indices=selected,
                            reciprocal_pairs=pairs,world_pair_gap_m=gaps,source_model_admitted=False,
                            learned_geometry_hypothesis_only=True,certified_free=False)
        records.append(dict(frame=i,eye=eye,file=name,time_s=now,source_original_points=len(packet['world']),
                            selected_pairs=len(pairs),selection_pair_latency_s=elapsed,
                            original_Hero_inference_not_repeated=True))
    print(i,len(pairs),'pairs',flush=True)
(D/'records.json').write_text(json.dumps(records,indent=2)+'\n')
(D/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),output_sha256={str(p):sha(p) for p in D.iterdir() if p.is_file()},no_truth_or_actions_input=True,all_three_pair_selections_before_assessment=True,old_source_REJECT_unchanged=True),indent=2)+'\n')
