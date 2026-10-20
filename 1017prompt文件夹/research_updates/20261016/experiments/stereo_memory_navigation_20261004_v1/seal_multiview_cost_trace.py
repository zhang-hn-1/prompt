"""Seal bounded sampling using pixel identity and saved confidence, never GT accuracy."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=json.loads((H/'multiview_rgb_verification.json').read_text())
for path,value in old['source_and_artifact_sha256'].items():assert sha(path)==value,path
q=np.load(H/'past_multiview_measurements.npz');run=json.loads((H/'multiview_rgb_predictions/manifest.json').read_text());rows=[]
for row in run['rows']:
    f=row['frame'];pix=q[f'frame_{f:03d}__source_pixels_yx'];n=len(pix)
    spatial=np.round(np.linspace(0,n-1,4)).astype(int)
    pred=np.load(row['prediction']);yy,xx=pix.T
    retained=np.flatnonzero(pred['confidence'][yy,xx]);retained=retained[~np.isin(retained,spatial)]
    assert len(retained)>=4
    selected=retained[np.round(np.linspace(0,len(retained)-1,4)).astype(int)]
    indices=np.concatenate([spatial,selected]);assert len(set(indices))==8
    rows.append(dict(frame=f,time_s=row['time_s'],sources=row['sources'],full_original_query_n=n,anchors=[dict(query_index=int(j),pixel_yx=pix[j].tolist(),stratum='spatial' if k<4 else 'retained_output') for k,j in enumerate(indices)]))
inputs=[H/'multiview_rgb.py',H/'multiview_rgb_protocol.json',H/'multiview_rgb_verification.json',H/'multiview_rgb_audit.json',H/'past_multiview.py',H/'past_multiview_measurements.npz',H/'multiview_rgb_inputs/manifest.json',H/'multiview_rgb_predictions/manifest.json',H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz']
inputs += [H/'multiview_rgb_inputs'/f'frame_{r["frame"]:03d}.npz' for r in rows]
inputs += [Path(r['prediction']) for r in run['rows']]
# Label payload files are registered but not read by the trace program.
inputs += [H/f'capture_transaction_development/hard_0_e1_0004/offline_truth/frame_{r["frame"]:03d}.npz' for r in rows]
protocol=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),role='ONE read-only sparse cost/support diagnosis of frozen V1, not a new candidate/navigation gate',
    sampling='4 raster-uniform full-source queries +4 raster-uniform saved-confident queries perframe, excluding duplicates; uses saved outputs but NEVER GT error/correctness/source visibility/cost; deliberately stratified, NOT population-weighted',
    budget=dict(anchors=32,max_per_frame=8,depth_bins=128,source_depth_anchor_scores=16384,new_dense_predictions=0,new_configs=0,new_simulations=0,new_control_runs=0),
    rows=rows,structure='same frozen optical projection/remap5x5/float32 box moments/ZNCC/validity/median/128inverse-depthbins/.5..3/confidence; no sparse tuning',
    inputs='sanitized actualRGB, known past/current poses,K,masks,currentmetadata only; metadata futureexcludedbeforepayload; knownpose privilegeddiagnostic, not estimatedpose',
    fidelity=dict(raw_winner_depth_exact=True,winner_view_bits_exact=True,confidence_exact=True,best_cost_atol=1e-5,interval_ray_width_atol=1e-6),
    labels='only AFTER all4 sparse curve NPZs and manifest saved and fidelity passes; sourceworld/renderer labels NEVER fed trace',
    hypothesis='depth-dependent source-set/validity switching may create falsely narrow or low-cost minima, but cannot assume sole cause; single-source000 and same-bitmap wrong matches are counterevidence',
    measurements='winner adjacent source-bit changes; invalid adjacent aggregate; competitive and full-finite interval raywidth; nearest true-depth-bin cost/validity/rankgap; fixed selected patches/source correspondence; no source selection based on truth',
    ruling='source-switch adjacency is association, NOT actual repaired counterfactual; gridlabels can miss exact curved correspondence. If no causal repair identified, reject source-switch-as-sole-cause and stop this audit branch; next decide bounded observation intervention, not another diagnosis loop',
    source_and_input_sha256={str(p):sha(p) for p in inputs})
(H/'multiview_cost_trace_protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
print('SEALED',protocol['sealed_utc'],'32anchors',len(inputs),'immutable inputs')
