"""One seven-exposure fixed input ablation, RGB-only inference; truth after seal."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from fast_stereo import FastStereo
from mask_stereo_inputs import masked_inputs
from stereo_geometry import restore_native
from masked_source_pair import paired_sources
from dynamic_self_mask import RobotSelfMask
H=Path(__file__).resolve().parent;O=H/'cad_masked_input_probe';O.mkdir(exist_ok=False);groups=[('retreat_observation_run',i) for i in [0,60,120,180]]+[('fresh_rolling_prior_shadow_current_run',i) for i in [0,60,110]];sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();old=json.loads((H/'retreat_observation_protocol.json').read_text());sources=dict(old['source_sha256'])
for p,s in sources.items():assert sha(p)==s,p
for p in [Path(__file__),H/'mask_stereo_inputs.py',H/'test_mask_stereo_inputs.py',H.parents[1]/'docs/superpowers/plans/2026-10-07-cad-masked-stereo-input-probe.md']:sources[str(p.resolve())]=sha(p)
inputs={}
for case,i in groups:
 for folder in ['online_inputs','known_poses','history','source_predictions','masks','offline_truth']:
  p=H/case/folder/f'frame_{i:03d}.npz';inputs[str(p)]=sha(p)
 for p in [H/case/'states.json',H/case/'sensor_packets'/f'source_{i:03d}.npz']:inputs[str(p)]=sha(p)
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,groups=groups,only_change='zero currentCAD-invalid copied RGB pixels before sameFast backbone',no_new_network_fit_or_thresholds=True,old_gate_unchanged=True,local_gate=dict(near_correct_fraction_min=.80,near_correct_divider_count_min=25,near_frames=[120,180],far_correct_fraction_no_regression=True,far_correct_count_no_regression=True),development_not_fresh=True,GT_values_not_deserialized_until_all_predictions_sealed=True)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');cad=RobotSelfMask();base=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');model=FastStereo();cache=[]
for n,(case,i) in enumerate(groups):
 o=H/case;rgb=np.load(o/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(o/'known_poses'/f'frame_{i:03d}.npz');hist=np.load(o/'history'/f'frame_{i:03d}.npz');states=json.loads((o/'states.json').read_text());m=np.load(o/'masks'/f'frame_{i:03d}.npz');t=float(rgb['time_s']);known=dict(left_camera_usd_world=pose['left_camera_usd_world'],right_camera_usd_world=pose['right_camera_usd_world'],root_position_world_m=states[i]['position'],time_s=t)
 def packet(pred):
  raw=dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t)
  return paired_sources(raw,known,hist['proprio_history'],cad,t)[1]
 # Cached raw-prediction reconstruction independent of any new GPU inference.
 oldpred=np.load(o/'source_predictions'/f'frame_{i:03d}.npz');cached=packet(oldpred);saved=np.load(o/'sensor_packets'/f'source_{i:03d}.npz')
 for k,v in cached.items():np.testing.assert_equal(v,saved[k])
 d=restore_native(oldpred['disparity_px']);y,x=np.indices(d.shape);v=y-np.where(np.isfinite(d),d,0);j=np.rint(np.clip(v,0,479)).astype(int);extra=(~m['left'])&(~m['right'][j,x]);filtered=dict(depth_m=oldpred['depth_m'],confidence=oldpred['confidence']&extra,disparity_px=oldpred['disparity_px']);equiv=packet(filtered)
 for k,v in cached.items():np.testing.assert_equal(v,equiv[k])
 l,r=masked_inputs(rgb['left_rgb'],rgb['right_rgb'],m['left'],m['right']);pair={}
 for name,images in [('baseline',(rgb['left_rgb'],rgb['right_rgb'])),('zeroCAD',(l,r))]:
  pred=model.predict(*images,base['left'],base['right']);s=packet(pred);np.savez_compressed(O/f'{n:02d}_{name}_prediction.npz',**pred,frame=i,time_s=t);np.savez_compressed(O/f'{n:02d}_{name}_packet.npz',**s);pair[name]=s
 cache.append((case,i,pair));print('PAIRED_INFERENCE_DONE',case,i,flush=True)
sealed={str(p):sha(p) for p in O.glob('*.npz')};(O/'predictions_pretruth_sha256.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256=sealed),indent=2)+'\n');geom=json.loads((H/'retreat_visibility_diagnostic.json').read_text());centres=np.array(geom['divider_centres']);rows=[]
for case,i,pair in cache:
 truth=np.load(H/case/'offline_truth'/f'frame_{i:03d}.npz');r=dict(case=case,frame=i)
 for name,s in pair.items():
  q=s['query_pixel_id'].astype(int);yy=q//640;xx=q%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(s['world']-actual,axis=1)
  good=np.isfinite(err)&(err<=.1);proxy=np.zeros(len(q),bool)
  for c in centres:proxy|=(np.linalg.norm(actual[:,:2]-c[:2],axis=1)<=.355)&(actual[:,2]>=-.005)&(actual[:,2]<=1.005)
  r[name]=dict(queries=len(q),correct_10cm=int(good.sum()),correct_fraction=float(good.sum()/len(q)) if len(q) else 0.,finite_truth=int(np.isfinite(err).sum()),correct_divider=int((good&proxy).sum()))
 b=r['baseline'];c=r['zeroCAD'];r['diagnostic_gate_pass']=bool(c['correct_fraction']>=.8 and c['correct_divider']>=25) if case=='retreat_observation_run' and i in [120,180] else (bool(c['correct_fraction']>=b['correct_fraction'] and c['correct_10cm']>=b['correct_10cm']) if case!='retreat_observation_run' else None);rows.append(r)
for p,s in sealed.items():assert sha(p)==s
accepted=all(r['diagnostic_gate_pass'] for r in rows if r['diagnostic_gate_pass'] is not None);result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,candidate_accepted_for_fresh_source_validation_only=accepted,old_source_gate_unchanged=True,controller_not_changed=True,no_navigation_run=True,whole_research_complete=False,cached_packet_reconstruction_and_outputCAD_equivalence_all_seven_pass=True,paired_same_RGB_baseline_evaluated=True)
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
