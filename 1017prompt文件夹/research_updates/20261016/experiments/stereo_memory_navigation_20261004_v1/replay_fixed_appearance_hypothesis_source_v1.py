"""Frozen old hypothesis pipeline applicability; development, never fresh/nav."""
from pathlib import Path
import json,hashlib,datetime,copy,numpy as np
from imu_gravity_rolling import GravityRolling
from partial_cue_source_v2 import PartialCueSetSource,inflate_xy
from earlier_source_opportunity import EarlierSourceOpportunity
from bilateral_core_memory import BilateralCoreMemory
from ray_contradiction_memory import contradict_cores
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
H=Path(__file__).resolve().parent;D=H/'native_fixed_appearance_probe_run_v1';O=H/'fixed_appearance_hypothesis_source_v1';O.mkdir(exist_ok=False)
for n in ['possible_sets','class_masks']:(O/n).mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
actor=json.loads((H/'partial_set_uncertainty_actor_v1.json').read_text());assert actor['radius_m']==actor['maximum_radius_m']==.2 and actor['is_model_hypothesis_not_observation']
packets=json.loads((D/'native_imu_packets.json').read_text());gauge=json.loads((D/'initial_gauge.json').read_text());identity=json.loads((D/'native_imu_identity.json').read_text());native=json.loads((D/'native_predictions.json').read_text());R0=np.asarray(gauge['rotation']);g=GravityRolling(identity['initial_normal']);vel=[]
for p,r in zip(packets,native):
 v=g.update(p);np.testing.assert_array_equal(R0@v['rotation'],r['rotation_world']);position=np.asarray(gauge['position'])+R0@v['position'];position[2]=.4000000059604645;np.testing.assert_array_equal(position,r['position_world']);vel.append(R0@v['velocity'])
vel=np.asarray(vel);np.savez_compressed(O/'native_velocity_pretruth.npz',velocity_world_m_s=vel,time_s=[p['time_s'] for p in packets],flat_height_velocity_is_prior=True)
source=PartialCueSetSource(actor['radius_m']);op=EarlierSourceOpportunity();cores=BilateralCoreMemory();cad=RobotSelfMask();pairs=[];rows=[];inputs={};event=None
for i in range(0,141,5):
 name=f'frame_{i:03d}.npz';fs=[D/'sensor_packets'/f'source_{i:03d}.npz',D/'source_inputs'/name,D/'estimated_poses'/name,D/'history'/name]
 for f in fs:inputs[str(f)]=sha(f)
 packet,rgb,pose,history=[dict(np.load(f)) for f in fs];t=float(packet['time_s']);assert t==float(rgb['time_s'])==float(pose['time_s']);mask=cad.project(joints_from_proprio(history['proprio_history']))
 last5=vel[np.arange(max(0,i-4),i+1)*5,:2];pair=op.update(packet,last5,t,i)
 if pair is not None:event=dict(frame=i,time_s=t,pair=pair)
 q=source.update(packet,rgb,pose,mask,pair is not None,t,i,pose['root_position_world_m'][:2],np.array([2.2,.15]),np.array([-2.2,.15]));cores.update(q,t);pairs.append((copy.deepcopy(packet),copy.deepcopy(pose)));np.savez_compressed(O/'possible_sets'/name,**q)
 for (frame,eye),cl in source.class_masks.items():np.savez_compressed(O/'class_masks'/f'birth_{i:03d}_{eye}_{frame:03d}.npz',potential_hazard_mask=cl,exposure_time_s=float(packets[frame*5]['time_s']),hypothesis_birth_time_s=t)
 rows.append(dict(frame=i,time_s=t,qualified_seed=pair is not None,event=copy.deepcopy(event),opportunity=copy.deepcopy(op.diagnostics),pending_seed=bool(q['pending_seed']),raw_cores=len(q['raw_world_voxel_centres']),support_frames=q.get('support_frames',np.empty(0,int)).tolist()))
if cores.cores:
 pool=cores.freeze(t);raw=pool['raw_world_voxel_centres'];veto,proof=contradict_cores(raw,pairs,t);seed=dict(pool,raw_world_voxel_centres=raw[~veto],possible_xy=inflate_xy(raw[~veto,:2],.2),source_consumer='frozen_negative_ray_history_new_appearance_DEVELOPMENT',not_measured_free=True)
 np.savez_compressed(O/'unfiltered_pool.npz',**pool);np.savez_compressed(O/'seed_pretruth.npz',**seed);np.savez_compressed(O/'core_veto.npz',veto=veto);(O/'ray_proofs.json').write_text(json.dumps(proof,indent=2)+'\n')
else:seed=None
(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'core_provenance.json').write_text(json.dumps(dict(windows=cores.windows,cores=cores.core_provenance()),indent=2)+'\n')
if source.model is not None:np.savez_compressed(O/'online_appearance_model.npz',**source.model)
inputs.update({str(D/n):sha(D/n) for n in ['native_imu_packets.json','native_predictions.json','initial_gauge.json','native_imu_identity.json']});inputs[str(H/'partial_set_uncertainty_actor_v1.json')]=sha(H/'partial_set_uncertainty_actor_v1.json')
dependencies={str(H/n):sha(H/n) for n in ['replay_fixed_appearance_hypothesis_source_v1.py','imu_gravity_rolling.py','partial_cue_source_v2.py','earlier_source_opportunity.py','bilateral_core_memory.py','ray_contradiction_memory.py','dynamic_self_mask.py']}
(O/'actor_seal_pretruth.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,source_sha256=dependencies,output_sha256={str(f):sha(f) for f in O.rglob('*') if f.is_file()},new_actor_truth_labels_not_read=True,old14_source_labels_previously_used_so_NOT_FRESH=True,first_qualified_pair_only=True,radius_not_refit=True,online_class_initialization_by_unchanged_code=True,no_NN_or_GPU_or_navigation=True),indent=2)+'\n');print(json.dumps(dict(event=event,model_birth=None if source.model is None else source.model['time_s'],unfiltered_cores=len(cores.cores),filtered_cores=0 if seed is None else len(seed['raw_world_voxel_centres']),no_actor_label_reads=True,not_fresh=True),indent=2))
