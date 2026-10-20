"""Bounded open-plane pose-entry intervention; not a navigation result."""
from __future__ import annotations
import argparse, dataclasses, hashlib, json, math, traceback
from pathlib import Path
import numpy as np
from benchmark.policies import OraclePathPolicy, EpisodeContext
from benchmark.kinodynamic_planning import KinodynamicWaypoint

ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
OLD=ROOT/'research/ball_wm/runs/benchmark_round8_instrumented_b0_20260921_v1'

def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if dataclasses.is_dataclass(x):return plain(dataclasses.asdict(x))
    if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [plain(v) for v in x]
    return x

def write(path,data):
    path.write_text(json.dumps(plain(data),ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def align_reference(ref,index,position,yaw):
    angle=float(yaw-ref[index].yaw_rad)
    r=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    origin=np.array([ref[index].x_m,ref[index].y_m])
    shift=np.asarray(position)-r@origin
    out=[]
    for w in ref:
        xy=r@np.array([w.x_m,w.y_m])+shift
        out.append(dataclasses.replace(w,x_m=float(xy[0]),y_m=float(xy[1]),yaw_rad=float((w.yaw_rad+angle+math.pi)%(2*math.pi)-math.pi)))
    return tuple(out),{'rotation':r,'translation':shift,'angle':angle}

def canonical_xy(xy,transform):
    return (np.asarray(xy)-np.asarray(transform['translation']))@np.asarray(transform['rotation'])

class EventOracle(OraclePathPolicy):
    def reset(self,context):
        super().reset(context)
        self.initial_direction=self._active_drive_direction
        self.handoff_index=None
    def _handle_direction_change(self,direction,speed):
        previous=self._active_drive_direction
        action=super()._handle_direction_change(direction,speed)
        if self.handoff_index is None and previous!=self._active_drive_direction:
            self.handoff_index=int(self._path_index)
        return action
    @property
    def ready(self):
        return self.handoff_index is not None and self._path_index>self.handoff_index and self._pending_drive_direction is None and self._active_drive_direction==self._reference[self._path_index].drive_direction

def snapshot(runtime,policy,step):
    pos,quat=runtime._robot.get_world_pose()
    return dict(position=np.asarray(pos),quaternion=np.asarray(quat),linear_velocity=np.asarray(runtime._robot.get_linear_velocity()),angular_velocity=np.asarray(runtime._robot.get_angular_velocity()),joint_pos=np.asarray(runtime._robot.get_joint_positions()),joint_vel=np.asarray(runtime._robot.get_joint_velocities()),smoother=runtime._action_smoother.snapshot(),previous_action=runtime._previous_action.copy(),proprio_history=step.observation.proprio_history,action_history=step.observation.action_history,policy={k:v for k,v in vars(policy).items() if v is None or isinstance(v,(str,int,float,bool))})

def run(case,condition,out):
    from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
    from benchmark.drive_model import DriveModel
    from scripts.run_round4_fixed_reference import load_transformed_reference
    from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
    from training.action_protocol import load_action_protocol
    source=OLD/f'episode_hard_0_e1_{case}'/'states.npz'
    ref,provenance=load_transformed_reference(source,case_id=case)
    episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
    protocol=load_action_protocol(DEFAULT_CONTRACT)
    out.mkdir(parents=True,exist_ok=False)
    write(out/'config.json',dict(case=case,condition=condition,source=provenance,post_event_s=8,diagnostic_only=True,intervention='main-speed floor 1.0 rad/s in first posthandoff normal direction segment only',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    runtime=IsaacBenchmarkRuntime(IsaacRuntimeConfig(scene_root=DEFAULT_SCENE_ROOT.resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,record_video=False,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),diagnostic_open_plane=True,diagnostic_physics_trace=True,diagnostic_auxiliary_target_rate_rad_s=math.radians(30),diagnostic_auxiliary_velocity_limit_rad_s=.30))
    runtime.set_reference_visualization_path(ref.points_xy)
    code=1
    try:
        kwargs=dict(lookahead_m=.5,yaw_rate_feedback_gain_s=.15,drive_model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE))
        if condition=='candidate':
            from candidate import RecoveryOracle
            policy=RecoveryOracle(protocol,**kwargs)
        else:policy=EventOracle(protocol,**kwargs)
        policy.reset(EpisodeContext(episode_id=episode.episode_id,privileged=True,oracle_path_xy=ref.points_xy,action_protocol_id=protocol.protocol_id,kinodynamic_reference=ref.waypoints))
        step=runtime.reset(episode)
        positions=[];proprios=[];quats=[];actions=[];times=[];event=None;anchor=None;before=None;after=None
        transform={'rotation':np.eye(2),'translation':np.zeros(2),'angle':0.}
        for block in range(351):
            obs=step.observation
            if event is None and policy.ready:
                event=block;anchor=int(policy._path_index)
                before=snapshot(runtime,policy,step)
                if condition=='aligned':
                    new,transform=align_reference(policy._reference,anchor,obs.position_world_xy,obs.yaw_world_rad)
                    policy._reference=new
                    policy._reference_xy=np.asarray([(w.x_m,w.y_m) for w in new])
                    policy._oracle_path_xy=policy._reference_xy.copy()
                after=snapshot(runtime,policy,step)
                write(out/'entry.json',dict(block=block,anchor=anchor,handoff_index=policy.handoff_index,before=before,after=after,transform=transform))
            positions.append(obs.position_world_xy.copy());proprios.append(obs.proprio_history[-1].copy());quats.append(obs.quaternion_world_wxyz_history[-1].copy());times.append(step.simulation_time_s)
            if block==350 or (event is not None and block-event>=80):break
            goal=policy._reference_xy[-1]-obs.position_world_xy
            c,s=math.cos(obs.yaw_world_rad),math.sin(obs.yaw_world_rad)
            pg=np.array([c*goal[0]+s*goal[1],-s*goal[0]+c*goal[1]])
            action=policy.act(obs,pointgoal_base_xy=pg)
            actions.append(action.copy())
            step=runtime.execute(action)
        if event is None:raise RuntimeError('handoff-progress event not reached')
        canonical=np.asarray(positions).copy()
        canonical[event:]=canonical_xy(canonical[event:],transform)
        np.savez(out/'trace.npz',position=positions,canonical_position=canonical,proprio=proprios,quaternion=quats,action=actions,time=times,reference_xy=ref.points_xy,reference_yaw=ref.yaw_rad,reference_direction=[1 if w.drive_direction=='forward' else -1 for w in ref.waypoints],reference_command=[(w.main_command_rad_s,w.auxiliary_angle_rad) for w in ref.waypoints])
        np.savez(out/'physics.npz',**runtime.physics_diagnostics)
        write(out/'tracking.json',list(policy.tracking_records))
        write(out/'status.json',dict(complete=True,event=event,anchor=anchor,blocks=len(actions),policy_act_calls=len(policy.tracking_records)))
        print('PROBE_COMPLETE',case,condition,event,len(actions),flush=True)
        code=0
    except BaseException:
        write(out/'failure.json',{'traceback':traceback.format_exc()});raise
    finally:runtime.close(return_code=code)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--condition',choices=['real','aligned','candidate'],required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();run(a.case,a.condition,a.out)
