import json,hashlib,math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from benchmark.drive_model import DriveModel
from measure import geometric_curvature,first_sustained,reversal
HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'oracle_fixed_reference_pacing_20260928_v1'
ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark')
model=DriveModel.from_json(ROOT/'research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json')
out={};hashes={};fig,axes=plt.subplots(4,3,figsize=(16,13))
row=0
for case in ['0002','0004']:
    out[case]={}
    for group in ['real','reserve']:
        p=SOURCE/f'{case}_{group}'
        for name in ['physics.npz','trace.npz','tracking.json']:
            f=p/name;hashes[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
        ph=np.load(p/'physics.npz');z=np.load(p/'trace.npz');records=json.loads((p/'tracking.json').read_text());t=ph['simulation_time_s'];t=t-t[0]
        rt=z['time'][:-1]-z['time'][0]
        event=reversal(records)
        assert event is not None
        block,sign=event;et=rt[block];pi=int(np.searchsorted(t,et-1e-6));stop=min(pi+400,len(t))
        channels={'request':ph['clipped_target'][:,1],'smoothed':ph['smoothed_target'][:,1],'applied':ph['applied_action'][:,1],'actual':ph['joint_position_rad'][:,1]}
        timing={}
        for name,a in channels.items():
            j=first_sustained(a[:stop],np.radians(10),sign,pi)
            timing[name+'_opposite10_s']=None if j is None else float(t[j])
        j=first_sustained(channels['actual'][:stop],0,sign,pi)
        timing['actual_zero_s']=None if j is None else float(t[j])
        geo=geometric_curvature(ph['position_world_xy']);ref=z['reference_xy'];dense=np.concatenate([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.01))+1)) for a,b in zip(ref[:-1],ref[1:])]);error=cKDTree(dense).query(ph['position_world_xy'])[0]
        valid=np.array([x['source']=='tracking' for x in records]);win=(np.arange(len(rt))>=block)&(np.arange(len(rt))<block+40)&valid
        rows=[x for x,w in zip(records,win) if w]
        actual_end=first_sustained(channels['actual'][:stop],np.radians(10),sign,pi)
        geometry_event=first_sustained(geo[:stop],.1,-sign,pi,count=15)
        out[case][group]={'travel_until_actual_opposite10_m':None if actual_end is None else float(np.linalg.norm(np.diff(ph['position_world_xy'][pi:actual_end+1],axis=0),axis=1).sum()),'offline_geometry_opposite_curvature_s':None if geometry_event is None else float(t[geometry_event]),'lookahead_requested_m_at_event':records[block]['lookahead_requested_m'],'event_request_s':float(et),'event_record_index':block,'reference_index_at_event':records[block]['projection_index'],'target_sign':sign,**timing,'actual_angle_at_event_deg':float(np.degrees(channels['actual'][pi])),'event_xy_speed_m_s':records[block]['world_xy_speed_m_s'],'event_error_m':float(error[pi]),'next4s_peak_error_m':float(error[pi:min(pi+201,len(t))].max()),'next4s_tracking_blocks':len(rows),'next4s_outside_v1_support_blocks':sum(x['requested_curvature_outside_v1_support'] for x in rows),'next4s_protocol_bound_blocks':sum(x['angle_target_at_bound'] for x in rows)}
        for name,a in channels.items():axes[row,0].plot(t,np.degrees(a),label=name)
        req=np.array([x.get('requested_curvature_1_m',np.nan) if x['source']=='tracking' else np.nan for x in records])
        axes[row,1].plot(rt,req,label='requested');axes[row,1].plot(t,geo,label='XY chord curvature');axes[row,1].set_ylim(-2.5,2.5)
        axes[row,2].plot(t,error,label='same reference distance m')
        for ax in axes[row]:ax.axvline(et,color='k',ls=':');ax.set_xlim(max(0,et-4),min(35,et+8));ax.grid(alpha=.3);ax.legend(fontsize=8);ax.set_xlabel('s')
        axes[row,0].set_ylabel(case+' '+('B0' if group=='real' else 'pacing')+' angle deg')
        row+=1
    commands=z['reference_command'][:-1];angles=np.clip(commands[:,1],-math.pi/6,math.pi/6);s=np.linalg.norm(np.diff(ref,axis=0),axis=1);transitions=[]
    for i in range(1,len(commands)):
        if np.sign(commands[i,0])!=np.sign(commands[i-1,0]):continue
        duration=float(s[i]/model.speed(*commands[i]));change=abs(angles[i]-angles[i-1]);targettime=change/np.radians(30);nominalservo=change/.3
        transitions.append({'reference_index':i,'from_aux_deg':float(np.degrees(angles[i-1])),'to_aux_deg':float(np.degrees(angles[i])),'next_primitive_nominal_s':duration,'target_slew_time_s':float(targettime),'nominal_030_rate_time_s':float(nominalservo),'target_slew_exceeds_whole_next_primitive':bool(targettime>duration),'nominal_030_time_exceeds_whole_next_primitive':bool(nominalservo>duration)})
    out[case]['reference_transitions']=transitions
fig.tight_layout();fig.savefig(HERE/'turn_response.png',dpi=150)
(HERE/'measurement.json').write_text(json.dumps(out,indent=2)+'\n')
(HERE/'source_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
for case in out:
 for key,v in out[case].items():
  if key!='reference_transitions':print(case,key,json.dumps(v))
 ts=out[case]['reference_transitions'];print(case,'transition count',len(ts),'slew warnings',sum(x['target_slew_exceeds_whole_next_primitive'] for x in ts),'rate warnings',sum(x['nominal_030_time_exceeds_whole_next_primitive'] for x in ts))
