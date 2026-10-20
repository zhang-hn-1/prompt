"""Postseal descriptive prospectiveA vs already-known developmentB; not a full3arm trial."""
from paths import *
import json,hashlib,numpy as np
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
B=W/'experiments/four_camera_weak_baseline_20261017_v1';A=D
for root in [A,B]:assert json.loads((root/'evaluation_receipt.json').read_text())['verified_before_after']
ar=json.loads((A/'e1_result.json').read_text());br=json.loads((B/'e1_result.json').read_text());asup=json.loads((A/'supported_memory_result.json').read_text());bsup=json.loads((B/'supported_memory_result.json').read_text())
records={}
for name,root,result,supported in [('A',A,ar,asup),('B',B,br,bsup)]:
    policy=json.loads((root/'e1_run/policy_records.json').read_text());req=np.load(root/'e1_run/requests.npz')['requests']
    no=np.array([r['time_s']>=6 and max(abs(v) for v in r['request'])==0 and r['plan'].get('status')!='success' for r in policy])
    run=0;first=None;maxrun=0
    for i,b in enumerate(no):
        run=run+1 if b else 0;maxrun=max(maxrun,run)
        if run>=50 and first is None:first=i-49
    records[name]=dict(root=str(root),termination=result['termination'],contact_stop=result['termination']=='physical_contact',actions=result['actions'],frames=result['frames'],sim_time_s=result['sim_time_s'],net_goal_progress_m=result['net_goal_progress_m'],path_length_m=result['path_length_m'],request_reverse_time_s=result['request_reverse_time_s'],tracker_pending_direction_time_s=supported['tracker_pending_direction_time_s'],source_supported_blind_denominator=supported['blind_previously_source_supported_target_queries'],source_supported_blind_missing=supported['missing_queries'],source_supported_blind_missing_fraction=supported['missing_fraction'],false_map_cell_seconds=result['memory']['false_map_cell_seconds'],max_false_map_cells=result['memory']['max_false_map_cells'],first_sustained_no_route_policy=first,first_sustained_no_route_time_s=policy[first]['time_s'] if first is not None else None,max_sustained_no_route_s=maxrun*.1,total_no_route_s=float(no.sum()*.1),source=result['source'],full_chain_P50_P90_P95_s=result['end_to_end_policy_execute_render_perception_P50_P90_P95_s'])
ra=np.load(A/'e1_run/requests.npz')['requests'];rb=np.load(B/'e1_run/requests.npz')['requests'];n=min(len(ra),len(rb));different=np.flatnonzero(np.max(abs(ra[:n]-rb[:n]),axis=1)>1e-8)
first=int(different[0]) if len(different) else None
# Pixel equality is evaluated, never assumed from seed. Only the common pre-divergence source prefix has matched actionhistory.
aps=json.loads((A/'e1_run/source_seal.json').read_text())['rows'];bps=json.loads((B/'e1_run/source_seal.json').read_text())['rows'];ac=json.loads((A/'e1_run/capture_seal.json').read_text())['rows'];bc=json.loads((B/'e1_run/capture_seal.json').read_text())['rows'];rgb=[]
for i,(s,t) in enumerate(zip(aps,bps)):
    block=ac[i]['request_blocks_executed']
    if first is not None and block>first:break
    with np.load(A/'e1_run/frames'/ac[i]['frame']) as x,np.load(B/'e1_run/frames'/bc[i]['frame']) as y:
        per={}
        for eye in ['front_left','front_right','rear_left','rear_right']:
            aa=x[eye+'_rgb'];bb=y[eye+'_rgb'];delta=abs(aa.astype(float)-bb.astype(float));per[eye]=dict(byte_equal=bool(np.array_equal(aa,bb)),MAE_uint8=float(delta.mean()),changed_pixels=int(np.any(aa!=bb,axis=2).sum()))
    rgb.append(dict(source_frame=i,request_blocks=block,per_eye=per))
Arec=records['A'];Brec=records['B'];gates=dict(minimum_visual_observation_support=bool(ar['minimum_observation_support'] and br['minimum_observation_support']),collision_noninferiority=not Arec['contact_stop'],blind_missing_noninferiority=Arec['source_supported_blind_missing_fraction'] is not None and Brec['source_supported_blind_missing_fraction'] is not None and Arec['source_supported_blind_missing_fraction']<=Brec['source_supported_blind_missing_fraction']+.05,latency_noninferiority=Arec['full_chain_P50_P90_P95_s'][1]<=Brec['full_chain_P50_P90_P95_s'][1]*1.1,net_progress_benefit=Arec['net_goal_progress_m']>=Brec['net_goal_progress_m']+.5,false_memory_benefit=Arec['false_map_cell_seconds']<=Brec['false_map_cell_seconds']*.8)
result=dict(complete=True,arms=records,first_action_divergence_policy=first,first_action_divergence_time_s=first*.1 if first is not None else None,shared_action_prefix_RGB_comparison=rgb,predeclared_development_gates=gates,all_development_gates_pass=all(gates.values()),scope='ProspectiveoneA vs knownpreviousB; source/control nominallyfixed; pixelprefix explicitlychecked, latertrajectories/source/targetdenominators diverge. Singledevelopmentrun perarm, no significance/generalization orfull3armC. DiscardedhistoryUNKNOWN not observedfree.',C='REJECT_OFFLINE_NO_PHYSICS',reliable_navigation=False,learning_increment=False,whole_research_complete=False,source_navigation_one_import_change=json.loads((A/'integration_receipt.json').read_text())['one_line_exact_import_change'])
(D/'comparison.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print('COMPARE',records,gates,'FIRST_DIVERGENCE',first,flush=True)
