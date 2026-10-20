"""Native data completeness, source identity and causal-input clock checks."""
from pathlib import Path
import json,hashlib,numpy as np

D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def validate(condition,status):
    root=D/(condition+'_run');cfg=json.loads((D/'protocol.json').read_text());spec=cfg['acquisitions'][condition]
    assert status['status']=='finished' and status['child_returncode']==0,status
    assert not (root/'exception.txt').exists(),'Capture exception'
    terminal=json.loads((root/'terminal.json').read_text())
    assert terminal['exit_code']==0 and terminal['termination'] in ['request_program_complete','physical_contact']
    seal=json.loads((root/'capture_seal.json').read_text());assert seal['complete']
    names=['front_left','front_right','rear_left','rear_right'];assert seal['camera_names']==names
    rows=seal['rows'];assert len(rows)==terminal['frames'] and len(rows)>1
    native=json.loads((root/'native_imu.json').read_text());q=np.load(root/'requests.npz')['requests']
    assert np.array_equal(q,np.load(spec['request_program'])['requests'][:len(q)])
    assert len(native)==len(q)*5+1 and len(q)==terminal['executed_blocks']
    t=np.array([x['time_s'] for x in native]);assert abs(t[0])<1e-8 and np.max(abs(np.diff(t)-.02))<1e-6
    assert all(x['qualified'] and x['causal_before_next_action'] and not x['orientation_read'] for x in native)
    physics=np.load(root/'physics.npz');assert len(physics['simulation_time_s'])==len(native)
    assert np.max(abs(physics['simulation_time_s']-t))<1e-8
    previous=[-1]*4;times=[];maxpair=0.
    for row in rows:
        path=root/'frames'/row['frame'];assert sha(path)==row['sha256']
        a=np.load(path);clock=row['clock'];times.append(float(a['source_time_s']))
        assert abs(times[-1]-clock[0])<1e-8
        im=native[row['imu_index']];assert row['imu_index']==int(a['native_packet_index'])
        assert abs(im['time_s']-times[-1])<1e-8
        assert row['imu_index']==row['request_blocks_executed']*5
        maxpair=max(maxpair,row['pair_error']);assert maxpair<=2e-5
        for j,name in enumerate(names):
            assert a[name+'_rgb'].shape==(480,640,3) and a[name+'_rgb'].dtype==np.uint8
            assert a[name+'_depth_label'].shape==(480,640) and not np.isnan(a[name+'_depth_label']).any()
            assert np.isfinite(a[name+'_camera_usd_world_label']).all()
            expected=np.array(im['estimated_base_world'])@np.array(seal['initial_local_identity'][j]['local_matrix'])
            assert np.max(abs(a[name+'_camera_usd_world_estimated']-expected))<1e-12
            frame=int(a[name+'_render_frame']);assert frame>previous[j];previous[j]=frame
            assert abs(float(a[name+'_render_time'])-clock[2])<1e-5
            assert abs(im['end_raw_s']-clock[2])<1e-5
    assert abs(times[0])<1e-8 and abs(times[-1]-terminal['time_s'])<1e-8
    assert np.all(np.diff(times)>0) and max(np.diff(times))<=.500001
    assert rows[-1]['terminal'] and rows[-1]['request_blocks_executed']==len(q)
    if terminal['termination']=='request_program_complete':assert len(q)==spec['request_blocks']
    # Verify all recorded videos contain one image for each saved exposure.
    import cv2
    videos={}
    for name in names:
        c=cv2.VideoCapture(str(root/(name+'.mp4')));count=0
        while c.read()[0]:count+=1
        c.release();assert count==len(rows),(name,count,len(rows));videos[name]=count
    for p,h in cfg['source_sha256'].items():assert sha(p)==h,p
    for p,h in spec['input_sha256'].items():assert sha(p)==h,p
    return dict(complete=True,termination=terminal['termination'],frames=len(rows),images=len(rows)*4,
        native_IMU_packets=len(native),request_blocks=len(q),time_s=times[-1],max_pair_error=maxpair,
        four_video_frame_counts=videos,all_input_source_frame_SHA_verified=True,
        clock_and_causal_estimated_pose_checks=True,model=0,fit=0,navigation=0)
