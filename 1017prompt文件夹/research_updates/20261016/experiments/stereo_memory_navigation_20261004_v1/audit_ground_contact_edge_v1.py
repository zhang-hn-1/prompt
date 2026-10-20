"""Postseal exact base edge projection diagnostic; not an image detector."""
from pathlib import Path
import numpy as np,json,hashlib
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';protocol=json.loads((H/'ground_contact_edge_protocol_v1.json').read_text());a=np.arange(protocol['samples'])*2*np.pi/protocol['samples'];world=np.c_[.35*np.cos(a),1.8+.35*np.sin(a),np.zeros_like(a)];rows=[]
for f in protocol['frames']:
    poses=np.load(D/'known_poses'/f'frame_{f:03d}.npz');gt=np.load(D/'frames'/f'frame_{f:03d}.npz');eyes=[]
    for eye in ['left','right']:
        T=poses[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1.,-1.,-1.]);camera=(world-T[:3,3])@R;uv=camera[:,:2]/camera[:,2,None]*584.8888888888889+np.array([319.5,239.5]);valid=(camera[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479);error=np.full(len(world),np.inf)
        ids=np.flatnonzero(valid);u,v=np.rint(uv[ids]).astype(int).T;z=gt[eye+'_depth_m'][v,u];measured=T[:3,3]+np.c_[(u-319.5)/584.8888888888889*z,(v-239.5)/584.8888888888889*z,z]@R.T;error[ids]=np.linalg.norm(measured-world[ids],axis=1)
        eyes.append(dict(eye=eye,projected_in_image=int(valid.sum()),actual_depth_matches_within1cm=int((error<=.01).sum()),visible_sample_ids=np.flatnonzero(error<=.01).tolist()))
    rows.append(dict(frame=f,eyes=eyes))
r=dict(protocol=protocol,rows=rows,no_actor_use=True,not_RGB_validation=True,no_model_admission=True,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(H/'ground_contact_edge_assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(rows,indent=2))
