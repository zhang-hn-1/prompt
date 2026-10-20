"""Descriptive authored raw convex mesh check; does not replace the sealed gate."""
from pathlib import Path
import datetime, hashlib, json
import numpy as np
from scipy.spatial import ConvexHull
from scene_ray_geometry import camera_rays, convex_mesh_depth
from audit_static_scene_labels import metric

H = Path(__file__).resolve().parent

def main():
    protocol = json.loads((H/'scene_mesh_representation_protocol.json').read_text())
    def verify():
        for path, sha in protocol['inputs_sha256'].items():
            assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == sha, path
    verify()
    shapes = json.loads((H/'static_scene_geometry.json').read_text())['shapes']
    meshes = np.load(H/'static_scene_child_meshes.npz')
    prior = np.load(H/'static_scene_ray_measurements.npz')
    manifest = json.loads((H/'transaction_development_manifest.json').read_text())
    start = datetime.datetime.now(datetime.timezone.utc).isoformat()
    rows=[]; arrays={}; pooled={}; hulls={}
    for row in (r for r in manifest['samples'] if r['sequence']=='hard_0_e1_0004'):
        truth=np.load(row['truth_file']); frame=int(Path(row['rgb_file']).stem.split('_')[1])
        for eye in ['left','right']:
            prefix=f'{eye}_{frame:03d}'
            origin,rays,yy,xx=camera_rays(truth[eye+'_camera_usd_world'],480,640,584.8888888888889,319.5,239.5,2)
            norm=np.linalg.norm(rays,axis=-1)
            core=prior[prefix+'_sphere_core_selection']; ids=prior[prefix+'_expected_scene_id']
            mesh_depth=np.full(core.shape,np.inf); object_rows=[]
            for i in np.unique(ids[core]):
                i=int(i); shape=shapes[i]; selected=core & (ids==i)
                if i not in hulls: hulls[i]=ConvexHull(meshes[f'shape_{i:03d}_points_local']).equations
                rotation=np.asarray(shape['rotation'])
                local_origin=(origin-np.asarray(shape['centre']))@rotation
                local_rays=rays[selected]@rotation
                mesh_depth[selected]=convex_mesh_depth(local_origin,local_rays,hulls[i])
                actual=truth[eye+'_depth_m'][yy,xx]
                with np.errstate(invalid='ignore'):
                    error=np.abs(actual-mesh_depth)*norm
                error[~np.isfinite(error)|(actual<=0)]=np.inf
                object_rows.append(dict(path=shape['path'],raw_mesh_current=metric(error[selected]),hull_planes=int(len(hulls[i]))))
            comparisons={}
            for label_key, label in [('current',truth[eye+'_depth_m'][yy,xx]),('pretransaction',truth[eye+'_pretransaction_depth_m'][yy,xx])]:
                with np.errstate(invalid='ignore'): error=np.abs(label-mesh_depth)*norm
                error[~np.isfinite(error)|(label<=0)]=np.inf
                comparisons[label_key]=metric(error[core]); pooled.setdefault((eye,label_key),[]).append(error[core])
                arrays[prefix+'_'+label_key+'_raw_mesh_error_m']=error
            with np.errstate(invalid='ignore'):
                delta=(mesh_depth-prior[prefix+'_expected_axis_depth_m'])*norm
            finite=delta[core & np.isfinite(delta)]
            rows.append(dict(frame=frame,eye=eye,n=int(core.sum()),comparisons=comparisons,objects=object_rows,
                raw_minus_ideal_ray_m=dict(finite_n=int(finite.size),median=float(np.median(finite)) if finite.size else None,
                    p90=float(np.percentile(finite,90)) if finite.size else None,peak=float(finite.max()) if finite.size else None)))
            arrays[prefix+'_raw_mesh_axis_depth_m']=mesh_depth
            print('RAW_MESH',frame,eye,comparisons['current'],flush=True)
    result=dict(started_utc=start,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,
        groups={eye:{label:metric(np.concatenate(v)) for (e,label),v in pooled.items() if e==eye} for eye in ['left','right']},
        original_gate_preserved=False if json.loads((H/'static_scene_label_audit.json').read_text())['geometric_label_gate_passed'] else True,
        original_geometric_gate_passed=False,mesh_gate_defined=False,selection='same full original independent sphere core; no removed failed labels',
        limitations=protocol['scope']+'; raw convex hull is not exact Catmull-Clark or primary renderer equivalence; not an independent RGB semantic/silhouette label',
        new_simulations=0,new_inference_calls=0,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'scene_ray_geometry.py',H/'scene_mesh_representation_protocol.json']})
    np.savez_compressed(H/'static_scene_mesh_measurements.npz',**arrays)
    (H/'static_scene_mesh_decomposition.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    verify()

if __name__=='__main__': main()
