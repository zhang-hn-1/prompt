"""Verify immutable prior artifacts, all denominators, metrics and version snapshots."""
from pathlib import Path
import datetime,hashlib,json
import numpy as np
from audit_static_scene_labels import metric
from scene_ray_geometry import camera_rays
H=Path(__file__).resolve().parent

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
    n=0
    for name,key in [('scene_ray_protocol.json','input_sha256'),('scene_mesh_representation_protocol.json','inputs_sha256'),('stereo_causal_verification.json','source_and_artifact_sha256')]:
        for path,expected in json.loads((H/name).read_text())[key].items():
            assert sha(path)==expected,path
            n+=1
    ideal=json.loads((H/'static_scene_label_audit.json').read_text()); mesh=json.loads((H/'static_scene_mesh_decomposition.json').read_text())
    assert not ideal['geometric_label_gate_passed'] and ideal['applicable_cases']==6
    assert not mesh['original_geometric_gate_passed'] and mesh['original_gate_preserved'] and not mesh['mesh_gate_defined']
    assert sha(H/'scene_ray_geometry_ideal_v1.py')==ideal['source_sha256'][str(H/'scene_ray_geometry.py')]
    for path,expected in mesh['source_sha256'].items(): assert sha(path)==expected,path
    for path,expected in ideal['source_sha256'].items():
        if path!=str(H/'scene_ray_geometry.py'):assert sha(path)==expected,path
    assert '86 passed' in (H/'scene_mesh_green.log').read_text()
    source=np.load(H/'static_scene_ray_measurements.npz'); raw=np.load(H/'static_scene_mesh_measurements.npz')
    samples=json.loads((H/'transaction_development_manifest.json').read_text())['samples']; pooled={}; counts={}; residual={}
    for row,mrow in zip(ideal['rows'],mesh['rows']):
        eye=row['eye'];frame=row['frame'];prefix=f'{eye}_{frame:03d}'
        assert (eye,frame)==(mrow['eye'],mrow['frame'])
        selected=source[prefix+'_sphere_core_selection']
        assert int(selected.sum())==row['independent_core_n']==mrow['n']
        sample=next(s for s in samples if s['sequence']=='hard_0_e1_0004' and Path(s['truth_file']).stem==f'frame_{frame:03d}')
        truth=np.load(sample['truth_file'])
        _,rays,_,_=camera_rays(truth[eye+'_camera_usd_world'],480,640,584.8888888888889,319.5,239.5,2)
        norm=np.linalg.norm(rays,axis=-1)
        for label_key,label_name in [('current',eye+'_depth_m'),('pretransaction',eye+'_pretransaction_depth_m')]:
            label=truth[label_name][::2,::2]
            with np.errstate(invalid='ignore'): expected=np.abs(label-raw[prefix+'_raw_mesh_axis_depth_m'])*norm
            expected[~np.isfinite(expected)|(label<=0)]=np.inf
            np.testing.assert_allclose(expected[selected],raw[prefix+'_'+label_key+'_raw_mesh_error_m'][selected],rtol=1e-14,atol=1e-12)
            assert metric(expected[selected])==mrow['comparisons'][label_key]
            pooled.setdefault((eye,label_key),[]).append(expected[selected])
        counts[eye]=counts.get(eye,0)+int(selected.sum())
        err=raw[prefix+'_current_raw_mesh_error_m'][selected]
        residual[eye]=residual.get(eye,0)+int((err>.01).sum())
    for (eye,label),errors in pooled.items(): assert metric(np.concatenate(errors))==mesh['groups'][eye][label]
    assert counts=={'left':73383,'right':23430} and residual=={'left':73,'right':893}
    artifacts=['scene_ray_geometry.py','scene_ray_geometry_ideal_v1.py','test_scene_ray_geometry.py','export_static_scene_geometry.py','audit_static_scene_labels.py','audit_static_scene_mesh.py','plot_static_scene_audit.py','verify_static_scene_audit.py','scene_ray_protocol.json','scene_mesh_representation_protocol.json','static_scene_geometry.json','static_scene_child_meshes.npz','static_scene_label_audit.json','static_scene_ray_measurements.npz','static_scene_mesh_decomposition.json','static_scene_mesh_measurements.npz','static_scene_geometry_export.log','static_scene_label_audit.log','static_scene_mesh_decomposition.log','scene_ray_red.log','scene_ray_green.log','scene_mesh_red.log','scene_mesh_green.log','static_scene_figures.log','static_scene_review.json','figures/static_scene_fixed_frame010.png','figures/static_scene_all_eye_frames.png']
    paths=[H/p for p in artifacts]+[H.parent.parent/'reports/benchmark_static_scene_geometry_and_depth_labels_20261004.md']
    result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=n,
        current_cpu_tests_passed=86,actual_archived_pairs=4,actual_eye_frames=8,applicable_ideal_gate_frames=6,
        original_ideal_gate_passed=False,mesh_gate_defined=False,denominators_verified=counts,remaining_mesh_failures_gt_0p01m=residual,
        original_ideal_helper_byte_exact_preserved=True,figures_inspected=True,new_simulations=0,new_model_inferences=0,rgb_admitted=False,whole_research_complete=False,
        source_and_artifact_sha256={str(p):sha(p) for p in paths})
    (H/'static_scene_verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print('VERIFIED',result['verified_utc'],'old SHA checks',n,'new artifacts',len(paths),'all denominators and metrics exact')
if __name__=='__main__':main()
