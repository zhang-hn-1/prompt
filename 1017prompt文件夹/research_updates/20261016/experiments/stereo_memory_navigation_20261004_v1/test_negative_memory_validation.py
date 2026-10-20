import numpy as np,pytest

def test_same_class_output_does_not_hide_different_ray_inputs_or_between_capture_physics(tmp_path):
 from negative_memory_validation import archives_equal
 a=tmp_path/'a.npz';b=tmp_path/'b.npz'
 np.savez(a,world=np.array([[0.,0.,1.]]),hypothesis=np.array([[0.,0.]]));np.savez(b,world=np.array([[0.,0.,2.]]),hypothesis=np.array([[0.,0.]]));assert not archives_equal(a,b)
 aa=np.zeros((701,3));bb=aa.copy();bb[347,0]=.1
 np.savez(a,root=aa);np.savez(b,root=bb);assert not archives_equal(a,b,prefix_samples=701)

def test_nominal_true_clearance_catches_segment_crossing_between_safe_endpoints():
 from negative_memory_validation import true_cylinder_clearance
 assert true_cylinder_clearance([[-2,0],[2,0]])==pytest.approx(-.35)
 assert true_cylinder_clearance([[-2,2],[2,2]])==pytest.approx(1.65)

def test_each_arm_errors_preserved_and_existing_evidence_never_overwritten(tmp_path):
 from negative_memory_validation import save_arm_errors_once
 historic=tmp_path/'bank_rock_observation_errors_posthoc_v1.npz';historic.write_bytes(b'historical')
 a=save_arm_errors_once(tmp_path,'R0',{'error':np.array([1.])});b=save_arm_errors_once(tmp_path,'R1',{'error':np.array([2.])})
 assert a!=b and historic.read_bytes()==b'historical'
 with pytest.raises(FileExistsError):save_arm_errors_once(tmp_path,'R0',{'error':np.array([99.])})
 assert np.load(a)['error'][0]==1.
