from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent
src=(H/'replay_fixed_appearance_hypothesis_source_v1.py').read_text()
# Both tests reconstruct the first source-ready prefix, without refitting a candidate.
for variant in ('truncate','poison'):
 s=src.replace("O=H/'fixed_appearance_hypothesis_source_v1'",f"O=H/'fixed_appearance_hypothesis_prefix_{variant}_v1'").replace('range(0,141,5)','range(0,71,5)').replace('zip(packets,native)','zip(packets[:351],native[:351])')
 insertion="\npackets[351:]=[];native[351:]=[]\n" if variant=='truncate' else "\nfor future in packets[351:]:\n future.clear();future.update({'FUTURE_POISON':float('nan')})\nfor future in native[351:]:\n future.clear();future.update({'FUTURE_POISON':float('nan')})\n"
 s=s.replace('R0=np.asarray(gauge',insertion+'R0=np.asarray(gauge')
 exec(compile(s,str(H/'replay_fixed_appearance_hypothesis_source_v1.py'),'exec'),{'__file__':str(H/'replay_fixed_appearance_hypothesis_source_v1.py')})
 O=H/f'fixed_appearance_hypothesis_prefix_{variant}_v1';original=H/'fixed_appearance_hypothesis_source_v1'
 for i in range(0,71,5):
  a=np.load(O/'possible_sets'/f'frame_{i:03d}.npz');b=np.load(original/'possible_sets'/f'frame_{i:03d}.npz');assert set(a.files)==set(b.files)
  for k in a.files:np.testing.assert_array_equal(a[k],b[k],err_msg=f'{variant} {i} {k}')
 np.testing.assert_array_equal(np.load(O/'native_velocity_pretruth.npz')['velocity_world_m_s'],np.load(original/'native_velocity_pretruth.npz')['velocity_world_m_s'][:351])
a={'decision':'PASS_FIRST_QUALIFIED_PREFIX_CAUSALITY_ONLY','cutoff_frame':70,'compared_source_epochs':15,'native_samples':351,'future_native_payload_poison_and_truncation_exact':True,'future_RGB_packets_not_opened_by_prefix_loop':True,'not_all29_output_poison_test':True,'not_source_quality_or_navigation_pass':True}
(H/'fixed_appearance_hypothesis_prefix_assessment_v1.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
