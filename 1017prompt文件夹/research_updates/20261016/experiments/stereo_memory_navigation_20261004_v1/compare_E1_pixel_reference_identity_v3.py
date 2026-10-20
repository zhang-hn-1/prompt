"""Canonical absence of reference; preserve earlier null/empty adaptation failures."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_current_pixel_navigation_run_v1';B=H/'minimum_prefix_transition_navigation_run_v1';n=json.loads((D/'policy_records.json').read_text());o=json.loads((B/'policy_records.json').read_text());sh=json.loads((D/'B0_shadow_until_fork.json').read_text());different=[];historical=[]
def canon(v):return np.asarray([] if v is None else v).reshape(-1,2)
for i,row in enumerate(n):
 ref=canon(row['installed_reference_xy_after_act']);s=canon(sh[i]['B0_reference']);oldref=canon(o[i]['installed_reference_xy_after_act'])
 if ref.shape!=s.shape or not np.array_equal(ref,s):different.append(i)
 if ref.shape!=oldref.shape or not np.array_equal(ref,oldref):historical.append(i)
v={'same_stream_reference_different_frames':different,'historical_reference_different_frames':historical,'first_historical_reference_difference':None if not historical else historical[0],'no_policy_recall':True,'canonical_absence_only':True,'earlier_v1_v2_preserved':True};old=json.loads((D/'reference_identity_comparison_v2.json').read_text());assert old['same_stream_reference_different_frames']==different and old['historical_reference_different_frames']==historical;(D/'reference_identity_comparison_v3.json').write_text(json.dumps(v,indent=2)+'\n');print(len(different),len(historical))
