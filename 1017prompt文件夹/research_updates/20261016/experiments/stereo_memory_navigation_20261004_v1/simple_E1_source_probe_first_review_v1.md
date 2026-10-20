# Simple E1 source probe — one scoped first review

Reviewed 2026-10-08. Read-only code review; only this findings file was written. No physics process was launched. Scope: the prescribed plan, new driver/stage/test/protocol, and directly called capture/source/identity/cleanup/runtime interfaces. This is the first review only, not a review loop.

## Critical

None found.

## Important

1. **The task episode export contradicts the actual original E1 / 14 s identity.** The driver calls `export_task_identity(out/'task_episode.json', episode)` at line 52. `native_probe_identity.py:5` unconditionally writes `actual_task='native_current_motion_readonly_preknown60s'` and `not_original_E0_E1_E2=True`. Therefore this new run will publish mutually inconsistent identities: its actual episode is `golden_e1_0001`, while its task export denies original E1 and names a different 60 s experiment. The new `identity.json` role does not correct the earlier artifact. Use an explicit probe-specific task identity (or parameterize the helper) retaining no-navigation / SPL-inapplicable qualifications while stating original E1, 14 s, prescribed commands, and current GT-pose registration.

2. **Collision on the final action is sealed as complete despite the driver's incomplete stop classification.** At driver line 180, any collision sets `termination='collision_stop_incomplete'`. If it occurs on block 139, all three count checks at line 188 nevertheless pass; lines 189–190 then write `complete=True` and return code 0. Stage lines 27–29 consequently classify and post-close-seal that output as complete. Earlier collisions fail the count check, so the last block is an inconsistent edge case. Completion must also require the normal budget termination, or collision must have an explicitly consistent completion policy throughout termination, identity, exit status, and post-close manifest.

## Minor

1. **The new focused test checks strings rather than the changed behavior.** `test_simple_E1_source_probe_v1.py:5–17` proves textual presence/absence but cannot catch either incorrect task identity or final-action collision completion above. Add a small behavior check of probe-specific identity and a completion predicate for normal versus collision termination. This does not require simulator startup.

## Verified evidence and scope limits

- All 2,010 source hashes and all 32 input hashes matched when read. The new driver is present in the source hash set; hash maps were checked programmatically rather than reproduced here.
- `BenchmarkEpisode.from_dict` succeeded in the existing ball_wm Python environment. `golden_e1_0001` verifies its original `${SCENE_ROOT}/benchmark_e1/e1_detour.usda` and `e1_detour_bodyband.npz` hashes. Start is approximately `(-3.3, 0, .45)`, goal `(+3.3, 0, .45)`, quaternion `(1,0,0,0)`; no episode replacement is present.
- The frozen request artifact is finite float32 `(140,10)` and exactly equals the first 140 rows of the sealed TaskH parent `live_paired_texture_navigation_run_v1/requests.npz` `(505,10)`. It is saved to the new output before runtime launch. The loop uses each prescribed row directly; no planner, adaptive observation request, appearance binding, moving-goal rescue, or navigation-success stop is present.
- Current GT camera transforms and root position feed registration, explicitly marked upperbound in source rows and final role. CAD selection uses current public joint columns 9:12 after clearing privileged proprio columns 0:9. Native IMU pose is exported separately and marked unused by control. Frozen PID `.4/.1/.7`, inner caps, camera profile, and source components are retained.
- Per-NN-exposure prediction, packet, partial observation, and provenance are written before GT depth is read for the diagnostic frame. Full error-label assessment must remain after the fresh whole-sequence post-close seal, as prescribed; it is not implemented by this driver. Generic height and paired texture support replace the old colour selector; no old cylinder-only semantic acceptance is executed.
- The stage uses one status/log/output identity, one subprocess, a 600 s actual wait and bounded termination/kill cleanup, with no retry. Driver finalization uses the existing export-then-close helper; the stage seals after child closure. Counts check 140 requests / 141 captures / 29 NN exposures. The runtime's existing five ticks per block and initial trace imply 701 physics rows, but final runtime verification should check that count and final time explicitly.
- No test suite was rerun here. Parent reported 13 passed / 4 import errors from missing pxr path and is handling native-environment focused tests/import validation. This review does not assert fresh runtime validity or source geometry/semantic admission.

Assessment: fix the two Important findings before the single authorized physics run. No Critical finding and no request for another review loop.

## Late scoped assessor inspection (same first review)

The parent supplied `assess_simple_E1_partial_source_probe_v1.py` during this review; it was also inspected without executing it. It checks the stage-recorded post-close seal digest and every sealed file, freezes current input/source hashes, compares actual requests, verifies 141 capture clocks / states, 701 physics rows, 29 source rows, 141 decoded video frames, and current-GT pose equality before evaluating frozen subset depth-error gates. Height-band precision is clearly diagnostic and full semantic/coverage admission remains separate. This is consistent with post-seal error labeling.

Additional **Minor**: assessor lines 19–29 check only row count and `i%5==0`; they do not enforce the exact unique source frame sequence `[0,5,...,140]`, row times, or `current_GT_pose_registration` declarations, and line 14 checks only `identity['complete']`. Enforce the exact source indices and probe/episode identity so duplicated/misidentified records cannot satisfy qualification counts. This is evidence validation hardening; the inspected driver presently emits the expected indices and GT flags.

Parent subsequently reported the corrected native USD environment suite passed 17/17 and a failing identity regression was added for the required eighteenth test. Those are parent-provided results; this review itself did not rerun tests.
