# First independent active-course observation review

Result: no critical control-flow findings; one important experiment-identity finding blocks launch until corrected. This is the FIRST review, including the prelaunch scope supplement, not another review of neutral or ablation results.

## Important I1 — Generated execution labels remain neutral

`run_active_course_observation_v1.py:182` generates `neutral_bank_visibility_probe=True`; line 184 labels the artifact manifest `qualified_hazard_with_one_neutral_bank_sensing_probe`. The scope supplement correctly supersedes inherited protocol plan/experiment/controller labels and matches the protocol hash, but is not represented in these output artifacts. The observation request changes both main speed and bank; correct output identity matters for downstream interpretation. Replace these generated labels with explicit active-course labels and record the observer override separately from the normal B0 cap. Update frozen runner SHA. This requires a metadata fix, not a scientific retry or repeat experiment.

## Personally checked

- Six small probe tests passed: 6 passed in 0.20s, bytecode disabled, pytest cacheprovider disabled.
- All 1290 source SHA entries match the filesystem; no inherited neutral source entry changed.
- Original runner comparison matches the supplied diff. Source/perception, camera rig/light, PID gains/runtime, supervisor and handoff code are inherited unchanged.
- Source/current clock checks occur before velocity/nominal payload conversion in the new helper. Original detector validates monotonic frame/time identity, so repeated source frames cannot supply the second deficient update. Handed-off latch gates the trigger.
- Event course comes only from current world XY velocity at speed >=0.08. Runtime capture takes world velocity into the latest proprio row; policy time comes from the current captured packet. No future velocity/action/truth enters the new control consumer.
- Low event speed fixes event course to None: zero requests through the six-second deadline, no later course invention, circular wait, or forced start.
- Positive wrapped current course difference >=10 degrees permanently stops turn requests; negative differences and low current speed do not count. Four-second turn deadline and six-second observation deadline depend on immutable event_time; no retrigger/reset.
- B0 act occurs once per frame. Nominal, pre-probe, effective request and execution snapshots are separately available. Physical behavior still requires post-run execution evidence; command alone cannot establish bank, course, displacement, or stopping.
- +.5/-30 maps to desired bank -22.5 by the original .75 interface. No new dynamic response model is consumed, and code explicitly logs physical_response_prediction_used=False. This is a bounded observation experiment, not a single-factor PID A/B or source/model/navigation admission.
- New stage wrapper was read only: one output/status path, review closure and source checks before child launch, 900-second timeout plus up to 30-second graceful termination. No launch performed. Active run output did not exist at check time.

## Evidence limits

Author full-suite log reads 516 passed in 24.42s; reviewer did not rerun the full suite. Reviewer did not rerun steady-response calibration, simulation, inference, or motion tests. Source acceptance, contact-free movement, >=0.05m displacement, >=10-degree real course and camera-view evidence remain pending actual execution and assessment. The plan explicitly permits one action-block quantization at the six-second boundary and does not promise physical stopping during the final two seconds.

Machine-readable findings and reviewed-file hashes are saved beside this report in findings.json. No author files changed, no commits/deletions, and no rereview loop requested.
