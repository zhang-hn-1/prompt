# Short obstacle branch — one first-pass review

**Critical 0 / Important 0 / Minor 2.** The completed two-arm branch and nine-exposure visibility audit support a preserved negative result. Neither arm passes the shortened obstacle task, and reliable memory navigation benefit is not established. Whole research remains incomplete.

This reviewer was reused after a new-agent capacity failure and retains historical context from the earlier occluded-memory review. This is **not an empty-context review**. It is the first and only review pass for the new short-obstacle branch. The new ongoing far-approach audit, any subsequent candidate, and other estimated-pose work are outside this review. No simulator/GPU run, controller change, scientific-code edit, old sealed-byte rewrite, or commit was made.

## Personally checked evidence

Read the report, short-obstacle spec/plan, task helper/tests, runner and stage driver, protocol/status, both completed assessments, assessor, comparison, visibility preanalysis/diagnostic, audit implementation, camera/stereo conventions, and relevant model disparity-update code. Compared the runner to its frozen masked-memory ancestor and reversed every declared derivation substitution to obtain the exact base text. The runner hashes also match the derivation.

Personally verified **every hash** in the 1,272-entry source manifest, 888 current-arm post-close entries, 2,407 history-arm post-close entries, 41 visibility-preanalysis inputs, and nine author-closure entries: all match, covering 4,588 distinct files. Source plus artifact counts reproduce 2,160/current and 3,679/history. The separate JSON records the manifests' identities and all checked hashes. This review uses file SHA closure; it does not invent a Git commit comparison for the previously unborn workspace.

Ran 16 bounded CPU tests: short task endpoints/validation, same-stream shadow, masked current shadow, stereo geometry, and prior-RGB policy contracts. **16 passed in 6.11 s.** This does not claim a reviewer rerun of the 410-test prelaunch suite.

For every call of both arms, matched the saved policy-input request to the primary record and all five subsequent physical raw-command pairs. Confirmed opposite-memory shadow roles, read-only/unexecuted status, same source ID/time, no alternative-future trajectory labels, current/past source age, original source rejection, and no global-reference or terminal-candidate consumption. Maximum source age is 0.3999999910593033 s. The runtime returns the primary request after the shadow and retains state/clock immutability assertions. This is not a physical shadow-on/off determinism experiment.

Independently recomputed task endpoints and actual results:

| Arm | Calls / captures / physics / source inferences | Termination | Final distance | Goal-line progress | Contact samples | Entirely zero request blocks |
|---|---|---|---:|---:|---:|---:|
| Current | 120 / 121 / 601 / 25 | collision at 12 s | 2.049916869436414 m | 0.550131422281265 m | 6 | 75 / 120 |
| History | 350 / 351 / 1751 / 71 | timeout at 35 s | 2.0555776632014413 m | 0.5444229662418363 m | 0 | 350 / 350 |

Neither actual trajectory crosses the 1.3 m goal-line midpoint. History's complete ten-component request is zero throughout. Its physical main applied command is also zero; auxiliary roll-PID correction reaches about 0.03070 rad. Thus its displacement is passive motion with closed-loop stabilization, not commanded forward navigation. At frame 60 history records 58 historical occupied cells, zero current occupied cells, and `observed_obstacle_blocks_route`. These facts support the report's failure interpretation, not validation of retained hazards.

The declared new task has a 2.6 m span, unchanged original height/quaternion, 35 s budget, 0.2 m goal tolerance and no-contact criterion, with a further actual midpoint-crossing check. Inspected unchanged policy/controller, 0.3 main request cap, 0.3 auxiliary limit, CAD masking, rig, lighting, priors, and stereo source. Original 6.6 m failures and nominal/terminal rejections are not rescored. Inherited oracle-path metadata and SPL are explicitly excluded from acceptance. Renderer truth is retrieved after the source packet is fixed and is used only for retrospective scoring.

Recomputed source endpoint accuracy across all 25/71 saved source packets: current has 1,958 candidate queries and history 9,004, with **zero candidate endpoints within 0.10 m** of finite renderer truth in either arm. Old packet counts are 2,820/11,019, also with zero correct endpoints. Query/partner identities and old/new retained-world machine-epsilon comparisons remain consistent. This is independent scoring from saved packets, not a new network inference or CAD reconstruction.

At both frame 0 and frame 60, current/history saved poses agree exactly while left and right RGB differ. The physical-arm comparison therefore retains a reset/RGB confound; it cannot isolate a unique causal memory effect. The far-start historical comparison also changes approach pose, including yaw, and supplies bounded observability evidence rather than a controlled distance-only effect.

## Visibility audit verification

Independently recomputed all nine exposures using homogeneous USD camera-to-world matrices and an explicit inverse right-camera transform. Applied the USD-to-native optical axis conversion `(x, -y, -z)`. Back-projection into the left image agrees within 0.000071 pixels, providing a numerical check on the transform/sign interpretation. The frozen scene's divider cylinders all have radius 0.35 m and height 1 m; the audit's radius/height allowance remains an offline geometric proxy, not an object-instance ID.

All discrete counts checked against the saved diagnostic agree exactly. For the six short-arm exposures at frames 0, 60 and 110, left unmasked proxy counts are 157,187–160,938, and **right-inside count is zero**. Projected right vertical coordinates are outside the native image, with even the nearest sampled upper bound below −43 pixels. At current frame 60, two accepted proxy queries exist, but neither is within 0.10 m of truth.

Far-start frames 0/60/110 have respectively **31,830 / 23,252 / 21,987** bilaterally unmasked, right-depth-consistent proxy pixels, and **1,599 / 2,914 / 3,910** source proxy queries accurate within 0.10 m. The 0.02 m right-depth agreement rule is a diagnostic classification; it is not a revised source gate. The nine sampled exposures do not prove every exposure shares the same geometry.

Inspected native vertical rectification and the model forward path: `max_disp` defines cost-volume extent; recurrent updates add residual disparity and the inspected output path does not clamp disparity to 416. Accordingly, fB/416 is a nominal search scale, not a hard output-depth bound. fB/479 is only a necessary idealized native-overlap scale. The actual projected coordinates establish this sampled field-of-view loss; increasing the network search extent alone cannot create missing physical correspondence. No claim is made that a future maneuver or memory change has already fixed the problem.

## Inherited evidence, not personal reruns

Full CAD mesh projection/source-packet reconstruction (550/1,562 exact packet/mask fields), primary-map reconstruction (720/2,100 arrays), opposite-memory shadow diagnostics (1,440/4,200 fields), complete decoding of the four 121/351-frame videos at 10 Hz, and the 410-test prelaunch suite remain author evidence. Their stored results and sealed identities were inspected, but these computations were not independently rerun here. The earlier reviewer CPU environment lacked `pxr`, so no CAD reconstruction is implied by the saved-packet scoring above.

## Findings

### Critical

None found within this review's scope.

### Important

None found within this review's scope. The negative scientific verdict is correct; review acceptance is not navigation acceptance.

### Minor 1 — movement-period metrics become a single final sample when no movement is requested

`assess_short_obstacle_navigation.py:66–67` sets `move=N` when no nonzero main request exists, then computes `cross[N:]`. Because states contain N+1 samples, history's `movement_cross_RMS_m` and `movement_cross_peak_m` are both 0.0016086449846630657 m from **one final state**, despite there being no commanded movement interval. These values are misleadingly named and should not be compared to current's 61-state post-request interval. The report does not promote them into a pass, and the actual task gate does not depend on them.

Recommended disposition: append a correction declaring history movement-period metrics unavailable/no commanded interval, recording zero qualifying requests and the one-sample provenance of the old values. Retain the correctly calculated full-trajectory cross-track metrics. Preserve sealed assessment bytes; any future assessor should explicitly represent this case as unavailable.

### Minor 2 — sealed identity retains a 2.0 m local-goal label

`run_short_obstacle_navigation.py:158` retains `local_goal_m=2.0` from its ancestor, and both sealed `identity.json` files contain that value. The actual episode, protocol and independent endpoint recomputation instead specify the new 2.6 m start-to-goal span. The actual gate and policy consumed the correct goal, so this is an evidence-metadata inconsistency rather than an executed control error.

Recommended disposition: append an explicit identity correction pointing to `task_episode.json` and the 2.6 m declared span. Do not rewrite the frozen runner or sealed artifacts to hide the mismatch.

One review pass only. Both shortened-task gates remain FAIL, reliable memory benefit remains unestablished, and whole research remains incomplete.
