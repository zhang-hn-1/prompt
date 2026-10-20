# Frozen velocity-pursuit candidate: native post-goal physical measurement

**REJECT under prospectively frozen V2 measurement gates.** Candidate improves localtracking and qualifies five low-speed inside-goal samples after2.1s, but briefly exits the original20cm goal radius (peak20.2711cm). Original V1 end-progress REJECT also remains unchanged. No candidate installed, no further equivalent repetitions/gain scans, no formal35s navigation result. This task is a four-point known localreference, not the original obstacle-crossing path.

## Measurement question and unchanged implementation

V1 nominal runs ended at different goal-disk entry times. The earlier closed geometric audit explains why their end-progress values cannot alone prove which moved slower; it does not revise V1 gate or admit the candidate. The prior engines were closed and had never measured postgoal physical behavior, so V2 ran exactly two wholedevelopment/regression sequences to obtain that missing tail, preserving the same knownprefix/fullreference/currentcandidate/rig/light/source/PID/caps. Prefixes already informed this measurement and are not freshheldout/unseen scene.

Existing native `RunnerConfig(diagnostic_settling_budget_s=3.)` retains original35s total, .2m goal tolerance, .05m/s and five distinct10Hz low+inside samples. At nominal goal entry, the runner sends zero request blocks and does not call policy.act. Current speed or leaving the goal resets the low counter. No new stop-controller policy or threshold. Recorded mask/actualdispatch distinguishes this tail from cached old tracking fields. Native diagnostic timing does not redefine nominal time-to-goal. No claim beyond the last recorded qualification/budget sample.

Protocol frozen01:08:02UTC, actual two-arm stage01:08:55–01:18:37UTC, both exit0. Source/code/config identities checked before initialization and afterclose. Runner derivation changes only protocol/output names and the existing opt-in settling option. Copied metadata such as2.6m local_goal, observation task labels, inherited original reference-length/SPL, old recovery wording and old accepted-controller strings describe ancestral experiments; current actualgoal is[-1.06468233,.18290872], fixed four-point endpoint. They are not used to score currenttask, SPL or originalnavigation, nor evidence that a new recovery occurred. Actual policy class, reference hash, request stream, task_episode and this explicit scope govern conclusions.

## Actual physical outcomes

| Measurement | B0 | Frozen velocity-pursuit |
|---|---:|---:|
| Nominal entry |25.7s|25.5s|
| Physical observation ended |28.7s|27.6s|
| Zero-request tail blocks |30|21|
| Largest postentry goal distance |.195152m|.202711m|
| Final goal distance |.134690m|.194871m|
| Five low+inside qualification delay |not reached within3s|2.1s|
| Contact samples |0|0|
| Policy calls / captures / physics |257 /288 /1436|255 /277 /1381|
| Source inference updates |58|56|

On the independently measured commonwindow18–25.5s, reference progress .692783→.706974m, lateralRMS .058695→.033200m, peak .118410→.062459m. Both spatial checkpoints pass. Tracking improvement is real on this localtask. However, “all measured postentry positions remain inside originalgoal” is FALSE for candidate, so V2 overall REJECT; eventual qualification does not erase the excursion. Candidate cannot be installed or promoted to formalnavigation based on this result.

## Causal execution and physical separation

First real request difference remainsframe187/18.7s while measuredcoursevalid. Before it, 936 actual physics samples×17 fields match exactly, 188 execution records match (initialNone plus187 complete snapshots), and561 originalpolicy field comparisons match. 445 appliedticks differ over the common recorded physical duration. Each arm's entire oldV1 nominal prefix also reproduces exactly in raw requests, states and17 physics fields: additional tail did not silently change earlier behavior.

Afterzero rawrequests, applied main first becomeszero .62s later, consistent with existing acceleration smoothing. Maximum actual speed .20611/.15000m/s, and even afterapplied mainzero peaks .19737/.13993m/s. B0 actualvelocity changes sign twice relative to its entrycourse; candidate zero such changes. Maximum low+inside run B0four vs candidatefive. B0bank ranges−.37066 to+.06320rad in the tail, candidate−.23876 to+.01986rad. These are measured execution/physical effects, not a proof that PID or smoothing alone causedfailure. No counterfactual stop-controller comparison was run.

## Verification evidence and limits

Both postclose assessors exited0 and verified allsource/CAD packets, masks/currentjoints, RGB/posewhitelist and timestamps, readonly map/shadow reconstruction and future-payload trap; 3460/3402 identity checks,1276/1232 packet/maskfields and1542/1530 primarymaparrays. Allactualdispatch blocks match physics; all30/21 tail requests exactlyzero, tailmasktrue and no newpolicycalls. Every frozen reference matches, no replanning/reset. Videos decoded completely: bothviews288/277 frames at10Hz. FullCPU suite469 passed23.24s (author); V1 independentreview469 passed24.03s, C0/I0/M1 with snapshot-count clarification appended. **That V1 review does not certify V2**; V2 author verification currently separate.

Actualcode/protocol/status/rawrecord/manifests/assessments/comparison/zero-tail audit and videos are under `experiments/stereo_memory_navigation_20261004_v1` with `velocity_goal_settling_*_v2` names. Plan `docs/superpowers/plans/2026-10-07-velocity-goal-settling-measurement.md`. Chart generated fromrawpositions/speeds/appliedtargets and personally inspected; originalbytes/gates remain.

![Physical goal-stop tail](../experiments/stereo_memory_navigation_20261004_v1/velocity_goal_settling_actual_v2.png)

The remaining navigation blockers include unreliable moving stereo-source coverage and incomplete estimated-pose integration, not merely reference tracking. Continue a bounded currentRGB corroboration of source-traceable memory, keeping originalgeometryage separate fromcurrentappearanceevidence. No more equivalent physics repeats or new PID/controller variant in this completedexperiment. Frozen geometrycandidate not installed; B0 stays unchanged. Fullreference/originalgoal/estimatednavigation/E0E1E2 and whole researchremain unfinished.


Independent reviewer identified an inherited metadata flag: sealed identity.json extra_physics_tail=False is WRONG for current native V2 diagnostic. Actual30/21 zero-dispatch blocks, settlingmasktrue, longer rawphysics/captures and current report are authoritative; preserved old identitybytes are not evidence of no tail. Source/timing and REJECT unchanged. 2026-10-07T01:24:20.875398+00:00 OldprefixSHA 11c01561148928a2cdb3034804121f814d3ef98a18f07a9af36af113ac5cd2dc


Bank-feedback scope clarification 2026-10-07T01:24:42.533040+00:00: zero_tail_audit bank ranges are10Hz execution snapshots of PID last.bank_rad (saved previous-tick feedback), not50Hz quaternion extrema of the whole physical tail. They characterize controller feedback samples only. All displacement/speed/qualification claims are independently measured; no solePID causal attribution. OldprefixSHA ec7382e2597aa30380098d2ba0a9cea47061870604f5f04cbac7ce3c7f0d6232


Independent first V2 review closed 2026-10-07T01:25:56.224826+00:00: Critical0/Important0/Minor2; both inheritedextra_physics_tailFalse and10HzPID-feedback scope already clarified by append. Reviewer personally checked1286sourceSHA/2174+2116artifactSHA, allsource/maps/videos,150/105 tailticks and30/21 snapshots exact;8 native settling checks pass1.08s. Independent50Hz distances confirm25 candidate samples outside.2, maximum.2027110815m, so REJECT stands. V1 comparisonSHA unchanged. This review does not certify RGB-memory diagnostic/onlinevalidation. OldprefixSHA 06345ad0b88defcbdcc2c1399fedc1c7874e87cdf5ccb1ad1b8475491da957dd
