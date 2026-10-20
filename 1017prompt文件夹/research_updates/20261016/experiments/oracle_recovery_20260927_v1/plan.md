# Oracle recovery implementation plan

Goal: obtain measured evidence and one bounded control/reference improvement, if supported, without changing the frozen baseline.
Spec: user's approved 2026-09-27 proposal; permission to proceed without additional approval.
Execution: inline; existing dirty worktree preserved; no commit or push of unrelated changes.

Constraints: V2 aux30, auxiliary target slew 30 deg/s, measured auxiliary supervisor 0.30 rad/s, main slew 0.5 rad/s²; formal horizon 35 s. No learning-model fitting. Old reports/results immutable; corrections in dated addendum.

- [ ] Verify old reverse-sign xfail with runxfail; replace erroneous assertion with signed-body/spatial/yaw-rate identities over both directions and turn signs. Policy untouched.
- [ ] Test then implement open-plane paired diagnostic: replay same B0 prefix; event is completed direction change followed by reference-index advancement. Real vs aligned reference-frame initial pose; preserve entire physical/joint/smoother history. Exactly two references x two conditions, 8 s post-event.
- [ ] Freeze evaluator independent of policy path_index. Compare geometry, actual progress, command layers, entry state and prefix parity. Alignment is diagnostic only, never a navigation improvement.
- [ ] Select ONE candidate from evidence; test before implementing. Preserve reference and limits for control candidate; preserve tracker for reference candidate. Stop if no supported candidate.
- [ ] Actual local A/B; if passes (RMS >=20% reduction both references, peak <= B0+0.05m, progress >= B0-0.05m, changed applied actions), run full fixed reference; then formal golden/hard_0 at 35s if full comparison passes. Failed candidate is rejected, not promoted.
- [ ] Save plots, source fingerprints, commands, test/run evidence and report; no claim of success based on alignment or index movement.

Ruling: the diagnostic uses an SE(2) transform of the fixed reference at the live event, equivalent in an empty horizontal plane to correcting robot XY/yaw without changing body-relative dynamics. This preserves actual joint/smoother/history instead of fabricating a steady state. It isolates entry pose only, and cannot prove full dynamical feasibility. The original reference geometry and command sequence are unchanged up to rigid transform; evaluation converts back to the frozen reference frame.

Review focus: direction-sign conventions; no double policy act; exact pre-event state/action equality; frozen reference evaluator; no future information; no false navigation success from diagnostic transforms.

## Execution ledger
- Curvature audit: old --runxfail assertion reproduced; signed-body/spatial/yaw-rate identities now covered in both directions and turn signs. 138 policy/audit tests pass, 3 pre-existing deprecation warnings. No policy action change.
- Four physical diagnostics completed; prefixes and entry physical/joint/smoother/policy state identical. Fresh B0 trajectories exactly match historical B0 over the measured duration.
- Entry alignment: 0002 RMS .2541 -> .0857m, progress 1.2613 -> 1.8933m; 0004 RMS .3357 -> .0930m, progress 1.0213 -> 1.7447m. This is NOT a realizable control intervention or navigation success.
- Selected one reference-source candidate: after the same live handoff-progress event, replan ONCE from actual pose to original goal. Existing Hybrid A*, same drive profile/limits/tracker; bounded 5000 expanded nodes; retain B0 on failure. Preserve direction latch and five-sample stop logic. Criterion remains original-reference 8s RMS/peak/progress, not candidate path_index.
- Ruling: first candidate run incorrectly used the all-free diagnostic fixture for replanning, allowing shortcuts through the original obstacle layout. Those two actual runs remain as invalid-map diagnostic evidence; they are not candidate acceptance evidence. Corrected callback uses the original map hash and exact source/open-frame transform. Same candidate and numeric parameters, no scan. Added two source-map tests; total six local tests pass.
- Correct-map candidate completed twice; planning succeeded but reselected reverse as initial direction. 0002/0004 spent 52/48 of 80 post-event blocks in direction-change braking; original-reference net progress -0.172/-0.183m. Both rejected; no full-reference or formal benchmark promotion. No second control candidate.
- Independent review confirms metrics, prefix/causality, sign correction and rejection. Real-time limitation: 4.377/3.318 seconds of blocking CPU replanning, physics paused.
- Final tests: 138 policy/audit tests + 20 focused planning/reference/probe/candidate tests passed (3 pre-existing warnings in first group). Not a full repository suite.
- Baseline snapshot check detects 22 source differences already accumulated since Sept22 (this turn changes one test among them). Old snapshot and old results untouched; current tree is not claimed identical to Sept22. New real-prefix trajectories independently match Sept21 B0 exactly.
