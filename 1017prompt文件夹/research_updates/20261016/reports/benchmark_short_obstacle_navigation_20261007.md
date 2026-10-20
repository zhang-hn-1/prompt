# Short obstacle navigation: actual two-arm results (2026-10-07)

## Scope and controller identity

New 2.6m known-pose E1-scene diagnostic, declared symmetric endpoints X=-1.3 and +1.3 before execution. This is not original 6.6m E1 or a formal E0/E1/E2 result. Same frozen PriorRGBPolicy, CAD bilateral masking, Fast stereo eager eight iterations, ground/safe-start priors and explicitly permitted unknown exploration, roll PID, main request cap .3 rad/s, auxiliary limit .3 rad/s, camera rig and lighting. Current-only versus 10s history is the sole primary-consumer difference. Failed nominal/terminal candidates are not installed. Inherited original oracle-length/classification metadata is archival; SPL is not used. No known global route or renderer depth enters policy. True pose remains privileged, and depth source admission remains rejected.

## Actual results and acceptance

| primary | termination | simulation seconds | final distance m | true progress m | contact samples | calls/captures/physics/sources |
|---|---|---:|---:|---:|---:|---|
| current | collision | 12.0 | 2.049916869 | 0.550131422 | 6 | 120/121/601/25 |
| history | timeout | 35.0 | 2.055577663 | 0.544422966 | 0 | 350/351/1751/71 |

Neither arm crosses the actual X=0 midplane or reaches the goal under unchanged 35s/.2m/no-contact criteria. Both task gates FAIL. History has all 350 ten-component raw request blocks zero; its .544m displacement is actual passive/closed-loop physical drift, not commanded forward route completion. Its smaller cross-track error cannot certify improvement when progress is absent. Current has 75 all-zero blocks. No reliable memory navigation benefit is accepted. Independent simulation resets produce different RGB predictions even at matching poses; this is a task comparison, not isolated causal attribution to memory. Opposite-memory same-live shadow requests were never executed or assigned counterfactual trajectory labels.

## Observation failure, before any new fix

A separately sealed nine-exposure retrospective audit uses only completed logs: each short arm at frames 0/60/110, and older far-start yaw20 control at the same frames. Divider surface is a geometric proxy from the frozen USD cylinder definitions (radius .35m, height1m), not a renderer instance ID. GT endpoints identify that proxy only offline; no truth enters runtime or map reconstruction. The source/mask packets were finalized and saved before truth retrieval. The audit does not alter old gates.

In all six short-arm samples, roughly 160k left-visible unmasked divider pixels exist, but **zero project inside the right native image**. Thus the short cold start has no measured binocular correspondence on those visible divider surfaces; history cannot create unseen valid depth. At current frame60 there are two accepted source queries on the divider proxy, but neither is within .10m of its true endpoint. Per-arm source reconstruction also reports zero .10m-correct candidate queries over the full runs; this is a failed-source diagnostic, not certification of ghost obstacles. At history6s the consumer holds 58 historical hazard cells and reports observed_obstacle_blocks_route while raw requests stay zero; absence of collision does not validate those hazards.

The older far-start samples have right-depth-consistent, bilaterally unmasked divider pixels 31830/23252/21987, and packet divider queries accurate within .10m 1599/2914/3910 respectively. This is bounded historical evidence that observable approach coverage differs from near cold-start coverage, not a fresh network-validation pass. Right depth agreement tolerance .02m is a diagnostic classification only. The whole-run older depth gate remains rejected.

The .6m baseline and native vertical epipolar geometry matter. fB/416=.843589744m is the nominal cost-volume search scale; **max_disp is not a hard final disparity clip**, since recurrent updates add residual disparity. The necessary full-native overlap scale fB/479=.732637439m alone cannot guarantee object correspondence because pose, image bounds and occlusion also matter. At short frame0, divider axial depth median .80351m and disparity median436.75px; actual point projection, not just that formula, establishes zero in-image right correspondence for the sampled visible surfaces. Do not claim increasing search range alone fixes physical field-of-view loss.

## Actual verification evidence

Author independently ran both completed whole-arm assessors: current 2160 / history 3679 exact source-and-closed-artifact SHA checks; paired source/mask fields 550/1562; primary maps 720/2100; opposite-memory shadow diagnostic fields 1440/4200. Each checks current/past source time, future-payload rejection, actual raw command blocks versus physics, unchanged endpoint derivation and gates, source/model identity, and all video frames. Four videos contain exactly121/351 frames per arm at10Hz, no scoring-tail frames. This is not a physical shadow on/off determinism experiment. Prelaunch full CPU suite was410 passing; scientific result is nevertheless rejected. Geometry audit is author-only unless a subsequent review explicitly covers it.

Files: short_obstacle_navigation_protocol.json, short_obstacle_navigation_status.json, per-arm assessment JSON/logs, two original run directories including physics/contact traces, RGB/depth namespace isolation, query/partner provenance, maps, smoother/execution states and videos. Geometry: short_obstacle_visibility_preanalysis.json / short_obstacle_visibility_diagnostic.json / audit_short_obstacle_visibility.py. Actual comparison and trajectory: short_obstacle_navigation_comparison.json / short_obstacle_actual.png.

## Next admissible work

Keep camera and lighting fixed, retain every original source/Oracle/IMU/M2/terminal refusal. First measure observability loss and actual retained hazard-source ages on the already completed far-start approach; distinguish never observed, invalid depth, expiring valid obstacle evidence and route/physical execution failure. This identifies whether one bounded active-observation maneuver or one provenance-preserving static-memory consumer is justified. No new threshold sweep, planner rewrite, network, formal benchmark or estimated-pose obstacle success is claimed. Learned memory/world-model increment and formal original35s E0/E1/E2 remain unfinished. Whole research is not complete.


## One independent first-pass review and append-only scope correction

Reviewer reused historical occluded-memory context because new-agent dispatch previously hit thread limits; this is not a blank-context claim. Critical0 / Important0 / Minor2. Personally ran16 tests6.11s and verified4588 unique SHA files (1272 sources,888current/2407historyclosedartifacts,41visibilityinputs,9authorclosure entries), actual requests→physics, both whole-arm source endpoint counts and all nine projection statistics with independent homogeneous USD inverse matrices. Approach visibility/age/route-length diagnostics and the new retreat probe are explicitly outside review scope. Author full410 suite is inherited evidence, not personally rerun by reviewer.

Minor1: history has no active main-request interval, but assessor movement_cross metrics select only the final state and emit .001608644985m. These fields are **N/A for active tracking** and must not support improvement. Raw values stay sealed unchanged; whole-episode/progress gates unchanged. Minor2: old runner-generated identity.local_goal_m still says2.0; actual task/protocol/runtime endpoints give2.6m. This is archival metadata error, not actual controller/input identity. Both original identities and manifests remain untouched. Report truthful scope takes precedence; no rescoring or second-review loop.

A subsequent author-only offline route-length audit adds an essential limitation: reducing endpoint span to2.6m did not shorten the divider detour to2.6m. Eight-neighbor grid path with the same .6m clearance, no corner cutting, is7.062741700m (point-grid diagnostic, not Dubins/full reference or formal SPL). Nominal required speed over29post-settling seconds is.243542817m/s, while R*.3=.12m/s is only the ideal nominal rolling scale, **not a physical XY bound**. This does not prove impossibility, but prevents calling the short case an easier time-qualified navigation task. The actual immediate visibility/collision and history-zero-request findings remain valid. Original35s rules and failed originaltasks are preserved.
