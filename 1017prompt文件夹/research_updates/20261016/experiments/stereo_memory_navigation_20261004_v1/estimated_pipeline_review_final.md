# Estimated-pose pipeline — single fresh final review

Review date: 2026-10-07 Asia/Shanghai. Verdict: **accept the bounded causal interface/local feasibility result, with one minor reporting clarification**. No Critical or Important finding. This is the single new reviewer pass requested after the author-only reports; those earlier statements accurately describe their publication state and must not be retroactively presented as independently reviewed then.

## Scope

Reviewed the frozen historical rolling domain, fresh yaw +20° read-only rolling shadow, rolling pose/camera bridge, historical estimated source registration, navigation observation isolation, and completed single estimated-pose local 2 m episode. Read the four named reports, six design specifications, four implementation plans, core implementations, tests, assessment code, source derivation records, and the base/derived runner diffs. The separate knownpose yaw +20° memory experiment is excluded, including its latest result and assessment. No other reviewer was launched.

## Critical

None found within the inspected scope.

## Important

None found within the inspected scope. This is not an all-domain navigation, collision-safety, native-IMU, pure-RGB, or full E1 admission.

## Minor

**M1 — State the geometric tracking sample window explicitly.** `assess_estimated_pose_local_navigation.py:37` computes lateral RMS/peak on `time_s >= 6 - 1e-6`, giving 841 samples after the inherited settling interval, whereas the report supplies the values without that denominator. Add the actual window/count to the report or an append-only clarification; preserve the original sealed spec/protocol and raw data. This is a reporting issue, not a failed gate: my independent full-sequence check also passes, with RMS 0.042038853 m and peak 0.128785110 m across 1141 rows. The original pose gate independently uses all 1090 rows satisfying the exact `time_s >= 1` comparison. No threshold or trajectory needs changing.

## Personally verified evidence

- Conda `ball_wm` Python ran the four bridge, two observation, two actual-hook, four shadow and five rolling tests: **17 passed in 0.64 s**. The exact five test paths are in the companion JSON.
- Inspected the actual `benchmark.policies.PolicyObservation` contract: proprio and quaternion histories become float32, position is float64, arrays are copied and immutable. Instantiated this real type, replaced original dynamic truth fields with unrelated finite poison values, and reconstructed six saved policy fields exactly at frames 0, 1, 6 and 227. The final-frame check builds its five-row history from frames 223–227. This personally verifies dtype and history behavior on those bounded samples; it is not a claim to have repeated the full author reconstruction.
- Checked 13 relevant source hashes and 11 selected local-run artifact hashes against the saved protocol/post-close manifest. All match. Manifest counts are 1264 sources and 1976 artifacts; only the selected hashes were personally reread, not the entire census.
- Both base/derived runner hashes match their derivation records. Inspected the complete source diffs: fresh shadow adds the sensor-only live hook, yaw initialization and recording; local consumer adds the bridge/adapter, estimated source poses, declared endpoint sensitivity, and labels. No hidden planner/gain/cap/rig change was identified. The original author `inverse_exact` execution remains inherited evidence because its construction script was not found in the reviewed workspace; direct sealed-hash and diff inspection is the reviewer's evidence.
- Directly checked local denominators: 228 actions, 229 captures, 1141 physics samples, 46 inference captures, 1090 pose-gate rows and 841 tracking rows. Saved actual outcome is success at 22.8 s, distance 0.199806685 m, collision 0 and runtime error 0.
- Independently recomputed local scores from saved physical truth and prediction arrays: 3D RMS 0.04617220479419692 m, peak 0.046574048796154774 m, attitude p90 0.6869078817740794°, lateral RMS 0.04879654939927168 m and peak 0.12878510951995903 m. These exactly equal the saved assessment. The 46 mm vertical contribution was retained.
- Checked historical/fresh denominators from saved arrays: historical 22.6 s = 1131 physics / 1080 pose-gate rows; historical 35 s = 1751 / 1700; fresh collision-terminated sequence = 1496 physics / 300 captures / 299 actions / 1445 pose-gate rows. Read its separate pose PASS and navigation collision FAIL; no completion-to-35 s claim is warranted.

## Causality and privilege findings

The rolling estimator receives only time, body-frame ideal gyro and main encoder, plus one gravity initialization. The gyro emulator uses the current true rotation solely to convert measured world angular velocity to the simulated body sensor. That is a disclosed ideal sensor privilege, not proof of native or noisy IMU operation.

The bridge admits an explicit initial world gauge and whitelisted rolling records. Strict increasing timestamps, exact-time lookup, rigid/finite checks and copies prevent accepting a nonexistent future exposure or mutating stored history through returned arrays. In the inspected synchronous runner, live physics updates precede capture and policy consumption; camera source registration uses the exact captured estimated pose. Historical precheck also selects exact saved exposure timestamps rather than interpolating or applying later truth corrections.

The navigation adapter reads only original columns 9:17 and measured RGB/action history. Dynamic position/yaw/velocity/angular velocity/quaternion/gravity and pointgoal consumed by the policy are rebuilt from the estimate. The CAD-mask adapter reads current joints 9:12. Runtime truth recording, contacts, success termination and offline labels remain present but are not routed into the inspected estimated navigation inputs. Initial global gauge and the unchanged low-level ideal measured-roll PID remain explicit privileges.

The added `.10 + range * sin(2°)` endpoint allowance is a heuristic sensitivity term, not a calibrated probability, guaranteed error bound or certified free-space observation. The source flag and report say so; mapper consumption uses the augmented sigma with zero additional pose sigma, so no second pose allowance was identified in that path. Ground prior and initial safe-zone prior remain assumptions. Legacy knownpose class/metric strings are disclosed; actual dataflow and saved estimated markers carry the stronger provenance evidence.

## Inherited evidence and limits

Read but did not rerun: the full 405-test suite; complete 1264/1976 identity census; all 1141 live sensor/prediction comparisons; all 2061 estimated pose, 460 source and 1368 navigation field comparisons; full video decoding; historical 71-source endpoint reconstruction and rig-matrix checks. Their counts/results are author evidence, not newly produced reviewer results. Narrow personally executed evidence is recorded in `estimated_pipeline_review_personal_checks.json`.

No GPU or new physical episode, parameter matrix, full raw-data hash census, map rebuild or full video decode was run. No code was modified. Old raw-depth/high-speed Oracle/native-IMU rejections remain intact. The 2 m goal is before the E1 obstacle and supports local estimated-pose consumer feasibility only; it cannot certify full-goal obstacle avoidance, a memory improvement, a learning improvement, or formal E0/E1/E2 success. The separate fresh knownpose yaw +20° collision result remains a navigation failure despite passing its pose gate.
