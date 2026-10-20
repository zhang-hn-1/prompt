# Reverse-bank observational coverage — first scoped review

C0 / I0 / M2. No Critical or Important prelaunch code blocker in current reviewed bytes. This review does not approve navigation or source reliability.

Personally ran the four focused tests: 4 passed in 0.26s. Personally checked all 1,981 source and 14 input hashes; no mismatch. Inspected the observer, positional call interface, exact driver diff against forward parent, stage/seal lifecycle and all four postclose verifiers. Did not run GPU actual, postclose verification on nonexistent new data, or the full CPU suite.

The observer calls its parent state update exactly once and replaces only requested motion signs for the distinct family. Axis feedback is native estimated current pose against the independently read current capture clock; only known coarse goal and past/current source counts enter control. Ground truth is saved after source output and is used by assessments only, apart from declared initial gauge and inherited ideal low-level roll feedback. Reverse-bank dynamics remain uncalibrated. Request40-block/.5m estimated stopping and later true.7m safe-start assessment remain separate. No planner, PID, mapping, network or acceptance tuning found in the driver diff.

The initial plan incorrectly said5deg while inherited protocol/verifier remained2deg. Author corrected the plan before actual and preserved old bytes. Current plan/protocol/verifier agree on2deg; no old gate was relaxed.

Minor1: protocol descriptive yaw remains0 despite authoritative starting quaternion yaw20. Driver correctly consumes the quaternion. Update metadata/reseal before launch.

Minor2: runtime verification decodes each mp4 but lacks an assertion for exactly stereo_view.mp4 and third_person.mp4. Add the exact expected set/count check before decoding so missing outputs cannot silently evade the required two-video check.

Existing parent rejections remain preserved. Source, pose, zero-contact, displacement and axis outcomes must be jointly assessed after actual; no partial pass implies later navigation admission. No second review requested; author can close these minor findings once with preserved bytes and identity evidence.
