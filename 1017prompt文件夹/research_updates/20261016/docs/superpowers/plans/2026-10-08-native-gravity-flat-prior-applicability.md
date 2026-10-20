# Frozen native IMU + explicit flat-height applicability preflight

Reuse the already frozen GravityRolling and native actual_interval_v2 packets from the CLOSED historical complex35 Oracle shadow. No repeat physical sequence, model fit, gravity gain, timestamp repair or sensor matrix. This is seen historical applicability under separately authorized task H; original strict native3D reject remains unchanged.

Inputs: native packet whitelist, one t0 world position/rotation gauge, existing floor0/radius .4000000059604645 prior. World position = t0 position + t0 rotation @ unconstrained relative estimate; replace only world Z by the explicit prior. Rotation unchanged. Prior is not an observation/full3D safety. Predict using frozen code, exact-reconstruct old live native outputs, truncate501 and replace all future sensor packets for causal equality. Save prediction artifact and source/input seal before loading dynamic world-pose labels.

Evaluate t>=1s all-angle p90<=2 degrees, 3D position RMS<=.05m/peak<=.10m and complete qualified packets, same task H gates; separately report XY and Z. No searching other height/radius/gain. Historical preflight pass only permits freezing at most ONE new read-only native-sensor known-command60s sequence using prior closed v2 requests as preknown immutable plan. Not navigation, no planner/NN/GT feedback actor, no new navigation success rate. If preflight fails, reject this reuse; no rescue. Original v2 navigation/trapezoid/strict native reject and formal35/learning incomplete remain.

Predictions are analysis replay, never called online evidence. Reuse of old raw data does not make a new held-out set.
