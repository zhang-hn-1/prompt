# Necessary reverse-straight domain coverage

Independent stage; retain the three-forward-point planner rejection, all source/pose/M2/learning rejections and original35 rules. No PID, rig, light, mapping, threshold or planner tuning. This is not a navigation success or full dynamic model.

Concrete gap: original start requires reverse escape; all supported forward primitive first edges hit the frozen MODEL margin. Current reverse cap is -.3 but old table covered -.5. Do not interpolate the old table into this gap.

Budget: exactly one developmental32s sequence yaw0. Exactly one fresh whole-sequence validation yaw20deg, precommitted before development, launched only if development passes fixed steady gates. Full reset at each sequence start, none inside. Commands zero0–6s, main-.3/aux0 from6–26s, rawzero26–32s. Same current roll-PID .4/.1/.7, limits and original runtime. Every physical tick, execution state, synchronized dual camera and videos retained.

Frozen measurement: original centered31 XY geometry; signed by main, tail21–25.5. Gates valid fraction>=.9, direction fraction>=.9, bank95–5 span<=3deg, curvature IQR<=max(.2,.3absmedian). All original sampled contacts zero; exact runtime request/smoother/applied reconstruction, clocks and source identities required. Actual target reach and five consecutive10Hz XY speeds<.05 stop (rebound clears count) measured separately, not substituted by target slope.

Development scalar speed/curvature is one point only. Seal it and fitting provenance before validation launch. Validation same steady gates, speed error<=.03m/s, curvature error<=max(.2,.3absdevelopment median). No future feedback prediction, no recursive transient claim. Failure rejects extension, no extra sequences, yaw or duration rescue.

Only if both pass may an independent four-point primitive source preflight be considered, with one reverse point plus the existing three supported forward points; old rejected source remains unchanged. At most one conditional navigation run after complete dependency seal and CPU verification; no automatic formal35/world-model admission. No extra reverse-steering or intermediate-angle coverage.
