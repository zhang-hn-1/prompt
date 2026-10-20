# Past point RGB corroboration — bounded development diagnostic

This readonly diagnostic supplies a limited piece of current visual evidence for remembered geometry when frozenFast no longer returns obstacle queries. It does not estimate new depth, refresh geometry age, certify free space, enter the mapper/controller, or establish navigation success. Existing source/model/control rejections remain unchanged. The previously inspected sequence is development evidence, not fresh held-out.

## Question and execution

On the closed 35s sensing sequence `anchor_validation_fresh_run_v1`, preserve all1785 hazard points and original source IDs/time from18s. At18.5/19/24/35s, use only currentRGB, current known camera transforms and CAD masks to project past points into both eyes. Camera poses are explicitly privileged; this is not the RGB+estimated-pose tier. A single frozen7×7 patch condition requires grayscale standard deviation≥5 and normalized correlation≥.8 in both eyes, with no spatial search or alternate threshold. Source-only predictions for all four frames were sealed at01:18:16UTC before loading current renderer labels; label assessment at01:18:37UTC verified all input/output hashes.

## Actual data

| Time | Past points geometrically in both unmasked views | Appearance corroborated | Both-eye current geometric label correct≤10cm | Current frozenFast hazards |
|---|---:|---:|---:|---:|
|18.5s|1781|267|267|1784|
|19s|1285|30|30|0|
|24s|0|0|N/A|0|
|35s|0|0|N/A|0|

Zero finite geometric mismatches and zero insufficient labels among accepted points; conservative precision100% on these two nonempty frames. Preregistered moving19s nonzero support and≥95% precision gates pass **only this readonly appearance diagnostic**. At19s, the30 pixels span only0.0766×0.0923m in worldXY, three canonical10cm XY bins; they are correlated samples along one small boundary, not30 independent obstacles. At18.5s support occupiesfive such bins. No broad scene/map coverage claim.

24/35s zero past-seed projection says nothing about all current scene geometry: earlier physical overlap audit found573 possible divider pairs at24s in other regions. Newly visible areas need new observed geometry; this method cannot recover them from old points. All retained geometry provenance remains18s; current appearance timestamp is recorded separately and never substituted for depth/source age. A successful patch match is not a fresh depth measurement.

## Evidence and next scope

Plan `docs/superpowers/plans/2026-10-07-past-point-RGB-corroboration.md`; actual scripts and frozen predictions in `experiments/stereo_memory_navigation_20261004_v1/probe_past_point_RGB_corroboration_v1.py`, `assess_past_point_RGB_corroboration_v1.py`, `past_point_RGB_corroboration_predictions_v1/all_predictions_seal.json`, assessment andscope JSON. Plot generated fromactual19s images and personally inspected; accepted points visibly occupy a narrow cylinder boundary. No simulation replay, parameter scan, new network or closed-loop intervention was performed. Author verification only; independent velocity-controller review does not certify this diagnostic.

![Actual source-only projected support](../experiments/stereo_memory_navigation_20261004_v1/past_point_RGB_corroboration_frame190_v1.png)

This supports testing a provenance-preserving current appearance layer for existing static memory, with independent original geometry age and no new-free inference. Estimated-pose error, wrong seed geometry, occlusion, other regions, fresh sequences and execution integration remain unvalidated. It cannot by itself rescue the old source/filter/nav gate or authorize formal E0/E1/E2.
