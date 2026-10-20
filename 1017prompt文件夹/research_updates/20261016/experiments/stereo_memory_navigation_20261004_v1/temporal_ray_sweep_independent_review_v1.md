# First independent temporal ray-sweep review

Independent read-only review completed 2026-10-07. Scope: the new fixed six-case development diagnostic, its source, five tests, frozen plan, protocol, records, prediction seal, and subsequently author assessor. This is ONE review; no repair/review loop, predictor rerun, simulation, tuning, install, commit, or modification to frozen sources or author scores.

**Decision: REJECT the single temporal ray sweep.** Personally recomputed three-endpoint geometry confirms the author rejection. Five of six nonempty eye/frame packets fail the frozen 95% necessary precision gate. Coverage and packet size pass but do not override precision. Known camera poses/CAD masks are privileged development inputs; this provides no fresh source/global RGB admission, estimated-pose admission, free-space certification, or navigation success. Existing ORB/LightGlue/Fast/current-known-memory/estimated-pose/navigation rejects remain; formal 35 s E0/E1/E2 validation remains pending.

## Personally verified

- Ran the five requested tests with the existing `/home/an/miniconda3/envs/ball_wm/bin/python`: **5 passed in 0.56 s**. No dependencies installed. Initial system Python attempts lacked Python/pytest; bundled runtime lacked cv2, before the existing environment was located. These environment failures did not run the predictor.
- Checked all **23** input/output SHA entries in prediction_seal.json and all four protocol-bound plan/predictor/module/full-suite-log SHA values; all match. Seal timestamp 02:48:45.648199 UTC precedes author assessment timestamp 02:49:21.417008 UTC and reviewer geometry assessment 02:50:28.032768 UTC. Hashes establish current byte identity, not an independent runtime filesystem-access trace.
- Used a separate reviewer script, without importing the original predictor/module, to reconstruct current world points from native UV/axial depth, project them into both past poses, and compare all three endpoints with nearest-pixel offline renderer depth. Saved per-query error/mask arrays in the independent namespace. Current reconstruction max absolute error <1e-10; saved past UV agree <1e-8.
- Verified unique native query IDs, even-pixel sampling, fixed two past frame IDs, causal timestamps with 1 s/2 s lag within clock precision, selected depth .5..3 m, score >=.8, stored uniqueness margin >=.1, source baseline >.05 m, height .05..1.5 m and root distance >.55 m. Derived root agrees with offline root within 1.044e-7 m per coordinate. No `certified_free` or source admission flags.
- Checked RGB/pose/mask timestamps agree for all five participating exposures. All selected current 7x7 patches have 49 CAD-free pixels. Reprojected all selected patches at their sealed depths and examined the actual OpenCV bilinear mask interpolation footprint in both past views.

| Frame / eye | Accepted | Correct at all 3 endpoints | Insufficient | Finite wrong | Precision | Correct novel points / bins | Masked bilinear footprint |
|---|---:|---:|---:|---:|---:|---:|---:|
| 235 left | 854 | 676 | 14 | 164 | 79.1569% | 676 / 5 | 2 |
| 235 right | 1119 | 674 | 189 | 256 | 60.2324% | 624 / 9 | 0 |
| 245 left | 601 | 518 | 12 | 71 | 86.1897% | 518 / 3 | 8 |
| 245 right | 860 | 637 | 99 | 124 | 74.0698% | 603 / 9 | 4 |
| 255 left | 900 | 887 | 0 | 13 | 98.5556% | 887 / 4 | 0 |
| 255 right | 1397 | 1253 | 86 | 58 | 89.6922% | 486 / 8 | 4 |

Totals: **5731 accepted / 4645 correct / 400 insufficient / 686 finite wrong**. Insufficient labels remain in the precision denominator. Novel bins use canonical `floor(world XY / .1)` absent the 18 s sensor seed; reviewer coverage uses only three-endpoint-correct accepted points and requires actual query time >23.5 s. Both eyes individually exceed 20 correct novel points in >=2 bins at frames 245 and 255. This is conditional diagnostic coverage, not global source recall or spatial free-space evidence.

## Findings: C / I / M

**Critical: none identified within this bounded review.**

**Important I1 — past RGB interpolation violates the all-CAD-free support contract** (`temporal_ray_sweep.py:30`). RGB uses INTER_LINEAR while mask validity uses INTER_NEAREST. A source location can pass nearest-mask validity while its RGB interpolation includes a masked neighboring pixel. All sealed selected patches pass the implemented nearest-mask check, yet **18** accepted query patches include at least one positive masked-pixel bilinear contribution in one or both past views. Seven of those 18 are geometrically incorrect, all from frame 245 left. No selected patch had image-border interpolation contamination. This is a real contract failure, not merely hypothetical. It does not explain all 686 finite wrong or 400 insufficient points; no sole-cause or causal counterfactual claim is made. Preserve the frozen diagnostic rejection; do not silently repair, refilter, or rerun this one-candidate branch.

**Important I2 — author coverage boolean does not implement the frozen correctness/time conditions** (`assess_temporal_ray_sweep_v1.py`, row/coverage expressions). The row `coverage_gate` counts all novel predictions/bins, instead of `novel & correct`; its cross-eye search includes frame 235 at 23.499999474734068 s, although the plan requires a query post-23.5 s. The script separately records the number of novel correct points but does not use it in that boolean or count correct bins. In this particular sealed result, the independently stricter calculation still passes coverage at frames 245 and 255, so neither the coverage conclusion nor overall rejection changes. The existing author score artifact remains untouched. Any report should describe strict reviewer coverage explicitly, rather than present the author boolean as direct evidence of the exact gate.

**Minor: none elevated.** Code/API limitations outside this fixed wrapper are not evidence of a defect in these six sealed inputs.

## Inspected rather than personally rerun

Source causality checks run before image/pose payload; the future Trap test passed personally. The fixed wrapper loads only current/past RGB, known camera poses and masks; no truth, action, online control, global plane or world-map input enters the predictor. Optical conversion `diag(1,-1,-1)`, ray projection and inverse projection are mathematically consistent with the independent actual-pose reconstruction and literal projection test.

The NCC implementation warps native past gray images with per-depth local frontoparallel patch hypotheses, uses 7x7 means/variances, requires both temporal scores >=.8 through their minimum, source/current standard deviation >=5 and parallax >=.5 degrees. The uniqueness competitor is the maximum eligible depth outside +/- .10 m; demanding a finite competitor is conservative when no alternative is eligible. These expressions were inspected, not independently rerun on all depth hypotheses. Sealed packets contain score/margin fields but no complete cost-volume trace, so the review verifies stored thresholds and formula, not an independent recomputation of every winner/competitor. No same-time binocular matching is claimed.

The author full-suite log states **482 passed in 22.81 s** and its SHA matches protocol. It was inspected, **not personally rerun**. Source checks/tests and independently rebuilt selected geometry do not establish runtime 10 Hz capability: the inspected records show roughly 11.0–11.3 s per eye/query case on the author's run.

After independent recomputation, the author assessor/assessment were inspected and all six n/correct/insufficient/finite-wrong/precision rows agree exactly. Reviewer evidence is kept separate; no author prediction or score bytes were changed. See `temporal_ray_sweep_independent_evidence_v1/independent_geometry.json`, `ancillary_checks.json`, `targeted_tests.log`, per-query geometry NPZs, reviewer script, and `evidence_sha256.json` for reproducibility and identity.
