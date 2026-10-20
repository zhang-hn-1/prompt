# Learned-memory reliability — one first-pass review

**Critical 0 / Important 0 / Minor 2. Candidate REJECT confirmed from the far sequence's frozen gates alone.** No online learned shadow, controller installation, source admission, navigation pass, or whole-mission completion is supported.

This is the first and only review pass for this learned-memory branch. The reviewer retains context from earlier memory/short-obstacle reviews and is **not a blank-context reviewer**; those earlier reviews do not certify this branch. The planned eight-exposure source subgroup audit and all other subsequent work are excluded. The reviewer performed CPU checks only: no training, GPU inference, simulator collection, scientific-code modification, sealed-byte rewrite, or second review loop.

## Personally verified

Read the frozen plan and report, causal memory and head, packet scalar codec, both feature builders, fitter, predictor, held assessor, collection runner/stage/schedule/protocol, post-close records, author checks, causality audit, failure diagnostic, and relevant fixtures. Source comparison shows v2 feature recovery adds scalar metadata normalization and a separate output namespace; it does not change the point arrays, feature formulas, model, or numerical gates. The failed v1 namespace and failure record remain present.

Verified all manifest entries used by this branch: 1,281 collection-source identities; 1,371 post-close artifacts per fresh sequence; 1,336 fit-feature inputs and 270 sealed fit-feature files; 380 held-feature inputs and 77 sealed held-feature files; 76 pre-truth prediction files; and the fit/predictor, weight, dataset, label and manifest references. **All 5,794 distinct file identities checked matched.** Full hashes and exact check groups are recorded in `learned_memory_review_personal_checks.json`. Weight SHA is `7dbffaecc56147fc30cc648569f29eb2cb4c6da5aca204f43fc5fd5a816d55eb`.

The recorded chronology is consistent with the intended boundary: both fresh collections closed at 23:37:04 UTC; fitting began 23:40:10; weights sealed 23:40:11; held features opened 23:43:17; all held predictions sealed 23:43:36; held assessment began 23:44:08 on 2026-10-06. Fit feature tables were sealed at 23:35:35 before fitting labels were consumed; building fit-only features while the second collection was still running is not model fitting. The inspected fitter reads held manifest/identity metadata, not held features, scores or label arrays. The held predictor has no occupancy/depth-label input. This is code/seal evidence for the pipeline boundary, not an OS-level historical file-read trace.

Ran **25 bounded CPU tests, all passed in 1.41 s**: seven causal memory fixtures, four head cases, one scalar codec, three confusion-metric cases, and ten schedule cases. This does not claim a reviewer rerun of the historical 450-test suite or its RED/GREEN chronology. The schedule tests remain GREEN-only in the historical development record.

Both actual collections contain 180 requests, 900 subsequent command substeps, 181 captures, 901 physics samples and 37 original source exposures, with one reset and zero contact samples. Every request matches the fixed schedule, saved policy input and physical raw-command trace: far has positive 0.3 main requests during 6–12 s; near has negative 0.3 during that interval; other phases and requested auxiliary channels are zero. At all 37 exposures per sequence, the feature context's previous request exactly equals the preceding executed request. The nominal RGB policy is called for diagnostics and its action is not applied. The preserved history shadow in these collection logs is the old explicit-memory shadow, not the new learned model. These are observation collections, not navigation A/B trials.

Recomputed whole-sequence saved-source endpoint scoring: far 100,089 within 0.10 m out of 112,487 accepted queries, 103,622 with finite truth; near 21,412 out of 23,482, 22,088 finite. These match the report. They aggregate all accepted heights/surfaces and do not establish obstacle-subgroup coverage or reverse the existing source rejection.

Reassembled the entire fit dataset from the five sealed fit-only feature sequences and their proxy labels: **16,111 rows, 3,721 positive**, exactly matching the saved dataset. Saved mean/scale exactly match fit-only float32 standardization. The stored head has 21→16(tanh)→1 structure; protocol fixes seed 7, Adam 0.003, batch 256, ten epochs and threshold 0.5. There are ten saved epoch entries. No model was refitted by this reviewer. The inspected split consists of five historical whole fit/development sequences and two new whole held sequences; within-fit batch shuffling is not a random frame train/held split.

Independently rebuilt all **74 held feature snapshots** from original saved packet/RGB/mask/pose/previous-action inputs, without opening truth files in that reconstruction. Features, cells, original last-query times, 10 s baseline labels and frozen-head scores all match the saved tables exactly. Each 6 s truncated stream reproduces all 13 earlier snapshots and scores; future-payload traps reject before value access and leave the prefix unchanged. An added truth key is rejected by the packet whitelist. The tests also cover immutable query identities, no replay count/age refresh, 35 s expiration and XY-translation invariance. No future/GT feature leakage path was found in the inspected production pipeline.

The head receives only the finite 21-column matrix. Cell/episode addressing, absolute XY and GT labels are not fed into its forward path. Relative geometry and calibrated predicted height remain permitted privileged-pose features. Baseline labels are stored alongside features but are not included in the model matrix. Snapshots remain model-supported static imputation, not observed-free or safety-certified output.

## Independent score and gate calculation

| Sequence | Method | TP | FP | FN | Precision | Recall | Eligible cell-time rows |
|---|---|---:|---:|---:|---:|---:|---:|
| Far | B0 | 747 | 366 | 3 | 0.671159 | 0.996000 | 1,118 |
| Far | Model | 596 | 288 | 154 | 0.674208 | 0.794667 | 1,118 |
| Near | B0 | 41 | 173 | 0 | 0.191589 | 1.000000 | 214 |
| Near | Model | 25 | 11 | 16 | 0.694444 | 0.609756 | 214 |

Recomputed every held score from the sealed head and every confusion count from the frozen occupancy proxy. Far fails precision ≥0.95, recall ≥0.80 and no-worse FN; its TP≥25 and no-worse FP clauses pass individually. **Far alone rejects the unique candidate.** Near has 41 eligible positive labels, so its recall is defined and the zero-positive N/A branch is inapplicable. Near-positive precision/recall/FN comparisons are descriptive, not extra pre-frozen rejection reasons.

The error-layer diagnostic was also recomputed: the fixed 0.10 m nearest-occupied-cell bins and percentiles match, including model FP counts 288/11. These retrospective distance bins do not relax the binary acceptance gate or establish a unique optical/learning cause.

## Limits and inherited evidence

The fit and held sequences use the **same static occupancy map** (identical label-file SHA). Thus this is fresh-sequence validation in a known static scene, not held-scene generalization or an unseen-map test. The shared map is used for offline proxy labels; the inspected features do not directly expose it. Counts are repeated eligible cell-time decisions, not independent obstacles or complete scene coverage. All old source/pose/navigation refusals remain outside this proxy's authority.

Original fitting, reconstruction of all historical fit features from sensor inputs, CAD source/mask reconstruction of 444 fields per collection, four complete video decodes (181 frames each at 10 Hz), historical full-suite results and development RED/GREEN evidence remain inherited author evidence. Their relevant sealed files were inspected; these computations were not rerun here. By contrast, full held causal reconstruction, frozen inference, confusion metrics, fit-dataset assembly/standardization, schedule-to-physics checks and source aggregate scoring above were personally performed.

## Findings

### Critical

None found in this bounded review.

### Important

None found in this bounded review. The model's frozen far-gate failure is valid and final for this candidate.

### Minor 1 — machine assessment still combines near-positive descriptions with acceptance checks

`assess_learned_memory_held.py:12–16` places the unplanned near-positive precision/recall/FN comparisons inside `checks`, computes a near `layer_pass`, and combines it into `accepted`. The human report correctly discloses that these near-positive clauses were not separately frozen. The sealed machine record does not itself carry that distinction. This cannot change this candidate's REJECT because far independently fails, but a reader of only `assessment.json` could wrongly call the near comparisons predeclared gates.

Recommended disposition: add an append-only criterion-provenance note identifying the far frozen failures as the decisive rejection, the near positive count of 41, and the near extra comparisons as descriptive/nonbinding. Preserve the original assessor and sealed assessment; do not refit, rescore to another threshold, or launch a shadow.

### Minor 2 — inherited collection protocol fields describe the near schedule as universal

`memory_validation_collection_protocol.json:1414–1415` still states a universal `0/-0.3/0` schedule, although the far collection deliberately and correctly executes `0/+0.3/0`. Its inherited `endpoint_definition` at line 1389 also says symmetric shortening with original yaw, whereas far uses original endpoints and both fresh collections use yaw −20°. Plan, schedule implementation, actual episode records, per-kind identity and raw commands agree on the intended operation, so this is metadata inconsistency rather than a control deviation.

Recommended disposition: append a per-kind protocol clarification for far/near start, goal, yaw and request schedule, marking inherited universal fields as archival. Preserve sealed protocol and artifact bytes.

One review pass only. Model REJECT, no learned online shadow, no source/navigation admission, and whole research incomplete.
