# One learned static-cell reliability memory candidate

**REJECTED on fresh whole-sequence validation.** Not installed in the controller, no online learned shadow triggered, no formal E0/E1/E2 or new navigation success rate. The existing depth-source gates and previous M1/M2/navigation refusals remain unchanged. This is one small discriminative learned-memory increment, not a generative world model, and was tested diagnostically despite the explicit-memory full-navigation baseline still failing. It does not satisfy the original plan's working-baseline/closed-loop increment milestone.

## Frozen design and causality

Original accepted Fast packet records plus current/past RGB, query/partner IDs, calibrated pose, mask, sigma, and previous actual requests feed deterministic .1m cell memory with original unique-query provenance and max35s age. Model21→16tanh→1sigmoid; seed7, Adam .003, batch256, class-balancedBCE, tenepochs, threshold .5 fixed before fresh scores. Five historical whole sequences fit/development, including shared physical prefixes; none are called held-out. Mean/scale fitted only there. 16111fit cell-time rows,3721positive labels. BCE .82665→.36306, not a fresh-performance claim. No alternate model, extra epochs, threshold search or new validation collection after failure.

Cell indices/absolute XY are addressing only, not feature inputs. Features retain relative spread/displacement, source counts/age, RGB texture/residual, uncertainty, source range and past request. No GT occupancy/depth, global reference, scene/episode ID, future pose/actions or feedback input. Predictions are model-supported static imputation, never observed-free/certified clearance; dynamic E2 not admitted. Current known pose is privileged in this diagnostic; no RGB-only/estimated-pose claim. GT proxy labels belong only to offline fitting/scoring.

Fresh complete sequences were both sealed BEFORE fitting: far originalstart yaw−20°,6zero/6main+.3aux0/6zero; near declaredshortstart yaw−20°,6zero/6main−.3aux0/6zero. One initial reset each, continuous PID/smoother/physics within each. Camera/light/source fixed. Actions are current-time schedules; nominal RGB policy is called once for diagnostics but never applied. Near and far are observation datasets, not navigation A/B, no obstacle-crossing success claim.

Fit weight seal UTC 2026-10-06T23:40:11.237913+00:00; all74fresh predictions sealed UTC 2026-10-06T23:43:36.103156+00:00; held labels opened for assessment after these seals. Weight SHA256 `7dbffaecc56147fc30cc648569f29eb2cb4c6da5aca204f43fc5fd5a816d55eb`. Feature tables sealed before fitting labels; fresh features opened only after weight seal. Source/runtime scalar decoder v1 failed on zero-dimensional NPZ metadata, preserved initial protocol/log; separate v2 normalizes only metadata `.item()`, arrays/features/gates unchanged. No refit.

## Fresh results at frozen .5 threshold

Static2.5D occupied-cell proxy, eligible current/past query cells only. This is not .10m ray accuracy or whole-scene collision geometry. Counts sum repeated cell-time decisions, not independent samples or unique obstacles.

|sequence|method|TP|FP|FN|precision|recall|eligible rows|
|---|---|---:|---:|---:|---:|---:|---:|
|memory_validation_far_run|baseline|747|366|3|0.671159|0.996000|1118|
|memory_validation_far_run|model|596|288|154|0.674208|0.794667|1118|
|memory_validation_near_run|baseline|41|173|0|0.191589|1.000000|214|
|memory_validation_near_run|model|25|11|16|0.694444|0.609756|214|

Far fails frozen precision≥.95, recall≥.80 and no-worse FN: .674208/.794667 and154FN versusB0 three. TP596≥25 and FP288≤366 pass only their individual clauses. **Far alone decisively rejects the candidate.** Near unexpectedly has41eligible positive cell-time labels, so zero-positive N/A branch does not apply. Its25TP/11FP/16FN improve false hazards but lose coverage. The assessor added descriptive precision/recall and FN comparisons for this nonzero near branch; those were not separately pre-frozen as near-positive gates and cannot be called additional frozen acceptance criteria. They do not determine rejection, already established by far. No gate amended after result. Neither sequence establishes actual full-obstacle coverage; frame tables explicitly retain total GT occupied cells and occupied cells outside eligible set.

## Actual collection and interface evidence

Author personally verifies1281source SHAs plus1371post-close artifacts per sequence; each180exact scheduled requests/900 actual command substeps,181captures/901physics,37original Fast sources,444 source/mask field reconstructions. Both zero contact, both stereo/third-person videos decode181frames at10fps. Whole-sequence source-depth diagnostic: far100089correct≤.10m of112487accepted queries (103622finite), near21412of23482 (22088finite). These all-height accepted-query aggregates do not pass the old subgroup/coverage source gate and cannot be substituted for obstacle-depth accuracy. They motivate a separate bounded surface/subgroup audit, not a source upgrade.

Actual fresh source prefixes rebuilt independently: all37exposures each yield exact saved21features/cells/originallast-query times/scores. Truncation after6s leaves13earlier predictions identical; replacing future timestamps/payloads rejects before value access and preserves prefix scores. Seven memory fixtures test replay noage-refresh, future Bomb, translation invariance, expiration, immutable IDs; fourhead fixtures reject truth dictionaries/nonfinite/wrong shape and safe weight roundtrip; onecodec roundtrip; three metrics fixtures cover undefinedprecision/recall. These and schedules are CPU tests, not online shadow on/off behavior. Uniquehead frozen before scores; no controller modification.

Literal missing-module RED/GREEN logs: memory7,head4,codec1,metrics3; combined head+memory+codec12PASS1.22s; metrics3PASS.17s. Schedule wrapper10tests were written after implementation, GREEN-only, not literal RED. Full450testsPASS21.49s. Existing old failed-protocol/logs preserved; no unrelated Git changes or deletion.

## Error layer and next concrete work

The execution and causal data-boundary checks pass, but the learned physical-map reliability proxy is insufficient: fit precision already .654914/recall .963451, fresh far precision .674208/recall .794667, fewer false hazards at the cost of more missed occupied cells. This is not evidence that training was excellent or that overfitting alone caused failure. Original raw depth is unchanged and still not globally admitted. The bounded post-rejection nearest-occupied-cell diagnosis uses only fixed .10m descriptive distance and never relaxes the binary gate. Details are in learned_memory_failure_diagnostic.json; no claim proves a sole optical or learning cause.

Next actual bounded work reuses exactly eight already closed exposures (two new validation sequences ×0/6/12/18s) to separate divider/body hazard queries from floor/background and quantify physical binocular overlap. This directly tests whether high aggregate source accuracy hides obstacle failure and whether the new body yaw restores relevant view. No new image/model inference, yaw scan, threshold or control change; datasets are now development for that source audit. It will decide whether an observation consumer is supported before new control. Full RGB+estimated-pose obstacle navigation and formal35sE0/E1/E2 remain unfinished. Prior slow-baseline/terminal and depth rejections retained. No alternative learned-memory network after this unique candidate failure.

Evidence in experiments/stereo_memory_navigation_20261004_v1: memory_validation_collection_protocol/status, two actual runs, learned_memory_fit_features_v2/seals, learned_memory_fit/weights/seal, learned_memory_held_features, learned_memory_held_predictions/pretruth seal, learned_memory_held_assessment, collection_author_checks, causality_checks, failure_diagnostic, tests/logs. At report creation this branch is author-verified; independent review separately records its personal checks and inherited scope.


## Append-only independent review and provenance clarification
ONE firstpass independentreview C0/I0/M2, reusedcontext. Personally25testsPASS1.41s/5794uniqueSHAs/all74raw-feature/B0label/score reconstruction exact/13prefix eachfutureBomb/actualcommands/fitstandardization16111rows3721positive/far gates confirmed. Full450suite21.49s/videos/CADprojection remain author evidence. Append-only criteria provenance: ONLYfar predeclared .95precision/.80recall/no-worseFPFN/25TP determines rejection; near-positive extra thresholdchecks in sealedassessor are descriptive, notpredeclared acceptance gates. Collectionprotocol inherited universalnegative schedule/originalyaw/symmetricendpoint fields are stale; actualfaroriginalstart/yaw−20/+.3,nearshortstart/yaw−20/−.3 per frozenplan/derivedcode/rawrequests/identity. Do not read inherited fields alone or altersealedprotocol. Model rejects without source/online/navadmission; fixedsceneoccupancy common acrossfit/held limits to same-scene whole-sequence pose/trajectorytransfer, not newscene generalization. No secondreview.
