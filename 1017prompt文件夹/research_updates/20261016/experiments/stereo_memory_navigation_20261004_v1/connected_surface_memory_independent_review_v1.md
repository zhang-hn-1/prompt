# Independent review 1: one source-connected surface memory diagnostic

Decision: the frozen ONEgraph hypothesis remains **REJECT_SINGLE_CONNECTED_GRAPH**. No Critical or Important issue changes that decision. The implementation and assessment reproduce the intended single, privileged-knownpose, development-only comparison. No source, action, free-space, learned-model, whole-sequence, physics, or formal-navigation admission follows.

## Findings

**Critical: none.**

**Important: none for the six frozen diagnostic packets and their reject decision.**

**Minor — interpolation threshold is not a universal all-taps-clear contract.** `predict_connected_surface_memory_v1.py:28` uses remapped complement-mask `>.999`. Actual OpenCV linear interpolation has a smallest nonzero two-dimensional weight of 1/1024; one masked corner can therefore give 0.9990234375 and pass. I personally reproduced that fixture. I separately evaluated strict remapped support `==1` at all 49 patch positions and both past exposures: zero support differences, zero F0 acceptance differences, and zero F1 acceptance differences in every frozen packet. Thus actual six-packet support matches the stricter contract, but the implementation should not be advertised as enforcing literal all-nonzero-taps-clear for arbitrary future inputs. This is a documentation/latent boundary limitation, not permission to rerun or tune this rejected hypothesis.

**Minor — seal ancestry is partly verified rather than embedded.** The graph seal records 17 directly loaded NPZ inputs, seven output artifacts, code/module/plan/test-log hashes. Its predictor verifies the upstream output manifest before reading source records; the upstream records file is in that manifest. It does not embed a hash of the upstream seal JSON itself in its input manifest, and postseal assessment/causal audit are outside the source prediction seal. This does not invalidate the checked frozen outputs or rejection, but the author source seal alone is not an entirely self-contained ancestry/assessment manifest. Any supplemental audit must be marked posthoc and must not rewrite the original seal.

## Personal validation

All work was written only to `connected_surface_memory_reviewer1_v1/` and this report; author code, sealed outputs, frozen data and logs were not edited. Runtime: `/home/an/miniconda3/envs/ball_wm/bin/python`, NumPy 1.26.4, SciPy 1.15.3, OpenCV 4.6.0; bytecode and pytest cache disabled. System Python lacked NumPy, and direct pxr import was unavailable; no package installation or physics execution was attempted.

1. Personally ran three original graph tests: **3 passed**. Their literal chain, 3D separation, invalid-node and ordering cases pass.
2. Personally replayed the predictor into a distinct destination by copying only the runner and replacing its fixed H/output namespace. Every array in all six replayed NPZ packets equals its author counterpart; every original upstream NPZ field is retained unchanged. Identity includes eye, frame, query pixel ID, current/past UV, source frames/times, selected depth and geometry availability time.
3. Personally replayed the assessor against that replay: entire JSON equals author assessment excluding the expected UTC field.
4. Personally verified all 17 source input hashes, all seven prediction output hashes, and predictor, graph helper, projection helper, fixed plan and author full-suite-log seals.
5. Personally wrote a separate oracle using sparse undirected connected-component labels rather than the author's traversal. It independently reconstructs both arms and reproduces every acceptance array. Own-history prior-anchor counts are F0/F1 = 1501/1501 at 235, 1653/2620 at 245, 1778/3475 at 255. Both eyes are one current epoch; only valid original current proposal nodes participate; F0 uses only direct distance to its own past accepted anchors and F1 takes within-current graph closure seeded by its own past anchors, at the same 0.10 m 3D distance.
6. Personally reconstructed three-endpoint scoring with a separate optical-ray expression. All twelve eye/epoch/arm rows, insufficient denominators, finite wrong counts, precision, novel correct points and canonical 0.10 m XY bins equal author results.
7. Personally checked current patch FOV bounds and all 49 current CAD mask positions, both source FOV/positive-depth tests and strict actual OpenCV bilinear support; no differences from stored validity. Checked source exposure times precede current availability and are within the original 2 s ray-source domain.

Personal evidence: `tests.log`, `prediction.log`, `assessment.log`, `verification.json`, `independent_oracle.json`, the replay scripts and `replayed_outputs/` in the reviewer namespace.

## Reproduced outcome and frozen gates

| Epoch | F0 left | F1 left | F0 right n / correct / precision | F1 right n / correct / precision |
|---|---:|---:|---|---|
| 23.5 s | 0 | 0 | 152 / 70 / 0.460526 | 1119 / 674 / 0.602324 |
| 24.5 s | 0 | 0 | 125 / 76 / 0.608000 | 855 / 633 / 0.740351 |
| 25.5 s | 0 | 0 | 784 / 699 / 0.891582 | 1389 / 1249 / 0.899208 |

The necessary precision gate (each nonempty F1 eye/epoch >= 0.95, insufficient included) **fails**. At least one F1 packet >= 100 **passes**. Same post-23.5 epoch bilateral correct novel-point/bin coverage **fails** because every left packet is empty. Shared nonempty precision nondecrease **passes**. Correct post-23.5 spatial bin union increases 5 to 11, so increase **passes**. Necessary conjunction therefore rejects. Empty left packets are neither scored as perfect precision nor glossed as bilateral coverage. No new gate was substituted.

## Inspected evidence and limits

Code inspection confirms separate F0/F1 histories and original insertion stamps, filtered by `0 <= now - stamp <= 10`; retaining a previous anchor does not refresh it. New accepted current nodes are stored at their original packet geometry-availability epoch. Frozen epochs end at 25.5 s, so no original 18 s seed actually reaches expiry here; expiry was inspected in code, not experimentally exercised by this six-packet replay. Stored outputs retain identities although the internal ephemeral distance cache needs only points/stamps.

The source runner reads only the original 18 s seed, sealed original ray proposals, knownposes and masks. The imported projection/helper modules do not read truth or actions. GT is read only by the separate assessor after prediction sealing. This is an old rejected V1 ray-source consumer with explicit knownpose privilege and already-inspected development labels; the mask check does not regenerate V2 depth predictions or rehabilitate the rejected source.

CAD provenance was inspected through `dynamic_self_mask.py`, capture code and `verify_qualified_hazard_collection_v2.py`: robot visible CAD triangles, current measured joints, frozen base-link mask, fixed camera calibration and knownposes; no scene-depth or image labels feed mask formation. I verified actual frozen mask file hashes and their interpolation use, but did not independently regenerate CAD masks from USD in this review. Author log reports **486 passed in 22.70 s**; this is inspected author evidence, not my personally rerun full-suite claim. Postseal component audit is causal development analysis only, not a new pass condition or source-seal item.

Review ends with the frozen rejection. No alternate radius, graph cap, gate, source/network hypothesis, new inference, physics, control installation, formal E0/E1/E2 execution, or rereview rescue is proposed.
