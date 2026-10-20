# Occluded obstacle memory navigation — one independent final review

Verdict: **Critical 0 / Important 0 / Minor 1** within the bounded scope inspected. The branch is acceptable as a preserved **negative experiment**, not as a successful navigation or memory-benefit result. Whole research remains incomplete. No GPU run, physical episode, matrix repeat, scientific-code edit, sealed-result edit, or commit was performed by this reviewer. The separate estimated-pose pipeline is excluded.

## Scope and evidence identity

Read the occluded-memory report, dated spec and plan, B0 and derived runner diff, derivation, protocol/status, both assessors and v1 failure log, v2 assessment, source-height/range diagnostics, and actual B0/history records. Git HEAD is unborn: `git rev-parse --verify HEAD` failed, so no commit-diff claim is made. Source SHA closure is used instead.

Personally rehashed 13 selected entries from the 1,314-source protocol and 11 selected entries from the 2,700-artifact post-close manifest; all matched. Independently reversed the six runner substitutions and obtained the exact B0 runner text; both runner hashes match the derivation. Full selected hashes, commands, counts, and the explicit inherited-evidence list are in `occluded_memory_review_personal_checks.json`. The complete 4,014-entry hash sweep is the author's evidence, not a reviewer rerun.

## Personally verified

- Six bounded CPU fixtures passed in 4.64 seconds: the three same-stream shadow tests, two current masked-memory shadow tests, and the source truth-field/proprioception whitelist test. This is not a rerun of the historical 405-test suite or the author's 1.85-second timing.
- History contains 350 policy calls, 351 state/camera captures, 1,751 physics samples, and 71 source inferences. All 350 saved policy-input request vectors equal the primary records; all 1,750 subsequent physical raw-command pairs equal those request vectors in order. The shadow's request vectors are not substituted into this actual-command trace.
- Against the 299-call B0, first actual request difference is frame 120 (12 seconds). The first 601 physical samples agree exactly across 17 fields, including applied action and physical state; the first 121 execution-state entries also agree. Excluded raw absolute clock fields are named in the checker, rather than being represented as equal.
- The independent current shadow has 210 differing request blocks, first at frame 140 (14 seconds), on the same source IDs/times. Every record declares current memory, read-only/unexecuted status, and no alternative-future truth; primary records declare history. Source age peaks at 0.3999999910593033 seconds. Runner inspection confirms that the primary action is returned after the shadow call and runtime/state immutability assertions.
- Recomputed from the actual states: history times out at 35 seconds, final distance 5.5074564000548705 m, final goal-line progress 1.1843653142452237 m, maximum progress 1.8216765344142911 m, path 3.4208497360622294 m, zero contact samples. B0 collides at 29.9 seconds, distance 4.018381692018365 m, goal-line progress 2.5961178362369535 m, with five recorded contact samples. Both success flags are false.
- Checked actual request phases: positive main request over frames 60–222, zero main request over 223–248, negative main request over 249–349. All post-settling plan statuses are success; no `no_route` termination explains this run. The observations do not establish a unique reversal/controller bug.
- Independently checked left and right RGB arrays at reset frames 0, 60, and 120: each differs between the two physical resets despite the equal pre-branch physics. Consequently, the executed-run contrast cannot establish a pure memory-only causal effect. The same-stream shadow cannot be assigned the history run's subsequent actual trajectory.
- Recomputed all 60 B0 saved-source height/range diagnostics from offline renderer depth after source production: 78,658 queries, 76,386 finite truth points, 76,236 predicted hazards; zero hazards with finite truth at the floor threshold, 735 hazards without sufficient truth, and 30 sensitivity-ambiguous hazards all at positive truth height. The range and error percentiles exactly match the diagnostic JSON, including zero finite truth endpoints within 0.55 m and error p90 0.07578431367882155 m. This does not identify object instances, certify source safety, or eliminate all self-body/geometry possibilities; the report's bounded interpretation is appropriate.
- Compared all 71 saved old/new source packet pairs: query/partner IDs remain exact; only `source_265.npz` has one non-bitwise world coordinate, with absolute difference 1.1102230246251565e-16 m. This reproduces the v1 failure's scale. Direct assessor diff confirms v2 applies 32-machine-epsilon scaling only to that cross-batch retained-world comparison. Each rebuilt packet versus its own saved packet, mask comparison, query/partner identity, primary map checks, and numerical scoring gates retain their existing exact/frozen checks. V1 code and its failure log remain present.

## Inherited evidence and limits

The full source/CAD mask rebuild, 2,100 history-map arrays, 4,200 current-shadow diagnostic fields, future-payload map trap, complete video decodes, and full historical test suite are read and inspected author evidence. A reviewer attempt to instantiate `RobotSelfMask` in the CPU environment stopped at missing `pxr`; no simulator launch or installation was used to bypass that limitation. Thus the 1,562 source/mask field reconstruction is **not** claimed as a personal rerun. The initial reviewer helper also encountered a settling-row schema difference (`plan.status` absent before frame 60); its post-settling check was corrected without modifying project code. These helper/environment issues do not change a scientific gate.

Known pose, initial safe-zone and flat-ground priors, original full-goal budget/tolerance, shared 0.3 cap, unchanged roll-PID and source/filter all remain the experiment's scope. The original source rejection, old pose gates, cap-1 rejection, lack of learned-model increment, and lack of formal E0/E1/E2 admission are not reversed. Reduced contacts, parking, delayed failure, or retained hazard cells cannot convert this full-goal failure into a navigation/memory pass.

## Findings

### Critical

None found in the reviewed scope.

### Important

None found in the reviewed scope. This is a review of the validity and closure of a failed experiment, not acceptance of its navigation performance.

### Minor — inherited role and continuation labels are stale

`run_occluded_obstacle_memory_navigation_stage.py:3` writes `shadow='same-source history readonly'`, which survives in the finished status file, while the actual runner uses `ReadOnlyCurrentMemoryShadow`. Protocol lines 1432 and 1440 still describe a current primary/history shadow or current-memory baseline, although lines 1449–1450 correctly specify history/current. `occluded_obstacle_memory_continuation.json:3` and the ledger retain an earlier running snapshot after authoritative status and raw outputs show completion. A reader or lightweight status consumer can therefore select the wrong arm description or believe another run is still needed.

Recommended disposition: append a clear correction/closure note pointing to the authoritative finished status and actual history/current roles; label the continuation/ledger text as historical. Preserve frozen protocol/source/status bytes and their existing hashes. No new experiment or scientific change is needed for this documentation issue.

One review pass only. Research mission and navigation/memory success remain incomplete.
