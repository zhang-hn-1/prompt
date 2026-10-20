# Static sparse memory and journal — one scoped first review

2026-10-08. ONE first review of this distinct new candidate, not a rereview of original E1 source-only or direct navigation. Scope: specified plan, grid/journal implementations and tests, archived CPU qualification/assessment, new runner and stage. No simulator launched; no source modified. This review file is the only written artifact. Protocol and author closure will be frozen after findings, so their final hash closure is outside this review.

## Critical

None found.

## Important

1. **Journal recovery errors prevent export of independent raw physical evidence.** `run_original_E1_static_sparse_memory_navigation_v1.py:226–230` closes and parses the journal before exporting requests at line 231, states/clocks/IMU/contact records at lines 232–233, physics at line 234, and failure videos at lines 235–237. `policy_row_journal_v1.py:46–47` deliberately rejects corrupt complete events, and `test_policy_row_journal_v1.py` explicitly verifies this error. Such an exception exits `partial_export`, skipping those independent exports. `export_then_close` still closes the runtime, but the experiment loses raw evidence needed to attribute the recovery failure and distinguish planned versus physically executed work. Isolate journal close/recovery in a guarded block, retain a journal-recovery error artifact and failed/incomplete status, and always proceed with the independent raw exports. Do not silently mark corrupt rows valid or promote pending rows. This is cleanup/error-path instrumentation; keep request/grid behavior unchanged.

## Minor

1. **Recorded action time excludes executed-journal persistence.** Runner line 207 records `end_to_end_wall_s` before line 208 serializes/flushes the executed row, and before terminal checks. Its name can therefore overstate coverage if used to certify every full action <=.1 wall s. Preserve the comparable existing timing metric if needed, but label its boundary or separately report journal write/flush overhead before any real-time claim. This does not alter physical requests.

2. **Journal schema validation permits decreasing action clocks.** `policy_row_journal_v1.py:12,18` enforce finite nonnegative time and consecutive frame IDs but do not compare a new planned row's time against the preceding completed row. The runner's observation adapter prevents backwards captures during normal execution, so no live regression is shown; a corrupted/replayed journal can nevertheless be accepted as causal. Add a monotonic completed-to-planned time assertion (and corresponding test) for standalone recovery evidence.

3. **One inherited comment contradicts the new run's goal stop.** Runner line 209 says goal is not navigation acceptance immediately before line 211 declares measured-goal success. The actual code correctly retains collision-before-goal stopping; update the comment to match known-pose conditional navigation scope.

## Verified evidence

- Ran both new component test files in the existing ball_wm interpreter without simulator startup: **9 passed in 1.52 s**. This is not a claim that final frozen/native integration checks or fresh-run outcome have passed.
- `static_sparse_point_risk_grid_v1.py:15–22` invokes the existing source validation/TTL mapper first, then merges past measured cell acquisition clocks with `maximum`. Empty later source records retain only already measured hazards. New measurements can legitimately give a cell a new birth; absent measurements do not refresh its birth. Future source is rejected before poisoned world payload access, and the test confirms no changed grid on that rejection.
- Labels remain `-1` unknown or `1` hazard; no `0` observed-free cells or finite floor clocks are created. Hazard codes distinguish current measured birth from past risk. No shape expansion, unseen geometry completion, depth-truth/scene occupancy import, or free relabeling appears in the new grid.
- Explicit static-scene assumption is mandatory. A fresh factory construction clears the cumulative clock; the runner constructs a new grid after its single policy/runtime initialization. The raw point provider remains `TentativePointMemory` with 10 s TTL and existing immutable source provenance.
- Archived qualification program validates the old B0 post-close file hashes, replays source packets only when their capture frame has elapsed, and compares cumulative cells/clocks at every archived action. Its saved CPU assessment reports exactly **546 journal rows and 546 static clock comparisons**, no fresh physics, no navigation admission, and `old_B0_complete60=False`. The five journal tests separately cover pending rows, immutable planned snapshots, changed requests, truncated tails, corruption rejection, and phase order.
- A runner diff against the preserved direct consumer shows the control change is replacing `PartialPointRiskGrid` with the persistent hazard subclass. Additional differences are candidate identity, matched 546-action/54.6 s budget, and planned/executed journal telemetry. Existing source, camera render/capture transaction, current GT root/camera registration, public-joint CAD, original scene/start/goal/material/light profiles, planner/search/reference generator, command cap, six-second settling, PID gains/caps/supervisor, and raw TTL provider remain unchanged.
- Source partial observations/packets and their provenance are committed before diagnostic GT depth reads. Current known-pose goal/kinematics are explicit TaskH privileged inputs; full occupancy and diagnostic depth are not passed to the actor. Maps and raw point snapshots are written before requests, preserving acquisition-clock records for post-close causal reconstruction.
- The loop allows at most **546 requests**. It stops at physical contact first, then true XY goal <=.2; budget exhaustion is `conditional_54_6s_timeout`, not success. The task artifact says matched B0 completed54.6 s prefix, not complete60 or strict35. Stage retains one sequence, one600-wall wait, no retry, bounded termination/kill cleanup, and post-child-close sealing.
- Geometry/selected membership/static assumption/local progress/no-contact versus navigation-success gates remain distinct in the plan. Old B0 and rejected source/learned-head/strict-benchmark scopes are preserved. New entire-trajectory source reconstruction/error/membership and pre-intervention request/physics comparison are future post-close obligations, not supplied by the archived CPU qualification.

Assessment: resolve the Important independent-export recovery issue before freezing/launching the single candidate. No Critical finding. This is one first review only; no further review loop or simulator run was performed.

## Author resolution received during first-review completion

The parent reported an author-side RED-to-GREEN correction for Important finding 1: independent raw exports now precede journal recovery; a recovery helper catches ValueError/OSError and writes `valid=False` without completed-row promotion on corrupt complete events. Parent reports **11 tests PASS**, including the new corruption recovery test. These are author-provided resolution/results, not a second reviewer pass. The original finding and initial inspected line references are retained above; author closure should bind the final bytes and document this resolution.
