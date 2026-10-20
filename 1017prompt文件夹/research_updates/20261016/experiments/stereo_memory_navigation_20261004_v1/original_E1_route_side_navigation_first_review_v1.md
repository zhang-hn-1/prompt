# Route-side consumer — one scoped first review

2026-10-08. DISTINCT new first review, not a second review of source-only/direct/static-memory runs. Scope: specified plan, route-side grid, read-only shadow, their tests and runner test, new driver/stage, nominal geometry and archived shadow CPU assessments; current actual policy/tracker MRO and waypoint semantics were inspected. No simulator was launched or source changed. Final protocol/review closure is not generated yet and must bind the final artifacts before execution.

## Critical

None found.

## Important

None found in this scope. In particular, the suspected initial forward request / cusp-skipping mechanism is **not present in the actual composed tracker**; evidence follows. This is no claim that the candidate nominal plan will physically track successfully.

## Minor

1. **Add the actual overlapping-cusp regression to the focused CPU suite.** `test_E1_route_side_runner_v1.py:5–16` checks runner strings; `E1_route_side_shadow_CPU_v1/assessment.json` reports fork 180 but does not state its direction or projection index. The nominal frame155 plan has a short reverse first edge and a forward return overlapping its start. This invites incorrect admission/rejection reasoning if tracker composition changes later. Freeze a CPU regression for the actual MRO: braking at installation, reverse first-fork request, projection window excludes overlapping forward waypoint2 while reverse is active. The current behavior passes the reviewer reproduction below; no controller modification is warranted.

2. **Inherited comment contradicts the conditional goal stop.** `run_original_E1_route_side_navigation_v1.py:218` says goal is not navigation acceptance immediately before lines 220–221 stop with `termination='success'` at the measured goal. The implementation is correct for the explicit known-pose TaskH conditional scope; correct the comment without changing the stop policy.

## Focused suspected-cusp investigation

The actual tracker MRO obtained from the new runner's exact factory composition is:

`OutgoingEndpoint -> ConfidentSpeed -> MappedBankOracle -> CuspBoundary -> OraclePathPolicy -> _PolicyBase`.

`cusp_projection.py:16–20` holds the current waypoint when its outgoing direction is not yet the accepted direction. `benchmark/policies.py:689–700` subsequently scans only through the FIRST next waypoint with a different accepted direction. It does not scan the later overlapping return waypoint. `outgoing_lookahead.py` uses outgoing-edge direction, including the next cusp endpoint as the reverse lookahead target. `depth_local_curve.py:54–56` installs replacement geometry at index0 while preserving direction-confirmation state.

Read-only CPU replay of the archived current/past inputs, with the exact current composition and no simulator, produced:

| Frame | Candidate request main / aux | Path index | Accepted direction | Interpretation |
|---|---|---:|---|---|
| 155 | 0 / 0 | 0 | forward | Installed reverse-first plan; direction-change braking |
| 156 | 0 / 0 | 0 | forward | Still holding the unaccepted reverse waypoint |
| 165 | 0 / 0 | 0 | forward | Braking retained |
| 179 | 0 / 0 | 0 | forward | Braking retained |
| 180 | -0.3000000119 / -0.5235987902 | 0 | reverse | First request fork accepts and commands reverse |

At frame180, the recorded reference's first three points are:

- index0 `(-1.707605362, .440354794)`, outgoing reverse;
- index1 `(-1.964349648, .285169868)`, outgoing forward cusp, .299999983 m from index0;
- index2 `(-1.728389736, .469276890)`, outgoing forward, near index0.

The replay's reverse lookahead index is1, pointing to the cusp, while projection remains0. The archived shadow artifact agrees: both candidate and B0 main requests are `-.3000000119`; their first difference is auxiliary angle (candidate `-.5235987902`, B0 `-.3757525086`). It is not a forward-before-reverse fork.

An additional isolated CPU projection query used the exact nominal frame155 plan and actual composed tracker with reverse accepted, placing the query exactly at the overlapping waypoint2. Projection still returned **index0**, distance **.0356156963 m**; waypoint2 was outside the scan window. Thus the suspected overlap shortcut does not invalidate this candidate's nominal admission. Tracking remains nearest-waypoint based, so this is not proof of exact physical traversal to the cusp or future closed-loop safety.

## Other verified evidence and limits

- Ran the three specified focused test files in the existing ball_wm environment without simulator startup: **7 passed in 2.14 s**. Archived inputs were hash-checked by the CPU replay prefix. No final protocol hashes, native runtime launch or new physical outcome are claimed.
- `route_side_commitment_grid_v1.py:31–36` returns the original first successful global route while deriving one immutable axis/first observed inflated component/side/birth decision. Lines 42–50 apply the commitment to global-route traversability only; observation labels/codes/acquisition clocks and the existing local Hybrid A* primitives/clearance remain unchanged. Tests verify first-route identity, growth behavior, no labels/time mutation, unsupported crossing, changed-goal rejection, release, and new-instance reset.
- Birth metadata is derived only from finite measured hazard clocks and asserted no later than current grid time (lines 22–24). Commitment/release are explicit planning decisions, not observed free space or completed obstacle volume. No depth-truth/full-scene actor interface is introduced; original source/camera/current GT registration/public-joint masking/static risk/provider TTL10/PID/gains/speeds/caps/six-second settling are unchanged in the runner diff.
- `readonly_boundary_shadow_v1.py:9–17` checks elapsed capture/envelope/record clocks before copied payload consumption. An independent frozen static policy acts exactly once per active frame on deep-copied inputs. Lines 19–22 deactivate on the first unequal request; subsequent calls read no payload. This excludes candidate-dependent future trajectory from B0 truth after fork. Saved archived on/off and future-truncation checks remain CPU interface evidence, explicitly not fresh alternate physical rollout.
- Runner lines 219–226 stop on actual physical-contact collision before goal; at action546 / 54.6 s, true goal remaining <=4.182900500 and true axis progress >=2.427377623 are required for continuation. Failure explicitly stops as `REJECT_LOCAL54_6_PROGRESS_STOP`; extension cannot rescue it. Goal before that boundary stops immediately, with actual final state/termination preserved. No path-index acceptance replaces physical progress.
- After a passing physical prefix, the same runtime may continue to at most1200 requests /120 s, under one1200-wall stage with no retry/reset. Budget exhaustion remains timeout, not success. Source prefix/whole-trajectory geometry and selected membership remain post-seal independent gates; strict35/pureRGB/estimated-pose/full-volume/learned-head admission is not claimed.
- Independent raw evidence is exported before journal recovery, preserving the prior author correction. The stage waits for child closure before sealing and preserves old source, nominal CPU and shadow prerequisites. `complete=True` denotes experiment closure and does not supersede termination, prefix gate, contact, source gates or benchmark scope.

Assessment: no Critical/Important finding requiring candidate rejection was found. The exact current tracker semantics reject the suspected forward-cusp shortcut hypothesis. Only one first review was performed. This permits the predeclared experiment entry after final identity/test closure; it does not admit nominal geometry as executable safety, future navigation success, or source quality on the new trajectory.
