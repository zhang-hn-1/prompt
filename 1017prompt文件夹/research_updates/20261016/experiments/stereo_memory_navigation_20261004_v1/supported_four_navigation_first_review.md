# First scoped four-point navigation review

Critical: **0**. Important: **1**. Minor: **1**. Read-only review; no GPU, actual launch, source edit or historical rescoring.

## I1: Required actual cross-track peak/RMS are absent

The committed plan explicitly requires actual crosspeak/RMS and evidence independent of index jumps/stalls. The assessment reports true goal/progress/clearance but no cross-track sample series, peak or RMS; none of the two verifiers computes them either. The empirical .2 tube therefore has no measured actual-error report for this unique run.

Requested fix: During post-close assessment, compute true XY distance to the applicable complete installed reference geometry independently of policy path_index, summarize peak/RMS with sample/reference scope and report stalls/index behavior without weakening native+true goal/progress gates. Use independent geometry tests; preserve all .8 nominal/.6 retained guard and source/pose gates.

Location: `assess_supported_four_navigation_v1.py:50`.

## M1: Explicit no-source-mutation test is missing

The plan names no source table mutation as a CPU test; the five supplied tests cover admitted/invalid/unsupported requests, feature-off equality and reverse reachability, but do not snapshot the frozen source/provenance before and after repeated queries. Code inspection shows no query mutation, so this is a coverage gap rather than observed corruption.

Requested fix: Add a small repeated-query immutability assertion in a future relevant test pass, or record the intentionally deferred coverage gap.

Location: `test_supported_four_primitives.py:6`.

## Verification and scope

- Explicit supported requests enumerate -.3/0 plus .5/0,+-pi/6; free-map reverse target test exercises -.3 rather than unreachable callback only.
- Source-disabled planner output equals original direction_root search in supplied fixture; diff confines search changes to command enumeration/curvature lookup. Consumer diff confines changes to import/enumeration; driver diff from negative_ray_memory_v2 preserves PID, BankMap, direction, limits and guard control chain.
- Driver uses with_command_domain_cap(with_supported_four_points(PriorRGBPolicy)); captured act is command_domain_cap.act, whose super act reaches unchanged PriorRGBPolicy / guarded reference tracker. Prefix i<140 uses observation requests only; i>=140 uses actual act.
- Current cleaned protocol is coherent: R1 only, max one actual sequence, 60s total including14s prefix, .8 nominal/.6 retained true guards, no strict AB or source equivalence, no formal35/navigation/model certification from scalar points.
- All current protocol 1883 source hashes matched. Historical preflight seals 1868 source hashes and14234 inputs including frame000/140, frozen seed, all inherited runtime dependencies and reverse/forward provenance matched; output pretruth seal matched and occurs before true clearance call.
- Reverse scalar was sealed development21-25.5 at16:43:48 UTC, validation prelaunch seal16:43:50 and validation launch16:43:53; fitted provenance contains development only. Both whole-sequence checks are admitted without rescoring.
- Historical preflight decision only,12 waypoints and true nominal clearance1.11218m; original three-forward rejection is left separate.
- Source95%/class95%/area6, pose RMS.05/peak.10/angularp90 2deg, original native+trueXYZgoal.2, axisprogress4, actual crossing, zero contact/collision and synchronized videos remain in scorer. Runtime capture contact is a physics-block latch, so sampled physics contacts are not lost merely by reading capture rows.

Personal reviewer tests: **5 passed in 1.42s**, focused CPU only. Author full-suite evidence is separate: first recursive attempt preserved19 collection errors; corrected project test scope log reports **650 passed in44.86s**.

The current cleaned protocol was assessed within this first review. No repeat review or final author freeze approval was performed. Unique actual navigation must wait for important finding closure and complete author freeze.
