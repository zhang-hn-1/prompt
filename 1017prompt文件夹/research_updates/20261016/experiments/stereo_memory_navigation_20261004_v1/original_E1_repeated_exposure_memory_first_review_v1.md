# Repeated-exposure memory: first whole-artifact review

Critical: **0**
Important: **0**
Minor: **0**

Assessment: the reviewed artifact is consistent with the bounded plan and has no blocking finding for freezing and ONE fresh actual sequence. This is an implementation/interface review, not evidence of navigation success, complete safety, or an effective repair of the entire prior failure.

## Scope and source identity

Reviewed all seven files in `original_E1_repeated_exposure_memory_review_source_identity_v1.json`: grid, tests, historical CPU qualifier, actual driver, stage launcher, freezer, and native preflight. Personally checked every file SHA256 against the supplied identity: 7 / 7 match. Also inspected the plan, parent grid/TTL memory, read-only boundary shadow, closed previous driver, historical CPU assessment/map, and native identity report. No implementation or unrelated artifact was changed.

## Findings and requirement checks

The candidate changes persistent promotion alone: at least two distinct actual capture frame IDs in the same existing raster cell, with a set preventing repeated old-record consumption or multiple same-frame points from counting twice. Recent singleton hazards retain the original inclusive TTL10 behavior. Expired unconfirmed cells become UNKNOWN, never observed free. Confirmed cells retain original acquisition clocks. Parent static-scene requirement, raster, nonbinding route hypothesis, planner clearance, and goal/route logic remain inherited. All acquisition clocks are checked by the partial-grid implementation before reading any world payload; envelope checks remain inherited. The immutable unique-frame raw memory supplies actual source identity, and changed frame timestamps are rejected.

The actual driver diff against the closed nonbinding driver changes run identities and the intended grid assignment: candidate RepeatedExposureMemoryGrid versus baseline NonbindingSideHypothesisGrid. Both arms retain the same policy and tracker composition, cusp acquisition, tube viability and command cap. Sensor generation, GT pose registration, diagnostic depth reads after source output, PID, speed, action loop, contact stop, success threshold, and frozen 54.6 progress gate are unchanged. No GT actor membership filter, future alternate physical label, new neural fit, forced early intervention, or gate extension was introduced.

The qualifier reconstructs the closed baseline exactly until the first request fork, uses separate candidate and baseline instances, stops at that fork, compares shadow on/off requests and references, and checks truncation/future poisoning at 65, 155 and 544 with guarded source/history reads. Candidate-start clearance comes from the candidate past-only map rather than the baseline. The stage/freezer pin source and input identities, require zero unresolved blocking review findings plus necessary CPU/native preflight results, preserve prior source/control rejections, allow one startup, and retain failure/no-retry closure and post-close sealing.

## Personally tested

Ran the five bounded related test modules (repeated exposure, partial-point TTL, static sparse grid, nonbinding hypothesis, cusp coast) with the specified ball_wm interpreter and USD import environment, bytecode disabled and pytest cache disabled: **22 passed in 2.24 s**. These exercise duplicate-frame nonpromotion, recent singleton hazard/expiry, distinct-frame persistence and clock preservation, future-before-payload rejection, timestamp identity, and related unchanged behavior.

Independently recomputed clearance from the saved candidate fork map and closed frame544 position: **0.894427190999916 m**, exceeding unchanged margin **0.8707106781186548 m**. Independently checked the saved candidate map has no observed-free labels. Personally verified all seven reviewed source hashes.

## Inspected evidence and prospective limits

Inspected, but did not rerun, CPU85499 historical qualification: PASS_NECESSARY_INTERFACE_ONLY, first fork544, 545 evaluated requests, exact baseline prefix, exact on/off requests/references, and all three future/truncation checks. Inspected the native import/class/reset preflight report with matching driver hash and matching candidate/baseline policy/tracker MRO; no simulation or NN was initialized by it. No physics, NN, actual sequence, freezer or stage launcher was run during this review.

The historical first fork leaves only 0.2 s before the frozen 54.6 gate. It does not establish that this correction can repair the earlier 14.9 s reverse/progress loss or the whole failed run. One fresh actual sequence tests true online memory consumption/intervention and unchanged physical criteria; fresh RGB may move the event. Nonintervention or an inadequate intervention window must be reported as such, not as proof of actuator-level ineffectiveness. There is no delayed-gate rescue, extra run to force an earlier event, or alternate-physics inference from the baseline logs. Correlated capture exposures do not imply independent errors, a completed obstacle volume, or a safety proof. Evaluator-only removal of the two historical singleton cells motivates the final-blockage hypothesis without becoming online actor filtering.
