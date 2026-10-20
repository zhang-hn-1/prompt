# First independent review: neutral-bank visibility branch v1

Assessment: suitable for the single bounded development sensing probe after the coordinator's full-suite gate. C=0, I=0, M=1. No code blocker found. This does not admit the model, source, controller, estimated pose, production use, or formal navigation success. No simulation was launched in this review.

## Scope and personally performed checks

Read the complete 2026-10-07 neutral-bank plan, helper, five tests, entire new runner, saved controlled runner diff, protocol, and the unchanged observation handoff, qualified-hazard consumer, PriorRGBPolicy and original roll PID. This is one first review of the new branch; no repeat review of earlier Hero or qualified-source results.

Personally ran `PYTHONDONTWRITEBYTECODE=1 /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q -p no:cacheprovider experiments/stereo_memory_navigation_20261004_v1/test_neutral_bank_visibility_probe.py`: 5 passed in 0.19 s. Output: `neutral_bank_visibility_reviewer1_v1/cpu_helper_tests.log`. Only CPU NumPy/pytest helper imports were used. No runner/model/simulator import, GPU use, weights deserialization, or network activity.

Personally SHA256-verified all 1287 protocol source identities: zero missing/mismatching files. All 1283 identities from the original protocol are present with unchanged hashes; four additions are the new runner/helper/test/plan. The original runner SHA256 is `84ee3c0ae2d1ac7d01c7670511bbcfaa9fe2baad544305e2164ccea0f0eb43a2`, matching the declared base. New runner SHA256 is `95be745aebfba49aac8bbd0faba77654defb6416392e54ea6a2ce111655c63cf`. Personally recomputed the exact unified diff; byte-for-byte equal to `neutral_bank_visibility_runner_diff_v1.txt`. Both runners parse as AST; unchanged CaptureRuntime and recorded_reset ASTs are exactly equal. Recorded_act contains exactly one original act call and exactly one probe.choose call. Evidence is in the reviewer1 JSON and recomputed diff files.

Inspected, but did not personally run, author RED/GREEN evidence and the full-suite log (510 passed in 26.61 s). The full suite is therefore author-run evidence, not an independent execution claim.

## Strengths and plan alignment

- Frame identity, increasing source time and current/source clocks are validated before converting/copying the action payload. Repeated calls on the same frame do not increment deficiency; distinct good updates reset deficiency; fewer than 100 qualified hazards on two distinct post-handoff updates trigger the latched event.
- Before handoff the helper leaves the original schedule request unchanged. The runner retains the exact original handoff and five XY-low confirmation/rebound reset code. The original handoff source must contain >=100 native hazards; qualification at the eligible initial seed retains that support. No artificial admission is added by the new consumer.
- Active output is all ten zeros during [event,event+4 s); at its end nominal output resumes permanently. No depth/stability wait, second trigger, repeated policy call, or reset was added. All-zero auxiliary input requests desired bank zero through the existing PID, and main zero reaches the original smoother. Physical settling/effective action change remain measurements for the actual probe, not assertions from these CPU checks.
- B0 is still called once on every observation, including the probe interval. B0 nominal_request, pre_probe_request and final local.request/policy_inputs.request are explicitly separated. The nested probe diagnostics retain the pre-probe request, source identity, event time/frame and active/finished status. The original policy request diagnostics remain nominal diagnostics; dispatch auditing must use the explicit final request and PID/execution records.
- Exact runner diff changes only protocol/output namespaces, import/instance/consumer insertion and explicit logging/identity labels. Rig, inference cadence, Fast model, qualification, original observation schedule, reset, gains, planner, tracking, caps and smoother remain frozen by identical code and source hashes.
- The new decision path consumes only current time/source identity, qualified count, handoff state and request. It has no GT input. Existing GT depth reads occur only after current RGB source/qualification is fixed and are saved in offline_truth. Known pose remains the explicitly privileged development condition.

## Issues

Critical: none.

Important: none.

Minor M1 — inherited protocol labels still describe the old consumer. `neutral_bank_visibility_protocol_v1.json:1396,1428-1431` retains accepted_controller without the probe, an old interruption-recovery description and `control_variant="source_qualification_only"`. New experiment/plan/source hashes and runner artifact labels identify the actual branch correctly, so this is not a runtime or scientific retry blocker. For provenance clarity, normalize these descriptive fields to the one-shot neutral-bank probe and retain the explicit no-scientific-retry constraint.

## Behaviors declined to judge

- Actual binocular visibility, neutral-bank settling, contacts, geometry precision and new-cell recovery: require the one actual moving-camera probe and its sealed offline assessment; no simulation was authorized for this reviewer.
- Enforcement by the outer persistent driver of 900 s wall time and no duplicate launch: outside the supplied new runner/helper diff; coordinator must apply the existing launch discipline.
- Production/formal navigation admission or improvement relative to the historical collision run: explicitly excluded by the plan and unequal source trajectories.

## Recommendation

Proceed through the full-suite gate to exactly one planned development probe. Use all window source updates and the predeclared sensor criterion, and keep the resulting sensor verdict separate from task navigation outcome. M1 is optional provenance cleanup; no C/I fix is required by this review.
