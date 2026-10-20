# ONE terminal-goal branch final review

Reviewer context: this is the reused reviewer that completed the unrelated static-scene geometry review. It is independent of terminal branch implementation and had not previously reviewed this branch, but is **not a newly empty context**. Two new reviewer dispatches failed with the agent thread limit. This is one terminal branch review pass; no sub-review or second review was dispatched.

Scope: terminal_goal_consumer.py and its tests, frozen nominal parent, local runner and derivation, protocol/status, local assessment and scoring derivations, conditional full-run admission, execution diagnosis, report, and saved local evidence. Root's subsequent short-obstacle actual two-arm task and memory-side audit are excluded. No GPU, simulation, model inference, policy/controller change, source change, or scientific gate change was performed. Only this review and personal_checks.json were saved; the checking script ran from /tmp.

## Findings

- Critical: 0.
- Important: 0.
- Minor: 0.

The scientifically rejected candidate is correctly retained as rejected evidence. A rejected experimental result is not an implementation defect or grounds to change its gate.

## Personally verified

- `terminal_goal_consumer.py:5–10,18–24`: current finite 2D goal only; <=.60 latch; only the five final main requests clipped to ±.3; auxiliaries, signed/zero requests and inputs preserved. One inherited actual act per call; latch cleared by reset. Personally reran the three terminal tests and four nominal tests: **7 passed in 1.97s**. The once-only/reset test isolates inherited act with a mock; actual saved evidence separately validates the request/physical chain.
- `run_terminal_goal_navigation.py:116–141` and `terminal_goal_runner_derivation.json`: six declared substitutions reversed byte-exactly to the frozen nominal runner. All 1280 presealed source identities match, including selected gains/runtime/protocol/rig/source identities. All 924 post-close actual artifacts match. Frozen old nominal owned identities: 17/17; author terminal owned: 11/11; pre-finish scoring identities: 7/7.
- `benchmark/runner.py:1224–1230` and `assess_terminal_goal_navigation.py:83–90`: independently rebuilt all 123 world-to-current-base goal vectors from each saved current position/yaw and the declared task goal, exactly. Rebuilt the latch and nominal-parent/final request chain, including all 615 raw physical substeps, exactly. All auxiliary requests unchanged. Source timestamps are causal at all 123 calls; RGB/known-pose namespaces contain no offline depth.
- `assess_terminal_goal_navigation.py:93–96`: first actual old-fast request difference is frame110. All **551 samples × 17 physical fields** and **111 execution states** before execution of that request match the old frozen fast trajectory exactly. First terminal distance is .5750375326463583m at the first <=.60 sample. The latch remains on through frames110–122; seven request blocks differ from the same-call nominal request. Remaining terminal blocks can equal a parent request already below the cap.
- `assess_terminal_goal_navigation.py:16–46`: rebuilt all 25 source frames without inference: 500 packet fields and 50 eye masks exact; removed points correspond to CAD masks. Personally rebuilt 738 map arrays, checked 1476 same-live slow-shadow map diagnostic fields, and 615 saved reference/source fields. Cross-batch selected world points were exact in this replay; the existing 32-epsilon allowance remains administrative, not a scientific scoring relaxation. The future-payload trap rejected a future packet before payload access. Runtime source code also checks full tracker/reference equality and no shadow effects on action/clock/physical state; no second actual controller was executed by this review.
- Independently scored saved physical state: **123 actions / 124 captures / 616 physical samples / 25 source inferences**, 12.3s, final distance **.1969799472001205m**, contacts0, runtime_error0. Movement starts at frame60; lateral RMS .025167067272231645m and peak .05544284582138115m pass .12/.30. Exact frozen slow baseline remains 22.6s/.19302877235323554m; old fast remains 12.1s/.1953819838888008m and rejected. The new candidate exceeds the frozen terminal baseline by .003951174846884953m and fails baseline+1e-6. TaskPASS, geometryPASS, strict terminalFAIL, candidateREJECT.
- `prepare_terminal_goal_full_navigation.py:5` rejects preparation unless saved local acceptance passes. Actual full-run protocol/output do not exist; saved assessment says full_goal_triggered=false. No original full6.6m or formal E0/E1/E2 admission occurred.
- `diagnose_terminal_goal_execution.py:7–10` and cause JSON: personally recomputed first execution at step551/11.019999753683805s, request .3000000119 versus smoothed/applied .9399999976. Smoothed/applied reach .3 only at goal termination12.299999725073576s. Measured main joint velocity .8987478018→.4548425078rad/s and XY speed .3610679805→.1857597381m/s agree with the saved trace. The 1.3s command-ramp calculation is retrospective; it is not future input, an added settled-stop result, or a new controller.
- Personally decoded both complete videos: stereo_view and third_person each **124 frames at 10Hz**. This verifies decoding and denominators, not a new qualitative safety certification.

## Inherited evidence and limits

Read, but did not personally recreate, the original RED import-failure chronology and fixture correction. Read terminal_goal_full_suite.log: **408 passed in 21.86s** is the terminal preflight result. The subsequent **410-test preflight belongs to the next short task** and is excluded from this terminal claim. Full controller internal reference equality and clock/state immutability assertions were read in the sealed runtime producer; personal replay covers stored current goals/requests, source/map/reference fields and physical traces, not a second stateful act execution.

The report correctly distinguishes current known-pose/CAD/task priors from pure RGB, observed request reduction from a settled physical stop, and local task success from candidate admission. Historical RGB restart differences prevent an unrestricted pure causal performance claim. Offline depth/contact/progress labels are used for retrospective assessment only. Gates, old rejections, source rejection and whole_research_complete=false remain unchanged.

## Declined to judge

- Root's new short-obstacle actual two-arm task and memory-side audit: explicitly outside this review assignment.
- Original full6.6m, E0/E1/E2, learned memory/model increment, general navigation reliability and safety: not executed/admitted by this terminal branch; no claim is made about them.

## Assessment

Ready to retain this branch as a completed, verified **rejected bounded terminal experiment**: yes. Ready to promote the terminal candidate to full navigation: no, because the unchanged strict terminal gate failed. No Critical/Important fix pass is required. Overall research remains incomplete. No second review is requested after this pass.

Detailed personal results and provenance are in personal_checks.json.
