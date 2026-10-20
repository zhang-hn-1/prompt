# First independent RGB-memory interface review

Review closed: 2026-10-07, after the author fresh assessment was actually present (assessment UTC 01:34:02). This is the first review of the new readonly RGB appearance-memory branch. Earlier velocity, anchor and learning reviews do not certify this branch. Scope: both 2026-10-07 RGB plans, the past-point report/probe/assessor, PastPointRGBMemory and its four tests, the online runner/stage/protocol/derivation/status, online assessor, and the collection-verifier code. No physics started, no original project writes, no Git operation, no full 473-test rerun, no author artifact modified.

**Interface/appearance decision: C0 / I0 / M2. The observed fresh result satisfies the limited preregistered readonly appearance gate. This is not new depth, free-space certification, mapper admission, action integration, estimated-pose validation, unseen-scene generalization, or navigation success.** Collection/source/physics/video results below distinguish author verification from personal checks.

## Personally checked

- Read both plans completely and the development report, module, all four tests, probe/assess scripts, online runner/stage, protocol metadata, derivation, status, online assessor and collection verifier. Read current status in research_mission.md (758–766), which still describes the running stage at review time.
- Ran the four targeted tests with bytecode and pytest cache disabled: **4 passed in 0.21s**. Test output: `newpersonal_checks/rgb_memory_tests.log`.
- Reconstructed all four old development outputs with the new module: **52 old saved fields exact**, without reading renderer labels during prediction reconstruction. The original source IDs and geometry time are retained.
- Independently checked **2759 post-close artifact hashes** against `artifact_manifest_post_close.json`. Reconstructed all **71 fresh source-update outputs / 1207 fields**, exact against the saved online predictions. Qualified seed was frame180, using five distinct captured 10Hz state times, each XY speed<.05, at least100 hazards, current source age0; same source frame/time as its RGB packet. Actual seed time17.999999597668648 uses the frozen 1e-6 tick tolerance around18s; it is not a late-seed age refresh.
- Queried future RGB and future pose payload traps at all71 source updates (**142 rejections before non-time payload access**), checked future seed source rejection, and71 duplicate replays without source-age refresh. Extended the independently reconstructed truncated prefix to **40 updates through frame195**, including seed and three subsequent nonempty frames, exact against whole-sequence output. These are input-local causal checks; they do not constitute archive-wide future-file poisoning.
- Independently recomputed all71 bilateral renderer-label rows *after* reconstruction and poison/prefix checks. Every count and precision agrees with the author assessment. Independently recomputed XY bins, speed, current hazards and original TTL/source-frame/time. The result is saved in `newpersonal_checks/rgb_memory_interface_results.json`; runnable readonly checker: `newpersonal_checks/rgb_memory_interface_check.py`.
- Tracked six author code/protocol/assessment files before/after personal checks; bytes unchanged. Verified source ownership copies and no consumer of `proof` in the RGB branch by direct code inspection. `proof` is written to its output namespace and summarized; actual requests come from the fixed scheduler. Runner inverse diff against its declared parent is limited to the new recorder, yaw−10 and output/identity namespaces.

| Fresh frame | accepted | both eyes correct≤.10m | finite wrong | insufficient | conservative precision | XY bins | current hazards | speed m/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|185|79|76|0|3|.962025|6|0|.031523|
|190|65|64|0|1|.984615|4|0|.087033|
|195|57|57|0|0|1|2|867|.108034|
|200|54|54|0|0|1|2|2170|.104216|
|205|31|31|0|0|1|1|1264|.097561|

All other66 source-update rows are empty and precision is N/A. Only frame190 meets the moving(.05+), zero-current-hazard, ≥20pixel, ≥2bin coverage requirement. Four missing labels remain in conservative denominators, not hidden as finite successes. All nonempty frames meet≥.95. Tiny spatial coverage and correlated pixels remain; there is no claim of65 independent hazards.

## Interface and execution boundary findings

Single successful seed; retained geometry and original query/partner IDs are owned copies, frozen thereafter. Current appearance time is separate; current known transforms project old3D into both cameras and a fixed7×7/std5/NCC.8 comparison performs no search or threshold scan. Geometry expires when its original age exceeds10s, with no reseed or refresh. Timestamp checks for source/RGB/pose occur before their non-time payload, reject future and mismatched timestamps, and prevent rewind. Empty/unseeded and expired outputs cannot assert `certified_free` or `source_model_admitted`.

The class receives qualification as a caller-provided boolean; class alone does not certify the five-state/hazard qualification. The actual runner enforces it using distinct current captured states and the current frozenFast/CAD packet; personal reconstruction confirms those rules for this collection. Current CAD masks are produced from the current source update and current joints. It is a privileged known-pose diagnostic, not an estimated-pose claim.

In the actual capture body, the appearance result is saved before `read_camera_depth` and before the current `offline_truth` output. The inherited base capture computes occupancy clearance/contact metadata earlier, but no such labels reach this appearance module or seed selection. In the independent checker, predictions are reconstructed before renderer depth is loaded. This supports the precise renderer-label separation claim, not a claim that the entire simulator never reads any GT metadata.

The RGB memory has no mapper/planner/action consumer. Nominal PriorRGBPolicy and the other existing readonly shadow still run for diagnostics, while executed requests are explicitly replaced by the fixed0/−.3/0/+.3/0 schedule. Saved physical-state/clock assertions surround capture and readonly work. The collection-verifier code checks all350 actual fixed requests/1750 dispatch ticks, and reconstructs1745 execution ticks from the actual execution snapshot and measured feedback; the initial5 ticks are explicitly not cloned. This reconstruction verifies software state/commands under measured feedback, not a new physical on/off counterfactual run. I inspected this verifier; I did **not** personally rerun its execution clone, CAD/source reconstruction, video decoder or physical simulator in this interface pass.

## Author evidence read, not rerun here

The closed stage status records exit0 at01:31:15.984740UTC for exactly one fresh near/yaw−10 whole35s sequence; collection verification reports350requests/351captures/1751physics/71source updates and passing source/CAD/map/video/dispatch reconstruction. Those are author-run results whose sealed artifact bytes I independently hash-checked; their expensive reconstruction/video checks were not rerun by this interface reviewer. No new uncontrolled physics or secondary seed/yaw collection was authorized or performed.

The development report gives267/30/0/0 appearance support at185/190/240/350, both nonempty precision1, three small bins at190, zero currentFast hazards there. I personally checked52 source-only fields exact; its old renderer-label counts/report visualization are read author evidence in this pass. Development used previously inspected yaw−20 data, cannot be fresh heldout. The yaw−10 collection is a fresh whole sequence after design in the same scene; it does not validate unseen geometry/scenes or sole yaw causality.

## Minor findings, preserved-byte clarifications recommended

**M1 — inherited protocol fields conflict with the actual single readonly task.** `online_RGB_memory_protocol_v1.json` still contains the earlier “two fresh whole-sequence learned-memory validation collections”, learning-fit/heldout fields, `shared_prefix_not_fresh_scene:true`, a shortened old `schedule_s`, and old precision/retention criteria. These conflict with the final `actual_task`, one-sequence limit, yaw−10, correct `single_sequence_schedule`, no learning, and the authoritative current plan/runtime identity. No execution defect found: the actual runner uses the final fixed schedule and one sequence. Preserve the prelaunch protocol bytes and append an authoritative scope clarification; do not present inherited learning fields as current claims.

**M2 — author future/truncation boolean needs its test-scope stated.** Author `future_poison_and_truncation_pass` is based on future RGB `Bomb` calls and a37-update prefix ending at frame180 (the seed), so it does not independently demonstrate seed-following nonempty-prefix invariance or archive-wide future-file poisoning. Personal checks here extend RGB+pose guards and prefix through frame195 with exact nonempty outputs, supporting local causal safety; still no archive-wide poison test was performed. Report the actual test range rather than expanding that boolean to unseen archive poisoning. This is a claim-scope clarification, not evidence of future leakage.

Current status should append the closed assessment and first independent-review result after the existing running entries, retaining old text. Whole research remains incomplete. No original source/filter/navigation gate is rescued by this readonly appearance pass.
