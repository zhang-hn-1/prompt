Verdict: **0 Critical, 0 Important, 2 Minor findings.** The saved evidence supports rejecting both candidates within the declared historical diagnostic scope. It does **not** establish a reliable depth source, successful VO, model admission, or navigation success. Whole research remains incomplete.

I performed this single review read-only. No source, report, prediction, protocol, or experiment artifact was modified.

Findings:

1. **Minor — the first plot’s vertical-axis label implies normalization, but the bars show counts.** In [spatial_vo_measurements.png](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/spatial_vo_measurements.png), “correct / entire source-query denominator” accompanies values 19/18, 12/10, 8/8 and 4/4. These are correct-query counts, not fractions of the complete denominators. The report table and numerical rejection remain correct. A separate clarification should identify the bars as counts and give the four denominators, or a future separately versioned figure could plot actual percentages. Preserve the reviewed original.

2. **Minor — latency is elapsed wall time, not CPU consumption.** The report’s [“CPU s” column](/home/an/Documents/ChatGPT/球型机器人导航/reports/benchmark_spatial_stereo_and_pose_observability_20261006.md:23) and figure’s “CPU seconds” label describe differences of `time.perf_counter()` in [run_spatial_cost.py](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/run_spatial_cost.py:24). These are elapsed seconds while running the CPU implementation. The measured values are accurate as wall-clock latency; they do not measure process CPU time. This affects performance labeling, not either scientific rejection.

Review-focus conclusions:

- **Observation versus prior:** Eight-path aggregation changes selection without changing the unary volume. Confidence still checks the original unary minimum, selected cost, complete competitive interval, ray-width conversion and interior label. Invalid labels remain invalid; wholly invalid predecessors restart paths. Raw hypotheses and qualified outputs remain separate. No free-space map is produced. “Evidence-qualified” must retain the report’s operational meaning: it is not ground-truth-certified observation; the strict provenance audit contains only one qualifying source query.
- **Clone equality:** I mechanically compared the entire original unary-building prefix against `spatial_cost_unary.py`, allowing only the function-name change and return boundary: exact match. I also decoded each saved unary volume and personally verified exact equality, including NaNs, of all six old baseline arrays on all four frames.
- **Actual causal sources:** Saved sources contain only the current opposite eye and strictly earlier captures, with the expected bit assignments. Future-payload rejection is exercised by the passing fixture. The runner and predictor have no truth-label loading path.
- **Serialization before evaluation:** Every saved prediction identity matches its manifest. Protocol sealing precedes the recorded prediction starts; all prediction end times precede evaluation start. The separate runner/audit sources enforce the documented ordering. These are consistent retained execution records and source boundaries, not an independent observation of the original execution.
- **Calibration versus trajectory:** All 181 sanitized feature packets contain exactly the permitted feature/frame/time fields and equal the corresponding producer arrays. Their original RGB provenance identities also match. Reconstructing the fixed optical rig transform using the source’s native float32 operations exactly reproduces the serialized calibration. Global pose cancels; subsequent trajectory poses are absent from the predictor payload and matching logic.
- **Pose convention and failures:** The synthetic geometry fixtures verify current-from-previous estimation and inverse accumulation. Once rejected, `PoseChain` remains unknown even if a later transform is supplied. Actual frames 1–180 all have unknown accumulated poses. All 180 pairs report insufficient stereo support; the maximum saved qualified-landmark count is five, below eight.
- **Complete denominators:** The spatial source-pixel arrays match the archived query arrays. I recalculated each frame’s denominator, qualified count, correct count and finite p90, and pooled p90. Results are 40/73,383 correct, 769 qualified, pooled p90 1.156351558 m. VO retains all 180 attempts, 121 pairs with current time at least 6 seconds, and all 181 accumulated-pose slots. There are zero finite moving-pair estimates and one valid initial gauge.
- **No accidental success claim:** The initial gauge’s approximately zero absolute error is explicitly identified as trivial. Although individual conditional absolute-error booleans are true, coverage and relative gates fail and combined acceptance is false. Both report and audit preserve rejection.
- **Response closure:** The ten cited response-audit source identities match. I checked the primitive definitions, BankMap current-speed query/encoding path and nominal circular-arc arithmetic; the stated endpoint gap is 0.003938582699 m. This supports only the bounded descriptive comparison, not full-table substitution or a controller-gain causal claim.

Personally verified:

- **22/22 fixtures passed in 0.40 s**, with bytecode and pytest cache disabled.
- Review-package identities: 12/12.
- Spatial protocol identities: 10 sources, 10 prediction inputs, 12 evaluation inputs and three document archives.
- VO protocol identities: seven sources, 182 prediction inputs, 182 producer references and 181 evaluation inputs.
- All 185 prediction files and four unary-volume identities.
- Four complete six-array baseline comparisons.
- All 181 sanitized feature payloads, their producer-array equality and original RGB provenance identities.
- Source recurrence, confidence boundary, causal selection, descriptor/triangulation/PnP interfaces, failure accumulation, calibration reconstruction, saved chronology, evaluation arithmetic and plot.
- Both specs, both plans, ledger, report, author-verification record and named implementations/audits.

Author evidence inspected but **not personally rerun**:

- The original four spatial predictions and 180 VO attempts.
- Their reported inference latencies.
- Full-directory **268 passed in 9.51 s**.
- Earlier RED/GREEN sequences and retained environment/scaffold failures.
- Prior physical runs and videos; their outcomes remain historical context.

My exploratory `python` invocation was unavailable, and system `python3` lacked NumPy. Neither executed science. I then used the supplied environment for the permitted small fixtures and array checks. One initial mechanical-prefix search used the wrong token; the corrected comparison completed and passed.

Explicitly declined scope and consequence:

- No target spatial inference rerun: original computation is assessed through retained outputs, source and identities.
- No target VO sequence rerun: the same limitation applies to execution reproducibility.
- No full regression rerun: 268-test status remains author evidence.
- No RGB feature re-extraction: extraction logic, provenance and cached-array equality were checked instead.
- No physical simulation, acquisition or video replay: no new physical or navigation conclusion.
- No parameter, depth-grid, texture, descriptor, source-set or PnP sweep: rejection cannot be rescued by tuning.
- No threshold reduction, pose reset, identity fallback or truth-based recovery: unknown states remain unknown.
- No fresh-sequence/generalization assessment: all results remain historical development diagnostics.
- No independent calibration measurement or rig-stability study: supplied fixed known calibration remains an explicit assumption.
- No new learned model, fit or training: existing rejected models remain rejected.
- No planner search, full BankMap replacement, PID change or control installation: nominal response arithmetic cannot admit these changes.
- No free-space, ground-sweep, collision-coverage or memory admission: source-query evaluation alone cannot establish them.
- No replay of the full original source-provenance labels: I checked the evaluation code, archived query identity and saved measurement arithmetic; this is narrower than regenerating all labels.
- No production robustness campaign outside the 22 fixtures: this is a bounded experimental implementation review.
- No Git integration claim, commit, cleanup, upload or deployment: source SHA identities define the reviewed boundary.
- No follow-up reviewer or second review: this is the single combined review requested.

The two Minor issues warrant separate presentation clarifications. They do not change the supported disposition: **spatial candidate rejected, stereo VO candidate rejected, previous rejection gates preserved, no navigation/model success, whole task unfinished.**
