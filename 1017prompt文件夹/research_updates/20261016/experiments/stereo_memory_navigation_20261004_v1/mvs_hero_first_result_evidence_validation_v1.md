# First independent validation of six new Hero result datasets

2026-10-07. Result evidence assessment: **C0 / I0 / M0** within the bounded scope below. Independently reproduced **REJECT_SINGLE_HERO_SOURCE**. This is the first validation of new result data, not a second source code review or a model/weights/GPU rerun. No author file was modified. Evidence and the reproducible CPU validator are in `reviewer_mvs_hero_results_v1/`.

All six nonempty rows fail the frozen precision >=0.95 gate. At least one packet >=100 points and strict bilateral post-23.5-second novel-region coverage pass. Missing/nonfinite truth support remains in the precision denominator. A point is correct only when the current and both past rounded-pixel renderer world endpoints are finite and all three distances to the predicted world point are <=0.10 m. No confidence/sky gate, scale alignment, threshold rescue or refit was applied.

| Frame | Eye | Points | Correct | Insufficient | Finite wrong | Precision | Correct novel points / XY bins | Strict coverage |
|---|---|---:|---:|---:|---:|---:|---:|---|
| 235 | left | 19,146 | 17,378 | 1,768 | 0 | 0.907656951844 | 17,378 / 9 | false |
| 235 | right | 22,475 | 9,417 | 8,211 | 4,847 | 0.418998887653 | 6,681 / 11 | false |
| 245 | left | 23,206 | 10,339 | 1,164 | 11,703 | 0.445531328105 | 10,339 / 5 | true |
| 245 | right | 24,077 | 7,750 | 6,706 | 9,621 | 0.321883955642 | 5,384 / 7 | true |
| 255 | left | 37,384 | 22,323 | 0 | 15,061 | 0.597127113204 | 22,323 / 8 | true |
| 255 | right | 35,620 | 13,233 | 7,222 | 15,165 | 0.371504772600 | 6,595 / 6 | true |

Totals: **161,908 points; 80,440 correct; 25,071 insufficient; 56,397 finite wrong**. Every row count, full precision value, novel count/bin count and gate agrees exactly with the sealed author's assessment. Frame 235 time is 23.499999474734068, so it correctly fails the strict >23.5 coverage condition; 245 and 255 are 24.499999452382326 and 25.499999430030584.

## Personally executed after all outputs existed

- Rehashed all 15 input NPZ files, 8 sealed outputs (six NPZ, records, runtime identity), 7 sealed code files and 2 plans. Rehashed the custody seal, source/artifact/DINO manifests and both camera clock authorities. Verified all 205 upstream source and 203 DINO source file hashes and the 2,420,601,860-byte checkpoint SHA256 without deserialization. All match.
- Independently recomputed world points and both past UV arrays from native USD poses and selected raw axial depth using optical +Z and fx=fy=584.8888888888889, cx=319.5, cy=239.5. Maximum world difference is 4.440892098500626e-16 m; maximum past projection difference is 3.410605131648481e-13 pixels. Verified unique query IDs, stride 2, exact raw-depth selection, native 480x640 output, range [0.5,3], root X=0.2/Y=+/-0.3/Z=0.05 calibration, hazard bounds and hypothesis/nonadmission flags.
- Checked every retained query's 49 current integer CAD taps and all 49 same-depth projected taps in each past image, in chunks of 2,048 queries. Positive past camera Z, strict FOV and OpenCV INTER_LINEAR support ==1 are required. **Zero support failures among all 161,908 queries**, encompassing 23,800,476 current/two-past taps. This validates the retained queries; it does not certify internal model attention or treat context-derived predictions as observations/free space.
- Checked actual native uint8 480x640 RGB schema, image/pose/CAD exposure stamps and CAD frame IDs for all six current/two-past requests, against the independent camera authority. Both same-eye pasts are approximately 1 and 2 seconds earlier; no future capture is in the sealed input inventory.
- Ran 11 CPU capture-clock, causal bridge and strict-bilinear support tests: **11 passed in 0.41 s**. Tests use payload traps to demonstrate rejection before RGB/pose/mask reads for invalid current/future/mismatched clocks and CAD metadata. The bridge also verifies native optical conversion and network half-corner K (320,240). No GPU/model import or checkpoint pickle was involved.
- Recomputed all three endpoint truth scores only after outputs had already been sealed; independently applied all frozen precision, minimum-size and bilateral novel-region gates. Used seed frame 180 height-filtered XY 0.1-m bins for novelty.
- Compared the two independently rehashed runtime identity JSONs: both report 557 exact state tensors and the identical final tensor-stream SHA256 `03008367e4f6955d43697d4296c5cd2282ed10e0deb1af4b76124ea264668f83`. This is a personally checked equality of the old/new reported digests, **not a new execution of the 557-tensor comparison**. The latter remains author execution evidence, consistent with the task's prohibition on model import/checkpoint deserialization.

Recorded chronology is whole-source input seal **04:01:17.262552 UTC**, runtime identity written before the first model call **04:01:25.311151 UTC**, all-six output seal **04:01:30.888917 UTC**, assessment **04:01:31.882410 UTC**. File mtimes independently agree. The inspected bounded predictor and sequential recovery driver place the input seal before inference and assessment after all six sealed outputs. These are local provenance evidence, not externally notarized timestamps.

## Inspected evidence and scope limits

Read both frozen development/recovery plans and the original source-only review (C0/I2), author guard/closure material and current scripts. The original review's two issues concern the earlier pre-inference capture guard and custody ordering; the current CPU tests and new custody chronology verify their corrected behavior in the result path. This report does not repeat a code-review loop.

The preserved original request failed its native output assertion and has no point packets/assessment. The documented one-time recovery applies official decoder `set_prediction_scale(1.0)` to checkpoint configuration 0.875 without changing learned state. The old failure remains preserved. Author source/output-contract RED/GREEN and full-suite **501 passed in 24.94 s** logs were inspected, not rerun. Author six inference latency/peak-memory fields were inspected, not independently benchmarked. One original failed GPU request plus six recovery requests is the recorded history; native one-refinement implementation entails two internal forward passes per request. No actual first-failure tensor shape is invented from the CPU fixture.

Video, fresh whole-sequence prediction, physics/controller execution, model internals and actual learned-tensor equality were not personally rerun. No production/global/free-space/control admission follows from these development outputs. Known poses are privileged, the development labels had previously been inspected, and CAD support filtering is not a certificate that model context excludes CAD. The rejected Hero hypothesis remains rejected; prior rejected sources and the formal navigation scope are unchanged. No further network, threshold, parameter or administrative retry is proposed.

## Reproduction and evidence custody

Run `reviewer_mvs_hero_results_v1/validate.py` with `/home/an/miniconda3/envs/ball_wm/bin/python`, `PYTHONDONTWRITEBYTECODE=1` and `OPENBLAS_NUM_THREADS=1`. This writes only the reviewer evidence JSON/log namespace and reads existing author inputs/outputs; it does not import the model. CPU tests are recorded separately in `cpu_guards.log`. Machine-readable per-row counts, SHA checks, chronology and geometric residuals are in `evidence.json`; `validation.log` contains the independently recomputed rows. All checks pass and the evidence issue list is empty.
