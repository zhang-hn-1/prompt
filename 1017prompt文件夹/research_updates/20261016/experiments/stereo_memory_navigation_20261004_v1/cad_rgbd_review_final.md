# Final read-only CAD-filtered RGB-D review

C / I / M: **0 / 0 / 0**. No actionable blocker found within this single historical candidate. The result supports **REJECT / no online admission**, not successful odometry. Reviewed the supplied final artifact scope and relevant imported implementations; these files are untracked, so this is not a claimed tracked Git base-to-head diff review.

## Personally verified

- Fresh bounded tests: four new RGB-D fixtures plus seven old stereo/PnP fixtures, **11 passed in 0.32 s**, exit 0. Cache/bytecode writes disabled.
- All **106/106** recorded source identities match; prediction completion hash matches; recorded protocol < completion < assessment timestamps are ordered.
- Exactly **122 raw RGB captures**, exactly **25 source exposures 0,5,…120**, exactly **25 five-field prediction inputs**. Raw RGB, source depth, corresponding CAD masks and exported payload timestamps/frame identities agree. Exported depth equals its same-exposure Fast source.
- Recomputed old ORB xy/descriptors for every exposure and all 25 native validity masks using confidence, finite range .5–3 m, disparity >1, bilateral CAD/native partner support, fixed-rig self-distance .55 m and .1 m precision. Exact agreement with saved inputs. Recomputed all 25 support-layer rows with exact agreement.
- Read predictor/runner/auditor: previous depth only, subpixel previous ray, previous/current descriptors, strict increasing timestamps; unchanged old temporal/PnP parameters. World/GT pose inputs absent from exported predictor payloads; labels enter only the separate auditor. PoseChain inverse-composes and permanently remains unknown after failure; tested old composition/failure fixture freshly.
- First 0→.5 s pair has **4** metric landmarks. All following accumulated poses are null. Complete coverage **1/25 = 4%**. **24 pairs = 6 accepted + 17 insufficient landmarks + 1 insufficient inliers**. Accepted increment endpoints are approximately 1, 2, 3, 4, 4.5 and 5 s.
- Independently recomputed all six saved finite pair errors from offline optical poses. The motion-period complete denominator is **13 pairs**, with **0 finite / 0 correct**, and null finite p90. No finite-survivor denominator or late gauge reset changes the rejection.
- Report, ledger and logs agree with these counts and retain the historical/development, no-online and old-rejections-preserved boundaries. Cause discussion correctly distinguishes source-feature/depth support from joint temporal support; it does not claim universal visual infeasibility.

## Inherited evidence and limits

I inspected the original RED/GREEN/run/assessment/support logs; I did not recreate the historical RED state, rerun the earlier 379-test suite, execute Fast/GPU inference, regenerate CAD masks from the physical scene, or repeat acquisition/controls. Hashes and matching timestamps establish current recorded consistency, not independent forensic proof of original execution order. Original Fast/stereo-VO/IMU/cap rejection evidence remains inherited; nothing in this candidate overrides it.

Declined to judge fresh-sequence generalization, useful accumulated-trajectory accuracy, online runtime/causality/navigation performance, other methods/gates, or the next cap .3 current-CAD full-goal baseline. The rejected cap 1.0 full-goal route receives no admission here. Subpixel-ray/nearest-depth sampling remains the explicit unvalidated approximation already stated in the report.

Only these review artifacts were saved. No implementation, prediction, assessment, report or ledger edits; no GPU, fit, threshold scan, candidate rerun or second review.
