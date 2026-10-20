# Per-eye RGB obstacle range candidate V1

This is one bounded offline candidate for the fixed, vertical stereo rig. It does not change a navigation action. Input at inference is one current RGB image, pixel coordinates, and eye identity. Rendered depth, camera pose, map, future images, episode ID, and goal are never inference inputs.

Fit: complete E0 direct and E2 direct sequences from the integrated-lighting capture (70 and 71 frames). Test: complete E1 direct collision sequence (30 frames), then historical hard_0 E1 0003 as a regression. The E1/hard_0 sequences have been used for earlier design and are **not fresh held-out**. Do not random-split neighboring frames. Use one small model, one training run, 10 epochs, fixed seed 20261002, and no hyperparameter sweep. Renderer depth and existing three-class geometry labels are training/evaluation targets only.

Frozen success gate for E1 before running candidate: for true front obstacle pixels at camera depth <=2.0 m, the joint `P(obstacle)>=0.5 && predicted_depth<=2.5m` recall must be >=0.85 across the complete E1 sequence and >=0.80 in each populated 0–2, 2–4, and 4–6 s bin. On known ground pixels, the same near-obstacle output rate must be <=0.05. On true obstacle pixels, median absolute range error <=0.50 m and p90 <=1.50 m. A populated time bin has >=100 true near-obstacle pixels. Unknown pixels remain unknown and cannot certify clearance. Any failed criterion rejects control use; no alternative model or threshold tuning in this version.

If this retrospective gate passes, collect a newly sealed complete-scene test before any action-changing A/B. If it fails, report the failure with denominators and leave the controller unchanged.
