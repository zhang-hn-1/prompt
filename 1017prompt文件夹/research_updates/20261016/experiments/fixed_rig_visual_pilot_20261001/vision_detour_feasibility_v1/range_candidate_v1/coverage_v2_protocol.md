# Coverage intervention V2 (frozen before fit)

V1 used only E0/E2 and had 34 front near-obstacle training pixels, versus 54,623 on historical E1 test. This V2 changes **only the complete training sequences**, keeping the V1 architecture, loss, RGB-only inference inputs, seed, ten epochs, and fixed `P(obstacle)>=0.5`/`predicted range<=2.5m` gate. It is a controlled data-coverage intervention, not a hyperparameter sweep.

Add E1 direct (30 frames), hard_0 E1 0001 (20 frames), and hard_0 E1 0002 (14 frames) to E0/E2. Evaluate on complete hard_0 E1 0003 (41 frames); it has been used diagnostically and is historical regression, not fresh held-out. Do not score E1 direct as a held-out result once it enters fit.

Accept for further **read-only validation only** if hard_0 E1 0003 has >=0.85 near-obstacle joint recall, <=0.05 false near-obstacle output on known central ground, median obstacle range error <=0.50m, and p90 <=1.50m. Also require >=0.80 recall in each 0–2, 2–4, 4–6s bin with >=100 near-obstacle true pixels. Failure stops this candidate. Passing would require a newly sealed complete episode before any change to navigation actions.
