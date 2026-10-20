# One closed-pair sensor prefix audit

The provided-MODEL physical identity pilot is already closed and rejected. No actual rerun, estimator change, sensor replacement in control, tolerance change, or admission is authorized by this audit.

Before the full comparison, inspection of the first two saved packets disclosed: the initial gauge is identical, but absolute SDK interval origin is 0.20 s for historical R1 versus 0.18 s for provided B0; first gyro and acceleration differ. This is a disclosed diagnostic observation, not fresh held-out evidence or proof of a sensor bug.

Compare all native gyro/acceleration/encoder/time packets only up to index 747 inclusive, the last common actual applied/physical prefix. Report first differences, counts and magnitudes, relative/absolute clocks and gravity-correction eligibility. Replay the frozen estimator on each measured prefix; then substitute one channel at a time in this same historical prefix (gyro, acceleration, encoder), plus only gyro packet zero. These are offline attribution probes, never alternative physical trajectories or improved pose scores. Exact reconstruction is the check; failure to isolate a channel is reported without additional combinations or fitting.

Preserve old rejection and seals. Use no future packet past the physical divergence. After closure, return to the fixed-rig perception task rather than repeat sensor, Oracle, network or threshold matrices.
