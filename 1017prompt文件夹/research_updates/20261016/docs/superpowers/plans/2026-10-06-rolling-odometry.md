# Inline rolling odometry continuation (unfinished)

Spec: docs/superpowers/specs/2026-10-06-rolling-odometry-design.md

Task1: personally verify current assetjoint1/base_link/link1/radius and live DOF_names; no archived name inference. TDD independent SO3/shell kinematic integrator. Expected callable RED→GREEN.
Task2: sanitize both901/1601sample past sequences to time/bodygyro/mainencoder only, t0gravity calibration; seal source/input/spec before recursive prediction. Save all trajectories before labels. Expected no automatic pass, no fits.
Task3: evaluate original full >=1s denominator, RMS/peak/attitude/GT-slip caveats and exact source/calibration identity. Pass only both; failure frozen. If pass, prepare one fresh read-only B0+sensor-shadow source identity/noninterference and actual run; no formal matrix or controller change.

Pre-flight: synthetic ideal body sensor creation may read CURRENT GT to emulate sensor, estimator cannot; offline GT label branch separate. Static radius/axis/initial normal are calibration, not dynamic truth correction. No wrapper speed/radius fit. Existing source-sparse RGB gate unchanged. Next fresh shadow is additional actual step, not old replay relabeled online.
