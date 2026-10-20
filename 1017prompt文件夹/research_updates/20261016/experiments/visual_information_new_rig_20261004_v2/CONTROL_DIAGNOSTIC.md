# Shared controller frozen before first actual run
2026-10-04. No modification of the successful Oracle. This separate geometric map baseline is diagnostic and not a validated deployment candidate.

Both modes use the same fixed 40×40m 0.10m grid, start-centered coordinates, 0.60m required clearance (robot radius0.40+margin0.20), same A* with no corner cutting, current PointGoal, one-meter geometric target, protocol bounds and frozen roll PID. Plan every0.5s; act at10Hz. Reversal requires five speed<0.05m/s samples, rebound clears count. No blind reverse timed script.

Map mode supplies actual full occupancy once; visible mode supplies only current stereo depth point hits, known calibration/current attitude and causal history. Floor-hit cells free, body-height hits occupied, unknown blocked. Body footprint seeds0.4m free but cannot bridge unknown. Same-grid/state/action equality tested. Test pre-flight does not establish physical tracking adequacy; actual map runs are the positive-control diagnostic of this limitation.

Four starts per mode, up to8 actual runs if both RGB candidates reject. Physical contacts remain physical_contact_v2; budget35s. No claim that rejection/parking proves visual navigation impossible. No control tuning from run outcomes.
