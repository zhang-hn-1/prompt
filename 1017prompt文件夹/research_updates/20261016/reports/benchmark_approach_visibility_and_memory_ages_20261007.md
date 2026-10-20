# Existing approach visibility and memory-age causal audit (2026-10-07)

Author-only bounded diagnosis: exactly14existing exposure frames, two completed original-goal yaw20runs at6/10/14/18/22/26/29.5s. All geometry/source/age inputs sealed before offline analysis. No new inference, simulation, control, threshold, fit or gate. Short-task independent review excludes this audit.

## Visibility and source layers

Current at6/10/14s has23252/25183/19518 true-right-depth-consistent unmasked divider pixels and2914/3095/2031 accurate accepted packet divider queries. At18s it still has10453 physically valid pairs but only1 accurate source query: this distinguishes network/source failure from complete geometric invisibility. At22s854 physically valid pairs/1 accurate query; at26and29.5s0physical pair/0accurate queries.

History at26s has374 physical pairs but no accurate accepted query. After actual reverse/reorientation,29.5s has16184 physical pairs and632 accurate queries: view can recover through real motion. This is actual executed history behavior; no imaginary future labels for shadow plans. The same rejected source model remains used, and proxy divider membership from geometric cylinder endpoints is not renderer object identification or all-domain validation.

## Original ten-second memory age

|case/time|past correct divider hazard cells|not older than10s|primary hazard among these alive cells|older correct cells|all primary hazard|
|---|---:|---:|---:|---:|---:|
|fresh_rolling_prior_shadow_current_run/18.0|44|43|1|1|2|
|fresh_rolling_prior_shadow_current_run/22.0|45|22|1|23|1|
|fresh_rolling_prior_shadow_current_run/26.0|45|16|0|29|0|
|fresh_rolling_prior_shadow_current_run/29.5|45|7|0|38|0|
|occluded_obstacle_memory_navigation_history_run/18.0|46|43|43|3|74|
|occluded_obstacle_memory_navigation_history_run/22.0|55|34|34|21|75|
|occluded_obstacle_memory_navigation_history_run/26.0|56|22|22|34|41|
|occluded_obstacle_memory_navigation_history_run/29.5|57|16|16|41|33|

Correctness is offline predicted endpoint distance≤.10m, true endpoint within frozen cylinder proxy and predicted z(.05,1.5]. Cell ages use original query timestamps/rounding. Hypothetical all-past correct-cell aggregate uses truth only for diagnosis; it is **not an admissible online memory algorithm**. Current clears mapper each frame; at26s16 earlier correct cells are still within the existing10s TTL but none remains in primary map. Thus complete current loss is not explained by TTL alone. History retains22alive correct cells at26s and16at29.5s, yet original episode still fails; simply lengthening TTL is not established as a navigation fix. At some locations old correct-cell age is expired but primary hazard remains because a newer possibly inaccurate query occupies the same cell; do not conflate cell timestamps with physical-instance tracking.

Prior history-side audit showed genuine new surface samples can invalidate an old route. Retaining old evidence forever or forcing the old reference is not justified. Near cold start has no proper divider correspondence to retain at all. Next actual experiment is ONE18s fixed retreat sensing probe (6zero/6negative-main-.3/6zero), not a navigation candidate, using unchanged rig/light/network/PID. It tests whether actual repositioning restores binocular and source support. No additional distance/duration scan after outcome. Controlled unknown-space authorization is explicit; no observed-free assertion.

Evidence: audit_approach_visibility.py/derivation/preanalysis/diagnostic and audit_approach_memory_ages.py/preanalysis/diagnostic; all originaltwo whole-run raw files kept. Failed original35s navigation/source gates remain; estimated-pose obstacle navigation/learningincrement/formal E0E1E2 pending. Whole research NOTcomplete.
