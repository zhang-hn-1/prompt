# Near-field ownership audit (historical, GT-only)

Authored E1 contains 71 static cylinders: a divider of overlapping cylinders, not an isolated short obstacle. The old local endpoint change did not shorten the physical divider. No online controller may consume this geometry.

Every failed later-motion appearance pixel was included. Native renderer depth and analytic nearest finite-cylinder camera rays were compared with frozen 0.02 m tolerance. Of 260,726 pixels whose reconstructed points are within 0.55 m of root, 169,802 are consistent with a static cylinder and 90,924 remain unresolved. Of 1,170,894 physical hazard pixels outside the old axial-depth domain, 811,935 are consistent with static cylinders and 358,959 unresolved. Unresolved does not mean robot. Close-to-root is not a body ownership label.

This is evidence against simply deleting nearby responses as self-occlusion. It does not repair colour-depth inference or change REJECT_LATER_MOTION_APPEARANCE, REJECT_APPEARANCE_CONE_GEOMETRY, source rejection or formal 35 s gates. No new physical or neural calls. Author personally ran the audit to exit 0 and inspected aggregate and per-eye output; no ownership sensor labels were collected.

Actual evidence: `experiments/stereo_memory_navigation_20261004_v1/near_field_static_ownership_v1/assessment_v1.json`, analytic authored geometry and `near_field_static_ownership_v1.log`. Complete script and input SHA are recorded in assessment.
