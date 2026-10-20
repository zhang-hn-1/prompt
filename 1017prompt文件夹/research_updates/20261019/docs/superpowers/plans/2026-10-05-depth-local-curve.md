# Privileged Depth Local Curve Implementation Plan

> Use superpowers:executing-plans inline; user chose autonomous execution. Retain artifacts/no unrelated Git commits in authorized H isolation.

Goal: one bounded local provenance-depth planner/controller navigation diagnostic.
Architecture: ProvenanceGrid -> existing partial-observed A* local target -> existing bounded Hybrid A* -> state-preserving reference adapter -> frozen mapped Oracle and roll PID.
Tech: existing NumPy/SciPy/Isaac SDK; no dependency/model installation.
Spec: docs/superpowers/specs/2026-10-05-depth-local-curve-design.md.

Global constraints: maxONE actual golden_e1_0001, original35s/350blocks/goal.2m/physical_contact_v2, six-second zero prefix, fixed rig/light/markers/smoother/PID. No full-map/global-reference input, unknown/free relabel, future truth, retuning, new fit or formal matrix. Declare Oracle request domain differs from old .3 unknown cap; no one-change attribution.

## Task 1: Local curve interface
- [ ] Create test_depth_local_curve.py first. Literal tests reject extra/future/nonfinite packets atomically, copied inputs, empty-map remains unknown with risky local motion, known hazard blocks local search, local curve is bounded/finite and obstacle-cleared, real inner tracker preserves pending/five-low state across replacement, denies full global context, future payload invariance. Expected RED missing module/callables.
- [ ] Create depth_local_curve.py: sanitize_packet(packet); local_curve(grid,xy,yaw,worldgoal,drive_model)->(plan,metadata); DepthLocalCurvePolicy(protocol).reset/set_current_packet/act; install_local_reference preserves inner direction state after first initialization. Reuse classes/functions rather than editing frozen sources. Expected focused GREEN and whole H CPU suite GREEN.

## Task 2: Actual causal integration
- [ ] Create run_depth_local_curve.py using existing complete capture runtime, both depth->world point projection, current-packet before next policy call, local-only context, actual one-call request log. Save every map/reference/request/physical layer. Create seal_depth_local_curve.py and phase driver/status; protocol includes exact source/SHA, input interfaces/gates/budget before data. Keep original old sources unchanged.
- [ ] CPU real packet/reference/curvature/request entry audit; no shadow prediction changed into labels. Expected known obstacle changes planning or blocks output, no oracle leakage/reset.
- [ ] Run once original35s golden E1, no alternate after failure. Expected valid actual artifacts/exit0 or explicit failure evidence, not guaranteed success.

## Task 3: Assessment and closure
- [ ] Recompute physical termination/contact/reference-independent progress, maps/provenance/local curve and clipping/latencies. Whole-policy causal replay/future truncation; evidence namespace separation. Save assessment/data/plot/report benchmark_depth_local_curve_navigation_20261005.md. Expected exact replay and explicit pass/reject scientific result.
- [ ] ONE fresh read-only reviewer per executing-plans; Critical/Important one tested fix pass, Minor disclosed. Author final identity verification and mission/mainplan/ledger sync. Do not append this presealed plan after sealing. Whole research stays ACTIVE until all required layers actually finish.

Review focus: no runtime global Oracle reference/map enters outer policy; local reference updates preserve direction/PID/smoother; every map labels unknown distinctly; nominal Hybrid A* not physical certificate; both camera depths/poses from same complete capture; original35s/no video tail and exact actual contact; cap/package difference explicit, not single-factor A/B or pureRGB; future payload doesn't affect a past decision.
