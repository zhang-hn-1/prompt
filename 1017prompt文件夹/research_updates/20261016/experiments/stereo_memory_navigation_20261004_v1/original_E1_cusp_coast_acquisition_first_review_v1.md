# Independent first review: E1 cusp coast acquisition v1

Scope: one first source review of the empirical incoming-cusp consumer, complete candidate driver/stage, CPU qualifier and assessment, compared with the actual rejected T driver. No simulation, source edits, calibration matrix, or second review was performed. This report is the only review output written. File hashes below identify the reviewed bytes; there is no Git SHA baseline.

## Findings

- Critical: **0**.
- Important: **0**.
- Minor: **0**.

No concrete unsafe/interface/specification defect was found in the reviewed implementation. There is therefore no defect reproduction to provide. This is a source/interface review pass for the planned one-sequence TaskH experiment, not a physical progress pass or safety admission.

## Source semantics and integration

- `cusp_coast_acquisition_v1.py:31-38` invokes the original projection exactly once, then considers the current installed reference and actual current observation. The only additional tracker mutation is `_path_index`. It does not assign active or pending direction, alter geometry, install a reference, or reset confirmation state.
- The original `OraclePathPolicy.act` projects before selecting the desired direction. The existing `_handle_direction_change` returns the zero request while measured XY speed has fewer than five consecutive samples below 0.05 m/s, resets the counter on rebound, and changes the accepted direction only on the fifth qualifying sample. The existing CuspBoundary override retains an already acquired opposite-direction waypoint while acceptance is pending. Early index acquisition therefore requests ordinary braking; it does not force actual reversal.
- The direction labels belong to outgoing edges (`outgoing_lookahead.py`). Scanning to the first opposite-direction waypoint and measuring the incoming polyline through that waypoint is consistent with this convention. The helper applies the frozen 0.37207621999361346 m arc window, 0.2 m cross-track limit, and current speed/joint/bank marginal domain.
- Policy reset wraps the newly constructed tracker once for that reset. `CuspCoastAcquisition.act` delegates once and clears `_coast_decision` before delegation; no-route/settling paths leave this call's diagnostic `None` instead of reusing a prior acquisition. Tracker `act` releases its temporary observation in `finally`. Reference/action boundaries do not add resets.
- Arming is inherited from T's measured reverse motion followed by accepted forward event. The consumer sees that latch on subsequent calls. The candidate composes command cap → cusp consumer → T tube retention → supported-four-point consumer → PriorRGBPolicy. The driver shadow composes the exact same T baseline without the cusp consumer, including T's tube wrapper.
- The driver diff preserves source capture, source registration, raw memory, map/prior construction, planning, PID/gains, request mapping, goal, initial conditions, and runtime limits. Source packet and current packet are separately copied into the read-only shadow. The shadow stops at the first request divergence and creates no alternate future physical truth.

## Evidence and launch boundary

The already saved CPU assessment records arm frame 229 and the first requested fork at frame 256: unchanged installed reference, candidate zero request, accepted direction still forward and pending reverse, exact historical T requests until the fork, and current start clearance 1.104536101718726 m ≥ the existing 0.8707106781186548 m margin. The qualifier stops at that fork; its future poisoning/truncation/file guards do not claim candidate rollout success. Shadow on/off checks explicitly compare requests and reference geometry; they are not a claim that a separate physical future was tested.

Independent lightweight verification: the five pure helper tests passed (`5 passed in 0.29s`) using the existing ball_wm Python environment with bytecode and pytest cache writes disabled. The initial system Python attempts were unavailable/no pytest; no environment installation occurred. Completed CPU matrices and historical qualification were inspected, not rerun.

The stage preserves one fresh directory/status/log, original physical progress gate at action 545 (54.6 s), fail-stop, conditional continuation to 120 s, 1200 s child wall timeout, and no retry. Driver contact stopping uses the existing physical-contact runtime: per-physics-step contact is latched into the capture collision flag, so an intervening contact cannot pass the progress gate. The stage requires the pending frozen protocol and author-review closure plus source/CPU/resource gates before child launch. Those artifacts and the actual new sequence were intentionally not yet produced at this review boundary; this review does not assert that launch preflight or native runtime imports have passed.

## Declined to judge

- Guaranteed stopping radius or complete initial-state coverage: outside this explicitly empirical marginal-domain TaskH diagnostic; the design, helper, CPU assessment and runtime identity expressly disclaim those guarantees.
- Physical post-fork progress, contacts, or reversal success: requires the single new physical sequence; historical physical future is not candidate truth.
- Formal 35 s, estimated-pose, learner, or whole-research admission: separately gated and not the requested experiment.
- Frozen protocol closure/native import preflight: subsequent prelaunch work, deliberately absent at the first-review boundary; do not treat this source pass as that work's completion.

## Assessment

**First review: PASS, with zero unresolved Critical/Important findings.** The implementation matches the one-consumer experiment and preserves handler ownership and T composition. Proceed through the remaining frozen/native/source/resource gates before the authorized one fresh run. Old T local54.6 REJECT and forward-only necessary-probe REJECT remain unchanged.

## Reviewed SHA-256 identities

Paths in this table are relative to `experiments/stereo_memory_navigation_20261004_v1`, except where noted.

| File | SHA-256 |
|---|---|
| `docs/superpowers/plans/2026-10-08-E1-cusp-coast-acquisition.md` (workspace relative) | `e04d2498e11aa22b2b5a17e75bb9e6f367c1bf0e668137e7225d9f06b358c8e1` |
| `E1_cusp_coast_acquisition_design_v1.json` | `1ee56dd525c9911564643960cd639dbd34aba70b25b3c9766391fbb7e39a7f66` |
| `cusp_coast_acquisition_v1.py` | `da7a3c3b979fdebb3b0c4e5ec4d0a5aa894f5dfc5db513b26fa6c09254dd424f` |
| `test_cusp_coast_acquisition_v1.py` | `4d5421cba14774392da9118109b68508012ede1e6796d1d6187a9bf56006b5f4` |
| `qualify_E1_cusp_coast_acquisition_shadow_v1.py` | `9b28dae3929c4117ee34fcd9673291f222f1ca060ce640174a5ebd86fe9680bf` |
| `E1_cusp_coast_acquisition_shadow_CPU_v1/assessment.json` | `abe95269c861e72874c72721f353b8d679bc00e47fa2a248df986fd1f85882de` |
| `run_original_E1_cusp_coast_acquisition_navigation_v1.py` | `0ad48fd69e828880739654e288612a93f1a0ec63ebf267b9a525ff3efe1d61c6` |
| `run_original_E1_cusp_coast_acquisition_navigation_stage_v1.py` | `faa2e2479da6d8b55b933bca8a1cf64c23e8f54ac69b586b4635bbb4b8a8ed1c` |
| `run_original_E1_tube_viability_navigation_v1.py` | `29c8b9956dec6c468e197095510fed3506bcad09c6e916824cc2266739b92f31` |
| `guarded_reference.py` | `30c8b79fef8bb097f2306f6579f51fe932288a6a320bc41604d725f56cec41bf` |
| `outgoing_lookahead.py` | `f0abeb4cd5d335be472fe43fefb4ee9e96bb21b9b3d8b1fca5495881e3e469c7` |
| `retained_reference_tube_viability_v1.py` | `7042246780df5545982a49923cf5e4c15dc0fa6f826b39ea644f46282082f6b1` |
| `/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/benchmark/policies.py` | `02961ecac3a2901f8236e87e8319515c84a2ebba3f8c98f1561d095d9ec8e84f` |
