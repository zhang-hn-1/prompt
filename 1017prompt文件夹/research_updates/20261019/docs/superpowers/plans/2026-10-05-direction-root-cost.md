# Current Direction Root Cost Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans inline; user autonomous authorization supersedes repeat approval pauses. Use test-driven-development and one fresh-context final read-only review.

**Goal:** isolate one current-direction root-cost intervention on saved frame175 without altering controller behavior.
**Architecture:** copied current snapshot→original route/crop request→old Hybrid and independent root-cost Hybrid→offline nominal evidence. No source checkout edits or policy.act calls.
**Tech Stack:** existing Python/NumPy/SciPy and frozen DriveModel; no dependencies.
**Spec:** docs/superpowers/specs/2026-10-05-direction-root-cost-design.md.

## Global Constraints
Original rig/light/PID/speed/mapper/protocol/smoother/five-low/rebound/oldgates/35s unchanged. One frame175; one grid search+two Hybrid searches only, nodebudget3000 each; original.5m switchcost/1.25reverse multiplier/.3primitive/.05sampling/.2tolerance. Both roots allowed; no forced forward or penalty scan. Known-pose+renderer-depth current map remains privileged/unknownrisk. No actual new control/sim/inference/fit/fresh claim. Archive all old evidence and failures; ACTIVE/not wholecomplete.

## Review Focus
1. Current accepted direction is not a future desired/request/applied value; report each independently and never use later feedback.
2. Both roots must remain admissible; charging root costs must affect frontier priority as well as g, without double-counting the chosen first direction.
3. Old/new dataclass identities and default configuration must be explicitly equivalent; only allowed source diff changes roots and validates initial_direction.
4. Nominal path/objective must not be called actual motion or dynamic safety; internal changes and initial switch are separate.
5. Snapshot copying and strict fields must prevent future/evaluation data entering the one fixed problem; reject malformed inputs before searching.

## Task 1 — isolated implementation / TDD
Create H/direction_root_planning.py (original exact source copy plus declared minimal change), H/direction_root_cost.py and H/test_direction_root_cost.py.
Consumes current ProvenanceGrid, frozen planner/DriveModel. Produces `snapshot_problem(map_arrays,current)` (copied strict snapshot), `prepare_request(snapshot)` (one original grid plan), `plan_kinodynamic_path(request,drive_model,*,initial_direction=None)` and `objective_summary(plan,request,initial_direction)`.
- [ ] Write tests: literal3 reverse primitives cost1.125+.5=1.625 relativeforward; same reverse1.125; forward→reverse .3+.375+.5=1.175; legacy None exact original plans; initialdirection zero-cost parity; reverse behind remains allowed; invalid enum before search; strict/copied current snapshot and futurepayload trap.
- [ ] Observe callable missing-feature RED; implement minimum, run focused then whole H pytest/cache disabled. Expected all pass; archive first failures.
- [ ] Verify exact source diff, separate private copy from frozen original; ledger interface/preflight and completion evidence. No unrelatedGitcommit/unborn repo.

## Task 2 — unique archived experiment / verification
Create H/seal_direction_root_cost.py, H/run_direction_root_cost_audit.py, H/verify_direction_root_cost.py.
Consumes Task1 and guarded_reference_run frame175 current map/input + pre-boundary accepted direction. Produces protocol, sanitized snapshot, two complete reference files and assessment.
- [ ] Seal source/config/input/document identities, field restrictions, frame175 and gates BEFOREsearch. Prior source86/2834 input/2790 stageartifacts/5doc identities must match; archive exact ledger prefix before new notes.
- [ ] Run once: exactly one grid and two Hybrid calls; original baseline reproduces savedtarget/route/completechosen reference/1821nodes. Expected explicit eligibility pass or reject; no other request search/control replay.
- [ ] Verify all saved nominal primitives on same inflated raster/terminal tolerance, hand objective/root provenance/defaultdiff/counts/SHA without searches. Save readable plot and report. Expected all interface checks correct, eligibility independent of navigation.

## Task 3 — one review / closure
- [ ] ONE fresh most-capable read-only review with above Review Focus verbatim and source/result package; no search/sim/replay. Each declined scope author ruling, one Critical/Important RED→GREEN repair if necessary, no second review/experiment retune.
- [ ] Preserve reviewed report body, append author scope; final identity/doc sync/ledger/mission/mainplan closure. Only this boundedstage complete, wholemissionFalse/ACTIVE. Nextstage determined by outcome, not another experiment here.
