# Nominal support implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline. Steps use checkbox syntax.

Goal: one conservative planner-only curvature support candidate and one actual current-snapshot CPU comparison. Architecture: DriveModel subclass overrides only curvature; standalone sealed runner uses original request/search twice. Tech: existing Python/numpy/pytest. Spec: docs/superpowers/specs/2026-10-05-nominal-support-design.md.

## Global Constraints
Original controls/rig/35s gates frozen; frame175 inputs only; max2searches,0physics,1candidate,0fits; domain screen not a physical model. No tracker model replacement.

## Review Focus
1. Domain intersection uses requested direction and both real calibrated rows, not weighted endpoints; zero/nonfinite/disjoint inputs reject.
2. Only planner curvature changes; original DriveModel and tracker speed/inverse/braking/current execution state remain frozen.
3. Current snapshot175 contains no future feedback; original archived trajectory is not candidate physical label.
4. Exactly two searches under identical original request and root direction; no rescue scans, candidate gate presealed and independent of path_index.
5. Geometry support admission is separated from dynamic/visual/navigation admission; failures preserved and a concrete next step follows evidence.

## Task1 adapter
Consumes original DriveModel and BankMap; produces make_support_drive(base,map)->DriveModel subclass and common_support(map,main). Create nominal_support.py/test_nominal_support.py. [ ] Write literal tests with real calibration for extremes/feasible/speed/original-isolation/input rejection and degenerate row ranges. [ ] Callable RED baseline uncapped. ExpectedbehaviorFAIL. [ ] Implement only cap. [ ] GREEN fixtures + full H suite with SDK LD. ExpectedallPASS. SHA ledger instead of unrelatedGitcommit.

## Task2 bounded comparison
Create frozen current175 packet, seal_nominal_support.py and audit_nominal_support.py. Produces original/candidate serialized paths, exact original match, domain/geometry/objective gate and plots. [ ] Freeze all inputs/source/gates before results. [ ] One original+one candidate search, save timestamps/status/results. Expected completed bounded result; eligibility canFAIL. [ ] Author hashes/counts/chronology/source isolation check, no second science.

## Task3 report/closure
[ ] Save report separates exact source mismatch, algebra support, local geometry, dynamic/navuntested. [ ] One readonly most-capable review, fresh if tooling allows, no searches/sim/rerun. [ ] Individual scope rulings, append docs and fresh SHAchecks. Preserveoldresults/unfinishedwholemission/automationACTIVE.
