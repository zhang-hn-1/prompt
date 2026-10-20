# Learned Visible Obstacles Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline. User directly approved the discussed learning experiment and autonomous bounded implementation; no additional design permission gate.

**Goal:** Learn current visible local obstacle surfaces from paired RGB and IMU, with explicit unknown and height bands.
**Architecture:** Fixed small CNN encodes current left/right RGB and IMU gravity, predicts a gravity-aligned local 40×60 raster. Output classes unknown, observed ground, and three obstacle-height bands. This is a current visible-surface target, not hidden-volume occupancy or free-space clearance. GT only labels/scores.
**Tech Stack:** Existing Python, PyTorch2.5.1, NumPy, current Isaac data/CAD; no new packages or pretrained download.
**Spec:** This document and experiment protocol.json implement the user's approved independent image-to-local-map proposal; supersedes unimplemented Census next step in v48.

## Global Constraints

- Current forward60cm rig; original camera/light/data immutable. New learning target, no old confidence model or153-memory rescue.
- Input pair at current exposure, 160×120 per eye, native IMU gravity; no GT position, layout ID, time/frame index, SGBM prediction or previous maps in network.
- Local coordinates forward[0,4), lateral[-3,3), resolution.1m. GT visible surfaces from finite current depth. Ground is a surface label and cannot certify a body path free. No-hit/occlusion/unlabelled remain unknown.
- Height classes world z .05–.5, .5–1,1–1.5m; flat-ground sim label convention disclosed; known calibration and initial gauge/ideal IMU provenance retained.
- Train H/E2+E0 entire sequences; E1 entirely excluded from fitting, normalization, weights, early stopping. All are previously seen research data, so E1 is held from fitting but not fresh independent research evidence. Identical/correlated physical groups never split.
- One seed2026, one fit, fixed30 epochs/batch16/Adam lr.001; final checkpoint only; no sweeps/early stopping/architecture replacement. Random batching only within train; no frame split.
- Pre-fit input-label coverage: all181 capture identities/clocks per sequence;>=90% frames with>=20 visible labelled cells; each train class>=100 aggregated cells; obstacle>=100 cells per group. Otherwise no fit; record exact data gap.
- Candidate probability>=.5 plus argmax obstacle; else unknown or ground. Consumer emits tentative surface risk with provenance, no permanent confirmation or history clearing.
- Gate per E1: visible-obstacle precision>=.90 and recall>=.80 (one-cell spatial tolerance), GT-ground false risk<=.05, unsupported predictions<=.10, cell localizationp90<=.15m, predicted visible-surface recall>=.70; each evaluated on complete sequence with denominators. Also report train and per-height metrics, unknown baseline and train-frequency baseline. Gates concern source only; passing requires fresh physical tests before claims of navigation gain.
- New physics cap4 inclusive collection/baseline/fresh verification, recovery<=1; fitcap1. Reject at inadequate source qualification, no need expend physics. Do not mix with old budgets.

## Review Focus

- All-nohit labels and outside-FOV cells remain unknown; ground not free.
- Target coordinates, signs, grid boundaries and yaw handling.
- Current IMU acquired at image time, privilege disclosure, no GT network inputs.
- Whole-sequence split and identical-input shared-prefix audit.
- Missing labels/classes blocks fitting, classifier overconfidence cannot hide missing coverage.

## Tasks

- [ ] Build supervised visible-surface raster and focused geometry/unknown tests; implement full sequence qualification and immutable input/label identities.
- [ ] Implement network and tentative consumer, train only after qualification; test shape/probability/GT input isolation; run bounded durable fit.
- [ ] Seal checkpoint and E1 predictions before scoring; quantify recall/precision/coverage/height with current baseline and direct RGB/true/predicted maps.
- [ ] Independent audit and keep/reject, then fresh actual closed-loop trials only if admitted. Update mission, plan, next entry, automation.
