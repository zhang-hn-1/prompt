# Round 2 Review

**Overall Score: 8.4 / 10**
**Verdict: NOT READY (blocking issues remain)**

## Scores

| Dimension | Score |
|---|---:|
| Problem Anchor Preservation | 9.5 |
| Technical Gap Clarity | 9.0 |
| Dominant Contribution Sharpness | 8.8 |
| Mechanism Concreteness | 8.1 |
| Focus / Scope | 8.4 |
| Validation Plan | 8.0 |
| Feasibility / TRO-Readiness | 7.8 |
| **Overall** | **8.4** |

## Drift Warning
**NONE — Problem Anchor preserved.**

## Blocking Issues
1. Training-planning mismatch: losses are one-step; planner uses multi-step closed-loop rollout → add multi-step overshooting
2. Coordinate system for r_t undefined: which frame? how initialized? how DUSt3R pseudo-pose aligns with IMU?

## Important (non-blocking)
- Add intermediate ablation baselines (explicit r_t concat without ray; ray without joint rollout)
- Handle DUSt3R teacher failure cases explicitly
- Tone down "FIRST" novelty wording
- Conservative expected gain statements unless pilot data exists

## Modernization Still Available
- Sin-cos parameterization for angle losses
- Multi-step latent overshooting loss
- Pseudo-label confidence weighting from DUSt3R
- Explicit platform normalization for transfer
