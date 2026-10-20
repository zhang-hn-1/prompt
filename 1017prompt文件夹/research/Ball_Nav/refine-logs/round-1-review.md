# Round 1 Review — GPT-5.4 (via OpenCode ultrabrain agent)

**Overall Score: 6.1 / 10**
**Verdict: REVISE**

## Scores

| Dimension | Weight | Score |
|---|---:|---:|
| Problem Fidelity | 15% | 8.5 |
| Method Specificity | 25% | 5.0 |
| Contribution Quality | 25% | 5.5 |
| Frontier Leverage | 15% | 7.0 |
| Feasibility | 10% | 5.5 |
| Validation Focus | 5% | 6.5 |
| Venue Readiness | 5% | 6.0 |
| **Overall** | 100% | **6.1** |

## Critical Issues

### CRITICAL 1 — Method Specificity (5.0)
- Stereo pair at single time step gives parallax, NOT temporal ego-rotation Δθ
- MPPI rollout needs FUTURE viewpoint evolution under candidate actions — not defined
- Fix: make viewpoint an explicit latent state with transition model (z_t, r_t, a_t) → (ẑ_{t+1}, r̂_{t+1})

### CRITICAL 2 — Contribution Quality (5.5)
- Δθ-modulated positional bias too weak: cannot explain token re-indexing, occlusion, perspective change
- Fix: replace bias with pose-conditioned ray embedding in latent token space

## Important Issues
- Feasibility (5.5): stereo weak supervision fragile in low-texture scenes; constrain to minimal rotation DoF
- Validation Focus (6.5): Claim 2 "zero-shot quadruped" conflates too many factors; narrow it
- Venue Readiness (6.0): paper needs precise state-update formalization, not just intuition

## Simplification Opportunities
1. Delete separate CNN viewpoint estimator; use DINO tokens + tiny pose head instead
2. Predict minimal necessary viewpoint DoF (yaw/roll for spherical), not always full R³
3. Merge contribution around one central mechanism: explicit viewpoint state in latent rollout

## Modernization Opportunities
1. Use DUSt3R/MASt3R as offline teacher for pseudo-labels, distill to lightweight onboard head
2. Use cosine/contrastive loss on DINO tokens instead of plain L2
3. Replace 2D positional bias with camera-ray-conditioned attention

## Drift Warning
NONE — recommended fixes preserve the original problem anchor.

<details>
<summary>Full raw review</summary>

The proposal targets a real failure mode: turning-induced viewpoint shift is concrete, operationally important, and directly harmful to latent planning.

The TRO risk is not the motivation; it is the mechanism. As written, the proposal is weakest exactly where reviewers will probe hardest: how viewpoint change is actually observed/predicted during rollout, and whether a Δθ-conditioned positional bias is expressive enough to correct token relocation/occlusion in DINO space.

[Full review text preserved in task output bg_e4af36a6]
</details>
