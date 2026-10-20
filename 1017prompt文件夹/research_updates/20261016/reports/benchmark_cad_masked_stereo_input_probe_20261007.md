# Single CAD-masked RGB input ablation and analytic depth check

Candidate **REJECTED**. Same frozen Fast checkpoint/eager8iterations/maxdisp416/LR1.5 and paired CAD post-filter; only copied robot-invalid RGB pixels zeroed before matching. Exactly seven historical/development exposures, two inferences per same RGB pair. All predictions sealed before truth; cached old source reconstruction and CAD-only-output filtering equivalence pass on all seven. No camera/light/PID/planner/action/threshold changes, no fresh source-validation or navigation triggered.

|case/frame|B0 correct/all|zeroCAD correct/all|B0 divider correct|zeroCAD divider correct|gate|
|---|---:|---:|---:|---:|---|
|retreat_observation_run/0|0/81|0/1|0|0|None|
|retreat_observation_run/60|0/0|0/309|0|0|None|
|retreat_observation_run/120|0/72|0/0|0|0|False|
|retreat_observation_run/180|0/0|0/0|0|0|False|
|fresh_rolling_prior_shadow_current_run/0|1599/1686|1940/2179|1599|1940|False|
|fresh_rolling_prior_shadow_current_run/60|2914/2920|1611/1611|2914|1611|False|
|fresh_rolling_prior_shadow_current_run/110|3910/4013|3517/3570|3910|3517|False|

Near12/18s fail frozen .80 unconditional correctness +25accurate-divider-query criteria: both zero output. Far0 loses precision .9484→.8903; far6/11 lose correct count2914→1611/3910→3517. Empty output is not perfect accuracy. No zero-fill/crop/fill variant scan. All old all-domain depth/source rejection remains.

Independent analytic sanity check, one closed18s SGBM group412rays: all ideal USD-cylinder/cap intersections finite; renderer-versus-ideal world error median .0116795/p90 .0154543m, versus SGBM-versus-ideal .1915174/.2033747m, zero≤.10m. This rules out a .18m label reinterpretation rescue for these rays. Mesh tessellation versus ideal analytic cylinders limits exactness; this is not all-domain renderer certification or source .10m admission. Same-proxy photometric correspondence alone remains insufficient.

Author-only evidence at creation: six mask-helper tests literal RED→GREEN .15s; original actual GPU inference exit0, exact frozen local gates and measured assessment; code/protocol/all predictions before GT/analytic inputs retained. Prior retreat independent review explicitly excluded this branch. No new navigation, formal E0/E1/E2 or research-completion claim.

Next branch independently pre-frozen small learned reliability head on original source (not this rejected preprocessing); predicted memory remains static imputed, never observed-free. Its result is a separate layer.


## Append-only independent review and provenance clarification
ONE firstpass independentreview C0/I0/M0, reused nonblank context; sixfixturesPASS.16s, allsealed sources/inputs/predictions/analyticclosure and21freshB0-vs-cache arrays/14CAD masks/70cached+70filter+140pairedpacketfields exact; sevenfrozen scores/gates and412ray independent11-cylinder geometry reproduced. GPU/REDcontrolflow inherited, no new inference/sim/model or learningreview. No source/navigationadmission.
