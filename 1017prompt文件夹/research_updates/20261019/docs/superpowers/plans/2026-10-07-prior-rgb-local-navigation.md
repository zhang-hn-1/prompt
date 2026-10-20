# Prior RGB Local Navigation Implementation Plan
> REQUIRED superpowers:executing-plans inline; ONE fresh final readonly branch review.
Goal: two actual35s RGB-knownpose localgoal runs, unique current→history memory intervention.
Architecture: frozenFast RGB every.5s→currentknownpose worldendpoints→PointSupportMap/provenancegrid→existing local_curve/Endpoint/rollPID. Explicit task assumptions, unknown risk ±.3 cap, all old gates unchanged.
TechStack: existing Isaac/Torch/NumPy/SciPy; no newdependency/model/fit.
Spec: docs/superpowers/specs/2026-10-07-prior-rgb-local-navigation-design.md
Constraints: originalrig/light/controller/smoother/.3auxvelocity/noGTonline; task2m pointgoal/original35s; exactly2 whole physical runs, <=71inferperrun; fixedB0, current/history same otherparameters; priors notobserved.
Review focus: cachedtimestamp/provenance; boolconfidenceNaN; runtime GTnamespace isolation; unknown risk vs safety; no source/fresh/formal/real-time overclaim.
Task1 create prior_rgb_navigation.py/test_prior_rgb_navigation.py, literalRED/GREEN for finiteboolguard/2mgoal/current-history persistence/future source rejection/cached no refresh/prior separate/sharedcap. Reuse readonlyPointSupportMap, local_curve, EndpointLookaheadPolicy without edits.
Task2 create RGB collector from prior synchronizedruntime, replacing depth control packet by frozenRGBsource only; sampleevery5frame, storeGT onlyaftercurrentcontrolpacket. Preserve currentroot/permittedpose, sourceframe/time, raw commands/maps/sourceparams. Freeze2arm protocol and all imports/model/robot/scene assets hash; CPUfixture fullH suite and syntax beforeGPU.
Task3 run current thenhistory once each; record status perarm and completepostclose artifact SHA. Evaluategoal/contact/physics/source IDs/actual actiondifference/progress; actualtwo videos/plots. Candidate equivalent orfailed→reject clearly, no scan/refit. ONEreview, authorfinalclosure/missionupdate; continue next admissible work ifavailable.
