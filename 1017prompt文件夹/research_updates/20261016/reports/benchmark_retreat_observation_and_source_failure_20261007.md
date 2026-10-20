# Fixed-mount retreat observation probe (2026-10-07)

One actual18s sensing sequence, no navigation candidate or success rate. Same near cold start X=-1.3/yaw0, frozen camera/light/rollPID/protocol/smoother/Fast depth/CAD. Zero requests0–6, main-.3/aux0 for6–12, thenzero12–18; one reset. Main/aux requests pass through actual supervisor/PID so applied targets and physical joint states are not assumed equal to requests. Nominal PriorRGBPolicy is called once for source/map diagnostics, its output NEVERcontrols this probe; nominal_request and execution_role explicitly separate it from actual scheduled request. Old planner/tracker diagnostic values are nominal only. Caller chooses actions by current time alone, not offline depth or future feedback. Backward unknown space retains authorized exploratory scope, not observed-free certification.

## Actual physics and execution validation

Exit0/no contact;180scheduledrequests181captures901physics37actualFastinferences. True rootX at0/6/12/18s: -1.300000429 / -1.266752362 / -1.839271426 / -2.017139435m. Six-second requested retreat actual displacement.572519064m, followed by.177868009m tail. First five consecutive10Hz measured XYspeed<.05 after12s is13.4s, with reset/rebound counting retained. This is not a stopping envelope guarantee. Geometry and model layers scored separately.

Author personally verifies2642source/closedartifactSHAs, all180requests against900actual command substeps and saved current inputs,444paired packet/mask fields, one reset/unchanged rig and18s diagnostic config, zero camera exposure delta, two181frame10Hz videos. 120blocks differ from nominal policy output, explicitly never reported as a navigation B0 execution. No scoring-tail frames. 9schedule fixtures literal missing-module RED thenGREEN.15s; full419tests21.90s. Actual before/after stereo image montage inspected: retreat_observation_actual_views.png.

## Repositioning restores geometry but does not fix the source

|exposure seconds|bilateral GT-depth-consistent divider pixels|accurate accepted Fast divider queries|
|---|---:|---:|
|0.0|0|0|
|6.0|0|0|
|12.0|5404|0|
|18.0|11629|0|

Thus actual retreat restores5404/11629physically supported divider pixel correspondences at12/18s, versus0at0/6s. However the existing Fast source has **0/2507** whole-probe accepted endpoints accurate within.10m (2499finite GT,8insufficient); no new depth-admission or navigation pass. At12s72accepted queries are not on the divider proxy in either eye;18snoaccepted Fast source query. Geometry-only recovery is insufficient. Source evaluation includes insufficient truth in unconditional denominator, not cherry-picked finite successes. All GT read only after query prediction/source sealing and never used to decide the retreat.

## Frozen existing matcher check, no network replacement

Exactlyfour same closed RGB exposures, old SGBM448/block5/LR1.5/padding parameters unchanged, bilateral dynamicCAD source filtering unchanged. All four predictions/packets SHA sealed before truth scoring. Accurate endpoints≤.10m:0/10,0/38,0/122,0/412 at0/6/12/18. At18s median/p90error .181574833/.194596063m. No threshold relaxation, no online SGBM substitution, old rejected SGBM/Fast gates unchanged; already used exposures are development/historical, not fresh validation.

One administrative runtime failure lacked USD Python path after writing empty probe protocol, second attempt correctly refused the existing directory; both logs/initialprotocol untouched. Independent v2 output namespace and correct bundled USD runtime used, same frames/matcher/criteria; completed once, no physical rerun. See retreat_sgbm_runtime_recovery.json.

Bounded pixel-pair diagnosis: at18s all412SGBM endpoints in both eyes belong to the same nearest geometric-cylinder proxy, none to different proxies; actual endpoint pair gap median.065697827m, selectedRGB mean-channel residual median1/p902.333levels. At12s97/122pairs hit the same proxy. This rejects the confident adjacent-cylinder-alias explanation for these selected packets. Low photometric residual and LR consistency do not guarantee .10m depth accuracy on these surfaces. Proxy is not renderer objectID, diagnosis does not establish a sole cause or permit a truth-filtered online gate. read_camera_depth uses distance_to_image_plane; no radial/axial reinterpretation was applied.

## Single next isolated source test

No navigation consumer is qualified yet. A new separately frozen seven-development-exposure test will zero CAD-invalid robot pixels in copied left/right RGB **before the same frozen Fast backbone**, retaining existing post-matching filters/weights/iterations/range and source coordinates. Robot pixels previously entered the matcher despite post-query exclusion. This hypothesis is not asserted as the cause. Exactlyonezero-fill preprocessing, no crop/fill/network/parameter alternatives. New local thresholds pre-frozen in2026-10-07-cad-masked-stereo-input-probe.md; require≥80%accurate source endpoints and≥25accurate divider queries atbothretreat12/18 plusfar-domainno-regression, before evenonefreshwhole-sequence validation. It cannot certify old all-domain sourcegate/navigation. 6pure-helpertestsRED→GREEN.15s. Not a controller modification.

Evidence: retreat_observation_protocol/status/derivation, originalretreat_observation_run allsources/physics/execute histories/maps/rawRGB/GTisolated/videos, retreat_observation_assessment.json, retreat_visibility_preanalysis/diagnostic, retreat_frozen_sgbm_probe_v2 protocol/predictions_pretruth_sha256/assessment, retreat_correspondence_alias_preanalysis/diagnostic, actual montage. Stage closed as **physical coverage restored; reliable depth/navigation NOTpassed**. Wholeproject/learningincrement/formal35sE0E1E2 unfinished; actualnextsourceablation mustcontinue, notreportprojectcompletion.


## Append-only independent review and historical role clarification
ONEfirstpass independentreview C0/I0/M1; reviewer reused prior Oracle context, not blank-context. Personally9tests .15s/2642source+artifacts/all180requests900command ticks/1081contact measurements/two181videos/370packetfields/sourceGT/4visibility/4SGBM/alias calculations. Full419suite and444pairedCADfields remain author evidence. Historical pre-close manifest rgb_control=True is stale metadata: executed actions came from the fixed current-time schedule, nominal RGB policy outputs were diagnostics only. Source, physics, sealed manifests and reviewed report bytes unchanged; no rerun. CAD input ablation/analytic depth/learning are separate new branches, outside this review.
