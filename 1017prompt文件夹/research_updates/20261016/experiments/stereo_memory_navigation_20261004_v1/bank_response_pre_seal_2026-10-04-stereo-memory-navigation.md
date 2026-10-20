# Fixed stereo, spatial memory and active observation implementation plan

> Execute inline using superpowers:executing-plans. User authorized the entire proposed research on 2026-10-04, without intermediate approval. Results, including rejected candidates, must be supported by actual evidence.

**Goal:** Establish whether the fixed binocular rig can support an executable memory-based local navigation baseline, then quantify the value of learned memory separately.

**Architecture:** Rectify the measured binocular rig, estimate metric disparity with actual correspondence and explicit confidence, accumulate gravity-aligned geometry with provenance, and select short executable motions using the existing roll PID. Advance through measured gates; never replace missing observations with ground truth under a visual label.

**Tech stack:** Existing Isaac runtime, Python/NumPy/OpenCV, official frozen Fast-FoundationStereo, existing protocol/smoother/supervisor and roll PID.

**Spec:** The preceding user-approved research proposal in this conversation and the preserved stage report `reports/benchmark_visible_surface_connectivity_20261004.md`.

## Global constraints

- Camera rig ball_stereo_front020_up005_v2 remains X=.20,Y=±.30,Z=.05m, 640×480, focal 584.8889px. Measured image geometry determines rotation and eye order.
- Old stages and source checkout remain frozen. New code and outputs live in `experiments/stereo_memory_navigation_20261004_v1/` in the already authorized independent research workspace; no commits of unrelated untracked material.
- Observation actions count in the original 35s formal budget. History-initialized and safe-start prior tasks are separately labelled and cannot silently replace cold start.
- RGB inference reads only both current/past RGB, calibration and permissible current state; renderer depth/full-map are offline labels or explicitly privileged diagnostic arms.
- Unknown, historical observed, current observed and predicted areas remain distinguishable. Floor support alone is not a complete 3D swept-body certificate.
- Model and geometry gates cannot be relaxed after seeing held-out results. Development replay is development; new fresh whole sequences must be sealed before fitting or tuning.
- A failed prerequisite is a research result and blocks dependent admission; it is not permission for a model/benchmark success claim.

## Review focus

1. Vertical native epipolar direction, rotated pixel centers and eye order: hand-derived reprojection fixtures.
2. Right-image occlusion/body mask: bilateral confidence must reject unsupported correspondences.
3. Startup unseen floor: no memory update may turn never-observed cells into observed free.
4. Pose/time drift: snapshot provenance and expired confidence must remain visible.
5. Simulation execution lag and stopping tail: score actual trajectory and all observation time, not request-only progress.

## Task 1: Binocular geometry and bounded RGB stereo audit

Files: create `stereo_geometry.py`, `test_stereo_geometry.py`, `audit_stereo.py`; download official frozen source to `third_party/fast_foundation_stereo/`.

Consumes: frozen new-rig 66-sample development manifest; native left/right RGB; independent renderer-depth labels. Produces rectified pair/depth/confidence and `stereo_audit.json` with source identities and measured accuracy/coverage/runtime.

- [ ] RED: hand-derived native vertical-baseline fixture must rectify to horizontal positive disparity; constant disparity yields z=f*.6/d; invalid/right-masked/out-of-bounds disparities rejected.
- [ ] GREEN: implement `rectify_pair(left,right)` and `disparity_depth(disparity,right_disparity,left_mask,right_mask, lr_tolerance_px=1.5)` without true depth inputs.
- [ ] Run all stage CPU tests. Expected all pass and save RED/GREEN evidence.
- [ ] Run fixed SGBM (one configuration, full disparity span justified by .6m baseline) on all 66 development pairs. Assess true bilateral geometry separately from algorithm matching.
- [ ] Inspect official Fast-FoundationStereo environment/weights and execute one frozen model if resources/dependencies permit; report measured performance, no hardware table substitution.
- [ ] Freeze a fresh-sequence validation gate before any model tuning. Original .3m error is descriptive only; admission requires body-height near-obstacle error p90≤.10m, ≥90% within .10m on at least 100 samples/subgroup, geometry overlap and finite conservative confidence. Reject if coverage cannot support collision checks.

## Task 2: Startup observability and measured observation motions

Files: `observation_protocol.json`, `observation_schedule.py`, `test_observation_schedule.py`, `run_observation.py`, `assess_observation.py`.

Consumes measured rig and existing physical controls; produces no more than four 12s complete open-plane sequences: zero control, positive/negative bounded bank oscillation, slow forward-stop and reverse-stop. Each resets once, internally retains PID/smoother/physics.

- [ ] RED/GREEN: schedules end with a braking interval, keep protocol bounds, and count observation time; unknown-start certificate cannot be fabricated from an empty memory.
- [ ] Freeze commands before results: zero; ±15° bank request for 2s each then zero; main +.3rad/s for3s then zero; main -.3rad/s for3s then zero. Six-second initial zero settling and remaining braking within12s.
- [ ] Actual synchronized camera/physics captures in open plane with fixed rig/lighting, markers hidden. Report actual translation, attitude, velocities, stopping qualification, near-field newly observed support and unknown footprint.
- [ ] If no certified cold-start action exists, state explicitly the need for a declared safe-start assumption, extra sensing or independently verified observation mechanism. Do not silently drive into unknown.

## Task 3: Spatial memory and traversability

Files: `spatial_memory.py`, `test_spatial_memory.py`, `audit_memory.py`.

Consumes points/poses/confidence at current timestamps; produces bounded gravity-aligned local occupancy, floor support, last observation/provenance and uncertainty.

- [ ] RED/GREEN: past obstacle survives an occluded current frame; never-observed remains unknown; obstacles override ground support; expired/uncertain poses cannot increase clearance confidence.
- [ ] Compare current-only vs memory on complete moving sequences using independent truth labels. Collision-body coverage, obstacle persistence and false-free rate are required; map-area gain alone is not acceptance.

## Task 4: Executable local planner and diagnostic closed loop

Files: `primitive_planner.py`, tests, `run_navigation.py`, frozen experiment manifest.

Consumes admitted memory, current physical state, local goal; produces a bounded request sequence scored by swept volume, stopping and visibility. Uses existing roll PID and empirically supported main/bank motions.

- [ ] RED/GREEN: unknown collision volume cannot be ranked as certified safe; braking extends the footprint; absent feasible motion returns explicit status; goal progress is independent of candidate path index.
- [ ] Use actual motion sequences from Task2 rather than ideal differential-drive integration. One bounded primitive family only.
- [ ] Run true-depth/true-pose diagnostic same-initial-state arms first: current-only, memory, memory+observation. Separately declare any safe-start prior task. Run formal35s only when prerequisites support it.

## Task 5: RGB + estimated pose and independent evaluation

Consumes stereo gate and diagnostic planner. Implement stereo geometric odometry with estimated uncertainty (or reject if insufficient correspondences), then run actual RGB closed loop. Do not present true-pose results as this layer.

- [ ] Isolate complete fresh sequences before tuning. Compare estimated pose to offline truth; reset/future substitution tests enforce causal inputs.
- [ ] Execute golden/hard cases and original E0/E1/E2 only after required admission gates. Record collisions, actual progress, end-goal distance, blind-zone passage, startup time, end-to-end latency and false-free rate.

## Task 6: Learned memory/world-model increment and research report

Only after a working explicit-memory baseline and adequate sealed data: one bounded action-conditioned recurrent residual/memory candidate, complete-sequence fit/validation, no future observations as online inputs. Compare against unchanged explicit memory; reject if no independent closed-loop improvement.

Final report separates stereo measurement, state estimation, mapping, startup assumption, diagnostic navigation, RGB navigation, learned increment and formal benchmark. Archive reproducible commands, hashes, videos and test evidence. No guaranteed positive research result; no skipped gate may be called completed validation.

## Initial rulings

- Workspace has an unborn master and years of unrelated untracked experiment artifacts; an independent new experiment directory is the isolation boundary. No Git history or external checkout is changed for this task.
- User explicitly requested autonomous inline execution. Skill plan approval pauses are superseded; implement and verify without another approval request.
- The proposal spans dependent subsystems; each gate is a bounded experiment. Failed admission preserves the finding and triggers causal diagnosis, not unbounded network/parameter scans.

## 08:20UTC bounded consumer amendment (existing gates unchanged)

Completed actual one-arm post-policy stopping-veto experiment. Interface passed with exact28.4s physical prefix and real changed final requests. Actualcontact avoided in this one E1trial, butgoalnotreached/referenceprogress andlateralregressed; navigation rejected. Historical.140884m stoppingtail also rejected as currentlargebank bound (1.5s radial.183946m). Report `reports/benchmark_local_stopping_consumer_20261004.md`,41CPUtests; nonewstereo/learning/formalnav acceptance. Next bounded work: reuse successfulOracle execution geometry and existingphysicaldata, freeze atmost2 targetedlargebankturn/stop sequences ifcoverage missing; one executable localshortsegment consumer, keep source/provenance/unknown risk separate. No furtherstop-radius scan/repeatE1arm/newnetwork. Wholeproject remains active.

## 09:21UTC targeted execution coverage amendment

Exactlytwo preregistered18s bank±30 encoded/main.3 turn-stop whole sequences completed,2exit0,44CPUtests. Measuredstartup lateral~.148m at1.5s vsheadingchange~.062rad; oldrootyawconstantarc misses transient motion. Bothstopradialat1.5s>.14,oldbound rejectionpreserved. Negativetailstabilityfails PRE-frozenbanktrend1deg/s; no gateadjustment. Report `reports/benchmark_large_bank_motion_coverage_20261004.md`,actualbundle notacceptedpredictor. OldsuccessfulOracle10scores remainhistorical;9wrapperhashessame but2currentruntime/runner hashesdiffer, identityportscope limited. Nextone guardedshortsegment consumptionusesmeasuredXY+stopping andcompletecurrent executionstate;whole fresh validation sealedbeforefit,old2nowdevelopment. Do notrepeatmatrix/stopradius/modelscan. Research remainsactive.

## 10:21UTC guarded response consumer amendment

One actual whole18s yaw+20fresh validation complete,exit0,49CPUtests. Completefrozenexecutioncloneundermeasuredfeedbackexact;guardedarchivedXY+itsown6sstopreallyoutputselectedrequests. Narrowcold-openplane yawtransfergatepasses(XYpeak.0003045m),notgeneralphysicalmodel/nav. HistoricalE1only1/255statesapplicable;195unknownselectedsweptcellsnotcertified. Report `reports/benchmark_guarded_measured_response_consumer_20261004.md`. Scopeexplicit12sindivisiblefiniteprimitive,notarbitrary1.5sstop/replanning. Nextboundatmost2whole targetedshort-prefix/matched-stop sequences,priorsealvalidation,nooldmatrix/yawrepeat/noM3/stopradius/gateloosen. Missionremainsactive.

## 11:22UTC short matching-stop / actual repeat gate amendment

Exactly2preregisteredwhole13.5/21s sequences completed,2exit0,53CPUtests. One1.5srequest+matching6sstop consumer reallyexecuted;freshwholeXYpeak.0000352m. Repeatselectionactuallytriggeredafter75blocksAND38actuallowcounts,butfullstate/PID/smootherfails→repeatREJECTED. Zero fallback drifts.14188m;notnavigationimprovement. Rawzeroafter7.5 makesXYspeedrebound.2177/stopradial.2420;bankreturn/stoppinglateral+ .1336 cancels90.64%turnsidewardnet. Report `reports/benchmark_short_matching_stop_and_repeat_gate_20261004.md`. Nextboundedonecandidateisolatestop-bankreferencesource(holdprior22.5degwhilemain0)withsameactualprefix/PID/config,baselinealreadyarchived;stop/physicalprogressgatespreseal. Noendstate-tableexpansion/gaterelaxation/newnetwork. Researchactive.

## 12:24UTC stop-bank reference controlled amendment

Exactlyonepreregistered13.5sstop-bankholdsequencecompletedexit0;55CPUtests. ArchivedB0notrerun. Same7.5s17nonrawphysics/75requests/76completeexecutionstates/sourcepackets,allmainraw/smooth/appliedsameentireepisode;300auxappliedtickschange. Actualfive-lowqualification8.8vs10.2(+1.3vs2.7delay),6sradial.17139vs.24202(29.2%less),path.18790vs.38608(51.3%less),all12pre-frozenbrakechecksPASS. Source-onlyrequestinterventioncausallysupportsstopping/sidewardcancellationeffect,notgeneralnav. Remainingdrift/bank24.58/internaljointmotionmeanqualificationnotreset. Old.140884boundstillreject. Report reports/benchmark_stop_bank_reference_hold_20261004.md;old41files/report/robotSHAunchanged. NextoneactualE1realstop-eventrequest-sourcevalidationwithsamefrozenbaseline,35sdiagnosticbudgetandindependentprogress/lateral/qualification/contactgates;noPID/mapper/planner/cap/vetoradiuschanges,noextraresponse-table/modelscan. NoRGB/learned/formaladmission,wholemissionactive.

## 13:25UTC actual online bank-source validation amendment

Onepresealedactual35sE1runcompleteexit0;59CPUtests. Onlinehistorylatchafterunchangedparentactactuallyholdspriorbankatzero-main event28.4,notID/time/preconstruct. Same1421samplephysical/parent/core/map/executionprefix,raworigin0;330auxchanges,allmainchainidentical. BrakegatePASS(qualification30.6→29.8,6sradial.23994→.14689),WHOLEreference/navREJECT(refprogress2.14889→2.13017,goalremaining4.08557→4.23038,both35timeout). Report reports/benchmark_online_bank_hold_navigation_20261004.md. All66posteventunderlyingmotionproposalsvetoed;A*start.6clearancepassesbutoldstopdiscseeshazard.6073,notlatchwaiting. Activeactualspeedmean.1183vs.3047neededfor8.83675reference/29remainingseconds;notuniversalbound. NextboundedOFFLINEcausalauditusingactualonlineA*paths/hazards/state:ownpathtrackingvsindependentreference,path-stopcompatibility,currentcourse/heading/executionandtask-time/oldphysicalcoveragebeforeONEsupportedconsumerchange. Noextra stopbankrun/radius/PID/modelscan/defaultbaselinechange;returntofailedRGBlayerafterexecutioncauseaudit. Wholemissionactive,oldgatespreserved.

## 14:27UTC route/execution audit amendment

One350frame causal archived-data audit complete,0policy.act/0actualruns;35maps/350targets/350protocolproposals EXACT,63CPUtests. CurrentpathRMS~.034m resetswith.5sreplan;frozen-windowexitRMS.04668,latecourse/path31.29deg vsbody3.80deg. SpecificA*.6 vsOLDstopdisc.71159 mismatch visibleinrequestedchord19s/route20s beforebodyveto28.4. OneofflinealternativeusingMATCHEDoldradiusfindscompatiblecurrent-mappaths19/20/25s,butstoppedstateblocked;notnewnavclaim. HistoricalsuccessE1meanXY.28757/requests.5–1.5differscurrent.3domain;identityscopeunchanged. Report reports/benchmark_online_route_execution_causal_audit_20261004.md. Nextoneactualevent-branchgridclearanceconsumer per docs/superpowers/plans/2026-10-04-route-stop-clearance-single-consumer.md;noPID/speed/radius/mapperchanges. AfteruniquecomparisonreturnfailedRGBvisibility/matchingcauseusingexistingpairs;noendlessexecutionloop. Missionactive.


## 15:29UTC unique route-consumer exit amendment

Exactlyonepresealedactual35sE1comparisoncompletedexit0,69CPUtests,37newartifactSHA/old15+39unchanged. Real20sevent/prefixmatched;routeschangebutlargelysaturatedrequests,XYsepbeforestoponly.000579m. Planner no-route28s replacesB0veto28.4:70no-routevs66veto,notnavigationimprovement;wholeprogress2.13017→2.08066,goal4.23038→4.25965timeoutboth. CandidateREJECTED,norepeat. Local fewer-veto6sgatedefect(retained):B0has0in20–26,cannotstrictlyless;independentmeaningful/wholegatesalsoreject. Report reports/benchmark_route_stop_clearance_consumer_20261004.md. ReturntoRGBusingarchived20alignedpairs/samefrozenpredictionsforonevisibility/photometricpartner/ambiguitycauseaudit,nofullmatrix/newmodel/camera/PID/radiusscan. Missionactive.


## 16:29UTC archived RGB visibility / matching amendment

One frozen20pair offline causal measurement executed,noinference/acquisition/nav. Oldpixelgate exactbyfields+membership;lefteligiblebodyshared45.07%/13.74%,floor16.54%/15.03%;mosthardlossrightFOV,selfmask. Rawhardstill1.54%accurate/final.76%. 4399stratifiedpatchsamples showlocalintegerSADambiguity,bute1likewiseambiguous yetbetter,notsoletexturecause. FreshreviewleftwholemaskwarpbugfixedRED2→GREEN78,pre_reviewpreserved;finitewarp3797supported,68oldinputs+52oldartifact/24newidentity verified. Report reports/benchmark_stereo_visibility_and_matching_audit_20261004.md. Nextboundedindependentknownscene/mesh-ray+silhouettechecksamearchive toboundall-surfacetruthlabelscope;afterthatONEevidence-supportedtextureorcausalpast-multiviewdiagnostic,nocamera/control/model/matrixscan. RGBgateunchanged,wholemissionactive.


## 17:31UTC bounded source-geometry audit complete

Report: `reports/benchmark_static_scene_geometry_and_depth_labels_20261004.md`. Four historical hard0004 pairs/eight eyes, one independent authoredray audit plus one presealed same-source mesh representation decomposition; no acquisition/inference/control. Original analytic gate failed right000/010 and remains immutable; right020/023 lack support. Raw mesh current samecore agree<=1cm99.9005%L/96.1886%R vsoldpretransaction47.53%/64.82%,p90~1e-6m; this independently narrows synchronization diagnosis to known sphere surfaces and exposes idealprimitive assumption error. Preserve73/893failures and ground exceptions, noall-surface/renderersubdivision guarantee, nooldRGBadmissionchange.86CPUtests, freshreadonlyreviewnoCritical/Important,46oldSHAchecks/28newartifacts. Byteexactidealhelper snapshot preserved; USDnative row/savedtransposedcolumn conventions correctly documented.

Next onlyONE offline causal past-multiview support audit: freeze budget/full denominator/earlier-time-only rules beforemeasure; actualpastleft+right views/provenance versuscurrentselfmask/outofview; separatetruevisibility fromreliableRGB depth. No GTonline, futureframes, freshclaim, texture+memoryjoint intervention ormodel/thresholdscan. Source-ray errors remainexplicit; pureRGB/estimatedpose/learnedincrement/formal35s results unfinished. Nojobleft, automationACTIVE.


## 18:31UTC causal past-view support complete

Report `reports/benchmark_causal_past_multiview_support_20261004.md`;4historicalhardpairs,73383fullsourcequeries,no fresh/dedupmaparea/onlineclaims. Currentstereo8379→causalunion24298,+15919;earlier-onlypairs andsourceframe/time/eyebits verified. Whole2x2/.01mgeometrylabel +frozenRGBbothvisible/validconfidence/.10m rule yieldscurrent/pastRGBreliable0. Truevisibilitypotentialdoesnotrepairmetricpredictions;naiveFastfusionREJECTED,nooldgatechanged.92CPUtests,newcode/data/2inspectedfigures,23sealedinputs/51oldSHAchecks/20newartifactidentity,initialarrays/source preserved. Freshread-onlyreviewnoCritical/Important;Minorcolorcaptionfixed,reportwrittenafterreviewexplicitlyauthorselfchecked. No sim/inference/control/jobs;automationACTIVE.

NextONE traditional causal multi-view RGB metric candidate atknownpose diagnostic layer:actualcurrent/pastbothRGB+K+current/pastknowncamera poses, nevergeometryGT/depth/partnercoords/future. Preseal one low-dimensional matchingstructure/search/visibility/no-parallax/uniquenessrejection+same4pairhistoricalbudget before evaluation;literalgeometry/causalityTDD. Maintain olderror/coverageadmissionandunknownsemantics;onlyifsourcequalitypassedvalidateoneprovenance-memoryconsumer,nottexture+network/controlscan. Candidatefailuremustrejectstructureandlayerexplicitly,notunboundednewconfig. NoRGB+estimatedpose/learnedincrement/formal35scompletionyet.


## 19:31UTC unique causal RGB metric candidate complete

Report `reports/benchmark_causal_multiview_rgb_plane_sweep_20261004.md`. ONEsealedV1implemented and4actualhistoricaldenseRGB+known-posepredictions; truth/scene/truepartners absent fromsanitizedpackets/predictor, allpredictionsserializedbeforelabels.98CPUtests, onefreshreadonlyreviewnoCritical/Important,56independentSHAchecks,final40old+40newidentity. TwoMinorreportcaveatsresolvedwithoutoldarray/gatechanges;inheritedinverse/floorintegerprojectionroundoffnotrobustedgecertificate. CPU7.1–41.2snot10Hz. No new sim/control/fit/video/freshclaim.

Necessarysourcequalitygateall4FAIL:73383fullqueries,796retained/43correct(.0586%all),p901.15745m;strictany/all1vsoldFaststrict0doesNOTadmit. Fastsame-centre2832/6,p90.34134;old.76%bodygateuntouched. Oneafterpredictiontrue-depthpatchlabel diagnostic showsgenuinegoodcostsources exist, butvisibilitydoesnotensurewinnercorrect:580allcentrecontributionsvisible/547wrong;000single-source167/150wrong. V1structure+confidenceREJECTED;no fusion/controladmission, no retune/descriptor/networkscan. Whole research remainsunfinished,automationACTIVE,nojobsremaining.

NextONEofflinecauseexperiment: freeze<=32deterministicanchorqueries (<=8/frame) independentofGTaccuracy andsame128-depthstructure beforemeasure; savedcost/support/texture/parallax/footprint/per-viewbitmapcurves BEFOREoffline truthattached. Useactualcurrent+strictpastRGB/knownposes only, diagnosewrongwinner/true-neighborhood/source-setchanges andsingle-source000ambiguity. Not a newdensepredictor/modeladmission orfutureGTconsumer. Ifsource-switchcausefails,explicitlyrejectit,donotloopendlessaudits;futureONElocalrepaironlywithevidenceandnewpreseal. NoV1four-dense/oldFast/physicalmatrix/PID/cap/radius/plannerrepeat. Estimatedpose/reliableRGBmemory/executableplanning/learnedincrement/formal35sE0E1E2stillseparateunfinishedgates.


## 20:31UTC sparse cost/support cause audit complete; close source-switch branch

Report `reports/benchmark_multiview_cost_support_trace_20261004.md`; ONEsealed32-anchor/128-depth frozenV1 trace, no dense/sim/control rerun. Deterministic16spatial+16savedconfident-outputanchors independentofGTaccuracy, latterhalfconditionedonoutput/notpopulation/fresh. Curves beforetruth; all32savedV1 output/cost/width exact,104CPUtests,onefreshreadonlyreviewnoCritical/Important. Final20:54:04UTC61oldSHA/26newartifactidentity. DeferredMinorinvalidborderauxpatchmoments disclosed, no validprediction/statistic effect; originaltraces/code preserved, nosecondreview.

17retained/16wrong:8source-switch/8noswitch;11truthnearestcostvalid/allhigherthanwinner+.05,6samebitmap,12allwinningcentrecontributionsactuallyvisible. SinglecurrentR000wrongwithoutsource-switch remains. REJECTsource-switch-onlyrepair; neither declarephotometry/exposure/texture unique rootcause nor addcandidate/anchor/thresholdscan. V1/oldgates frozenrejected; noRGBmemory/navadmission/10Hz/freshclaim.

NextONE actual moving highcontrast CalibrationOnlyBoard intervention, independentopenplane/one12sforward-stopsequence. Fixedrig/lighting/execution/transaction; presealnonrepeatingknownpattern/featurelayout/geometry/request/finiteframes/numericalgates. ActualRGBfeaturelocation versusknownboard offlinegeometry checks moving exposure/current-pastpose; only aftergeometrygate usefrozenV1limitedquality. No boardtruthinputtopredictor, no formaltexture/markerchanges, no originalstatic6renderrepeat. Narrowpass cannot certifyuntexturedscene ornavsuccess; onefailedwitness localizeslayer ratherthanunboundedtexture/network/lagscan. Research remainsACTIVE; reliableRGB+estimatedpose/onlineprovenancememory/executablelocalplanning/uniquelearnedincrement/original35sE0E1E2stillunfinished.


## 21:33UTC moving coded-board witness complete

Report `reports/benchmark_moving_coded_board_rgb_pose_20261004.md`; ONE actual12s forward-stop,120request/121capture/601physics/two121frame10fpsvideos. Startup sandbox GPU failure before capture archived/resolved; front authoring reflection corrected by literalCPUfixture and V2 sealed BEFORE data, samecodes/gates/requests.111CPUtests,onefreshreadonlyreviewnoCritical/Important. TwoMinor prose fixes world-vs-opticalaxis and10Hzcapture path/speed explicitlypostreview; no experiment code/raw/summary/gate changes. Author final2026-10-04T22:03:53.631110+00:00:43oldSHA/190new identities.17nonrawphysics exactoldsequence, raworigincurrent-old-.02s/-1, historyshiftsonce,clockage0. No jobleft, automationACTIVE.

All12presealedgeometrygroupsREJECT coverage:420expected/356recovered/all356<=2px,84.76%<90%. ConditionalRMS.620–.809/p90.794–1.065;motioncurrentposebetterthanprevious.1scapture, narrowtrueRGBfeaturealignment witness only.16missingGT-quaddiagnosticsstillrejected,blackborder5–13white>max2; no GT helped detections/admission, specular-only cause unproven. Six whole-tag eyeintersections0NOTallpixeloverlap0.0V1runs; retained72max<min100budget defectdisclosednotmodelperformancefailure. OldV1/Fast/RGB/navgatesunchanged; no fresh estimatedpose/memory/navresult.

NextONE board MATERIAL-only controlled A/B, currentdefaultB0 reused/samefull12sinitialcondition/request/code/layout/rig/light/capture/PID. One explicitlowreflection/highroughness materialintervention only, no exposure/light/camera/code/decoder/planner/benchmarktexture changes. Independentpreseal unchangedfullgeometrygate andfeasible finite downstreamsamplebudget BEFORE data; no retrospectiverepair. Ifmaterialdoesnotresolve rejectthiscausalexplanation, no endlessnewcolour/light/threshold scan. Onlygeometrypass admits samefrozenV1 boundedsource-quality test; boardtruthofflineonly, notformaluntexturedscene/navigationcert. ReliableRGB+estimatedpose/onlineprovenancememory/executablelocalplanning/uniquelearnedincrement/original35sE0E1E2 remainunfinished.


## 22:34UTC material-only board A/B and finite V1 complete

Report `reports/benchmark_coded_board_material_and_sparse_depth_20261004.md`. ONE presealed12s matte-board arm22:40:07→22:40:58 exit0 vs archived defaultB0; samecodes/layout/colors/rig/light/decoder/control/transaction. Full19physicalarrays/request/121states-history/boardgeometry EXACT; raworiginsame. Board-only actualUsdPreviewSurface/primvarbinding verified,no light/emission/formaltexture/physics changes.115CPUtests,onefreshreadonlyreviewnoCritical/Important; reviewedreportSHA962b39f84008ec487b15bf32b2fb4c669071c3ff945cbff50bbc0165bcd36a47 unchanged. DeferredMinor legacy matching placeholder explicitlysuperseded by stage_summary/v1 actualassessment; no oldsource/gate rewritten. Authorfinal2026-10-04T22:50:49.510274+00:00:213oldSHA/203newidentities,verifierNOTcomprehensivelyreviewed. No jobleft/automationACTIVE.

BOARD geometry passesunchanged all12gate420/420/420; old16RGBmissingallrestored/GTwarpborderwhite5–13→0. Current-past motionRGBprojectionsupports narrowcurrentposewitness only; notall-surface/originalhard materialcert. Sixwhole-tag eyeintersections0NOTzeroallpixeloverlap. Material intervention closes thisphotometricbranch; no continuedlight/material/threshold/source-cost scan.

Conditionalpresealednewbudget6×24=144/min100 permitted6actualfrozenV1 sparsequeries BEFOREboardlabels:132raw/35retained/33correct,p90.06932conditional,33/14422.92%<90 REJECT sourceadmission.34retainedpastparticipation issourceevidenceNOTcurrent-onlyAB/closedloopgain;nearesttruth-gridcurrentR25/144. Codesonlyanchorproposal, predictorgetsXYnotID/layout. GeometryfailedB0V1notrun; notquantifieddepthABimprovement. OldV1/old72defect/historygates preserved, noRGBnav/newsuccessrate.

NextONE causal feature-track/triangulation/provenancememory candidate atknownpose layer. One frozen traditional RGBfeature/descriptor and actualcurrent+strictpast matches, knowncalibration/poses, cheirality/parallax/fullmask/reprojection/uncertainty gates; noboardID/layout/truth-correspondence/future access orfloorprior. Preseal finiteoriginalscene whole-sequence-grouped archiveddata/fullquerydenominators beforedesign-specificvalidation; nofreshclaim/adjacentframesrandomsplit, no descriptor/network/ratio scan. Existingboarddata forimplementation witness only, originalhardRGB required; ifsourcesfail rejectcandidate/notblindlyinstall. Onlysourcepass→memoryexpiry/conflict/local-execution consumer, thenestimatedpose anduniquelearnedincrement/original35sE0E1E2. Wholemissionunfinished.


## 23:35UTC frozen causal feature/source-memory stage complete

`reports/benchmark_causal_feature_tracks_and_sparse_memory_20261004.md`。唯一冻结ORB→极线描述子匹配→已知位姿三角化→稀疏来源记忆候选，26归档双RGB对/52 current-history预测在23:42:19UTC全部保存后才读标签；无新仿真/训练/控制/视频。三历史完整采集序列归组但有限选帧，非完整10Hz流/fresh held-out。124CPU测试通过；一次独立只读审查无Critical/Important，独立重提取全部特征/52预测/未来payload陷阱/记忆、独立射线标签与误差/拒绝链/三图核对一致。作者最终2026-10-04T23:57:51.252990+00:00核验248去重旧SHA（51源/203旧产物重叠）与79新身份；作者验证器未被审查者运行，不混称全面背书。没有待续本阶段进程，automationACTIVE，全研究未完成。

来源准入全部拒绝：板current59/46→history343/318（全1536查询，20.703%准确，条件p90.084725m）；原E1 38/19→88/47（全1014，4.635%、p90.179543m，15保留无有效标签）；hard仅87查询<min100，1保留/0正确，p9011.003498m。历史正确query加301减1，非实体去重地图/导航增益。hard错误点几何门槛全过（敏感性.016727m/视差14.2108°/重投影.500016px），离线真中心相隔7.692849m；不能把重投影一致当正确对应保证。E1也有真中心近而重建偏差大的证据，非单一错伙伴解释。

记忆终态板330记录308正确、E173/33且15无标签、hard1错误旧点仍存；从未生成free/认证扫掠或接入动作。known pose诊断不是估计位姿。独立body标签数据不足、mask旋转方形不认证完整ORB滤波依赖，报告保留此限制而不删除坏样本。Minor deferred：NaN reference/provenance时间比较可能绕过因果检查；实测全有限、当前拒绝结论不变，未来复用须以RED→GREEN验证独立坏时间guard，旧候选和结果冻结。原审查正文SHA72bb508ab3212a608d440148eb2203bca8e0dc5abcabbc8070cfc7b1fa11644b，最终追加复核段是作者，不冒充二次审查。

下一项仅一个原场景固定主动观察动作fresh完整采集见证，最多18秒/一条预封存完整请求/逐0.1秒真实双RGB和曝光位姿。先核验实测短段初态与扫掠适用性，保持rig/原场景材质/PID/unknown规则，不用未知地图宣称安全；不适用则给具体阻塞不启动。旧ORB结构作为冻结回归见证保持拒绝，不能事后补帧/换ratio/描述子/网络；必要新输入guard只拒绝坏时间，不改变正常结果。验证运动是否能增加真实来源供给，不能把主动观察或更长停留当导航成功。结束材料/光源/成本/描述子扫描；RGB可靠消费、估计位姿、唯一学习/世界模型增量、原35秒E0/E1/E2仍未完成。全部新代码、数据、测试、来源/失败/图/SHA保存，常规推进保持安静。


## 00:36UTC original-scene active observation — final stage closure

完整报告 `reports/benchmark_original_scene_active_observation_20261005.md`。一次原 golden E1 的18秒完整采集，00:45:55.968560—00:47:12.551316UTC、退出0；180请求、181双目对、901物理采样、1081可用接触记录均无实际接触，两视频各181帧/10fps。相机、照明、材质、roll PID与执行链冻结。已知完整地图只作受控采集的二维扫掠前提，未成为视觉观测/自由区；这不是视觉决定动作或E1导航成功。

实际整段转弯和自身停车响应通过原运动门槛：XY peak .000072081m、六时域yaw最大.000694rad/bank .000632°；五个实际10Hz XY速度低于.05m/s的资格14.2秒，与原实测响应相同。同六秒前缀17非原始时钟字段301物理采样、61执行状态及历史逐值一致，raw原点相同。readonly报表初稿通配排除raw_漏检raw_command_target已保留；独立精确版本补齐17字段，所有点/标签/误差/运动/gate及其他字段完全不变，没有重采或重推理。

冻结ORB的362 current/history预测在00:48:04.371701UTC全部保存后才读离线标签；模型/gate采集前冻结，推理runner/367输入身份推理前封存。完整5531查询（5087有限renderer标签、444无标签）current 226保留/206正确、3.724%完整准确/p90 .044577m；history 568/512、9.257%/p90 .051520m。历史正确查询净增306，不是地图面积或导航增益。两臂原90%完整门槛均拒绝，不用91.150%/90.141%的条件比例宣布可用。终态249记忆记录、203有标签/200正确、46无标签/3错误、247旧查询；从未认证free。旧ORB、hard错误、V1/Fast/M2及所有历史结论保持。

128CPU测试通过，四个坏时间/整段停车支持/初态测试RED→GREEN。作者最终验证 2026-10-05T01:14:05.342714+00:00：479去重旧SHA、1131新产物身份，181双眼特征重提取、362预测/有限wrapper等价/未来payload替换、181记忆快照、180历史移位与17字段完整前缀核验。一次独立只读whole-stage审查及其实际范围/限制见 `active_observation_review.json`；初次作者核验把当时仍写出的stdout log按空文件散列，独立审查确认为Minor；初次JSON保留并明确此局限，最终改用已预留排除的final日志输出，重新散列已完成的初次log。候选/数据/验证器未修改，最终完整身份逐项匹配；最终log在文档同步记录单独封存。不将作者检查冒充审查者执行。四张实际图作者已检查，六事件大图有缩放，另检查原尺寸两事件图。最终报告SHA `dda3ef8b69fa3d54e656b704f57b83dc2727b36dbe62a3584670383ed6254a65`。

这是预封存后的完整新采集，但环境/初态来自已使用E1，不是新的独立环境泛化。known-pose诊断不是估计位姿；nearest-pixel renderer深度/亚像素射线、边界混合、body-ID不足与ORB完整滤波支撑限制保留。未新增全robot资产或DOF/gain读回审计，不将此单一初态响应迁移作通用安全证书。没有新导航率、学习/世界模型或原35秒E0/E1/E2结果。阶段实际进程均结束；全研究未完成、automation ACTIVE，常规不发完成通知。

下一项只做同181帧的只读“稀疏来源地图→完整前进与停车扫掠支持”诊断shadow。固定同一实测参考及完整181帧；current/history同口径，unknown/当前/历史/predicted与来源/位姿层次明确，不重采动作、不换网络/描述子/阈值、不用真值过滤或seed脚下free。这些181帧的标签已被本阶段评价使用，消费者设计继承此知识，只能称历史开发诊断，不能称fresh held-out。先冻结消费者语义/预算及仅观测输入，消费者输出保存后才重新读取标签作分层核验；明确全查询比例不能替代任务空间面积。若本段支持仍不足，给出具体冷启动/观测任务条件阻塞并关闭这一分支，避免无限局部审计。拒绝候选shadow不获得控制准入；只有来源与消费者各自验证通过才改变动作，再验证估计位姿、唯一学习增量和原35秒评测。


## 01:38UTC endpoint support shadow — final stage closure

先完整阅读 `reports/benchmark_point_support_shadow_20261005.md`。一次真实历史回放181帧/362地图；190点/代码/参考身份在01:47:24.005734UTC封存，全部输出01:47:26.673320UTC保存后才重读离线标签。没有重采、重推理、训练、改动作或新视频；旧实际18秒双目视频作为来源。两地图都接收完全相同的causal当前query估计，current无旧query记录、history10秒原query时间；current不等于单帧双目，既往深度增益不混作本轮地图记忆效果。

实际568保留点全部world-Z .936991—1.015382m、nominal floor0；height/uncertainty筛选前，也没有任何点的nominal XY进入完整扫掠333格。起点187/转弯300/自身匹配停车225/whole333格在每臂每181时刻全部未知，所有phase0次通过，floor格全0。6秒开始运动前仍无起点支持；18秒事后249记忆也未补足。strict unknown-blocked语义下消费者拒绝，不能用已经走过/无接触回填观测free或调半径/放大稀疏点。

history在6秒252记录/248正确/4错误；末态249记录/203有标签200正确/46无标签3错误247旧query。hazard代理峰值20末态17，current峰值4末态1；history与旧occupancy自由格重叠峰值11末态10只是2D/高度/跨格代理，不自动等于真实错误物体。false-free0仅因floor空集合，不是安全通过。no ray-carving/interpolation/bodyseed/model-imputed-area；sigma只是扰动敏感性/knownpose_sigma0层；floor代理不是全体积认证，source旧拒绝保持。

六个literal消费fixture RED6→GREEN6；文档完整身份fixture另RED1→GREEN，最终135CPU测试通过（实际日志保存）。作者最终 2026-10-05T02:13:11.325621+00:00：1248去重旧/预封存身份，362地图重建、181packet/旧有效FeatureMemory点等价/未来payload与截断，4冻结raster几何+4独立continuousprojection格集合、1448直接map索引分类。原计划上轮append closing使全文SHA变，原采集前prefixSHA75204b3a…和recorded closure SHAda551acb…都逐字节验证并归档，不略过任意变更；所有其他旧源/数据/报告匹配。新增fixture曾并入封存测试文件，已精确恢复原6fixture、另文件保存新1fixture/合并中间版；首次split缺pytest的失败日志保存，最终135 green。作者一次重复1448geometry核验被主动中断后改为4geometry+1448map-index，仅核验进程、非实验中断，不少查数据。所有失败与版本保存，模型/mapper/runner/190协议/362地图/数据/gate未改，没有实验重跑。

一次fresh-context只读审查范围和发现见 `point_support_review.json`，作者复核不冒充其执行。两图实际检查；所有输出日志另封存，避免循环自散列。报告最终SHA `14bd954cff7172e9f0cde720f6fcd715528e803248d0ff054289dcb30e9e6b42`；最终579新产物身份和文档同步单独保存。任务阶段完成、全研究未完成，automation ACTIVE，常规不发完成通知。

关闭“冻结ORB+这条主动观察+保守endpoint支持解决严格冷启动”分支；没有近场floor/扫掠支持是具体阻塞，不能推出所有像素/所有方法/固定rig永久不可导航。停止追加动作、sigma/半径/TTL/网络扫描和同一局部诊断循环。

下一项回到完整控制/在线系统基线：先检查是否已有相同当前rig与控制身份的完整Oracle结果，存在则复用，不重做矩阵；核验 `H/reusable_oracle_audit.json` 及 `experiments/oracle_feasible_route_20260929_v9/hybrid/` 的实际class/继承/act/PID/协议/环境身份。旧wrapper9同SHA、旧core两份变化/旧environment缺rig，不能由名称继承旧3/3、5/7。必要时最多一条原35秒完整golden E1的已知地图Oracle诊断，同时真正在线运行仅当前/过去RGB+本层knownpose的只读来源shadow；参考、增益、速度配置保留旧成功控制器，不将其当原InformationPolicy的单层修复或同时调参。shadow不改B0动作，全图/全局参考不得写进RGB观测地图，旧source/consumer gate不解除。保存完整路径/真实到达/接触/时延/图像/执行/身份，不报告纯RGB或正式E0/E1/E2矩阵成功率。Oracle成功只说明控制上限，不能替代视觉基线；若要safe-start/已知地面先验必须单列prior任务条件，禁止冒充观测。估计位姿/唯一学习增量/原35秒正式分层结果仍按总计划，未完成不宣称完成。


## 02:39UTC current-rig Oracle success / live RGB shadow — final stage closure

先完整读 `reports/benchmark_oracle_online_rgb_shadow_20261005.md` 及本轮冻结计划 `docs/superpowers/plans/2026-10-05-oracle-online-rgb-shadow.md`。ONE真实原35s goldenE1 Oracle完整导航在02:49:22.233268—02:52:07.139341UTC退出0，30.7s目标entry、距离.1954856m、实际1843接触样本0；307动作/308双目capture/1536physics，两视频各308帧10fps无legacy5尾步。全研究未完成、automation ACTIVE。

205旧environment没有录入当前rig的Oracle；实际旧ConfidentSpeed→MappedBankOracle→OraclePathPolicy(MRO/act源码身份明确)，BankMap/P040D070 PID(.4,.1,.7)、auxsupervisor.3、lookahead.5原样。预sealed58源/配置/精确缓存旧hybrid已选30waypoint(8.7m/0reverse/483展开节点)参考，不重跑hybrid完整搜索/修改planner，不以未来实际轨迹作输入。当前rig固定ball_stereo_front020_up005_v2/原照明/采集事务/hidden markers。307actions/308XY以及11旧共有状态/执行/roll/velocity/clock遥测数组EXACT历史已归档成功，raw起点也同；旧rig未记录/不宣称同RGB/不是控制候选独立A/B。独立参考进度8.657741m、横向RMS.089963/peak.233021，真实路径8.835802；arc声明8.7、折线8.688761、runnerA*评价8.836753分开。Goalentry成功不是末端停稳保持。

Genuinely LIVE RGB shadow每次capture之后/下一act前都真实执行冻结ORB+last3pastpairs、10s原query endpoint map，共308；只允许frame/time/双方RGB/knowncamera poses/K，拒绝map/reference/depth/goal/actions，复制隔离。不读futuretruth/实际futureapplied/supervisor。Oracle完整地图/参考ONLY控制基线，不写RGB观测，knownpose不冒充estimatedpose。无floorprior/raycarving/interpolation/GTfilter/predictedfree/动作准入。

全28120ORBquery/24742有限label/3378nolabel，current756保留426正确1.515%p90.167196；causal2044/1096正确3.898%p90.170100/1419past。BOTH旧90%/.10m必要gate仍REJECT；旧source/consumer拒绝不倒改。末态917记录688labelled352correct336wrong229nolabel912old；记录≠实体/面积。Floor格all308为0，hazproxy峰98/end81，整冻结1521格sweepminunknown1505/end1517unknown4hazard，0supportpass；只是固定空间诊断/不要求提前看见全路准入，仍无floorendpoint。有限nearestpixel/subpixelray/bodyID/fullpyramid/2.5D限制保留。Featurep50.04936/p95.16041/capturepartialp50.3188/p95.44927s，因果在线≠wall10Hz/完整端到端。

五fixtureRED5→GREEN，140CPUtests2.76s(独立审查实际测试另见reviewJSON)。作者full2026-10-05T02:58:19.550166+00:0058源+2478初始pre-close输出SHA（并非全部关闭后输出）、308feature/pred/map真实重建、307实际同输入Oracle on/off/request/diag/mapping EXACT、3真实cold/mid/finalRGBprefix futurepayload-trap/truncate；实际时钟0age/markershidden/无尾步。历史579旧产物身份最终匹配，所有科学源/协议/数据/gate未经审查后改变；最终轻量identity closure仅封存报告/审查/日志，避免无理由重复科学重算。初始manifest在runtime.close前保存，继承close之后才写bank_feedback，故它不含该文件；独立审查Minor记录此范围缺口，冻结runner/manifest保留，post-close supplement保存其SHA c3c56ea6ad00be94c6d4abf597b2eebdbccb30532c0b0c2757665987dde89ea2，作者2508-entry核验已包含它，最终identity再封存。不得把初始2478称为关闭后全部文件。审查范围/发现/declined rulings见 `oracle_rgb_shadow_review.json`；审查正文SHA `2093868fe8fd5ef3b114007207a17b95d6d9e7faf50500b7ab9f009d22c10cbf`，之后作者段落分开，最终报告SHA `a06e2c76256860e756904fc79257c5d2f1be5042c218df1ad9e39df7b53e5d44`。封存工具最初只碰到新stage ledger追加、原prefix精确归档；生成nestedclass缩进parse错在数据前修复/初稿保存，唯一真实仿真不重跑。新stage计划被protocol封存保持原字节，不追加closing破坏旧输入身份。

控制基线已复现成功；冻结ORB/endpoint冷启动分支关闭，视觉与估计位姿/世界模型未准入。下一项独立特权depth+knownpose局部来源记忆控制基线：复用本次成功曲率/roll跟踪组件，局部参考来自当前/过去来源map，全图/全局Oracle路线不得进入局部planner；全球goal允许。先CPU核验真实class/局部参考/曲率/请求链，事前封存一个完整package与原35s、同初态，最多一条E1。保留明确原unknown-risk任务条件/unknown标签，不seed脚下free、不冒称safe；不称InformationPolicy单层修复或纯RGB，不调PID/速度阈值/旧stopradius，旧异状态有限响应不强拼。该上限通过后再按任务条件回到RGB可靠障碍消费/estimatedpose/唯一学习增量，仍不启动正式E0/E1/E2矩阵或宣称总研究完成。若采用flat-ground/safe-start prior必须单列prior任务而非观测。常规安静，automation ACTIVE。


## 03:39UTC privileged depth local curve — final stage closure

先完整读 `reports/benchmark_depth_local_curve_navigation_20261005.md` 和本轮冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-depth-local-curve*.md`。本阶段ONE真实原35s goldenE1已结束03:52:31.736585—03:55:56.221204UTC、退出0；没有待续仿真或第二条运行。接口通过、导航REJECT(timeout35/4.486646m/路程5.890657m/实际2101可用接触样本0)。350动作/351双方capture/1751physics、两视频各351帧10fps/无5s视频尾步。全研究未完成，automationACTIVE，不能作为完整结果通知。

独立privileged rendererdepth+knownpose来源地图→局部target2.4m/crop±4m/原HybridA*node3000→ConfidentSpeed/MappedBankOracle/OraclePathPolicy→原rollPID(.4,.1,.7)、lookahead.5、auxsupervisor.3。外层privilegedFalse只拒绝runner全局Oracle输入，传感仍privileged；全图/全局参考不得入外层，全球goal允许。Unknown labels独立，规划副本风险探索不认证安全，无footprintseed/raycarving/GTfilter/futureinputs。初次局部reference初始化，后续仅替换几何，五低速/反弹/方向/历史/smoother状态保持。冻结Oracle .5/1/1.5主轴域与旧InformationPolicy .3 unknowncap不同，且6s零请求前缀计35s，不能单因素A/B归因或冒称RGB/完整3D认证。72source于03:52:11.003572UTC preseal后保持；post-close2762outputs覆盖bankfeedback，初始manifest只pre-close。

58plansALLsuccess；localtarget world-y符号7.5s(-1.4→+1.4)、16s(+2.2→-2.0)两次变动，与来源hazard变化关联，未隔离单个格因果或形式拓扑。仅一次请求forward→reverse16s，actual18.5s，原合理制动2.5s；不称方向反复/resetbug。265有效movingtracking，protocolclip186(70.19%)、bankcurvatureclamp183、V1outside89(33.58%)。独立旧评价参考finalprogress4.167298/RMS.497267/peak1.114722m，不作为当前planner输入或纯trackingerror。0.5s重新起始参考exitRMS.027668/peak.080303不能证长期成功；保存冻结5s描述窗口，无另一计划实际label冒用。当前/历史floor31/1715、hazard63/163、unknown158028；depth层有来源floor，不改变旧ORBfloor0/RGBsourceconsumer REJECT结论。planp50.797596/p951.243388s，policy p95.940341/capturepartialp95.297808；causalonline≠wall10Hz。

148CPUtests4.39s；作者full2026-10-05T03:57:09.067008+00:0072source+2762postcloseSHA、351双方depthpacket、350真实请求/地图/局部reference/historyshift精确重建，3finitepaststate clones futurepayload/truncation一致；不是3完整freshphysicsruns。全部1750raw physics请求对齐，实测feedback执行链1745ticks重建一致，首5ticks不克隆/不是递归物理预测。初次assessment pending保留，随后verification interfacePASS/navREJECT为当前状态。预数据fixtureV1aux0非理想直行、boundaryhazard首次未RED、seal terminalnewline/initialcapture-reset调用序问题均如实归档，全部封存前解决；没有数据后调参/重复候选。ONE独立只读review范围/发现/declined rulings见depth_local_curve_review.json；原审查正文SHA 94d7cdbfdfba68f8913f5a6606720503ef975624240730c2a362dfff24557e3d，最终作者报告SHA f2cb7daf73773957a910b2736f76cafc7bea365c2a2dbf08359e37fa82495596，后续作者append分开。最终轻量identity只封存/复查现有完整科学输出，旧2513SHA与新72源一致，不重做矩阵。

接续尚未执行：从本次完整58map/reference数据先核验合法参考连续性候选，已有走廊仍受观测支持时稳定reference消费；真实新hazard失效允许明确重建，不锁死绕行/强制方向。仅一个参考消费修改，经CPU几何证据/预sealed独立进度、空间检查点、目标、接触gate后，最多一条同起点/同depth-risk-domain/同6s前缀/同原35s候选对本次B0。保持planner算法、PID、speed、mapping、rig/oldgates，候选不合理则拒绝；本heartbeat不再仿真。局部层未通过，RGB+估计位姿/唯一学习增量/正式35sE0E1E2未触发。常规安静，ACTIVE。


## 04:41UTC reference continuity causal audit — final stage closure

先完整读 `reports/benchmark_reference_continuity_causal_audit_20261005.md`、`docs/superpowers/plans/2026-10-05-reference-continuity-audit.md` 及对应spec。ONE归档几何实验完成04:50:01.698168—04:50:02.974850UTC退出0，协议04:50:01.580440封存77源/2798旧输入；58replan/首N_A+57旧局部段/current+installation114几何；仅8gridA*queries两事件75/160(prior/currentmap×XY)，无额外search/Hybrid/sim/推理/训练/控制/新视频。原35s timeout4.486646m/所有RGB/M2/停止/导航gate保持，全研究NOTcomplete/automationACTIVE。

新观测地图足以使目标y两次切侧，保留旧图只改当前XY不切侧。新hazard3/2格使旧WHOLEgridroute各10顶点原.6栅格失效/min.4，故不能判定无理由planner抖动。但75时旧INSTALLEDprefix剩2.313414/误差.045706/continuous1.027154/raster1m仍有效；160prefix剩2.316381/误差.003219/raster.6通过，continuouspolylineproxy.567948不通过且155安装时已如此，不是新增hazard破坏这一prefix。连续polyline/rint格点/nominalprimitive/实际动态层次分开，不宣称源实现bug/真扫掠危险。57交接44necessaryscreen通过，7continuous不足/6raster不足不重叠，无lookahead/projection拒绝。预declared两个switch均可保留的提议REJECTED；无无条件固定侧/无删除检查/调整.6后跑物理。几何输入干预仅隔离目标侧选择，不隔离35s超时贡献/另一计划真实label。

六literalfixturemissingmodule→RED6callables→GREEN，154全Hsuitetests4.39s。初作者cwd+relativepath写入全部失败、file-not-foundlog保留，之后正确路径创建；旧源/数据无改。作者verify2026-10-05T04:52:36.920731+00:0077源2798旧输入/114scalargeometry/5809brutenearestcentre+paddedraster/4实际原route-target对角精确，无额外search。报告/源码/协议/58rows/8完整NPZ路线/图/旧实际视频引用/所有log保留；初始assessment不倒改。ONE freshreadonlyreview与每条declined scope裁决见reference_continuity_review.json；原bodySHAcc7a132a8e672b003456dc16b9a6e40d468de232a1c2db5373e843757b60a0b6，作者最终报告SHA52e64cc9788e4c72630d86eb4f4cffab0c2d8d52cca72bbaffe0fdda7caf14c2；作者append不冒称二次审查，最终identity/docsync单独核验。

接续单一控制工作尚未实现：只在旧局部suffix+currentbridge两检查通过、remaining>=.5/projection<=.5时保留reference；失效或耗尽则交回原planner重建，允许合法换向/原五低速反弹，不强制选侧或等无法启动条件。此为有有效性guard的新候选，不是把本次拒绝的两个switch一律保留筛选改为通过。先新版本TDD验证真实内层索引/方向历史/smoother状态和hazard/耗尽重建；预封存唯一reference替换时机改动/同depth-knownpose风险域/同6s前缀/同35s，最多一条同起点goldenE1对归档B0，其他planner算法/PID/速度/映射/rig/gates不改。真目标到达/独立进度/检查点/接触验收，不能停慢或跳index称成功。动态安全仍未认证，不继续新几何矩阵/网络/PID/阈值扫描。RGB可靠消费/估计位姿/唯一学习增量/正式35sE0E1E2仍待验证；常规安静/ACTIVE。


## 05:42UTC guarded reference retention — final stage closure

先完整读 `reports/benchmark_guarded_reference_retention_20261005.md` 和冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-guarded-reference-retention*.md`。ONE真实原35s goldenE1已结束05:51:21.407818—05:54:11.093362UTC、退出0，无待续仿真或第二条运行。仅参考替换时机改变；保持原planner/goal/map/rollPID/speed/rig/光照/五低速确认/反弹清零。接口PASS、事前限定改善PASS、导航REJECT(timeout35s/目标剩4.156980m/0实际接触)。全研究NOTcomplete/automationACTIVE，常规安静。

86源/2834旧输入05:51:05.605849封存；350请求/351双方capture/1751physics/2101接触样本，两视频351帧10fps/无尾步。58边界50保留8重建(6success/2failure)，旧suffix/current bridge .6连续和.6栅格、remaining>=.5/projection<=.5，只在原.5s边界检验，不认证动态/3D/unknown安全。首次动作分叉69/6.9s，此前69请求346physics17非raw字段/70完整execution+packet-history EXACT；raw起点-.02s/-1tick分开。249请求block/1393appliedticks实际改变，绝非raw规范化等价。独立冻结评价reference进度4.167298→4.855576m，目标距离4.486646→4.156980，RMS.497267→.174067/peak1.114722→.475266m；2/4m检查点24.9→13.9和34.2→25.3s，6/8未通过，不把索引/短local误差当导航成功。

原真实方向状态17.5—23s经历3.8s制动及1.7s反向跟踪，随后26—35s沿当前local曲线增1.397859m/RMS.045075/peak.102368，但main+.5/83.333%请求裁剪，独立参考偏差仍上升。moving全224帧clip54.464%，V1outside45.089%比旧33.585%更高；不称全链恢复/不把剩余失败全归PID。nominal planner根同时forward/reverse成本0且输入未含实际direction，是初态与执行接口待隔离假设，不是已证明bug或修复。knownpose+真实rendererdepth仍特权诊断层；所有RGBsource/consumer/M2/旧拒绝保持，estimatedpose/学习增量/正式E0E1E2未准入。

162全HCPUtests6.40s，7feature RED+1旧cadence回归；初GREEN fixture误把no-route rebuild当已安装时间，数据前仅修fixture，完整失败log保留。作者full 2026-10-05T05:55:08.554643+00:00 86source/2752post-close SHA，351bilateral packet、350causal地图/reference/request/diag/history重建，1745实测feedback执行链一致(首5ticks未克隆，非递归物理预测)，3finite-past checkpoints futurepayload-trap/truncate(非3完整freshphysicsruns)。独立review仅一次，范围/发现与逐条declined裁决见guarded_reference_review.json；原bodySHA a5f21c0d1f11dad7b9ec56bd20ad2c556bd7abf7208ac7267f9857dfc4540202，最终作者报告SHA 4dba7eecdaf6ca77a468f3552171f86c55c06197cf62bc471534272d2108d0a3，追加作者段不冒称再审查。最终closure不重跑科学矩阵；旧源/数据/旧gate不改，归档B0对照不是独立fresh环境泛化证据。

下一项尚未执行的有界任务：对本次17.5s冻结输入做一个根状态成本反事实，比较把原有.5m direction-switch成本应用于当时真实accepted executed direction的首次primitive是否改变选择与可行性。先独立版本TDD/同map/同goal/同原primitive代价和nodebudget，允许必要reverse，不强制forward/不扫描新惩罚/不改PID速度阈值/不再次重做矩阵；归档几何不冒用另一计划真实运动标签。有合理且真实改变的证据才另外封存唯一35s运行；本heartbeat不再新增search/物理序列。来源可靠性、估计位姿和唯一学习记忆增量仍待分层研究，未宣称整个任务完成。


## 06:44UTC current-direction root cost — final stage closure

先完整读 `reports/benchmark_direction_root_cost_audit_20261005.md` 与独立spec/plan `docs/superpowers/{specs,plans}/2026-10-05-direction-root-cost*.md`。一次归档frame175/17.5s名义几何对照完成06:55:35.435283—06:55:41.269461UTC退出0，preseal06:55:21.321515UTC94源/5630输入。仅1原grid+2Hybrid，同当前地图/pointgoal/XY/yaw、原node3000/.3primitive/.05sampling/.2goal、原reverse1.25/switch.5，不改实际动作/物理/rig/光照/mapper/PID/速度/guard/five-low/rebound。没有新仿真、推理、训练或视频，旧实际视频仅context，不能作候选真实标签。全研究NOTcomplete/automationACTIVE。

原planner完整1821节点/route/target/pose/direction/command EXACT归档。当前acceptedforward/pendingNone，与当时appliedmain+.7、mainjoint实测+.876695/XY纵向+.339162一致；当前afterplanningdesiredreverse和零请求direction_change_braking单独report-only，预测snapshot禁止它们/未来输入。独立copiedplanner仅机械import、initialenum参数/验证、rootg+priority改变，两根仍可选。候选2434节点success首R→F，path3.3→3.0，统一current-directionobjective4.375→4.075，终点误差.174093→.018737。原内部换向1→2，但包括初始交接两者均2；拒绝消除换向/已降低制动耗时解释，不把名义曲线称真实轨迹或导航改善。

作者2026-10-05T06:59:34.658791+00:00 无search验证94源5630输入/字段defaults/当前来源/严格未来payload，21primitive147采样端点<1e-15/原inflatedraster/终点.2均过。Primitive栅格min .632456→.6，当前整polyline连续min .642272→.614018，原guard栅格min .583095FAIL→.6PASS，层次不同/未知风险和动态3D未认证。接口PASS与预declarednominaleligibilityPASS仅给未来独立单次控制对照的必要证据；旧原35s超时距goal4.156980/0contact保持，全部RGB/sourceconsumer/M2拒绝不改，estimatedpose/学习增量/正式E0E1E2未准入。

16真实fixturecallableRED1.18→GREEN1.36s，178全Htests7.01s。作者两次cwd+relativepath写入全失败且无文件创建、缺文件logs保留，随后真正一次科学对照，不隐藏failure/不重搜；机械import用于独立副本/旧defaults原样。ONE只读独立review/每条declined范围裁决见direction_root_cost_review.json；审查bodySHA 71400a7d86431fd5e4212b0528d31ed8cad44f4053f990b7af4cd53d5a8fcd92，最终作者报告SHA dc7cfcdc7c0684bbef9e452b51e5b15da52426db175039fbee91f5ed1f68b017；作者append/封存另核验，不冒称再审查。全部代码、两完整参考、当前快照、协议、数据/测试/图/日志/来源与旧身份保存，不做无关Gitcommit/delete。

接续一项尚未执行的传统控制候选：仅在原guarded-reference policy每次正常rebuild向新planner传当前accepted-direction并应用原.5rootcost，retain不搜索/reset，不按episode/time触发、允许必要reverse。先CPU核验真实class/request类型转换/状态/五次与反弹及保持原guard来源；新协议事前封存最多一条同起点/同完整任务/原35s goldenE1对归档guarded B0，真实目标/独立进度/横向/空间检查点/接触/动作链验收，失败直接拒绝，无penalty/nodebudget/goal距离/速度扫描。该候选仍有2次换向，真实效果不能由当前几何gate代替。本heartbeat不新增控制run/更多search；可靠RGB、估计位姿和唯一学习记忆/世界模型仍待分层研究，全研究继续ACTIVE。


## 07:44UTC current-direction root cost online — final stage closure

先完整读 `reports/benchmark_root_direction_online_navigation_20261005.md` 和冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-root-direction-online*.md`。ONE同起点goldenE1实际原35s于07:54:40.229131—07:57:32.904552UTC结束exit0，无遗留仿真/第二候选，归档guarded B0不重跑。接口PASS，事前限定改善REJECT，导航REJECT(timeout35s/goalremain3.671378/0contact)。全研究NOTcomplete/automationACTIVE，常规安静，不替换已归档B0。

只有原guard rebuild向独立planner传PRE-planning accepted direction并应用旧.5rootcost；初次None，后11次forward；retention不搜索/reset。原reverse1.25/node3000/primitive.3/.05sampling/goal.2/localtarget2.4/crop±4/clearance.6、PID/speed/rig/light/协议/smoother/五低速反弹均冻结。真实depth+knownpose显式unknownrisk，不是RGB/safe/estimatedpose，外层privilegedFalse只拒绝全图/全局Oracle在线输入。无footprintfree/raycarving/GTfilter/futuretruth/新模型或参数扫描。

106源/5670旧输入07:54:23.300306 presealed，350请求351双方capture1751physics2101可用接触0，两视频351帧10fps无尾步/markershidden/0age。关闭后2762manifest含bankfeedback，原manifest pre-close范围单列。首分叉175/17.5s，175request876physics17nonraw/176execution+packet history EXACT，raworigin+.02s/+1tick初始化差异独立；116rawblocks738appliedticks改变。实际175 reference EXACT先前名义candidate，58boundaries46retain12rebuildALLsuccess，source类型/安装/继承act/reset和retention状态明确。独立评价reference进度4.855576→5.389775(+.534199)，goalremain4.156980→3.671378(-.485602)，但RMS.174067→.572390/peak.475266→1.105993，故预gate拒绝；4m空间点25.3→19.7s，6/8未实际通过，不把索引或停车当成功。

只读完整日志分段定位继承projection接口：frame181idx0F→idx1R pendingR/low0，182在accepted仍F/未资格时窗口放行下一forward段→idx3F、pending取消；version3未替换。17.5→23 raw/applied/mainjoint均无negative，最低applied+.45、mainjoint+.418757/root纵向速度+.161221，短reverse未实际执行。候选brake.1s不能解释为比B0合法3.8s换向更好。同期自己冻结version3 RMS.260636/peak.510890偏离；后来稳定local23→27/30.5→35 RMS.035565/.022107，但独立全参考末态偏离1.086392，不能把全部失败归首次交接/PID，也不能以local小误差认证导航。

六feature RED1.81→GREEN2.53/184全Htests8.45s。作者2026-10-05T07:58:39.955951+00:00 106源/5670旧输入/2762产物SHA、350causal policy/map/reference/request/diag、351双方depthpacket、350单次history shift、1745实测feedback执行链精确，首5ticks不克隆/非递归物理预测；3finite-past checkpoints futuretrap/truncate一致不是3完整freshphysicsruns。初assessment pending保留，后verification接口PASS/导航REJECT，旧gate不改。ONE独立只读review及每条declined裁决见root_direction_review.json；审查bodySHA 9da9130cd1ff5cc20d48512f8763cd086cb1cde82f1e1dad0b8d22d5dc0b37a3，作者最终报告SHA 5cce76e114febecf087c74deeebcb22506119fc8ba3a1e2d27843cde918d8b36，作者append/最终hashcheck不冒称二次审查。全部实际代码/数据/日志/视频/版本保留，无无关Gitcommit/delete。

接续尚未实施的一项工作：CPU精确复现未资格换向时投影越过短reverse段，核验waypoint出段/accepted/pending/窗口调用顺序；只设计一个最小投影方向边界修复，保持原五次低速/反弹/mapper/PID/speed/planner/rig/gates，不强制提前反转或循环等待。真实fixture验证/身份与预sealed独立进度/横向/目标/contact gate后，才在后续独立阶段考虑最多一条同初态35s对照，本heartbeat不新增仿真/搜索/调参。所有RGB/sourceconsumer/M2/旧停止包络拒绝保留；估计位姿/唯一学习记忆世界模型/正式35s E0E1E2未准入，全研究继续ACTIVE。


## 08:44UTC unaccepted cusp projection regression — final stage closure

先完整读 `reports/benchmark_cusp_projection_regression_20261005.md` 和H/cusp_projection_protocol.json/assessment/verification/review。上一轮全部来源/产物/文档身份保持，无遗留实际仿真，本阶段只有CPU最小修复+一次归档两时刻对照，无新sim/grid/Hybrid/训练/视频/成功率。Projection regression PASS；实际闭环/反向段完成/导航尚未验证，原35s timeout3.671378/RMS.572390/peak1.105993/0contact以及rootcandidate拒绝保留，较强guarded B0未替换。全研究NOTcomplete/automationACTIVE，常规安静。

唯一修复：已知当前reference方向!=accepted时保持当前idx并返回真实距离，否则调用原projection。原act/handler/five-low .05m/s五10Hz与反弹、速度/PID/BankMap/协议/smoother/planner/rootcost/guard/相机照明保持。正常初始化无pending仍能进入反向交接，资格后恢复原投影；合理新reference可取消过时pending，不永久锁旧路线/不强制提前反转。独立with_cusp_boundary基类仅覆写project，make_cusp_oracle保持actualwrapper组成，CuspProjectionPolicy继承RootDirectionPolicy仅episode reset装配，不action时reset。非法accepted投影拒绝；旧rootplanner入口None Minor仍未修，不把这一层检查冒称全部入口修复。

11真实fixture初callablebaseline7failed4passed RED1.25→11GREEN1.23/fullH195PASS8.72s；双向短F/R/F复现、实际act五低速/反弹/首次无pending/资格release、正常同向完整act/raw/protocol/diag精确、合法geometry replacement/实际MRO/非法None均覆盖，无planner或sim。109源8487旧/case输入于08:53:56.762833 presealed，一次2arms×2currentcalls08:53:58.904760→08:53:58.951319exit0。初态frame180过去9policy资格字段+121pastmapping、175已知参考、181/182当前观测；不是fullphysics/PID/smootherclone。原臂两请求/diag EXACT，181共同状态/动作同，182实际速度.231750候选保持idx1/pendingR/low0/zero，原idx3/pendingNone/main+.5/auxraw-.582204 protocol-.523599。分叉182即结束，不使用183+旧实际轨迹当候选labels；只是raw/protocol改变，不claimapplied/停稳/实际reverse完成。

作者2026-10-05T08:55:18.621803+00:00 hash/scalar/chronology/source检查109/8487、4calls/两原请求/共同前缀/新索引层语义与assessmentSHA；没有再act/仿真。决策图是已知参考和共同actualXY，视觉已检查，无候选后续运动。ONEfreshreadonlyreview范围/发现/每条declined裁决见cusp_projection_review.json；审查bodySHA 371ca02bcca7f03dfb6e82b6e4f6b0d85538df0b9470bb1a2123102e846d6122、作者最终报告SHA 91c88f066c63ff2a7146837c071c997d95a19b83ac11e98d25aedc285f419522；作者append/finalchecker单独归属未二次review。全部源码/case/日志/图/协议保留，无无关Gitcommit/delete；旧sourcegate/M2/RGB/stop拒绝与rig不变。

接续尚未实施的一项实际工作：预封存唯一CuspProjectionPolicy同初态/同6s前缀/原35s goldenE1，对归档root_direction_run仅projection改动因果比较，并同时报告较强guarded B0；胜过更差比较器不是baseline准入。原rootcost/geometry/mapper/PID/speed/guard/five-low/rig保持，首分叉前完整物理/关节/执行/history/SHA核验，之后测真实applied/XY及主轴方向/是否实际通过交接段、独立参考进度/RMS/检查点/goal/contact。CPU修复不保证实际primitive完成；资格后仍用原nearest投影，未证所有短段闭环完成。导航必须goal<=.2原35s0contact，旧限定改善阈值不放宽；失败如实拒绝，不扫参数/网络/重做矩阵。RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2尚未准入，本CPUheartbeat不再actualrun，全研究继续ACTIVE。


## 09:45UTC cusp projection actual35s — final stage closure

先完整读 `reports/benchmark_cusp_projection_navigation_20261005.md` 与独立冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-cusp-navigation*.md`。ONE同起点goldenE1原35s于09:49:35.045858→09:52:28.764418UTC结束exit0，无遗留仿真/第二候选，旧root/guarded不重跑。接口PASS，限定改善对两臂均REJECT，导航REJECT(timeout35s/goalremain4.756419/0contact)。全研究NOTcomplete/automationACTIVE，常规安静，不替换较强归档B0。

只安装冻结CuspProjectionPolicy投影边界修复：未accepted当前方向期间保持索引，资格后原投影；legitimate replacement仍取消过时pending，不强制提前反转。外层继承RootDirectionPolicy act/plan_boundary，只episode reset换tracker；inner真实ConfidentSpeed/MappedBankOracle/CuspBoundary/Oracle原act/handler。runtime5处机械diff；原planner/rootcost.5/reverse1.25/node3000/geometry/guard/mapper/PID/speed/five-low/rebound/rig/light保持。仍是双rendererdepth+knownpose/未知风险，不是RGB估计pose/认证安全，outerprivilegedFalse不洗掉sensor privilege；旧RGB/sourceconsumer/M2/停止包络拒绝保留。

119源/8509输入09:49:05.318957presealed，实际350请求351双方capture1751physics2101contactsamples0actual，两视频351帧无尾步/hiddenmarkers/0age，2752关闭后manifest含feedback。首分叉182/18.2s相对root：911physics17nonraw/183execution+来源historyEXACT，raworigin-.02s/-1tick另报；141rawblocks840appliedticks改变。不能称与guarded到182全共同前缀，它与root此前175已经分叉。

四次19.7/23.7/29.7/33.3 accepted F/R交接，均真实五个当前10HzXY<.05，原反弹语义保留。180负rawticks278负applied428实测main负各自分母；197acceptedR时main还+.040957/previousapplied0，215main-.460650/previousapplied-.5/XY倒车。version3/6各约.299mR段，自己reference寿命内实际最接近终点.044042/.013268m、有向投影max.359736/.390492m，不能仅用index或accepted认证primitive。离线描述auditpostseal不改control/gate，lifespan负ticks不专属单primitive。

独立全参考终态progress3.707562、RMS.135045/peak.260893；相对rootprogress-1.682213/goalremain+1.085041，相对较强guardedprogress-1.148014/goalremain+.599439。虽然误差/0contact三项过，进度/goal两项失败，两臂gate均拒绝；4m检查点27.4s，6/8未实际到达。74direction-change零块=7.4s，合理停稳不当bug；25/25.5s observed-obstacle route失败、26s新F/R/F导致第二循环，不把首交接当全部失败。19.7→21.5实际yaw-1.178943→-1.224909；23.7前向接受时reference-.737970/actual-1.198351误差.460381，24.9仍.433406。是真实欠对准证据，未证PID/符号/映射单因果。

作者2026-10-05T09:54:10.961008+00:00核验119源/2752actualSHA、350causalpolicy/map/reference/request精确、351双方packet/350单次history、1745实测feedback执行tick（首5不clone）、3finitepastfuturetrap/truncatePASS；非递归物理预测/非3freshphysics。195/8.72与11RED/GREEN是前阶段冻结证据未冒称新跑；wall10Hz不成立，planp95 3.444s/capturepartialp95.311s另披露。继承SDKhiddenmarkerprimvarwarnings留rawlog，actualexit0不称blocked。原机械plot继承错label文件保留，独立postruncomparison_v2三实际臂更正且视觉检查。ONEfreshreadonlyreview范围/发现/每条declined裁决见cusp_navigation_review.json；审查bodySHA ef9f8d28596758816edd89ce202ad76cdf21c019cdeb1f3c4b0fc070af81c33a、作者最终reportSHA a1f061300c41dd480eeee28bb8b13787454ed4ce5304fc37df6d36a883d5979a；作者append与最终hashcheck不冒称二次review。保存全部来源/数据/失败/测试/视频/版本，无无关Gitcommit/delete。

下一项尚未实施的有界CPU工作：针对version3/6实测R tracking和资格后F tracking，核验reference body/profile与PP/feedback实际曲率合成、V1请求方向、roll反馈/PID/applied、实际yaw/XYcourse，尤其23.7/33.3欠对准和短R名义yaw与实际变化；字面直线/圆弧/正反向/连续性fixture，不二次act/不拿braking陈旧值/不默认符号bug。明确错误才选择唯一传统tracking最小修复，无错则拒绝该假设并隔离动态欠对准/来源route阻塞证据，不扫PID阈值/换M2M3网络。实际候选需另事前封存，同初态35s与原进度/goal/横向/contact gate；本heartbeat不追加search/sim。RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2仍待分层推进，全任务未完成，保持ACTIVE。


## 10:45UTC outgoing-edge lookahead CPU audit — final stage closure

先完整读 `reports/benchmark_outgoing_lookahead_and_heading_chain_20261005.md` 与独立spec/plan `docs/superpowers/{specs,plans}/2026-10-05-outgoing-lookahead*.md`。只有CPU最小query修复+一完整归档审计，无新sim/search/fit/inference/视频/正式successrate。CPU query regression PASS，实际新候选导航未测；旧35s cusp timeout4.756419/进度3.707562/RMS.135045/peak.260893/0contact与两对照限定改善REJECT不改。全研究NOTcomplete/automationACTIVE，常规安静。

确定outgoing convention bug：planner _reconstruct在i点赋actions[i]，direction是i→i+1出段；Oracle lookahead先看nextdirection故丢掉当前最后edgeendpoint。version3/6单R段旧targetidx1本身/累积0，实际197PP+17.088480而原名义Rbodyk-.956781/yaw+.287034；还有175首F0→1R也旧targetcurrent导致PP0。独立with_outgoing_endpoint仅先消费CURRENT edge再到nextendpoint；不会进对向下一edge/不改pathidx或latch。make_endpoint_oracle包冻结actualclass，EndpointLookaheadPolicy继承CuspProjectionPolicy只episode reset装配，act/rootplan/原projection/five-low/rebound/PID/BankMap/速度/协议/smoother/planner/mapper/rig/light原样，未bundle入口None Minor。

15字面F/R直线圆弧±k/小误差/边界与终点/完整同向act/五低速反弹真实fixtures，callable RED13fail2pass1.39→GREEN15pass1.36；fullH210pass9.18。初写cwd失败missingfileexit4不当RED，初fullsuite test_board_material缺libtf.so collectFAIL后加已有SDKLD库全pass，fail logs保留无testdisabled。v1审计10:56:13.960111封存125/11310，frame74用了float64diagnostic速度代替原actfloat32proprio norm导致5e-9yaw差/1e-10assertFAIL，无输出data。旧源/protocol/fail日志不改；独立v2只恢复原CURRENT dtype层语义、不放宽gate/不改control，10:57:51.417566封存126/11312。共2尝试(1不完整fail+1complete)明确披露；完整归档审计10:57:52.429785→10:57:56.377453exit0，不冒称singlelaunch。

350records188有效tracking/162无效或早return，188原target/PP/V1inverse/BankMap/raw/protocol全一致，1750旧actualbank/PIDtarget/smoothed/applied与physics逐tick一致；非递归物理预测/不重放policy.act。61zero query改正52raw/32protocol/5main当前代数改变，既有speed策略因新k触发不是新speedpolicy。首次代数分叉175，其他188行各自条件旧实测状态what-if，不是candidate真实trajectory/future标签。R3 |PP|median17.4087→2.3645仍19/19clamp；R6 26.4472→.5216clamp15→6；后F13/17queries仍same/clamp。不能把raw不同且protocol相同当执行改善，不能以counterfactual夹边少称导航成功。

真实旧R3Euler yaw-.053950/world-z integral-.057851与名义+.287034不同，但rootEuler/worldz非同量；bank是asin2(wx+yz) bodyY worldZ，非完整Eulerroll，原PID实际量归属明确。旧R3bankerrRMS.452392/PIDclip100%/supervisor3.16%，F3 .353504/78.46%/4.62%；低supervisor比例不能证限制无影响或删除，PIDclip不能独断gain不足。已知上游target错误可以改变bank请求，欠对准/routeobservedblocked仍有其他因素。已检查图只CURRENTknown refs/实际positions/查询目标，无candidate未来运动。

作者2026-10-05T10:59:27.095315+00:00无science重算hash/唯一行/计数/时序检查126源11312inputs/188rows1750feedback；ONE fresh readonlyreview独立范围/发现/每条declined裁决见outgoing_lookahead_review.json。审查bodySHA 476a36f4fda57a81861e0ef4d2299c3c2a40ba24821045ce98c71d3f25142055，作者最终reportSHA 5bf025a06663ed9d8a5ff7c1e9ac70729832ac3eeb2e5c61e85a6e23a936aea5；作者append/finalcheck不冒称二审，全部源码/REDGREEN/suite/failure/v1v2data/图/旧视频/身份保留，无无关Gitcommit/delete。自动续做仍ACTIVE。

下一阶段唯一未执行物理工作：预封存EndpointLookaheadPolicy同起点/同6s前缀/原35s goldenE1，相对归档cusp solequery因果对照，同时报告较强guarded与已拒绝root。相机/地图/rootgeometry/gains/PID/speed/handler五低速反弹/原acceptance均保持。共同完整physical/joint/exec/smoother/history/source到真实firstdivergence(不能假定197，应可能175)，之后只测candidate自己的applied/XY/bank/yaw/出段/独立进度/RMS/peak/检查点/goal/contact，原改善5项/35sgoal<=.2 0contact不放宽，失败直接reject不扫PID/阈值/网络。CPU修复不等于navigation，RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2未准入，全任务继续ACTIVE。本CPUheartbeat不再phys/search/fit。


## 11:46UTC outgoing endpoint actual35s — final stage closure

先完整读 reports/benchmark_outgoing_endpoint_navigation_20261005.md 与独立spec/plan docs/superpowers/{specs,plans}/2026-10-05-endpoint-navigation*.md。冻结EndpointLookaheadPolicy查询已真正运行。仅query改动 vs cusp causal comparator；guarded强对照和root拒绝臂同时报告。全部旧源/rig/阈值/gate不改。一次预采集sandbox GPU/CUDA/Vulkan startupSIGSEGV失败无physics/data/metrics，原空namespace/status/log保留；事前GPU环境-only addendum/获权限后原runtime重试，2launch attempts/ONEcompletedphysical sequence，非两条导航/非调参。

137source/11342inputs于11:49:58.157440presealed；fresh15fixturePASS1.40s，旧RED13fail2pass/GREEN15/fullH210原证据保留未重做matrix。唯一实际goldenE1 2026-10-05T11:51:16.855484+00:00→2026-10-05T11:54:05.194954+00:00exit0，6s观察包含原35s/350requests351captures1751physics2101contactsamples0actual，两视频351frame10fps无tail，hiddenmarkers/0age。2762postclosemanifest含PID。runtime机械diff只policy导入/装配及artifactnames，实际MRO Endpoint/ConfidentSpeed/MappedBankOracle/CuspBoundary/Oracle，原act/handler/five-low/rebound/rootcost/geometry/planner/mapper/PID/speed/auxlimit/rig/light全部保持。仍特权双depth+knownpose/source memory/unknownrisk，不是RGB估计pose或3D动态safe。原RGB/M2/停止拒绝不倒改，全研究NOTcomplete/automationACTIVE。

first REAL rawdivergence175/17.5s：与cusp此前876physical×17nonraw/176fullPID/smootherexec+bilateralsource/history精确，rawclockorigin0/0tick。102rawblocks875appliedticks改变，行为非±35→30等价。之后只候选自己的actualfuture为标签，非旧188what-if或旧cusp未来轨迹。作者2026-10-05T11:55:27.055915+00:00核源137/实际2762，350因果policy/map/reference/request、351双方packet/350history移位、1745实测feedback执行（首5未clone）、3finitepastfuturepayloadtrapPASS；非递归physicalprediction/非3freshphysical。

真实进度5.028426/距goal3.961658/RMS.171278/peak.384271/0contact/timeout35s。相对cusp进度+1.320864/goal距离-.794760，但RMS/peak更大，原限定5gate拒绝；相对较强guarded进度+.172851/goal-.195321/RMS-.002789/peak-.090995，5gate均PASS；相对root进度/goal失败。联合三臂限定改善REJECT；实际goal<=.2原35sgateREJECT，不能装成成功baseline或新E0E1E2成功率。4mcheckpoint18.6 vs cusp27.4，6/8m未过，6mmin1.147126。全参考评估器独立候选索引，停止不当成功。

仅20.1F→R与23.7R→F两交接，各原五当前10Hz XY<.05，实际90negative raw/139applied/230mainjointtick(1750tick各层不同)。方向brake38blocks3.8s vs旧74，合理制动保留，不强制反转。version3单R段.298971m实际lifespan到335，minendpoint.033647/signedmax.342228连续描述，不由accepted/index另造成功gate。R20.1→21.9Euler yaw+.048570/worldZintegral+.060030，bankerrRMS.415646/peak.610090/PIDclip68.89%。相对旧负yaw只是不同完整轨迹，不单因果标定PID/sign。F23.7→35实际沿独立ref+1.565348m，横向.155513→.384271，currentbodyheading误差entry.292202/peak.528624/last.056954；bankRMS.126795/PIDclip73.805%。234validmoving protocolclip127(54.2735%)/V1outside103(44.0171%)，kmedian.812188。仍有正常同方向饱和欠转弯与完整35s预算问题，不默认首交接解释全部失败。

描述性headingv1误读legacy数组恒0，原script/sidecar保留并拒绝；独立v2只读取actual tracking.reference_body_heading_rad/验证每validrow同reference/projection及实际rootheading，无效earlyreturn None，原控制数据不改。PIDraw/protocol中间输出在原feedbackbanklog，完整v2/phase表单独保留。Euler/worldZ不同量。计划wallp952.814430s/capturepartialp950.303882s，因果在线不是real10Hz。4计划ALLsuccess，33.5originalrasterclearance重建，没有旧25/25.5routefailure那种再次循环；旧数据不能给新动作未来标签。

全部源码/preseal/diff/失败/环境addendum/fresh测试/新actual数据/PID/视频/图/hash保存，像素已检查。ONEfreshreadonlyreview与每条declined裁决在endpoint_navigation_review.json；作者append/finalhashcheck另归属，不冒称二审。旧报告/结论/验收未倒改，无无关Gitcommit/delete/upload。阶段是否已完成以endpoint_navigation_final_check freshcheck为准，整个项目未完成，不停止自动续做。

下一项未做的有界CPU工作：复用本次237→335正常同方向实际data，核验planned/profile k与实际PP、BankMap当前speed支持夹边、PID动态bank、真实XY几何曲率/航向响应及33.5rasterguard失败的真实位置/已测map来源。只读当前/过去knownreference，不重复标定/换M3/扫增益/减速/裕量，不先验认定PIDbug；明确一个可隔离消费约束后另预sealed最多一条物理对照。本heartbeat不再physics/search/fit。可靠RGB+estimatedpose、唯一学习记忆/世界模型、原35s正式E0/E1/E2仍待分层推进，全研究继续ACTIVE。



## 12:46UTC tracking capacity and guard-source audit — final stage closure

先完整读 reports/benchmark_same_direction_capacity_and_guard_source_20261005.md 与 docs/superpowers/specs/2026-10-05-tracking-capacity-audit-design.md、plans/2026-10-05-tracking-capacity-audit.md。本轮只完成一个预封存CPU测量/来源审计，不新增物理导航、planner search、policy act、模型拟合或控制候选。144sources/14148旧与实际inputs于12:56:02.888173UTC冻结，一次完整审计12:56:18.580804→12:56:19.612774退出0；RED4fail2pass.16s→GREEN6pass.12s。首次cwd写入失败无文件/无science，保留日志。原35秒结果、rig、mapper/PID/speed/guard及旧RGB/M2/停止拒绝不改。

237..334的98个有效同方向tracking：名义main+.5/aux-30°的V1机体曲率+.880052，而当前速度插值BankMap正端点.796201..796603；98/98名义超过加权端点、98/98正式map夹边、98/98协议夹边，80/98速度超出表域被夹边。加权端点不是各层逆映射顺序，也不认证动态物理上限。490真实PIDtick377clip/5supervisoractive；早期倾侧滞后明显，334实际bank-.393705已接近目标-.392699，仍请求k1.660542/离参考.425525m。不能单凭PID饱和认定增益不足，不能说限制无作用或删supervisor。

10个不重叠测量窗全部可估计。真实XY三点几何曲率与root Euler yaw/arc、world-z积分/arc分别报告；早期XY曲率负并不自动证明打滑/符号bug，后期bank达到不证明恒曲率稳态。窗口后续实测只作离线描述，不输入在线预测。数据、唯一行/分母、图、PID与当前reference字段源均保存；禁止使用legacy全零heading数组。

33.5s原guard的4个past/current位置330/335×map330/335组合，位置330均保留，335均因原rasterclearance拒绝；只说明这个局部screen，不证明map全程无影响。原连续suffix/bridge距离.640678893>=.6，原rounding EDT.583095189<.6，规则如实触发重规划而非碰撞终止。两失败sample是同一投影点，round位移.058137568m；最近hazard为历史code4、timestamp13.099999707/frame131，来自3个原实测depth点，高度.528327..643834。hazard不受floor10秒TTL删除，不是335当前视觉/脚下先验。本层是特权depth+knownpose，不能改写为纯双RGB性能。

作者12:59:10.556788UTC核144/14148及全部保存计数/来源时序，无science重跑；两PNG视觉核验。本轮无新视频，旧endpoint两条实际视频作为上下文。ONE readonly第二人审查见tracking_capacity_review.json。新fresh-context dispatch因agent thread limit被拒绝，实际复用上一阶段审查者并公开保留上下文；不称fresh或二次独立重跑。正文与审查/作者追加分开，作者append/hashcheck未二次review；最终审查发现与逐项scope裁决以附录为准。保存所有代码/失败/数据/图/SHA，无无关Gitcommit/delete/upload。阶段完成以tracking_capacity_final_check为准；整个研究仍未完成、automation ACTIVE。

下一项尚未实施：ONE有界CPU规划名义响应/控制consumer一致性实验。先审查原V1/BankMap曲率定义、命令域与steady nominal适用性；在一个完整已知当前快照（可用175首次重建前，不取未来335结果设计输入）上只改变一个名义曲率响应来源或必要screen，原搜索/mapper/PID/gains/speed/rig/guard不动。比较同起态原计划与唯一候选的几何/goal/clearance/长度/换向数/当前请求域支持，冻结接受门槛后最多一次搜索对照；不把当前速度预测全部未来、不用旧实际轨迹当候选label、不强制禁reverse或循环等待。CPU资格通过才另阶段最多一条原35秒同初态实际对照。当前不再science/search/sim，RGB+estimatedpose、唯一学习记忆/世界模型与正式E0/E1/E2仍未准入，不宣称导航成功率。



## 13:47UTC nominal support CPU comparison — final stage closure

先完整读 reports/benchmark_nominal_support_and_planner_consistency_20261005.md 与 docs/superpowers/{specs,plans}/2026-10-05-nominal-support*.md。一个独立封存CPU候选：只在planner使用DriveModel子类，将V1曲率裁到BankMap相应方向low/high共同数值域，不装进tracker，不改PID/speed/guard/.6/rig/五次停稳/35秒gate。forward范围[-.8272253275,.7815023661]，reverse[-.7978797555,.7672796845]，不是动态物理上限。历史/当前baseline导航失败、RGB/停止/M2拒绝保持。

11fixture callableRED7fail4pass.29s→GREEN11pass.25s，完整H227pass9.43s。两次不同cwd/relative source或行政写入错误在science之前，无运行结果；第一missingfile log封存前被正确RED覆盖，工具失败转录另存，不当原pytest证据。152sources/14187旧与实际inputs于13:57:06.100102UTC封存；明确一次sharedgridroute+两次HybridA*，同完整frame175/map17.4999996088s、当前pointgoal/XY/yaw/acceptedforward，execution175身份另存无未来物理输入。

唯一completed CPU comparison13:57:43.268009→13:57:51.059837exit0。原参考175 pose/command/direction/expanded2434精确复现；候选2482nodes。原10primitives/F1R1F8、path3m/objective4.075/localgoal.018737/两换向/7域外，候选12/F4R8、path3.6/objective4.7/localgoal.174290/一次换向/0域外。两臂raster samples均0occupied/outside，共同名义域candidate三次mapquery各曲段均no curvatureclip。原代价/路径只允许+.3m的冻结门槛均FAIL，所以CPU资格REJECT，不触发物理A/B，不放宽、不续扫。当前共同175PP+.622023→-.400175、两map均no curvature/speedclip，只是参考当前几何查询，非fullscheduledraw/protocol/smoother/applied或candidate实际未来。

定义来源postresult只读核验：activev9表与v2bank_mapping JSON逐字节相同，v2fit/measurement用Savitzky-Golay XY几何k*sign(main)定义body-named k，bank输入是请求参考，不能叫root Euler yaw/arc响应。当前V1消费者/静态payload明确，但历史2026-09-12生产者身份未证，当前probe同时有legacyyawratio与newXYgeometry；不借当前源码认证历史算法。生产者来源独立SHA保存，不重跑fit或标定，也不倒改旧source/report/gate。

作者13:59:48.329266UTC核152/14187及计数/gate/时序，assessmentSHA188a81f06e8796ae459c38856aed84c14776a3e55b51ec8642ba8acd3bc44aef，无science重跑；PNG已查看，无新视频，旧actual35s视频上下文。ONE fresh-context独立审查见nominal_support_review.json，评审正文和作者追加分开；作者收尾/hashcheck另署，不称二审。全部源码/REDGREEN/失败/完整现状/两名义路径/数据/图/身份保留，无无关Gitcommit/delete/upload；stage完成以nominal_support_final_check为准，整个研究NOTcomplete，automation ACTIVE。

下一项有界工作：追踪已经归档成功Oracle time_planning/hybrid与当前local direction_root length-cost的真实消费入口/继承/units/速度来源，针对同一个已知完整175快照做独立冻结的路线时间预算核验。距离/倒车代价不能代表实际35秒执行仅是待检验假设；本轮没有证明它是实际导航主因。先明确steady nominal与交接延迟/实际XY速度条件，不能把静态时间认证为动态期限。仅证据充分才试一个已有时间代价消费改动，保持原V1/mapper/PID/speed/map/rig/guard，不将已拒绝cap夹带安装、不同时更换曲率来源。不拿旧B0未来轨迹给新计划标签，不重复本轮支持域扫描；任何新物理候选另封存独立CPU资格后，同起态原35s actual进度/goal/RMS/peak/contact对照。RGB+estimatedpose/唯一学习记忆或世界模型/正式E0E1E2仍待分层准入，不报新导航成功率。


13:47UTC行政身份补充：首次finalchecker旧文档路径比对失败；原字节存pre_close，独立v2别名校验与当前文档分开。nominal_support_final_identity_v2/document_sync_v2与final_check.json为最新行政证据，v1保留；所有科学结论与拒绝不变，无science重跑。


## 14:47UTC time objective source audit and single candidate — final stage closure

先完整读reports/benchmark_time_objective_source_and_candidate_20261005.md 与冻结docs/superpowers/{specs,plans}/2026-10-05-time-cost*.md。本次ONE CPU候选仅替换base edge为 L*vmax/max(.05,V1speed)，normalized米单位；原42V1几何/heuristic/request类型/accepted-root/reverse1.25/switch.5/node3000保持。没有旧.75filter、13:47已拒绝曲率cap、tracker/PID/speed/rig/地图/guard或换向五低速反弹改动。旧RGB/M2/停止/全部导航拒绝及原35s gate不追改。

实际callable RED9fail1pass.31s→GREEN10pass.26s，全H237pass9.44s；真实tinyfixture代价改变main1.5→.5但同端点，非实际机器人数据。162source/14200inputs14:59:55.445625UTCpreseal；旧mutable文档输入采用既有archivealias/currentdoc另存preseal字节，从开始分离避免修改旧SHA。ONE已知current175/map17.4999996088s/currXY-yaw-pointgoal-acceptedF，同2.4m目标/原.6clearance/±4crop，1sharedgrid/1newcandidateHybrid/0B0targetrepeat。候选14:59:58.491629→15:00:02.239808，完整audit14:59:56.519387→15:00:02.241984exit0。候选node_budget_exhausted3000无完整路径，所有success-dependent预sealedgate失败，CPU资格REJECT；0physics/0act/0fits/0rescue，不加节点/换heuristic救分，不触发正式E0E1E2。

归档B0与原actualreference175精确：2434nodes/10primitive/3m/4.075distanceobjective/2switch/7common-domain-out/.018737localgoal。weighted normalizedarc proxy13.693638/unweightedarc11.580005/archivedchooser chord+2 13.549481，当前35sremaining17.500000391；仅局部goal且未包含真实执行动态，不证明全局35s可行，不证明缺timecost是导航失败主因。failed空result path0/switch0不可作成功量。历史14source当前12match/ninewrappermatch/two corechanged，历史整套runtime不认证；旧timeplanner额外.75curvefilter/root缺失/heuristic秒转换不能单层移植。chooser episodeinitialfull-map fixed25s筛baseline，任何success timefallback不再检查25，字面actualfunction26/28选28、20/18保留20仅语义测量，不称当前剩余预算bug。

作者15:01:54.224984核162/14200/机械diff/typealiases/42primitive几何/代数/时序PASS，无science重跑。作者verifierv1_unused nonexistent import行政FAIL源/log保留，独立v2只删未用import；科学/协议/gates不变。PNG已查看，无新视频，原endpoint actual视频仅context。ONE第二人readonly审查0C0I0M，个人10fixturePASS.25s/162source14200input/三代理/图；freshdispatch因threadlimit拒绝，实际保留前nominalreview上下文公开，不冒称fresh/全suite重跑/二次review。20declined scopes与作者逐项裁决见time_cost_review.json/报告作者附录。Python3.10tomllib行政read失败转录保留，unique-status-line只读实际ACTIVE确认；设置未改。审查bodySHAe4dc994b4427aa0ea8a3836f8d193e395fde0664c96b1c42e345305e153c0874，最终报告/源/图/测试/失败/输入SHA在time_cost_final_identity/document_sync，freshfinalcheck决定stageclosure。阶段plan/spec保持原sealed字节，Taskcompletion另入time_cost_tasks.json；无无关Gitcommit/delete/upload。

下一项尚未执行：关闭当前timecost候选，不继续same175搜索救分。以已有BankMap完整生产者与实际endpoint/开放平面数据核验原V1名义响应与当前roll-PID编码bank请求链的系统身份/单位/稳态条件/XY响应，列明确闭环覆盖缺口。仅确有未覆盖项才预sealed一条必要完整roll-PID运动序列（最多一条，不重做矩阵）；不能用Euler yaw替代XY曲率或宣布BankMap共同域动态安全。然后只选择一个有测量依据的名义响应消费环节改动，未证匹配不移植。原Fast直接融合/causal多视角/ORB来源拒绝保持，不重复RGB矩阵，不把先验写成观测；可靠RGB+estimatedpose/唯一学习记忆或世界模型/正式原35s E0E1E2仍待分层完成。全研究NOTcomplete，automationACTIVE，常规安静。当前阶段不再science/search/sim。
