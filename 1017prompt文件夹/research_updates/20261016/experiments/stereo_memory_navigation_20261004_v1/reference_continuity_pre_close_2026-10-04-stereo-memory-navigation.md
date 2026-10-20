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
