# SDD ledger — plan: docs/superpowers/plans/2026-10-07-prior-rgb-local-navigation.md
Ruling: user approved newtask and immediateexecution; no repeatedstageapproval. Original source gates frozenREJECT. New2mgoal task is distinct from completeE1/formal. Existing independentlyauthorized H is isolation boundary; readonlyP, no unrelatedGit/deletion.
Preflight interface: Fast finitebooldepth→point_packet worldsource times; sampledsource caches oldpose; mapper currenttime must preserve sourceage; planner grid labels nevertaskprior; actioncap identicalarms. Existing SpatialMemory permanenthazard semantics notused; readonly PointSupportMap exactTTL10s used forbotharms.


## 2026-10-07 已实际启动首个RGB+已知位姿局部闭环对照

用户要求现在开始。spec/plan 2026-10-07-prior-rgb-local-navigation已冻结：goldenE1同场景/起态、沿原goal方向2m局部目标（非完整E1），原35s含6s观察，固定FastB0每5capture预测，source/currentknownpose→PointSupportMap→旧Endpoint/rollPID；flatground/初始safe区分离标签，未知探索共享main±.3cap。仅current→10s-history记忆变化，最多2physicalseq/各<=71infer，无fit/改rig/light/controller增益/完整map输入。旧source/pose/完整nav拒绝保留。

5 literalREDfail1.81→GREEN5pass2.43，完整H366passed12.55s；1245sources/权重/脚本预seal，原goal定义与posthoc独立goal点合法性审查各自留证，不用地图在线挑路线。初行政goalchecker用systempython无numpy退出1，独立conda只检查同目标未改参数。GPU driver已真实启动，H/prior_rgb_navigation_status.json、prior_rgb_{current,history}_run.log和run_prior_rgb_navigation_stage.py为接续入口；检查实际session/状态/目录，勿重复启动。Parent按fixedarm顺序运行，runtime失败会停并保存原因，科学失败不换候选。完成后独立统计实际goal/contact/progress/动作差异、同步来源/GT边界与视频，不称纯视觉/实时/正式成功率。wholeNOTcomplete，automationACTIVE。

Task3 actualcurrent22.6s/goal.193029/contact0 PASSlocal; history35s/goal.597168/contact0 REJECTadmission. 5270SHA/117source576map3456arrays/fourfullvideos passed. Scientific causality limitation: firstbranch13.5s exactphysicalprefix but RGB outputs differ; pairedsource latesthistory stillFAIL/sharedcurrentprefixhistoryPASS, so no purememory attribution. Three diagnostic failure versions retained, no controlchanges.
Ruling: numeric runtime_error0.0 means noerror, fixed posthocassessor beforefirstexecution; gate unchanged, cost if wrong wrongpass. Ruling: reject RGBbyteidentity hypothesis afteractualfailure, no repeatedphysicalmatrix; add same-stream readonlyshadow next rather than claimcausalAB. Ruling: restore NPZ0Dframe→int in posthocloader; actualrunner inputs alreadyint; cost ifwrong lostreplay.
Taskscope/runtime details and originalgateREJECT remain in report. Tests366pass preseal source1245, nofullretest afterpureposthocfiles. Source/sourcepacket/mapreplay personal, review pending.

Final review C0I0M1; personal5fixture2.44s/fourSHA/JSON; inheritedfullauthorchecks, upstreamdeclinedacceptedscope. Final minor(deferred): stale causalstdout only, appendreportqualification; no rereview. Task3 complete narrowresult + rejectedcausalincrement, wholeNOTcomplete; next complete-goal same-streamreadonlyshadow in progress.
