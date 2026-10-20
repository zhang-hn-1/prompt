# 原稠密双目域适配 Implementation Plan

> For agentic workers: REQUIRED SUB-SKILL superpowers:executing-plans, inline; ONE final fresh readonly review.

Goal: 一个原Fast decoder域适配候选及独立whole-sequence B0比较。
Architecture: 固定backbone/cost与rig，纯fit标签→唯一decoder权重；独立RGB-only预测→保存所有输出→offline evaluator。
Tech Stack: current torch/conda/officiallocalFast+Isaac，不安装升级。
Spec: docs/superpowers/specs/2026-10-06-dense-stereo-domain-adapter-design.md
Global constraints: ONEprobe/ONEfit/ONEfresh32s yaw−20 sequence，300attempt lr1e−5/8iters/max416/32examples，原camera/light/controller/gates冻结；留旧字节。
Review Focus: wrong-right flip/sign; invalid/selfmask/occlusion labels; trainableprefix/frozenbuffers; heldout complete separation; predictions no labels/pose and before evaluator. 以下Task1 tests及Task3身份与sequence checks覆盖。

## Task1 Labels/fit helper and real gradient
Files H/dense_adapter.py,test_dense_adapter.py,probe_dense_adapter.py.
Interfaces training_target(depth,partner_depth,masks) returns rectifiedpositiveD/valid,select_trainable(model) freezes all exceptthree prefixes,sequence_loss(preds,target,valid).
- [ ] Literal RED: synthetic disparity2 with occlusion/bounds/masks, right mirror,invalid; toy parameter/buffer closure and maskedloss gradient.
- [ ] Minimal helpers GREEN then Hsuite.
- [ ] Preseal source/oldoneRGB/label before ONEactualgradientprobe; no optimizer or savedweight. Failreject; successactualgrad metrics.

## Task2 Fresh whole visual collection
Files H/run_dense_adapter_heldout.py,stage wrapper,collection protocol.
- [ ] mechanically copy existing interval/semantic collection; onlynew namespace+E2yaw−20; noextraIMU/noPIDchange.
- [ ] presealcode/runtime/config/schedule and originalgains/scene identities.
- [ ] ONE actualGPU; verify65frames/321capture/1601physics/currentrender/clocks/twovideos/whole-sourceSHA; sealbeforefit.

## Task3 Sole fit, complete predictions, scoring
Files H/fit_dense_adapter.py,predict_dense_adapter.py,assess_dense_adapter.py.
- [ ] build32 fitexamples oldonly; labels target tests. Freeze wholefreshdata/spec/hyperparam/source/transitiveevaluator/mask/officialweight identity BEFOREfit.
- [ ] ONE300attempt fit finalweight only; immutable oldcheckpoint; verifyfrozenparams/buffers EXACT and validfinite losses; actualprogress saved.
- [ ] ALL65 pairsB0+candidate RGB-only preds save beforelabels; SAME bilateral/selfmask/gates.
- [ ] scorefullquery groups originalaudit semantics, gateprecision/coverage/p90 literal and insufficientrows; onecause diagnostic/PNG+fullvideo decode.
- [ ] WholeHsuitetest; actualdata report, onefreshfinalreview/regrade+authorappend, masterprefixarchives+append; no repeatedfit/candidate.
- [ ] Continue nextmission task as evidence allows, no stage-only stop, concretecontinuationifexecutionlimit.
