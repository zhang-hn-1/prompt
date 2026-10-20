# ONE first-pass CAD-masked stereo input review

This reviewer reuses the existing static-scene/terminal reviewer context. It is **not a newly blank context**, and neither earlier review certifies this new branch. The reviewer did not implement or previously review this CAD-input branch. This is its ONE first-pass independent review; no second review or reviewer was dispatched.

Scope: reports/benchmark_cad_masked_stereo_input_probe_20261007.md, the matching plan, probe producer, input helper/tests, sealed protocol/predictions/assessment, and the bounded retreat analytic depth check and its preanalysis/diagnostic. The learning branch remains excluded. Only this review and cad_masked_input_review_personal_checks.json were saved. No GPU, new inference, simulation, controller/source change or gate change was performed.

## Findings

- Critical: 0.
- Important: 0.
- Minor: 0.

The retained REJECT result agrees with the saved evidence and the pre-frozen local criteria. It does not certify all-domain source reliability or navigation.

## Personally checked

- `mask_stereo_inputs.py:4–8`: same-shape uint8 RGB and matching boolean masks are validated, both images copied, only their current CAD-invalid pixels zeroed, other pixels and inputs preserved. Personally reran all **six helper fixtures: 6 passed in 0.16s**.
- `probe_cad_masked_stereo_inputs.py:9–18`: exactly seven fixed historical/development pairs, two source prediction arms per pair, one input change. All **1280 source hashes**, **51 input hashes**, **28 prediction/packet hashes**, **four analytic-input hashes**, and **five author-closure hashes** match. Frozen checkpoint and stereo/source code identities are included. No old source or scientific gate was rescored.
- Personally compared all seven newly saved unmodified baseline depth/confidence/disparity arrays with historical B0: **21 arrays exact**. This independently strengthens the producer's equivalence check; there is no changed baseline in the measured comparison.
- `probe_cad_masked_stereo_inputs.py:20–31`: current CAD masks rebuilt from the saved current joint history match **14 eye masks** exactly. Historical cached source packets reproduce **70 fields**; the CAD-only output confidence filter preserves the same **70 fields**; both saved arms reconstruct from their sealed predictions into **140 packet fields** exactly. Only current saved poses/root/joints are used in those conversions. The CPU reconstruction never instantiates FastStereo or calls a network.
- `probe_cad_masked_stereo_inputs.py:33–45`: personally checked protocol UTC < prediction-seal UTC < assessment UTC, and all sealed files still match. Reviewed the frozen producer's order: GT file bytes are hashed for identity before inference, but GT arrays/values are deserialized only after all **14 predictions + 14 packets** are saved and sealed. Divider geometry is also read after that seal. Hashing the GT file does not feed its values into inference. These are local execution provenance records, not an external timestamp attestation.
- Independently recomputed every baseline/candidate query count, finite-label count, unconditional correct fraction, divider-proxy correct count, and each local gate. All seven assessment rows match exactly. Failed/nonfinite truth queries remain in the emitted-query denominator; empty output has correctness fraction0 and fails the near coverage criterion. Confidence/post-filtering still determines emitted queries, so this fraction is not dense image coverage or all-surface accuracy.
- Near12/18s candidate outputs are both empty, failing .80 correctness and >=25 correct divider source queries. Near6s emits309 queries with0 correct, which remain failures. Far0 precision drops .9483985765→.8903166590 despite a larger correct count; far6 and11 correct counts drop2914→1611 and3910→3517 despite improved fractions. All five scored local gates fail. These local diagnostic gates were frozen before this candidate inference and remain separate from the old all-domain source-admission rejection. The result correctly admits neither fresh source validation nor navigation.
- `audit_retreat_analytic_depth.py:4–12`: personally opened the frozen source USD offline, confirming units1m/Z-up and all **11 divider cylinders** with exact authored world centres, Z axes, radius.35m, height1m and identity rotations. Independently intersected all412 saved18s SGBM rays using a separate cap/side solver (`scene_ray_geometry.cylinder_depth`), with literal cap/side fixtures checked in this pass. This does not borrow the earlier static-scene review's certification.
- Independent analytic result: **412/412 finite**; renderer-versus-ideal median/p90 **.01167952744483414/.015454341917735332m**, 176 within1cm; SGBM-versus-ideal median/p90 **.19151736592313784/.20337465434477509m**, **0 within.10m**. Metrics match the saved diagnostic within1e-13m. The axial ray parameter and Euclidean discrepancy conversion are consistent. This is an ideal finite-cylinder sanity check on this group, not full scene occlusion, exact tessellated mesh equivalence, or all-domain renderer certification; the report preserves those limits.

## Inherited evidence

The original RED import failure, author GREEN0.15s and original GPU execution are read from retained logs and the hashed producer. They were not rerun by this reviewer. The original GPU calls' preprocessing and pretruth order are inferred from that sealed code/control flow and its saved products; this review cannot directly observe the past calls. Personally reconstructed packets and scores establish consistency of those saved products without another model call. No earlier retreat or terminal review is used to certify this branch.

## Declined to judge

- Learning/reliability-head and learned-memory branch: explicitly excluded and still under separate verification.
- Fresh source-validation, navigation, formal E0/E1/E2, general source reliability and research completion: not triggered/admitted by this rejected seven-pair diagnostic.

## Assessment

Ready to retain as a verified, scientifically **rejected bounded CAD-input probe**: yes. Ready to adopt the preprocessing, promote source validation or claim navigation success: no. No Critical/Important fix pass is required. Whole research remains incomplete. No second/follow-up review is requested after this pass.

Detailed personal versus inherited evidence is saved in cad_masked_input_review_personal_checks.json.
