"""Paired actual physics/geometry and AFTER-DETECTION photometric diagnostics."""
from pathlib import Path
import datetime,json,hashlib
import cv2,numpy as np
from coded_board import project_points,world_corners,decode_patch
from PIL import Image,ImageDraw
H=Path(__file__).resolve().parent;b=H/'moving_coded_board_run';o=H/'matte_coded_board_run'
p=json.loads((H/'matte_coded_board_protocol.json').read_text());sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
checks={**p['source_sha256'],**p['previous_artifacts']}
for f,s in checks.items():assert sha(f)==s,f
status=json.loads((H/'matte_coded_board_status.json').read_text());assert status['exit_code']==0 and status['start_utc']>p['sealed_utc']
oldphy=np.load(b/'physics.npz');phy=np.load(o/'physics.npz');same=[k for k in phy.files if k not in ['raw_simulation_time_s','raw_physics_step_index']]
assert len(same)==17
for k in same:np.testing.assert_equal(phy[k],oldphy[k])
for f in ['requests.npz']:
 x=np.load(b/f);y=np.load(o/f)
 for k in x.files:np.testing.assert_equal(x[k],y[k])
oldstates=json.loads((b/'states.json').read_text());states=json.loads((o/'states.json').read_text());assert oldstates==states
for i in range(121):
 x=np.load(b/'history'/f'frame_{i:03d}.npz');y=np.load(o/'history'/f'frame_{i:03d}.npz')
 for k in x.files:np.testing.assert_equal(x[k],y[k])
g=json.loads((o/'board_geometry.json').read_text());assert g==json.loads((b/'board_geometry.json').read_text())
old=json.loads((b/'geometry_assessment.json').read_text());new=json.loads((o/'geometry_assessment.json').read_text());codes=np.array(p['codes'],np.uint8)
assert new['detections_saved_utc']<new['labels_evaluated_utc']
comparison=[]
for r0,r1 in zip(old['rows'],new['rows']):
 assert (r0['frame'],r0['eye'],r0['eligible_tags'])==(r1['frame'],r1['eye'],r1['eligible_tags'])
 comparison.append(dict(frame=r0['frame'],eye=r0['eye'],expected=r0['expected_corners'],old_recovered=r0['recovered_corners'],new_recovered=r1['recovered_corners'],new_correct=r1['correct_corners_2px'],old_p90_px=r0['current_p90_px'],new_p90_px=r1['current_p90_px'],old_gate=r0['gate_pass'],new_gate=r1['gate_pass']))
# Baseline missing set is fixed before intervention, never selected using candidate quality.
rows=[];panels=[]
for r in json.loads((b/'missing_photometry_labels.json').read_text())['rows']:
 i=r['frame'];eye=r['eye'];tag=r['tag'];patches=[];metrics=[]
 for folder in [b,o]:
  a=np.load(folder/'frames'/f'frame_{i:03d}.npz');gray=cv2.cvtColor(a[eye+'_rgb'],cv2.COLOR_RGB2GRAY)
  xy,_=project_points(world_corners(g['centres'][tag],.28)[g['canonical_corner_permutation']],a[eye+'_camera_usd_world'],584.8888888888889,319.5,239.5)
  patch=cv2.warpPerspective(gray,cv2.getPerspectiveTransform(xy.astype(np.float32),np.array([[0,0],[139,0],[139,139],[0,139]],np.float32)),(140,140));patches.append(patch)
  cells=np.array([[np.median(patch[rr*20+6:rr*20+14,cc*20+6:cc*20+14]) for cc in range(7)] for rr in range(7)],np.uint8)
  threshold,_=cv2.threshold(cells,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU);bits=(cells>threshold).astype(np.uint8);border=np.r_[bits[0],bits[-1],bits[1:-1,0],bits[1:-1,-1]]
  d=decode_patch(patch,codes);saved=np.load(folder/'rgb_detections'/f'frame_{i:03d}_{eye}.npz')
  metrics.append(dict(border_white_cells=int(border.sum()),contrast=int(cells.max())-int(cells.min()),black_border_median=float(np.median(np.r_[cells[0],cells[-1],cells[1:-1,0],cells[1:-1,-1]])),threshold=float(threshold),truequad_same_id=bool(d is not None and d['id']==tag),independent_rgb_detected=bool(tag in saved['ids'])))
 rows.append(dict(frame=i,eye=eye,tag=tag,baseline=metrics[0],matte=metrics[1]))
 panel=Image.new('RGB',(300,174),'white');draw=ImageDraw.Draw(panel);draw.text((4,3),f'{i:03d} {eye} ID{tag} | B0 vs matte',fill='black')
 for col,patch in enumerate(patches):panel.paste(Image.fromarray(patch).convert('RGB'),(col*150,24))
 panels.append(panel)
fig=Image.new('RGB',(1200,696),'white')
for i,panel in enumerate(panels):fig.paste(panel,((i%4)*300,(i//4)*174))
fig.save(H/'figures/board_material_paired_missing_labels.png')
material=json.loads((o/'material_identity.json').read_text())
result=dict(assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=len(checks),nonraw_17_exact=True,states_all_121_exact=True,history_all_121_exact=True,raw_origin_delta_s=float(phy['raw_simulation_time_s'][0]-oldphy['raw_simulation_time_s'][0]),raw_index_delta=int(phy['raw_physics_step_index'][0]-oldphy['raw_physics_step_index'][0]),geometry_same_complete_denominator=True,old_geometry_gate_pass=old['geometry_gate_pass'],new_geometry_gate_pass=new['geometry_gate_pass'],comparison=comparison,paired_baseline_missing_labels=rows,material=material,
  expected=sum(r['expected'] for r in comparison),old_recovered=sum(r['old_recovered'] for r in comparison),new_recovered=sum(r['new_recovered'] for r in comparison),new_correct=sum(r['new_correct'] for r in comparison),
  old_missing16_now_rgb_detected=sum(r['matte']['independent_rgb_detected'] for r in rows),old_missing16_now_truequad_decoded=sum(r['matte']['truequad_same_id'] for r in rows),scope='one material-only diagnostic, baseline known missing tags offline labels; geometry frozen gate governs V1, no new navigation/admission')
(o/'material_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:x for k,x in result.items() if k not in ['comparison','paired_baseline_missing_labels','material']},indent=2))
