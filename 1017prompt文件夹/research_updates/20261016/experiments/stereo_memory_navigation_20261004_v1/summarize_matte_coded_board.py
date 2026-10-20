"""Post-detection diagnostics/figures, never input to RGB detector or matching."""
import json,datetime
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw
from coded_board import project_points,world_corners,decode_patch
H=Path(__file__).resolve().parent;o=H/'matte_coded_board_run';p=json.loads((H/'matte_coded_board_protocol.json').read_text());g=json.loads((o/'board_geometry.json').read_text());a=json.loads((o/'geometry_assessment.json').read_text());codes=np.array(p['codes'],np.uint8)
masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');diagnostics=[];panels=[];common=[]
for i in p['selected_indices']:
    frame=np.load(o/'frames'/f'frame_{i:03d}.npz');seen={}
    for eye in ['left','right']:
        row=next(r for r in a['rows'] if r['frame']==i and r['eye']==eye)
        detected=np.load(o/'rgb_detections'/f'frame_{i:03d}_{eye}.npz');ids=detected['ids'];seen[eye]=set(ids.tolist())
        rgb=frame[eye+'_rgb'];gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);canvas=Image.fromarray(rgb);draw=ImageDraw.Draw(canvas)
        for tag,corners in zip(ids,detected['corners']):
            draw.line([tuple(v) for v in corners]+[tuple(corners[0])],fill=(255,60,60),width=2);draw.text(tuple(corners.mean(0)),str(tag),fill=(255,255,0))
        for tag in row['eligible_tags']:
            pts=world_corners(g['centres'][tag],g['side_m'])[g['canonical_corner_permutation']]
            xy,_=project_points(pts,frame[eye+'_camera_usd_world'],584.8888888888889,319.5,239.5)
            draw.line([tuple(v) for v in xy]+[tuple(xy[0])],fill=(0,255,80),width=1)
            if tag not in ids:
                # GT-guided warped patch is explicitly offline failure attribution, NOT a new detector.
                patch=cv2.warpPerspective(gray,cv2.getPerspectiveTransform(xy.astype(np.float32),np.array([[0,0],[139,0],[139,139],[0,139]],np.float32)),(140,140))
                d=decode_patch(patch,codes)
                diagnostics.append(dict(frame=i,eye=eye,missing_id=tag,offline_true_quad_decoded_id=None if d is None else d['id'],offline_true_quad_same_id=bool(d is not None and d['id']==tag),pixel_range=[int(patch.min()),int(patch.max())]))
        top=Image.new('RGB',(640,510),'white');top.paste(canvas,(0,30));ImageDraw.Draw(top).text((8,8),f'{i:03d} {eye} | decoded {len(ids)} eligible {len(row["eligible_tags"])} | red RGB / green label',fill='black');panels.append(top)
    common.append(dict(frame=i,left_decoded=sorted(seen['left']),right_decoded=sorted(seen['right']),whole_tag_overlap=sorted(seen['left']&seen['right'])))
fig=H/'figures';fig.mkdir(exist_ok=True);montage=Image.new('RGB',(1280,3060),'white')
for n,panel in enumerate(panels):montage.paste(panel,((n%2)*640,(n//2)*510))
montage.save(fig/'matte_coded_board_all_frames.png')
focus=Image.new('RGB',(1280,1020),'white')
for n,k in enumerate([0,1,4,5]):focus.paste(panels[k],((n%2)*640,(n//2)*510))
focus.save(fig/'matte_coded_board_static_motion.png')
physics=np.load(o/'physics.npz');states=json.loads((o/'states.json').read_text());position=np.array([r['position'] for r in states]);velocity=np.array([r['velocity'] for r in states]);v=np.linalg.norm(velocity[:,:2],axis=1)
video=[]
for name in ['stereo_view','third_person']:
    cap=cv2.VideoCapture(str(o/(name+'.mp4')));video.append(dict(name=name,frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),fps=cap.get(cv2.CAP_PROP_FPS)));cap.release()
n=sum(r['expected_corners'] for r in a['rows']);got=sum(r['recovered_corners'] for r in a['rows']);correct=sum(r['correct_corners_2px'] for r in a['rows'])
summary=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),expected_corners=n,recovered_corners=got,correct_2px=correct,unconditional_fraction=correct/n,
    geometry_gate_pass=a['geometry_gate_pass'],missing_quad_diagnostic=diagnostics,missing_tags=len(diagnostics),missing_true_quad_decodable=sum(r['offline_true_quad_same_id'] for r in diagnostics),
    current_whole_tag_overlap=common,physics_samples=int(physics['simulation_time_s'].size),captures=len(states),requests=len(np.load(o/'requests.npz')['requests']),video=video,
    root_xy_net=(position[-1,:2]-position[0,:2]).tolist(),root_xy_path_m=float(np.linalg.norm(np.diff(position[:,:2],axis=0),axis=1).sum()),actual_xy_peak_speed_mps=float(v.max()),
    post6_xy_net=(position[-1,:2]-position[60,:2]).tolist(),matching_run=False,matching_block_reason='presealed geometry coverage gate failed; no V1 override',
    matching_protocol_budget='new independent presealed max6*24=144/min100; old max72 defect preserved',
    scope='GT quadrilateral warp only AFTER RGB detections saved; labels do not change detector outputs/admission. Whole-tag overlap is NOT all-pixel stereo overlap; planar calibration is not original obstacle or navigation.')
(o/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
