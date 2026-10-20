"""Decode RGB without pose; only then attach independent board-world labels."""
import json,hashlib,datetime
from pathlib import Path
import cv2,numpy as np
from coded_board import detect_tags,world_corners,project_points
H=Path(__file__).resolve().parent;out=H/'moving_coded_board_run'
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'moving_coded_board_protocol.json').read_text())
for f,s in p['source_sha256'].items():assert sha(f)==s,f
codes=np.array(p['codes'],np.uint8);mask=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
detections=out/'rgb_detections';detections.mkdir(exist_ok=False);saved=[];decoded={}
for i in p['selected_indices']:
    a=np.load(out/'frames'/f'frame_{i:03d}.npz')
    for eye in ['left','right']:
        tags=detect_tags(a[eye+'_rgb'],mask[eye],codes)
        ids=np.array([r['id'] for r in tags],int);corners=np.array([r['corners'] for r in tags]).reshape(-1,4,2)
        decoded[i,eye]=(ids,corners)
        f=detections/f'frame_{i:03d}_{eye}.npz';np.savez(f,ids=ids,corners=corners)
        saved.append(dict(frame=i,eye=eye,tags=len(ids),path=str(f),sha256=sha(f)))
detection_time=now();(detections/'manifest.json').write_text(json.dumps(dict(saved_utc=detection_time,rgb_only=True,geometry_not_loaded=True,rows=saved),indent=2)+'\n')
# World geometry and current/past poses are evaluation labels only.
g=json.loads((out/'board_geometry.json').read_text());states=json.loads((out/'states.json').read_text());focal=584.8888888888889;cx=319.5;cy=239.5;rows=[];details={}
for i in p['selected_indices']:
    for eye in ['left','right']:
        pose=np.array(states[i][eye+'_pose']);old=np.array(states[max(i-1,0)][eye+'_pose'])
        ids,corners=decoded[i,eye];lookup={int(tag):j for j,tag in enumerate(ids)}
        eligible=[];errors=[];past=[];allerrors=[];matches=0
        for tag,centre in enumerate(g['centres']):
            points=world_corners(centre,g['side_m'])[g['canonical_corner_permutation']];xy,z=project_points(points,pose,focal,cx,cy)
            if tag in lookup:
                measured=corners[lookup[tag]];e=np.linalg.norm(measured-xy,axis=1);allerrors.extend(e.tolist())
            inside=np.isfinite(xy).all() and np.all(z>0) and np.all(xy[:,0]>=2) and np.all(xy[:,0]<638) and np.all(xy[:,1]>=2) and np.all(xy[:,1]<478)
            if not inside:continue
            poly=np.zeros((480,640),np.uint8);cv2.fillConvexPoly(poly,np.rint(xy).astype(int),1)
            if np.any((poly!=0)&mask[eye]) or cv2.contourArea(xy.astype(np.float32))<400 or np.min(np.linalg.norm(xy-np.roll(xy,1,axis=0),axis=1))<20:continue
            eligible.append(tag)
            if tag not in lookup:continue
            oldxy,_=project_points(points,old,focal,cx,cy)
            e=np.linalg.norm(corners[lookup[tag]]-xy,axis=1);pe=np.linalg.norm(corners[lookup[tag]]-oldxy,axis=1)
            errors.extend(e.tolist());past.extend(pe.tolist());matches+=int((e<=2.).sum())
        n=4*len(eligible);recovered=len(errors)
        rms=float(np.sqrt(np.mean(np.square(errors)))) if errors else None
        p90=float(np.percentile(errors,90)) if errors else None
        passed=n>=12 and recovered>=.9*n and matches>=.9*n and rms<=1.5 and p90<=2.
        row=dict(frame=i,time_s=states[i]['time_s'],eye=eye,detected_tags=len(ids),eligible_tags=eligible,expected_corners=n,recovered_corners=recovered,correct_corners_2px=matches,
            current_rms_px=rms,current_p90_px=p90,current_peak_px=max(errors) if errors else None,past_rms_px=float(np.sqrt(np.mean(np.square(past)))) if past else None,past_p90_px=float(np.percentile(past,90)) if past else None,
            all_decoded_corner_p90_px=float(np.percentile(allerrors,90)) if allerrors else None,gate_pass=bool(passed))
        rows.append(row);details[f'{i:03d}_{eye}']=dict(ids=ids.tolist(),corners=corners.tolist(),eligible_errors=errors,past_errors=past,all_decoded_errors=allerrors)
assessment=dict(detections_saved_utc=detection_time,labels_evaluated_utc=now(),geometry_gate_pass=all(r['gate_pass'] for r in rows),rows=rows,
    comparison='Current versus precisely previous measured .1s capture pose only; no best-lag selection, no pose fit',role='calibration board acquisition/geometry witness, not formal navigation or all-surface certificate')
(out/'geometry_assessment.json').write_text(json.dumps(assessment,indent=2)+'\n');(out/'geometry_details.json').write_text(json.dumps(details,indent=2)+'\n')
print(json.dumps(assessment,indent=2),flush=True)
