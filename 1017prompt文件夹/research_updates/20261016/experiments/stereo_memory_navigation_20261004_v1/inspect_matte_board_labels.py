"""ONE label-only rejection decomposition of16 missing tags; does not alter detector."""
from pathlib import Path
import json,cv2,numpy as np
from PIL import Image,ImageDraw
from coded_board import world_corners,project_points
H=Path(__file__).resolve().parent;o=H/'matte_coded_board_run';s=json.loads((o/'summary.json').read_text());p=json.loads((H/'matte_coded_board_protocol.json').read_text());g=json.loads((o/'board_geometry.json').read_text());codes=np.array(p['codes'],np.uint8);rows=[];panels=[]
for r in s['missing_quad_diagnostic']:
    i=r['frame'];eye=r['eye'];tag=r['missing_id'];f=np.load(o/'frames'/f'frame_{i:03d}.npz');gray=cv2.cvtColor(f[eye+'_rgb'],cv2.COLOR_RGB2GRAY)
    corners=world_corners(g['centres'][tag],.28)[g['canonical_corner_permutation']];xy,_=project_points(corners,f[eye+'_camera_usd_world'],584.8888888888889,319.5,239.5)
    patch=cv2.warpPerspective(gray,cv2.getPerspectiveTransform(xy.astype(np.float32),np.array([[0,0],[139,0],[139,139],[0,139]],np.float32)),(140,140))
    values=np.array([[np.median(patch[rr*20+6:rr*20+14,cc*20+6:cc*20+14]) for cc in range(7)] for rr in range(7)],np.uint8)
    threshold,_=cv2.threshold(values,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU);bits=(values>threshold).astype(np.uint8);border=np.r_[bits[0],bits[-1],bits[1:-1,0],bits[1:-1,-1]]
    scores=sorted((int(np.count_nonzero(c!=np.rot90(bits[1:6,1:6],k))),j,k) for j,c in enumerate(codes) for k in range(4))
    row=dict(frame=i,eye=eye,tag=tag,cell_contrast=int(values.max())-int(values.min()),otsu_threshold=threshold,white_border_cells=int(border.sum()),best_id=scores[0][1],best_hamming=scores[0][0],margin=scores[1][0]-scores[0][0],canonical_expected_hamming=int(np.count_nonzero(bits[1:6,1:6]!=codes[tag])))
    row['rejected_by']=[name for name,bad in [('contrast',row['cell_contrast']<40),('border',row['white_border_cells']>2),('hamming',row['best_hamming']>1),('margin',row['margin']<3)] if bad];rows.append(row)
    panel=Image.new('RGB',(160,180),'white');panel.paste(Image.fromarray(patch).convert('RGB'),(10,30));d=ImageDraw.Draw(panel);d.text((4,4),f'{i} {eye} tag{tag}',fill='black');d.text((3,171),'/'.join(row['rejected_by']),fill='black');panels.append(panel)
image=Image.new('RGB',(640,720),'white')
for n,panel in enumerate(panels):image.paste(panel,((n%4)*160,(n//4)*180))
image.save(H/'figures/matte_coded_board_missing_labels.png')
(o/'missing_photometry_labels.json').write_text(json.dumps(dict(role='GT-assisted quadrilateral OFFLINE label measurement; NOT valid predicted corners/new detection/admission',rows=rows),indent=2)+'\n')
print(json.dumps(rows,indent=2))
