"""Copy only archived RGB + explicitly privileged known poses to sanitized packets."""
from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent
p=json.loads((H/'multiview_rgb_protocol.json').read_text())
for name,sha in p['input_sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha,name
m=json.loads((H/'transaction_development_manifest.json').read_text())
out=H/'multiview_rgb_inputs';out.mkdir(exist_ok=True);rows=[]
for r in sorted([r for r in m['samples'] if r['sequence']==p['sequence']],key=lambda r:r['rgb_file']):
    rgb=np.load(r['rgb_file']);known=np.load(r['truth_file'])
    frame=int(Path(r['rgb_file']).stem.split('_')[1]);path=out/f'frame_{frame:03d}.npz'
    t=float(rgb['simulation_time_s']);assert t==float(known['simulation_time_s'])
    np.savez_compressed(path,left_rgb=rgb['left_rgb'],right_rgb=rgb['right_rgb'],
        left_pose=known['left_camera_usd_world'],right_pose=known['right_camera_usd_world'],time_s=t,frame=frame)
    assert set(np.load(path).files)=={'left_rgb','right_rgb','left_pose','right_pose','time_s','frame'}
    rows.append(dict(frame=frame,time_s=t,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        rgb_source=r['rgb_file'],known_pose_source=r['truth_file']))
(out/'manifest.json').write_text(json.dumps(dict(layer='RGB+known camera pose diagnostic, no depth/geometry fields',rows=rows),indent=2)+'\n')
print('SANITIZED',len(rows),'packets; known poses only, no depth/source geometry')
