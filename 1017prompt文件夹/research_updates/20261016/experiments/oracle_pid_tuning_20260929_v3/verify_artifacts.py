from pathlib import Path
import hashlib,json,cv2,numpy as np
h=Path(__file__).resolve().parent
identity=json.loads((h/'navigation_source_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items())
rows=[]
for p in sorted(h.glob('navigation_*/episode_*/*.mp4')):
 if '.partial' in str(p):continue
 cap=cv2.VideoCapture(str(p));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS);first,_=cap.read();cap.set(cv2.CAP_PROP_POS_FRAMES,max(0,n-1));last,_=cap.read();cap.release();row=dict(path=str(p),frames=n,fps=fps,first_decodes=first,last_decodes=last);rows.append(row)
 assert n>0 and fps>0 and first and last,str(p)
expected=2*len(json.loads((h/'navigation_comparison.json').read_text()));assert len(rows)==expected,len(rows)
(h/'video_verification.json').write_text(json.dumps(rows,indent=2))
assert len(json.loads((h/'navigation_comparison.json').read_text())) in (3,10)
print('Verified unchanged frozen source and',len(rows),'videos with readable first/last frames')
