"""Pinned static checkpoint files only. No remote code execution or installation."""
from pathlib import Path
import hashlib,json,datetime,urllib.request
H=Path(__file__).resolve().parent;dest=H/'semantic_floor_model';dest.mkdir(exist_ok=True)
repo='nvidia/segformer-b0-finetuned-ade-512-512';revision='cc2189cbf3173593452db75308756036c990bc44'
files=['config.json','preprocessor_config.json','pytorch_model.bin'];rows=[]
for name in files:
 path=dest/name;url=f'https://huggingface.co/{repo}/resolve/{revision}/{name}'
 if path.exists():raise RuntimeError('pre-existing file requires explicit identity inspection, no silent replacement')
 with urllib.request.urlopen(url,timeout=30) as r:content=r.read()
 path.write_bytes(content);rows.append(dict(file=str(path),url=url,sha256=hashlib.sha256(content).hexdigest(),bytes=len(content)));print('STATIC_MODEL_FILE',name,len(content),flush=True)
(H/'semantic_floor_download_identity.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),repo=repo,revision=revision,files=rows,remote_code_executed=False),indent=2)+'\n')
