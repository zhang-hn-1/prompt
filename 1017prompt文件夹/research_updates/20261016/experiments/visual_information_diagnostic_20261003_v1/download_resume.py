"""Recover exact official bytes after broken transfer; size and SHA256 gate."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import requests,hashlib,json,shutil
from huggingface_hub import hf_hub_url,get_hf_file_metadata
H=Path(__file__).resolve().parent;D=H/'models/unidepth_v2';PARTS=D/'transfer_parts';PARTS.mkdir(exist_ok=True)
meta=get_hf_file_metadata(hf_hub_url('lpiccinelli/unidepth-v2-vitl14','model.safetensors',revision='52b349b514bd8b47642f67ac78cb7b5dc5c51dd9'))
assert meta.size==1415383604 and meta.etag=='ba73d3de735302ccc64a50f1e557122050c4b1893e6060b28dba05d6af3e67c6'
prefix=list((D/'.cache/huggingface/download').glob('*.incomplete'))[0]
start=prefix.stat().st_size;chunk=32*1024*1024
ranges=[(a,min(a+chunk,meta.size)-1) for a in range(start,meta.size,chunk)]
print('Verified official size/hash; prefix',start,'ranges',len(ranges),flush=True)
def fetch(pair):
 a,b=pair;dest=PARTS/f'{a}_{b}.part'
 if dest.exists() and dest.stat().st_size==b-a+1:return
 for attempt in range(3):
  try:
   with requests.get(meta.location,headers={'Range':f'bytes={a}-{b}'},stream=True,timeout=(30,60)) as r:
    r.raise_for_status();assert r.status_code==206 and r.headers['Content-Range']==f'bytes {a}-{b}/{meta.size}'
    with dest.open('wb') as f:
     for block in r.iter_content(1024*1024):f.write(block)
   assert dest.stat().st_size==b-a+1
   print('completed range',a,b,flush=True);return
  except Exception as e:
   print('transport retry',a,attempt,type(e).__name__,flush=True)
   if attempt==2:raise
with ThreadPoolExecutor(max_workers=4) as ex:list(ex.map(fetch,ranges))
temp=D/'assembled.safetensors';digest=hashlib.sha256()
with temp.open('wb') as out:
 for f in [prefix]+[PARTS/f'{a}_{b}.part' for a,b in ranges]:
  with f.open('rb') as src:
   while block:=src.read(8*1024*1024):digest.update(block);out.write(block)
assert temp.stat().st_size==meta.size and digest.hexdigest()==meta.etag,'official byte integrity failed'
temp.rename(D/'model.safetensors')
(H/'unidepth_download_integrity.json').write_text(json.dumps(dict(revision=meta.commit_hash,size=meta.size,sha256=digest.hexdigest(),range_recovery=True,prefix_bytes=start),indent=2)+'\n')
print('official SHA256 passed',flush=True)
