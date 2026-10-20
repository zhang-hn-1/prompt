from pathlib import Path
import requests,json,hashlib
D=Path(__file__).resolve().parent;assert json.loads((D/'oracle_qualification.json').read_text())['pass_gate']
repo='nvidia/segformer-b0-finetuned-ade-512-512';m=requests.get('https://huggingface.co/api/models/'+repo,timeout=30);m.raise_for_status();revision=m.json()['sha'];W=D/'semantic_weights';W.mkdir(exist_ok=True);files={}
for name in ['config.json','preprocessor_config.json','pytorch_model.bin']:
 r=requests.get('https://huggingface.co/'+repo+'/resolve/'+revision+'/'+name,timeout=(20,180),stream=True);r.raise_for_status();p=W/name
 with p.with_suffix(p.suffix+'.part').open('wb') as f:
  for b in r.iter_content(1024*1024):f.write(b)
 p.with_suffix(p.suffix+'.part').rename(p);files[name]={'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()};print(name,files[name]['bytes'],flush=True)
(D/'semantic_identity.json').write_text(json.dumps({'repo':repo,'revision':revision,'files':files,'licence':'NVIDIA SegFormer non-commercial research/evaluation','fit':0},indent=2)+'\n')
