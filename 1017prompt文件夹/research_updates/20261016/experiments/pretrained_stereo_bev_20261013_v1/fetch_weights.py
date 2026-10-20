from pathlib import Path
import sys,json,hashlib
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D/'download_dependencies'))
import gdown
p=D/'weights/23-51-11/model_best_bp2.pth';result=gdown.download(id='1Yh_2o9QCUrVqZrnAXZ7RUr0zTp3JrMKe',output=str(p),quiet=True,resume=True,timeout=45,retries=0);assert result
(D/'weight_identity.json').write_text(json.dumps({'path':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'cfg_sha256':hashlib.sha256(p.with_name('cfg.yaml').read_bytes()).hexdigest(),'official_drive_id':'1Yh_2o9QCUrVqZrnAXZ7RUr0zTp3JrMKe'},indent=2)+'\n');print('official_weight_complete_and_hashed',flush=True)
