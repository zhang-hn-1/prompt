"""Bounded isolated acquisition, no execution of downloaded source/weights."""
from pathlib import Path
import urllib.request,json,hashlib,datetime,time,tarfile,io
H=Path(__file__).resolve().parent;D=H/'mvsanywhere_source_preflight_v1';STATUS=D/'artifact_acquisition_status_v1.json';assert not STATUS.exists();start=time.monotonic();identity={}
def status(stage,**kw):STATUS.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage=stage,elapsed_s=time.monotonic()-start,**kw),indent=2)+'\n')
def req(url):return urllib.request.Request(url,headers={'User-Agent':'ball-navigation-research'})
def small(url):
 with urllib.request.urlopen(req(url),timeout=40) as r:return r.read()
try:
 status('pinning_DINOv2_source');meta=small('https://api.github.com/repos/facebookresearch/dinov2/commits/main');commit=json.loads(meta)['sha'];(D/'DINOv2_commit.json').write_bytes(meta);identity['DINOv2_commit']=commit
 archive=small('https://api.github.com/repos/facebookresearch/dinov2/tarball/'+commit);assert len(archive)<64*1024**2;(D/'DINOv2_source.tar.gz').write_bytes(archive);identity['DINOv2_tar_sha256']=hashlib.sha256(archive).hexdigest()
 with tarfile.open(fileobj=io.BytesIO(archive),mode='r:gz') as t:
  for m in t.getmembers():
   rel=Path(*Path(m.name).parts[1:]);p=D/'DINOv2_source'/rel;assert '..' not in rel.parts and not rel.is_absolute()
   if m.isdir():p.mkdir(parents=True,exist_ok=True)
   elif m.isfile():p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(t.extractfile(m).read())
 identity['DINOv2_files_sha256']={str(p.relative_to(D)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (D/'DINOv2_source').rglob('*') if p.is_file()}
 (D/'DINOv2_source_identity_v1.json').write_text(json.dumps(identity,indent=2)+'\n')
 url='https://storage.googleapis.com/niantic-lon-static/research/mvsanywhere/mvsanywhere_hero.ckpt';p=D/'weights/mvsanywhere_hero.ckpt';p.parent.mkdir(exist_ok=True);partial=p.with_suffix('.ckpt.partial');assert not p.exists() and not partial.exists();size=0;digest=hashlib.sha256()
 with urllib.request.urlopen(req(url),timeout=40) as r,partial.open('wb') as f:
  total=int(r.headers.get('Content-Length','0'));assert 0<total<3*1024**3;status('downloading_unique_Hero',bytes=0,total=total,DINOv2_commit=commit)
  while True:
   if time.monotonic()-start>900:raise TimeoutError('fixed900s acquisition budget; partial evidence preserved')
   data=r.read(1024**2)
   if not data:break
   f.write(data);digest.update(data);size+=len(data)
   if size%(8*1024**2)==0:status('downloading_unique_Hero',bytes=size,total=total,DINOv2_commit=commit)
 assert size==total;partial.rename(p);identity.update(Hero_url=url,Hero_bytes=size,Hero_sha256=digest.hexdigest(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),no_source_or_weights_executed=True)
 (D/'artifact_identity_v1.json').write_text(json.dumps(identity,indent=2)+'\n');status('acquisition_complete',bytes=size,total=total,Hero_sha256=digest.hexdigest());print('ACQUISITION_COMPLETE',size,digest.hexdigest(),flush=True)
except Exception as e:
 status('acquisition_failed_preserved',exception=type(e).__name__,reason=str(e));raise
