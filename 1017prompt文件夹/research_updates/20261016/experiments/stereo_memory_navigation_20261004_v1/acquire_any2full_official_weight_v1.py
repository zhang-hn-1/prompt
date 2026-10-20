"""One bounded public artifact acquisition; never imports or executes checkpoint."""
from pathlib import Path
import requests,json,datetime,hashlib,os,time,shutil
H=Path(__file__).resolve().parent;A=H/'any2full_predicted_geometry_preflight_v1';status=A/'weight_acquisition_status_v1.json';assert not status.exists()
expected='ea2809c9c490bdf37292f55e773fc5a935bcfc024cbd1eee6268b3ee3b744b41';size=2069029116;name='checkpoints/Any2Full_vitl.pth.tar';now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();begin=time.perf_counter()
row=dict(start_utc=now(),stage='resolving_official_revision',pid=os.getpid(),maximum_wall_s=900,expected_size=size,expected_sha256=expected,scientific_inference_count=0,model_candidates=1,whole_research_complete=False)
def save():status.write_text(json.dumps(row,indent=2)+'\n')
save()
try:
 assert shutil.disk_usage(A).free>=size+1000000000,'insufficient disk for bounded artifact'
 meta=requests.get('https://huggingface.co/api/models/zhiyuandaily/Any2Full',timeout=(20,30));meta.raise_for_status();info=meta.json();revision=info['sha'];(A/'official_model_revision_v1.json').write_text(json.dumps(info,indent=2)+'\n');row.update(stage='downloading_once',revision=revision,bytes_received=0);save()
 out=A/'Any2Full_vitl.pth.tar.partial';assert not out.exists();digest=hashlib.sha256();last=0
 with requests.get(f'https://huggingface.co/zhiyuandaily/Any2Full/resolve/{revision}/{name}',stream=True,timeout=(20,45)) as response:
  response.raise_for_status()
  with out.open('xb') as fp:
   for chunk in response.iter_content(1024*1024):
    if not chunk:continue
    assert time.perf_counter()-begin<=900,'bounded transfer wall time exceeded'
    row['bytes_received']+=len(chunk);assert row['bytes_received']<=size,'artifact exceeds frozen size';fp.write(chunk);digest.update(chunk)
    if time.perf_counter()-last>=15:last=time.perf_counter();save()
 assert row['bytes_received']==size and digest.hexdigest()==expected,'artifact identity mismatch'
 out.rename(A/'Any2Full_vitl.pth.tar');row.update(stage='downloaded_verified_no_load',actual_sha256=digest.hexdigest(),end_utc=now(),elapsed_wall_s=time.perf_counter()-begin);save();print(json.dumps(row,indent=2))
except Exception as exc:
 row.update(stage='failed_preserved_no_model_load',failure=type(exc).__name__+': '+str(exc),end_utc=now(),elapsed_wall_s=time.perf_counter()-begin);save();raise
