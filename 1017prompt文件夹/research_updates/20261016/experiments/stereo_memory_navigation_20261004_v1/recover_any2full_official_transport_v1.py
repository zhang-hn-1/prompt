"""ONE administrative transport recovery, same artifact; no scientific inference."""
from pathlib import Path
import os,json,datetime,time,subprocess,hashlib
H=Path(__file__).resolve().parent;A=H/'any2full_predicted_geometry_preflight_v1';out=A/'transport_recovery_status_v1.json';assert not out.exists();old=json.loads((A/'weight_acquisition_status_v1.json').read_text());assert old['stage']=='failed_preserved_no_model_load';start=time.monotonic();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
row=dict(start_utc=now(),pid=os.getpid(),stage='official_hf_xet_same_artifact',original_failure=old,maximum_wall_s=900,model_candidates=1,scientific_inference_count=0,revision=old['revision'])
def save():out.write_text(json.dumps(row,indent=2)+'\n')
save();env=os.environ.copy();env.update(HF_HOME=str(A/'hf_home'),HF_XET_CACHE=str(A/'hf_xet_cache'),HF_HUB_DISABLE_IMPLICIT_TOKEN='1',HF_HUB_DISABLE_TELEMETRY='1',PYTHONDONTWRITEBYTECODE='1')
code="from huggingface_hub import hf_hub_download; print(hf_hub_download(repo_id='zhiyuandaily/Any2Full',filename='checkpoints/Any2Full_vitl.pth.tar',revision='"+old['revision']+"',token=False,cache_dir='"+str(A/'hf_cache')+"'),flush=True)"
try:
 result=subprocess.run([os.sys.executable,'-c',code],env=env,timeout=900,capture_output=True,text=True);(A/'transport_recovery_stdout_v1.log').write_text(result.stdout);(A/'transport_recovery_stderr_v1.log').write_text(result.stderr);assert result.returncode==0,result.stderr[-1500:];p=Path(result.stdout.strip().splitlines()[-1]);assert p.stat().st_size==old['expected_size'];digest=hashlib.sha256(p.read_bytes()).hexdigest();assert digest==old['expected_sha256'];target=A/'Any2Full_vitl.pth.tar';assert not target.exists();os.link(p.resolve(),target);row.update(stage='downloaded_verified_no_load',actual_sha256=digest,artifact=str(target),end_utc=now(),wall_s=time.monotonic()-start);save();print(json.dumps(row,indent=2))
except Exception as exc:
 row.update(stage='failed_recovery_preserved_no_load',failure=type(exc).__name__+': '+str(exc),end_utc=now(),wall_s=time.monotonic()-start);save();raise
