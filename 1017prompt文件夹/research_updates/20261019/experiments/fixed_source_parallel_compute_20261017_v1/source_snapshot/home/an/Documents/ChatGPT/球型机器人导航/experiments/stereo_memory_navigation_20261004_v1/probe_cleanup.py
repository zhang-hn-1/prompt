"""Bounded source-probe shutdown; partial evidence never implies complete run."""
from pathlib import Path
import signal,json,hashlib,datetime

def install_sigterm_cleanup():
 def terminate(signum,frame):raise SystemExit(128+signum)
 signal.signal(signal.SIGTERM,terminate)

def export_then_close(runtime,export,code):
 try:export()
 finally:runtime.close(return_code=code)

def seal_existing(directory,metadata):
 d=Path(directory);d.mkdir(exist_ok=True)
 p=d/'artifact_manifest_post_close.json'
 assert not p.exists()
 files={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in d.rglob('*') if f.is_file()}
 p.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact_sha256=files,whole_research_complete=False,formal_triggered=False,**metadata),indent=2)+'\n')
 return p
