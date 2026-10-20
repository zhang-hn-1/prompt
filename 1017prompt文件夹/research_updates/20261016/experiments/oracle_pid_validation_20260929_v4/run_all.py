import subprocess,sys
from pathlib import Path
h=Path(__file__).resolve().parent
for name in ['run_fresh.py','assess_fresh.py','run_navigation.py','analyze_navigation.py','audit_execution.py']:
 r=subprocess.run([sys.executable,str(h/name)],cwd=h); print(name,r.returncode,flush=True)
 if r.returncode: sys.exit(r.returncode)
