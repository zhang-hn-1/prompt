from pathlib import Path
import subprocess,sys
D=Path(__file__).resolve().parent
for script in ['features.py','labels.py']:subprocess.run([sys.executable,'-B',str(D/script)],check=True)
