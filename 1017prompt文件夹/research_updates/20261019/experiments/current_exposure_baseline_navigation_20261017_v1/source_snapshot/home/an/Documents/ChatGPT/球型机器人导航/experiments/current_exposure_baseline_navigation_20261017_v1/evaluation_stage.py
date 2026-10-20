from pathlib import Path
import subprocess,sys
D=Path(__file__).resolve().parent
for script in ['evaluate_A_sealed.py','compare.py']:subprocess.run([sys.executable,'-B',str(D/script)],check=True)
