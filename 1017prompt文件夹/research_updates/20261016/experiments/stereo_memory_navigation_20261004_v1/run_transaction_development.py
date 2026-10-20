"""Exactly the two preregistered full command histories, sequential GPU use."""
import json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent
status=[]
for row in json.loads((H/'transaction_development_protocol.json').read_text())['sequences']:
    seq=row['sequence'];print('START',seq,flush=True)
    with (H/f'transaction_development_{seq}.log').open('w') as stream:
        process=subprocess.run([sys.executable,str(H/'capture_transaction_development.py'),'--sequence',seq],stdout=stream,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
    status.append(dict(sequence=seq,exit_code=process.returncode))
    (H/'transaction_development_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISH',seq,process.returncode,flush=True)
    if process.returncode:raise RuntimeError(seq)
