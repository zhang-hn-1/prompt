import subprocess,sys,os,json
from pathlib import Path
H=Path(__file__).resolve().parent
rows=json.loads((H/'corrected_capture_protocol.json').read_text())['sequences'];status=[]
for row in rows:
    name=row['sequence'];print('START',name,flush=True)
    assert not (H/'capture_corrected'/name).exists()
    with (H/f'corrected_capture_{name}.log').open('w') as f:
        r=subprocess.run([sys.executable,str(H/'capture_corrected.py'),'--sequence',name],stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
    status.append(dict(sequence=name,exit_code=r.returncode));(H/'corrected_capture_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISH',name,r.returncode,flush=True)
    if r.returncode:raise RuntimeError(name)
print('SIX_CORRECTED_ACTUAL_CAPTURES_COMPLETE',flush=True)
