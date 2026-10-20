"""Hash only completed, actually archived transaction captures."""
import json,hashlib
from pathlib import Path
H=Path(__file__).resolve().parent
rows=[]
for seq in json.loads((H/'transaction_development_protocol.json').read_text())['sequences']:
    root=H/'capture_transaction_development'/seq['sequence']
    summary=json.loads((root/'capture_summary.json').read_text());assert summary['completed']
    assert summary['selected_indices']==seq['selected_indices']
    for index in summary['selected_indices']:
        rgb=root/'online_inputs'/f'frame_{index:03d}.npz';truth=root/'offline_truth'/rgb.name
        rows.append(dict(sequence=seq['sequence'],rgb_file=str(rgb),truth_file=str(truth),rgb_sha256=hashlib.sha256(rgb.read_bytes()).hexdigest(),truth_sha256=hashlib.sha256(truth.read_bytes()).hexdigest()))
result=dict(role='two development full command histories with accepted capture transaction, 20 selected pairs; not fresh validation',samples=rows,source_protocol_sha256=hashlib.sha256((H/'transaction_development_protocol.json').read_bytes()).hexdigest())
(H/'transaction_development_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
print('MANIFEST_PAIRS',len(rows))
