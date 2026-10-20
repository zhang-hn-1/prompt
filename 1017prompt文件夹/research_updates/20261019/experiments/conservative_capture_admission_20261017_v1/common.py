from pathlib import Path
import json, hashlib, datetime
import numpy as np
D = Path(__file__).resolve().parent
W = D.parents[1]
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p): return json.loads(Path(p).read_text())
def put(p, x): Path(p).write_text(json.dumps(x, indent=2, allow_nan=False) + '\n')
def now(): return datetime.datetime.now(datetime.timezone.utc).isoformat()
