from pathlib import Path
import json, hashlib
import numpy as np
from reference import build_cost, aggregate
from evidence_decode import evidence_decode
from self_support import self_support
D = Path(__file__).resolve().parent

def main():
    cfg = json.loads((D / 'protocol.json').read_text())
    for sec in ['input_sha256', 'code_sha256']:
        for p, h in cfg[sec].items(): assert hashlib.sha256(Path(p).read_bytes()).hexdigest() == h, p
    rng = np.random.default_rng(20261019); a = rng.integers(0, 256, (23, 31, 3), dtype=np.uint8); b = a.copy(); ds = np.arange(-4, 5)
    c, s = build_cost(a, b, ds); empty = np.zeros(a.shape[:2], bool)
    checks = dict(empty_CAD_exact=np.array_equal(self_support(s, empty, empty, ds), s))
    mask = empty.copy(); mask[11, 15] = True
    one = self_support(s, mask, empty, ds)
    checks['query_full_BT_block_rectangle_excluded'] = not one[8:15, 11:20].any()
    partner = self_support(s, empty, mask, ds)
    checks['positive_partner_x_minus_d'] = not partner[11, 19, 8] and bool(partner[11, 24, 8])
    checks['negative_partner_x_minus_d'] = not partner[11, 11, 0] and bool(partner[11, 6, 0])
    inva = a.copy(); invb = b.copy(); inva[mask] = 255-inva[mask]; invb[mask] = 255-invb[mask]
    cc, ss = build_cost(inva, invb, ds); avail = self_support(s, mask, mask, ds)
    checks['supported_costs_self_RGB_invariant'] = np.array_equal(c[avail], cc[avail])
    total, _ = aggregate(c, avail); total2, _ = aggregate(cc, avail)
    checks['aggregated_costs_self_RGB_invariant'] = np.array_equal(total, total2)
    checks['decoded_self_RGB_invariant'] = np.array_equal(evidence_decode(total,c,avail,ds), evidence_decode(total2,cc,avail,ds), equal_nan=True)
    none = self_support(s, np.ones_like(mask), empty, ds); t, _ = aggregate(c, none)
    checks['all_self_UNKNOWN'] = not none.any() and np.isnan(evidence_decode(t,c,none,ds)).all()
    assert all(checks.values()), checks
    (D / 'interfaces_result.json').write_text(json.dumps(dict(checks={k:bool(v) for k,v in checks.items()},GT_reads=0,real_RGB_calls=0),indent=2)+'\n')
    print('INTERFACES', checks, flush=True)
if __name__ == '__main__': main()
