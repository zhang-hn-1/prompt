import pytest
from tracking_tube_launch_gate import verify_frozen_review

def fixtures():
 identities={'planner':'one','driver':'two','stage':'three'};return {'review_source_paths':list(identities),'source_sha256':identities},{'source_sha256':identities},{'unresolved_critical':0,'unresolved_important':0,'closed_source_sha256':identities},identities

def test_closed_review_matches_current_sources():
 p,s,r,h=fixtures();verify_frozen_review(p,s,r,h.__getitem__)

def test_regenerated_protocol_cannot_reuse_stale_closure():
 p,s,r,h=fixtures();p['source_sha256']=dict(h,planner='changed');s['source_sha256']=p['source_sha256'];h=dict(h,planner='changed')
 with pytest.raises(ValueError):verify_frozen_review(p,s,r,h.__getitem__)

def test_missing_review_source_cannot_pass_zero_counts():
 p,s,r,h=fixtures();r['closed_source_sha256']={'driver':'two'}
 with pytest.raises(ValueError):verify_frozen_review(p,s,r,h.__getitem__)
