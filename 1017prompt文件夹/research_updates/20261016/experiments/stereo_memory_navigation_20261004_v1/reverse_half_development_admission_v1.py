"""Validation can only follow a completely measured and sealed development."""
import json,hashlib
from pathlib import Path
def require_development_point(directory,protocol):
    h=Path(directory);load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    point=load(h/'reverse_half_fitted_point_v1.json')
    names=['reverse_half_development_status_v1.json','reverse_half_development_assessment_v1.json','reverse_half_development_verification_v1.json','reverse_half_development_run_v1/post_close_manifest.json','reverse_half_protocol_v1.json']
    if point['decision']!='DEVELOPMENT_POINT_SEALED' or point['utc']<=protocol['utc']:raise ValueError('development point chronology')
    for name in names:
        p=h/name
        if point['provenance_sha256'].get(str(p))!=sha(p):raise ValueError('development provenance '+name)
    status=load(h/names[0]);assessment=load(h/names[1]);verify=load(h/names[2]);manifest=load(h/names[3])
    if status['status']!='finished' or status['child_returncode']!=0:raise ValueError('development not closed0')
    if assessment['decision']!='PASS_POINT_ONLY_PENDING_EXECUTION_VERIFY' or not all(assessment['checks'].values()):raise ValueError('development gates failed')
    if verify['feedback_execution_ticks_exact']!=1600 or verify['physical_contacts']!=0:raise ValueError('development execution incomplete')
    if verify['assessment_sha256']!=sha(h/names[1]) or assessment['raw_manifest_sha256']!=sha(h/names[3]):raise ValueError('assessment linkage')
    if point['measured']!=assessment['measured']:raise ValueError('point differs from measurements')
    for p,s in manifest['artifact_sha256'].items():
        if sha(p)!=s:raise ValueError('development raw changed')
    return point
