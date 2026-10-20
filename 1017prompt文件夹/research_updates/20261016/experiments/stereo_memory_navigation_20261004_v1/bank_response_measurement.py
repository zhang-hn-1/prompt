"""Fixed current-controller coverage sequence; offline producer-definition measurement."""
from pathlib import Path
import importlib.util
import numpy as np
_file=Path(__file__).resolve().parent.parent/'oracle_bank_mapping_20260929_v2/measurement.py'
_spec=importlib.util.spec_from_file_location('_frozen_bank_response_geometry',_file);_measurement=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_measurement)
def schedule():
    commands=np.zeros((320,5,2),np.float32)
    commands[60:260,:,0]=.5;commands[60:260,:,1]=-np.pi/6
    return commands.reshape(320,10)
def summarize(time,xy,q,velocity,yaw,main,start,end):
    time=np.asarray(time,float);xy=np.asarray(xy);q=np.asarray(q);velocity=np.asarray(velocity);yaw=np.asarray(yaw)
    n=len(time)
    if n<31 or time.shape!=(n,) or xy.shape!=(n,2) or q.shape!=(n,4) or velocity.shape!=(n,2) or yaw.shape!=(n,):raise ValueError('matching complete measured arrays required')
    if not all(np.isfinite(a).all() for a in [time,xy,q,velocity,yaw]) or not np.isfinite([main,start,end]).all() or main==0 or start>=end:raise ValueError('finite nonzero main and valid measured window required')
    if not np.allclose(np.diff(time),.02,atol=1e-6,rtol=0):raise ValueError('frozen producer50Hz sampling required')
    mask=(time>=start)&(time<=end)
    if mask.sum()<31 or time[0]>start+1e-6 or time[-1]<end-1e-6:raise ValueError('complete predetermined window required')
    speed,k=_measurement.geometry(xy);b=np.degrees(_measurement.bank(q));valid=mask&np.isfinite(k);count=int(valid.sum());fraction=count/int(mask.sum())
    if not count:return dict(accepted=False,valid_fraction=0.,failure='no measurable geometric curvature',samples=int(mask.sum()))
    values=k[valid]*np.sign(main);med=float(np.median(values));iqr=float(np.ptp(np.percentile(values,[25,75])));span=float(np.ptp(np.percentile(b[valid],[5,95])));signed=velocity[:,0]*np.cos(yaw)+velocity[:,1]*np.sin(yaw);direction=float(np.mean(signed[valid]*np.sign(main)>0))
    checks=dict(valid_fraction=fraction>=.9,curvature_iqr=iqr<=max(.2,.3*abs(med)),bank_span=span<=3.,direction=direction>=.9)
    return dict(window_start_s=float(start),window_end_s=float(end),samples=int(mask.sum()),valid_samples=count,actual_speed_m_s=float(np.median(speed[valid])),actual_bank_deg=float(np.median(b[valid])),curvature_body_1_m=med,curvature_geometric_1_m=med*float(np.sign(main)),curvature_iqr=iqr,bank_span_95_5_deg=span,valid_fraction=float(fraction),direction_match_fraction=direction,checks=checks,accepted=all(checks.values()),measurement_scope='offline centered31sample XY geometry; not online physical prediction')
