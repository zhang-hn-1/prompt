"""Shared offline reference evaluator; never uses either controller's cursor."""
import numpy as np
from route_geometry_audit import project
from straight_pacing_measurement_v1 import stop_qualification

def measure(states,path,install_s=6.):
 times=np.array([r['time_s'] for r in states]);xy=np.array([r['position'][:2] for r in states]);vel=np.array([r['velocity'] for r in states]);path=np.asarray(path,float)
 sel=times>=install_s-1e-5
 arc,error,_=project(xy[sel],path);endpoint=np.linalg.norm(xy[sel]-path[-1],axis=1);hits=np.flatnonzero(endpoint<=.2)
 return dict(reference_samples=int(sel.sum()),real_arc_max_m=float(arc.max()),real_arc_final_m=float(arc[-1]),real_arc_gain_m=float(arc[-1]-arc[0]),cross_track_RMS_m=float(np.sqrt(np.mean(error**2))),cross_track_peak_m=float(error.max()),endpoint_min_m=float(endpoint.min()),endpoint_final_m=float(endpoint[-1]),first_endpoint_passage_from_install_s=None if not len(hits) else float(times[sel][hits[0]]-install_s),terminal_stop=stop_qualification(times,vel,install_s))

def full_contact_evidence(rows,requests,end_s):
 if len(rows)!=requests*5+(requests+1):return False
 if not rows or not all(r.get('available',False) and r.get('contact') is False for r in rows):return False
 t=np.array([r['simulation_time_s'] for r in rows],float)
 if not np.isfinite(t).all() or abs(t.min())>1e-5 or abs(t.max()-end_s)>1e-5:return False
 # Every physical20ms instant must have evidence, not just endpoints/count.
 return np.array_equal(np.unique(np.rint(t/.02).astype(int)),np.arange(requests*5+1))
