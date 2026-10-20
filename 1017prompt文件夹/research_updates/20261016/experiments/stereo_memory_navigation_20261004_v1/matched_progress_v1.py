"""Compare measured states at a shared cutoff, never an unmatched terminal state."""
import numpy as np

def matched_progress(candidate, base, cutoff, start, unit):
 a=max((r for r in candidate if r['time_s']<=cutoff+1e-5),key=lambda r:r['time_s'])
 b=max((r for r in base if r['time_s']<=cutoff+1e-5),key=lambda r:r['time_s'])
 return dict(axis_gain_m=float((np.asarray(a['position'][:2])-np.asarray(b['position'][:2]))@np.asarray(unit)),candidate_sample_time_s=a['time_s'],base_sample_time_s=b['time_s'])
