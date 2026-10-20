"""Frozen task safety gates independent of native arrival flag."""
def safety_checks(actual):
 return dict(no_contact=actual['contact_frames']==0,
             no_collision=actual['collision_frames']==0,
             within_task_budget=float(actual['elapsed_s'])<=60.+1e-5)
