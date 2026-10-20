"""Immutable proposal and supervision eligibility only; no predictor or runtime."""
from dataclasses import dataclass
import math

def pairs_finite(values):
 return all(len(row)==2 and all(math.isfinite(float(x)) for x in row) for row in values)

@dataclass(frozen=True)
class Proposal:
 time_s:float
 raw_targets:tuple
 def __post_init__(self):
  if not math.isfinite(self.time_s) or self.time_s<0:raise ValueError('finite current time')
  if not isinstance(self.raw_targets,tuple) or len(self.raw_targets)!=25 or not all(isinstance(row,tuple) for row in self.raw_targets):raise ValueError('immutable25pairs required')
  if not pairs_finite(self.raw_targets):raise ValueError('finite command pairs required')

def qualify_label(proposal,*,times,positions,raw_targets,complete):
 if not all(math.isfinite(float(t)) for t in times) or not pairs_finite(positions) or not pairs_finite(raw_targets):raise ValueError('finite supervision required')
 if not complete:return dict(scorable=False,reason='censored_execution')
 if len(times)!=26 or len(positions)!=26 or len(raw_targets)!=25:return dict(scorable=False,reason='missing_samples')
 if any(abs(t-(proposal.time_s+i*.02))>1e-7 for i,t in enumerate(times)):return dict(scorable=False,reason='clock_mismatch')
 # Exact raw target equality, not a post-hoc approximate or different hold program.
 if tuple(tuple(row) for row in raw_targets)!=proposal.raw_targets:return dict(scorable=False,reason='program_mismatch')
 return dict(scorable=True,reason='exact_completed_program')
