"""Planner-only common calibration-domain cap; no physical prediction."""
import math
from dataclasses import fields
from benchmark.drive_model import DriveModel

def common_support(bank_map,main):
 main=float(main)
 if not math.isfinite(main) or main==0:raise ValueError('finite nonzero main required')
 direction='forward' if main>0 else 'reverse'
 try:rows=[bank_map.directions[direction][key]['curvature_body_1_m'] for key in ('low','high')]
 except (KeyError,TypeError):raise ValueError('both calibrated rows required') from None
 if any(len(row)<2 or not all(math.isfinite(float(x)) for x in row) for row in rows):raise ValueError('finite calibration rows required')
 lower=max(min(row) for row in rows);upper=min(max(row) for row in rows)
 if lower>=upper:raise ValueError('nonempty common calibration domain required')
 return float(lower),float(upper)

class SupportLimitedDriveModel(DriveModel):
 def curvature(self,main_command_rad_s,auxiliary_angle_rad):
  k=super().curvature(main_command_rad_s,auxiliary_angle_rad)
  direction='forward' if float(main_command_rad_s)>0 else 'reverse'
  lo,hi=dict(self._nominal_support)[direction]
  return min(hi,max(lo,k))

def make_support_drive(base,bank_map):
 if not isinstance(base,DriveModel):raise TypeError('DriveModel required')
 domains=tuple((direction,common_support(bank_map,main)) for direction,main in [('forward',1.),('reverse',-1.)])
 result=SupportLimitedDriveModel(**{f.name:getattr(base,f.name) for f in fields(base) if f.init})
 object.__setattr__(result,'_nominal_support',domains)
 return result
