import bootstrap
import numpy as np
from types import SimpleNamespace as N
from cursor_guard import reference_geometry_status,leg_interval
from reference_continuity_audit import reference_geometry_status as old
path=np.array([[0,0],[-1,0],[0,.1],[1,.1]],float);grid=np.full((101,101),-1);xy=[-.1,.02]
t=N(_path_index=1,_reference=[N(drive_direction=d) for d in ['reverse','forward','forward','forward']],_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking')
a=reference_geometry_status(xy,path,grid,[-5,-5],.1,segment_interval=(1,2));b=old(xy,path,grid,[-5,-5],.1);assert a['projection_segment_index']>=1 and a['projected_arc_m']>b['projected_arc_m']
z=reference_geometry_status(xy,path,grid,[-5,-5],.1);assert all(z[k]==v for k,v in b.items())
# Future opposite segment remains in suffix; only projection candidates are restricted.
c=reference_geometry_status([-.5,0],path,grid,[-5,-5],.1,segment_interval=(0,0));assert c['suffix_xy'][-1]==[1.,.1]
print('folded path projection and full future suffix tests pass')
