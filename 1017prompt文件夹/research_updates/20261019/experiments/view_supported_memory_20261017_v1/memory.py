"""One promotion change: source-view novelty replaces exposure-count promotion.
Recent positive risk10s remains. No measured-free label or calibrated probability.
Confirmed risk still persists: this candidate does NOT solve every false memory.
"""
from pathlib import Path
import sys,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';sys.path.insert(0,str(H))
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from partial_point_risk_grid import PartialPointRiskGrid
from types import SimpleNamespace
class ViewSupportMemoryGrid(RepeatedExposureMemoryGrid):
    def __init__(self,origin,retention_s,*,static_scene_assumption):
        super().__init__(origin,retention_s,static_scene_assumption=static_scene_assumption)
        self._seen_source=set();self._view_witnesses={};self._view_clock=np.full(self.labels.shape,-np.inf)
    def consume_records(self,records,now,root):
        # Check causal metadata before consulting payload or changing state.
        for r in records:
            t=float(r['time_s']);f=r['frame']
            if not np.isfinite(t) or t>now or isinstance(f,bool) or not isinstance(f,(int,np.integer)) or f<0:raise ValueError('causal immutable frame required')
            for k in ['body_center','pair_center']:
                a=np.asarray(r[k],float)
                if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('estimated source centers required')
        PartialPointRiskGrid.consume_records(self,records,now,root)
        for r in records:
            f=int(r['frame']);t=float(r['time_s']);name=r['query_camera'];key=(f,name)
            if key in self._seen_source:continue
            self._seen_source.add(key)
            world=np.asarray(r['world'],float).reshape(-1,3);cells=self.cells(world[:,:2]);inside=self.inside(cells);world=world[inside];cells=cells[inside]
            body=np.asarray(r['body_center']);camera=np.asarray(r['pair_center'])
            for cell in np.unique(cells,axis=0):
                ij=tuple(cell);anchor=world[np.all(cells==cell,axis=1)].mean(axis=0)
                witness=self._view_witnesses.setdefault(ij,[])
                if any(w['frame']==f for w in witness):continue # both eyes/pairs same exposure not separate confirmations
                supported=False
                for w in witness:
                    # Fixed anchor avoids inventing parallax from noisy endpoint changes.
                    a=w['anchor']-w['pair_center'];b=w['anchor']-camera;den=np.linalg.norm(a)*np.linalg.norm(b)
                    angle=np.arccos(np.clip(a@b/den,-1,1)) if den>1e-12 else 0.
                    if np.linalg.norm(body-w['body_center'])>=.2 and angle>=np.deg2rad(5.):supported=True;break
                witness.append(dict(frame=f,time_s=t,body_center=body.copy(),pair_center=camera.copy(),anchor=anchor))
                if supported:self._view_clock[ij]=max(self._view_clock[ij],t)
        clock=np.maximum(self.hazard_acquisition_time,self._view_clock);active=np.isfinite(clock)
        self.labels.fill(-1);self.labels[active]=1;self.codes.fill(0);self.codes[active]=9;self.codes[active&(clock==float(now))]=8
        self.hazard_acquisition_time=clock;self._confirmed_clock=self._view_clock.copy();self._persistent_clock=self._view_clock.copy();self.last_seen=clock.copy()
        self.memory=SimpleNamespace(floor_time=np.full(self.labels.shape,-np.inf),hazard_time=clock.copy())
    def diagnostics(self):
        d=super().diagnostics();d.update(hazard_lifetime='same_recent10s_plus_persistent_two_geometry_separated_view_witnesses',confirmation_rule='body_translation>=.2m AND anchor_ray_angle>=5deg; duplicate exposure/pair not independent',geometry_separated_confirmation_cells=int(np.isfinite(self._view_clock).sum()),calibrated_independence_probability=False,explicit_conflict_clearance='none; source LR conflicts are unknown, no free-ray carving; confirmed positive risk retained',reobserve_condition='recent hypothesis expires without geometric confirmation; emit unknown, no action override')
        return d
