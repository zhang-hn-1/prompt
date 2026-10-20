"""Observe the existing force matrix, preserving its original collision decision."""
import json,numpy as np

def with_contact_audit(base,output):
 class ContactRuntime(base):
  def _physical_contact(self):
   if not hasattr(self,'contact_rows'):self.contact_rows=[];self.contact_geometry={}
   if self._contacts is None:return False
   try:
    forces=np.asarray(self._contacts.get_contact_force_matrix(dt=1./self._contract.schedule.physics_hz),dtype=np.float64)
    if forces.size==0 or forces.shape[-1]!=3 or not np.all(np.isfinite(forces)):raise RuntimeError('invalid filtered contact-force tensor')
   except Exception:
    self._contact_available=False;return False
   norm=np.linalg.norm(forces,axis=-1);pairs=np.argwhere(norm>1.)
   if len(pairs):
    view=self._contacts._contact_view;groups=view._regex_filter_paths;sensors=list(self._contacts.prim_paths)
    import omni.usd
    from pxr import Usd,UsdGeom,UsdPhysics
    stage=omni.usd.get_context().get_stage();cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy]);data=[]
    for si,fi in pairs:
     sensor=sensors[int(si)] if int(si)<len(sensors) else 'unresolved';group=groups[int(si)] if len(groups)>1 else groups[0];obstacle=group[int(fi)];data.append(dict(sensor_index=int(si),filter_index=int(fi),sensor=sensor,obstacle=obstacle,force_N=forces[si,fi].tolist(),norm_N=float(norm[si,fi])))
     for path in [sensor,obstacle]:
      if path in self.contact_geometry:continue
      prim=stage.GetPrimAtPath(path);bound=cache.ComputeWorldBound(prim).ComputeAlignedRange();colliders=[]
      for child in Usd.PrimRange(prim):
       if not child.HasAPI(UsdPhysics.CollisionAPI):continue
       box=cache.ComputeWorldBound(child).ComputeAlignedRange();colliders.append(dict(path=str(child.GetPath()),type=child.GetTypeName(),world_min=list(box.GetMin()),world_max=list(box.GetMax()),attributes={a.GetName():str(a.Get()) for a in child.GetAttributes() if any(k in a.GetName().lower() for k in ['radius','extent','approximation','offset','collisionenabled'])}))
      self.contact_geometry[path]=dict(type=prim.GetTypeName(),world_min=list(bound.GetMin()),world_max=list(bound.GetMax()),colliders=colliders)
    self.contact_rows.append(dict(time_s=float(self._clock_metadata()[0]),position=self._robot.get_world_pose()[0].tolist(),force_shape=list(forces.shape),sensor_paths=sensors,filter_groups=groups,pairs=data))
   return bool(len(pairs))
  def close(self,*args,**kwargs):
   output.mkdir(parents=True,exist_ok=True);(output/'contact_pairs.json').write_text(json.dumps(dict(rows=getattr(self,'contact_rows',[]),geometry=getattr(self,'contact_geometry',{})),indent=2));return super().close(*args,**kwargs)
 return ContactRuntime
