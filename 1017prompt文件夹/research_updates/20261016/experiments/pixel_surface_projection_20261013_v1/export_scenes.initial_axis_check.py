"""Read-only authored USD identities for offline surface label qualification."""
from common import *
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics

def main():
    out=D/'scenes';out.mkdir(exist_ok=False)
    for name,(R,cp) in GROUPS.items():
        cfg=json.loads(cp.read_text());ep=json.loads((R/'task_episode.json').read_text())['actual_episode']
        path=Path(ep['scene_uri'].replace('${SCENE_ROOT}',cfg['scene_root']))
        assert sha(path)==ep['scene_sha256'],(name,path)
        st=Usd.Stage.Open(str(path));cache=UsdGeom.XformCache()
        assert UsdGeom.GetStageMetersPerUnit(st)==1 and UsdGeom.GetStageUpAxis(st)=='Z'
        assert np.allclose(np.asarray(cache.GetLocalToWorldTransform(st.GetDefaultPrim())),np.eye(4))
        assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in st.Traverse())
        shapes=[];covered=set();visible=set();ground=[]
        for p in st.Traverse():
            if p.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible':visible.add(str(p.GetPath()))
            if p.GetTypeName()=='Mesh' and str(p.GetPath()).endswith('/defaultGroundPlane'):
                pts=np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get());T=np.asarray(cache.GetLocalToWorldTransform(p)).T
                pts=pts@T[:3,:3].T+T[:3,3];assert np.max(np.abs(pts[:,2]))<1e-8;ground.append(pts);covered.add(str(p.GetPath()))
            if p.GetTypeName() not in ['Sphere','Cylinder'] or UsdGeom.Imageable(p).ComputeVisibility()=='invisible':continue
            T=np.asarray(cache.GetLocalToWorldTransform(p)).T;rot=T[:3,:3]
            assert np.allclose(rot.T@rot,np.eye(3),atol=1e-8) and np.linalg.det(rot)>.999
            geo=UsdGeom.Sphere(p) if p.GetTypeName()=='Sphere' else UsdGeom.Cylinder(p)
            row=dict(path=str(p.GetPath()),type=p.GetTypeName(),centre=T[:3,3].tolist(),rotation=rot.tolist(),radius=float(geo.GetRadiusAttr().Get()))
            if row['type']=='Cylinder':assert geo.GetAxisAttr().Get()=='Z';row['height']=float(geo.GetHeightAttr().Get())
            shapes.append(row);covered.add(str(p.GetPath()))
            for child in p.GetChildren():
                if child.GetTypeName()=='Mesh':covered.add(str(child.GetPath()))
        assert visible<=covered,(name,visible-covered)
        # The external scene loader also adds a ground at z0. Saved views are inside the authored +/-30m floor.
        lo=np.concatenate(ground)[:,:2].min(0).tolist() if ground else [-30.,-30.]
        hi=np.concatenate(ground)[:,:2].max(0).tolist() if ground else [30.,30.]
        save('scenes/'+name+'.json',dict(scene=str(path),scene_sha256=sha(path),shape_count=len(shapes),shapes=shapes,visible_geometry_covered=True,ground_z_m=0.,ground_xy_min_m=lo,ground_xy_max_m=hi,ground_source='authored+runtime floor' if ground else 'runtime floor; qualification conservatively limits to +/-30m',label_scope='analytic parent-surface compatibility; not renderer instance-ID labels',task_sha256=sha(R/'task_episode.json')))
        print(name,len(shapes),'authored surfaces',flush=True)
if __name__=='__main__':main()
