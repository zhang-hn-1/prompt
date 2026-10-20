from source import *
import json
# Exact calibrated pinhole pair sign, native projection, inverse rotation, units.
checks=[]
for q,p in PAIRS:
    A=LOCAL[NAMES.index(q)];C=LOCAL[NAMES.index(p)];rel=(A[:3,:3]@AX).T@(C[:3,3]-A[:3,3])
    assert np.allclose(rel,[0,B,0],atol=1e-10)
    assert np.allclose(A[:3,:3],C[:3,:3],atol=1e-10)
    point=A[:3,3]+(A[:3,:3]@AX)@np.array([.1,0.,2.])
    uv=[]
    for T in (A,C):
        optical=(point-T[:3,3])@(T[:3,:3]@AX);uv.append(F*optical[:2]/optical[2]+[319.5,239.5])
    assert abs((uv[0][1]-uv[1][1])-F*B/2)<1e-10
    assert abs(uv[0][0]-uv[1][0])<1e-10
    checks.append(dict(query=q,partner=p,baseline_optical_m=rel.tolist(),two_m_disparity_px=uv[0][1]-uv[1][1]))
for sign in (1,-1):
    a=np.full((640,480),sign*96.,np.float32);b=-a;m=np.zeros_like(a,bool)
    z,v,d=depths(a,b,m,m,sign)
    assert v.sum()==640*(480-96) and np.allclose(z[v],F*B/96)
    assert np.isnan(z[~v]).all()
# CAD depends on joints/known body only, no depth or scene truth.
model=FourSGBM();masks=model.masks(np.zeros(3));assert all(m.shape==(480,640) for m in masks.values())
(D/'interface_result.json').write_text(json.dumps(dict(pass_required_interface=True,pairs=checks,focal_px=F,baseline_m=B,known_fixture_not_performance=True,CAD_zero_joint_pixels={n:int(m.sum()) for n,m in masks.items()}),indent=2)+'\n')
print('INTERFACE_OK',checks,flush=True)
