import copy
import numpy as np,pytest
from paired_current_pixel_surface_v1 import PairedCurrentPixelSurfaceMemory
from paired_texture_surface import PairedTextureSurfaceMemory

def fixture():
 y,x=np.indices((480,640));im=np.repeat(((x+y)%256).astype(np.uint8)[:,:,None],3,axis=2);q=np.array([100*640+100,150*640+150]);r=np.array([90*640+100,140*640+150]);p=dict(frame=0,time_s=0.,world=np.array([[1.,0,.2],[1.1,0,.2]]),sigma_m=np.ones(2)*.01,query_pixel_id=q,partner_pixel_id=r);return p,im.copy(),im.copy(),np.ones((480,640))*10

def test_identical_when_no_black():
 p,l,r,d=fixture();a=PairedTextureSurfaceMemory().update(p,l,r,0.,{'f':'0'*64},d);b=PairedCurrentPixelSurfaceMemory().update(p,l,r,0.,{'f':'0'*64},d)
 for k in ['world','query_pixel_id','partner_pixel_id']:np.testing.assert_array_equal(a[k],b[k])
 assert b['certified_free'] is False and b['pixel_support_not_correspondence_certificate']

@pytest.mark.parametrize('eye',[0,1])
def test_exact_black_end_removed_but_other_point_and_raw_kept(eye):
 p,l,r,d=fixture();old=copy.deepcopy(p);ids=p['query_pixel_id' if eye==0 else 'partner_pixel_id'];[l,r][eye][ids[0]//640,ids[0]%640]=0;a=PairedCurrentPixelSurfaceMemory();out=a.update(p,l,r,0.,{'f':'0'*64},d);assert len(out['world'])==1;np.testing.assert_array_equal(out['world'],p['world'][1:]);np.testing.assert_array_equal(p['world'],old['world']);assert len(a.at(0.)[0]['world'])==1

class Poison:
 def __array__(self,*a,**k):raise AssertionError('payload accessed')
def test_future_before_payload():
 a=PairedCurrentPixelSurfaceMemory();p=dict(time_s=1.,world=Poison(),query_pixel_id=Poison(),partner_pixel_id=Poison())
 with pytest.raises(ValueError):a.update(p,Poison(),Poison(),0.,{},Poison())
 assert not a._records

def test_bad_pixel_does_not_commit_state():
 p,l,r,d=fixture();p['partner_pixel_id'][0]=640*480;a=PairedCurrentPixelSurfaceMemory()
 with pytest.raises(ValueError):a.update(p,l,r,0.,{},d)
 assert not a._records

def test_stale_exposure_before_payload():
 a=PairedCurrentPixelSurfaceMemory();p=dict(time_s=0.,world=Poison(),query_pixel_id=Poison(),partner_pixel_id=Poison())
 with pytest.raises(ValueError):a.update(p,Poison(),Poison(),1.,{},Poison())
 assert not a._records
