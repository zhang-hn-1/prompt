import numpy as np,pytest
from mask_stereo_inputs import masked_inputs

def test_zero_only_invalid_and_copy():
 a=np.arange(36,dtype=np.uint8).reshape(3,4,3);b=a+1;lm=np.zeros((3,4),bool);rm=lm.copy();lm[0,1]=True;rm[2,3]=True;aa=a.copy();bb=b.copy();l,r=masked_inputs(a,b,lm,rm);np.testing.assert_equal(a,aa);np.testing.assert_equal(b,bb);np.testing.assert_equal(l[lm],0);np.testing.assert_equal(r[rm],0);np.testing.assert_equal(l[~lm],a[~lm]);np.testing.assert_equal(r[~rm],b[~rm]);assert not np.shares_memory(a,l) and not np.shares_memory(b,r)
@pytest.mark.parametrize('bad',[np.zeros((3,4),np.uint8),np.zeros((3,4,4),np.uint8),np.zeros((3,4,3),float)])
def test_bad_RGB(bad):
 with pytest.raises(ValueError):masked_inputs(bad,np.zeros((3,4,3),np.uint8),np.zeros((3,4),bool),np.zeros((3,4),bool))
@pytest.mark.parametrize('mask',[np.zeros((3,4),np.uint8),np.zeros((2,4),bool)])
def test_bad_mask(mask):
 with pytest.raises(ValueError):masked_inputs(np.zeros((3,4,3),np.uint8),np.zeros((3,4,3),np.uint8),mask,np.zeros((3,4),bool))
