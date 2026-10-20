import unittest
import numpy as np
from height_interval import HeightIntervalSurfaceMemory

class Trap:
    def __array__(self,*a,**kw): raise AssertionError('payload read before clock check')

class TestHeightInterval(unittest.TestCase):
    def inputs(self):
        ids=np.array([100*640+200,110*640+220,120*640+240])
        p=dict(frame=0,time_s=0.,world=np.array([[1,0,.08],[1,0,.3],[1,0,.15]]),
               sigma_m=np.array([.04,.02,.1]),query_pixel_id=ids,partner_pixel_id=ids-30)
        rgb=np.random.default_rng(1).integers(0,256,(480,640,3),dtype=np.uint8)
        return p,rgb,np.roll(rgb,-30,axis=1),np.full((480,640),30.)
    def test_aligned_filter_and_history(self):
        m=HeightIntervalSurfaceMemory();p,l,r,d=self.inputs();original=p['world'].copy()
        out=m.update(p,l,r,0.,{},d)
        self.assertEqual(len(out['world']),1)
        self.assertEqual(out['query_pixel_id'].tolist(),[110*640+220])
        self.assertEqual(out['height_interval_ambiguous_count'],2)
        self.assertFalse(out['certified_free'])
        np.testing.assert_array_equal(p['world'],original)
        np.testing.assert_array_equal(m.at(0)[0]['world'],out['world'])
    def test_future_before_payload(self):
        p,l,r,d=self.inputs();p.update(time_s=2,world=Trap())
        with self.assertRaises(ValueError):HeightIntervalSurfaceMemory().update(p,l,r,0.,{},d)
    def test_invalid_uncertainty_rejected_without_mutation(self):
        for sigma in [np.array([np.nan,.02,.1]),np.array([-.1,.02,.1]),np.array([.1])]:
            m=HeightIntervalSurfaceMemory();p,l,r,d=self.inputs();p['sigma_m']=sigma
            with self.assertRaises(ValueError):m.update(p,l,r,0.,{},d)
            self.assertEqual(m.at(0),[])
    def test_empty_and_unavailable_do_not_retract_history(self):
        m=HeightIntervalSurfaceMemory();p,l,r,d=self.inputs();m.update(p,l,r,0.,{},d)
        p.update(time_s=1,frame=10)
        for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:p[k]=p[k][:0]
        out=m.update(p,l,r,1.,{},d)
        self.assertEqual(len(out['world']),0);self.assertEqual(len(m.at(1)[0]['world']),1)

if __name__=='__main__':unittest.main()
