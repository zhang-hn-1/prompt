from pathlib import Path
import os,importlib,math
import numpy as np
import pytest
m=importlib.import_module(os.environ.get('BANK_RESPONSE_MODULE','bank_response_measurement'))
def test_fixed_sequence_shape():
 x=m.schedule();assert x.shape==(320,10) and x.dtype==np.float32
@pytest.mark.parametrize('start,end,main,aux',[(0,60,0.,0.),(60,260,.5,-np.pi/6),(260,320,0.,0.)])
def test_literal_phases(start,end,main,aux):
 x=m.schedule().reshape(320,5,2);np.testing.assert_allclose(x[start:end,:,0],main,rtol=0,atol=1e-8);np.testing.assert_allclose(x[start:end,:,1],aux,rtol=0,atol=2e-8)
def test_independent_schedule_arrays():
 x=m.schedule();x[:]=99;assert np.max(abs(m.schedule()))<1
def fixture(sign):
 t=np.arange(501)*.02;s=sign*.18*t;k=.8;yaw=k*s;x=np.column_stack((np.sin(yaw)/k,(1-np.cos(yaw))/k));v=sign*.18*np.column_stack((np.cos(yaw),np.sin(yaw)));bank=-np.pi/8;q=np.zeros((len(t),4));q[:,0]=np.cos(bank/2);q[:,1]=np.sin(bank/2);return t,x,q,v,yaw
@pytest.mark.parametrize('sign',[1,-1])
def test_literal_xy_body_sign_and_bank(sign):
 t,x,q,v,yaw=fixture(sign);r=m.summarize(t,x,q,v,yaw,sign*.5,3.,7.5)
 assert r['accepted'] and r['curvature_body_1_m']==pytest.approx(.8,abs=.002);assert r['actual_bank_deg']==pytest.approx(-22.5,abs=1e-9);assert r['actual_speed_m_s']==pytest.approx(.18,abs=.0001);assert r['direction_match_fraction']==1
@pytest.mark.parametrize('bad',['dt','short'])
def test_invalid_time_or_window_rejected(bad):
 t,x,q,v,yaw=fixture(1)
 if bad=='dt':t=t*.8
 with pytest.raises(ValueError):m.summarize(t,x,q,v,yaw,.5,3.,3.2 if bad=='short' else 7.5)
