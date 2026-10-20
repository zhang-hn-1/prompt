import time,numpy as np,pytest
from async_stereo_worker import AsyncStereoWorker,WorkerFault

def job(i,t=None,**extra):return dict(frame=i,time_s=float(i if t is None else t),value=np.array([i]),input_sha256={'capture':'a'*64},**extra)
def await_results(w,n=1,now=100.):
 end=time.monotonic()+8;out=[]
 while len(out)<n and time.monotonic()<end:
  out+=w.poll(now=now);time.sleep(.01)
 assert len(out)==n
 return out
class Poison:
 def __getitem__(self,k):raise AssertionError('future payload read')
def test_future_capture_rejected_before_payload_and_explicit_no_source():
 with AsyncStereoWorker('async_worker_cpu_fixture','Backend') as w:
  assert not w.has_delivered_source
  with pytest.raises(ValueError):w.submit(Poison(),frame=1,birth_time_s=2.,now=1.)
  assert w.trace==[]
def test_spawn_copy_birth_and_latest_pending_nonblocking():
 with AsyncStereoWorker('async_worker_cpu_fixture','Backend') as w:
  p=job(1,delay_s=.25);w.submit(p,frame=1,birth_time_s=1.,now=1.)
  p['value'][:]=99
  # Await startup and dispatch without waiting for inference.
  end=time.monotonic()+8
  while not w.busy and time.monotonic()<end:w.poll(now=1.);time.sleep(.01)
  assert w.busy and w.start_method=='spawn'
  begin=time.monotonic();w.submit(job(2),frame=2,birth_time_s=2.,now=2.);w.submit(job(3),frame=3,birth_time_s=3.,now=3.)
  assert time.monotonic()-begin<.15
  out=await_results(w,2)
  assert [x['packet']['frame'] for x in out]==[1,3]
  assert out[0]['packet']['time_s']==1 and out[0]['available_time_s']==100.
  np.testing.assert_array_equal(out[0]['packet']['value'],[1]);assert w.dropped_frames==[2] and w.has_delivered_source
  with pytest.raises(ValueError):w.poll(now=99.)
def test_child_failure_is_explicit_and_cleanup():
 w=AsyncStereoWorker('async_worker_cpu_fixture','Backend');pid=w.pid
 try:
  w.submit(job(1,fail=True),frame=1,birth_time_s=1.,now=1.)
  with pytest.raises(WorkerFault):await_results(w)
 finally:w.close()
 assert not w.is_alive and pid>0
def test_delivery_clock_does_not_refresh_expired_birth():
 from causal_source_delivery_memory import CausalSourceDeliveryMemory
 with AsyncStereoWorker('async_worker_cpu_fixture','Backend') as w:
  w.submit(job(1),frame=1,birth_time_s=1.,now=1.)
  result=await_results(w,now=12.)[0]
  packet=dict(frame=result['packet']['frame'],time_s=result['packet']['time_s'],world=np.array([[.2,.1,.4]]),query_pixel_id=np.array([1]),partner_pixel_id=np.array([2]),input_sha256=result['packet']['input_sha256'])
  memory=CausalSourceDeliveryMemory();memory.deliver(packet,available_time_s=result['available_time_s'],now=12.)
  snap=memory.snapshot(12.);assert snap['source_time_s']==1 and snap['records']==[]
