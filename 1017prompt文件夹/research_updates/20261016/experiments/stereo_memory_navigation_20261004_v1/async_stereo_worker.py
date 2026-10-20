"""Bounded spawn transport. Birth is immutable; parent stamps availability.

One executing job, one pending latest capture, one result. CPU poll/submit never
wait for inference. The factory imports GPU/CAD only inside the spawned child.
"""
import copy,importlib,multiprocessing as mp,queue,math,traceback
class WorkerFault(RuntimeError):pass

def _child(module,factory,inputs,outputs):
 try:
  backend=getattr(importlib.import_module(module),factory)()
  outputs.put(('ready',None))
  while True:
   item=inputs.get()
   if item is None:return
   result=backend(item)
   outputs.put(('result',result))
 except BaseException:outputs.put(('fault',traceback.format_exc()))

class AsyncStereoWorker:
 def __init__(self,module,factory):
  ctx=mp.get_context('spawn');self.start_method=ctx.get_start_method()
  self._in=ctx.Queue(maxsize=1);self._out=ctx.Queue(maxsize=1)
  self._process=ctx.Process(target=_child,args=(module,factory,self._in,self._out),daemon=True)
  self._pending=None;self._inflight=None;self._ready=False;self._closed=False;self._fault=None
  self._last_now=-math.inf;self._last_birth=-math.inf;self._last_frame=-1
  self.trace=[];self.dropped_frames=[];self.has_delivered_source=False
  self._process.start()
 @property
 def pid(self):return self._process.pid
 @property
 def is_alive(self):return self._process.is_alive()
 @property
 def busy(self):return self._inflight is not None
 def _clock(self,now):
  n=float(now)
  if not math.isfinite(n) or n<self._last_now:raise ValueError('finite monotonic client clock')
  if self._closed:raise WorkerFault('closed worker')
  if self._fault:raise WorkerFault(self._fault)
  return n
 def submit(self,packet,*,frame,birth_time_s,now):
  n=self._clock(now);b=float(birth_time_s);f=int(frame)
  if not math.isfinite(b) or b>n or b<=self._last_birth or f<=self._last_frame:
   raise ValueError('past monotonic capture before any payload read')
  if int(packet['frame'])!=f or float(packet['time_s'])!=b:raise ValueError('capture envelope mismatch')
  snapshot=copy.deepcopy(packet)
  if self._pending is not None:
   dropped=int(self._pending['frame']);self.dropped_frames.append(dropped);self.trace.append(dict(event='drop_pending',frame=dropped,now=n))
  self._pending=snapshot;self._last_now=n;self._last_birth=b;self._last_frame=f
  self.trace.append(dict(event='submit',frame=f,birth_time_s=b,now=n));self._dispatch(n)
 def _dispatch(self,n):
  if self._ready and self._inflight is None and self._pending is not None:
   self._in.put_nowait(self._pending)
   self._inflight=(int(self._pending['frame']),float(self._pending['time_s']))
   self.trace.append(dict(event='dispatch',frame=self._inflight[0],now=n));self._pending=None
 def poll(self,*,now):
  n=self._clock(now);self._last_now=n;results=[]
  while True:
   try:kind,value=self._out.get_nowait()
   except queue.Empty:break
   if kind=='fault':self._fault=value;self.trace.append(dict(event='fault',now=n));raise WorkerFault(value)
   if kind=='ready':self._ready=True;self.trace.append(dict(event='ready',now=n))
   elif kind=='result':
    if self._inflight is None or (int(value['frame']),float(value['time_s']))!=self._inflight:
     self._fault='worker changed acquisition identity';raise WorkerFault(self._fault)
    if float(value['time_s'])>n:self._fault='future result';raise WorkerFault(self._fault)
    results.append(dict(packet=value,available_time_s=n));self.has_delivered_source=True
    self.trace.append(dict(event='deliver',frame=self._inflight[0],birth_time_s=self._inflight[1],available_time_s=n));self._inflight=None
  # Queue feeder can lag process exit: allow one final drain next poll, never
  # manufacture successful data after worker exits.
  if not self.is_alive and not results:
   self._fault=f'worker exited without result/status: {self._process.exitcode}';raise WorkerFault(self._fault)
  self._dispatch(n);return results
 def close(self):
  if self._closed:return
  self._closed=True
  try:self._in.put_nowait(None)
  except queue.Full:pass
  self._process.join(timeout=1.)
  if self.is_alive:
   self.trace.append(dict(event='terminate_on_close'));self._process.terminate();self._process.join(timeout=2.)
  if self.is_alive:
   self.trace.append(dict(event='kill_on_close'));self._process.kill();self._process.join(timeout=2.)
  if self.is_alive:raise WorkerFault('worker failed to exit after kill')
  for q in (self._in,self._out):q.cancel_join_thread();q.close()
 def __enter__(self):return self
 def __exit__(self,*args):self.close()
