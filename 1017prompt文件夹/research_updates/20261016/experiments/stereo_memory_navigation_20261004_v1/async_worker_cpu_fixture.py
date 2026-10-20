"""CPU-only transport fixture; never used as physical/source evidence."""
import time
class Backend:
 def __call__(self,job):
  time.sleep(job.get('delay_s',.05))
  if job.get('fail'):raise RuntimeError('fixture failure')
  return dict(frame=job['frame'],time_s=job['time_s'],value=job['value'],input_sha256=job['input_sha256'])
class IgnoringTerminationBackend:
 def __init__(self):
  import signal
  signal.signal(signal.SIGTERM,signal.SIG_IGN)
 def __call__(self,job):
  time.sleep(30.)
  return job
