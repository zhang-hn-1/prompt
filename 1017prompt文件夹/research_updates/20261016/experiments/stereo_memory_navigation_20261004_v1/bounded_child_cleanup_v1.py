"""Bounded direct-child execution; timeout first gives SIGTERM cleanup a chance."""
import subprocess

def run_bounded(args,log,timeout,grace=45,env=None):
 p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,env=env)
 result=dict(pid=p.pid,timeout=False,forced_kill=False)
 try:p.wait(timeout=timeout)
 except subprocess.TimeoutExpired:
  result['timeout']=True;p.terminate()
  try:p.wait(timeout=grace)
  except subprocess.TimeoutExpired:
   result['forced_kill']=True;p.kill();p.wait(timeout=10)
 result['returncode']=p.returncode
 return result
