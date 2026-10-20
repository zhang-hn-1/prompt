"""Write-ahead request dispatch protocol; callback acknowledgement is not physics proof."""
import os,json,hashlib,sys,threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'execution_tube_qualification_20261012_v1'))
from tube_contract import Proposal
class Journal:
 def __init__(self,path):
  self.path=Path(path);self.fd=os.open(self.path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600);self.program=None;self.index=0;self.closed=False;self.lock=threading.Lock();self.poisoned=False;self.owner=threading.get_ident()
  directory=os.open(self.path.parent,os.O_RDONLY)
  try:os.fsync(directory)
  finally:os.close(directory)
 def require_owner(self):
  if threading.get_ident()!=self.owner:
   self.poisoned=True;raise ValueError("owner thread required")
 def append(self,record):
  self.require_owner()
  data=(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n').encode()
  while data:
   n=os.write(self.fd,data)
   if n<=0:raise OSError('short write')
   data=data[n:]
  os.fsync(self.fd)
 def commit(self,snapshot,program):
  self.require_owner()
  if self.program is not None or self.closed or type(snapshot) is not bytes:raise ValueError('one immutable commitment')
  canonical=Proposal(0.,program,'dispatch',hashlib.sha256(snapshot).hexdigest()).raw_targets
  try:self.append(dict(type='commit',snapshot_hex=snapshot.hex(),snapshot_sha256=hashlib.sha256(snapshot).hexdigest(),program=canonical))
  except BaseException:self.close();raise
  self.program=canonical
 def consume(self,callback):
  self.require_owner()
  if self.program is None or self.closed or self.index>=25:raise ValueError('no pending committed request')
  if not self.lock.acquire(blocking=False):
   self.poisoned=True;raise ValueError('reentrant or concurrent dispatch')
  try:
   if self.closed or self.poisoned:raise ValueError('dispatch closed')
   self.append(dict(type='dispatch_intent',index=self.index,pair=self.program[self.index]))
   callback(self.program[self.index])
   if self.poisoned:raise ValueError('dispatch poisoned by reentry')
   self.append(dict(type='callback_returned',index=self.index))
   self.index+=1
   if self.index==25:self.close()
  except BaseException:
   self.close();raise
  finally:self.lock.release()
 def close(self):
  self.require_owner()
  if not self.closed:self.closed=True;os.close(self.fd)
