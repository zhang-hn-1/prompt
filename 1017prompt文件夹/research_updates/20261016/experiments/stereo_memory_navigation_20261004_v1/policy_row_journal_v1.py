"""Append-only planned/executed rows. Pending actions are never promoted.
No policy, observation, or simulator calls occur here.
"""
import json,math
from pathlib import Path

_TIMING={'end_to_end_wall_s','execute_capture_NN_wall_s','wall_includes_policy_execute_render_NN_and_capture'}
def _snapshot(row):
 return json.loads(json.dumps(row,allow_nan=False))
def _validate(row):
 i=row['frame'];t=row['time_s'];req=row['request']
 if type(i) is not int or i<0 or not math.isfinite(t) or t<0 or len(req)!=10 or not all(math.isfinite(x) for x in req):raise ValueError('invalid action row')
 if row.get('source_time_s',0)>t:raise ValueError('future source row')
def _apply(state,event):
 row=event['row'];_validate(row);i=row['frame']
 if event['frame']!=i:raise ValueError('frame mismatch')
 if event['phase']=='planned':
  if state['pending_row'] is not None or i!=len(state['completed']):raise ValueError('nonconsecutive or duplicate planned row')
  state['pending_row']=row
 elif event['phase']=='executed':
  previous=state['pending_row']
  if previous is None or i!=previous['frame']:raise ValueError('missing planned row')
  if {k:v for k,v in row.items() if k not in _TIMING}!={k:v for k,v in previous.items() if k not in _TIMING}:raise ValueError('execution changed planned action fields')
  state['completed'].append(row);state['pending_row']=None
 else:raise ValueError('unknown phase')

class PolicyRowJournal:
 def __init__(self,path):
  self.file=Path(path).open('x');self.state={'completed':[],'pending_row':None}
 def _append(self,phase,row):
  event={'frame':row['frame'],'phase':phase,'row':_snapshot(row)}
  # Validate against a temporary state before appending, so rejected events leave no bytes.
  candidate={'completed':self.state['completed'].copy(),'pending_row':self.state['pending_row']}
  _apply(candidate,event)
  self.file.write(json.dumps(event,separators=(',',':'),allow_nan=False)+'\n');self.file.flush()
  self.state=candidate
 def planned(self,row):self._append('planned',row)
 def executed(self,row):self._append('executed',row)
 def close(self):self.file.close()

def read_journal(path):
 state={'completed':[],'pending_row':None};tail=False
 lines=Path(path).read_text().splitlines(keepends=True)
 for n,line in enumerate(lines):
  if n==len(lines)-1 and not line.endswith('\n'):tail=True;break
  try:_apply(state,json.loads(line))
  except (KeyError,TypeError,json.JSONDecodeError) as exc:raise ValueError('invalid journal event') from exc
 return {'completed':state['completed'],'pending':[] if state['pending_row'] is None else [state['pending_row']['frame']],'incomplete_tail_ignored':tail}

def recover_to_files(journal_path,rows_path,recovery_path):
 """A corrupt journal must not prevent independent raw physics export."""
 try:
  recovered=read_journal(journal_path)
  Path(rows_path).write_text(json.dumps(recovered['completed'],indent=2)+'\n')
  result=dict(valid=True,pending=recovered['pending'],incomplete_tail_ignored=recovered['incomplete_tail_ignored'],executed_rows=len(recovered['completed']))
 except (ValueError,OSError) as exc:
  result=dict(valid=False,error=repr(exc),completed_rows_admitted=False)
 Path(recovery_path).write_text(json.dumps(result,indent=2)+'\n')
 return result
