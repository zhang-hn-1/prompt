"""One frozen early turn termination and measured matching stopping family."""
import copy
import numpy as np

def development_schedule():
    result=np.zeros((135,10),np.float32)
    result[60:75]=np.tile(np.asarray([.3,np.pi/6],np.float32),5)
    return result

def whole_short_entry(template,xy,times,requests):
    times=np.asarray(times,float);requests=np.asarray(requests,np.float32)
    if requests.shape!=(75,10) or abs(times[0])>1e-5 or abs(times[-1]-7.5)>1e-5:raise ValueError('whole matched1.5s turn+6s stop required')
    expected=development_schedule()[60:]
    if not np.array_equal(expected,requests):raise ValueError('different plan requires independent physical response')
    e=copy.deepcopy(template);e['name']='short_positive';e['xy']=np.asarray(xy,float).copy();e['times']=times.copy();e['requests']=requests.copy()
    e['scope']='1.5s turn + its measured matching6s stopping tail; no arbitrary early truncation'
    return e

class CompletionGate:
    """Actual speed qualification doesn't restore the initial dynamics state."""
    def __init__(self,plan_blocks):self.plan_blocks=int(plan_blocks);self.executed_blocks=0;self.low_count=0
    def observe(self,current_speed):
        self.executed_blocks+=1
        self.low_count=self.low_count+1 if np.isfinite(current_speed) and 0<=current_speed<.05 else 0
        done=self.executed_blocks>=self.plan_blocks;stop=self.low_count>=5
        return dict(plan_exhausted=done,actual_stop_qualified=stop,low_speed_count=self.low_count,decision_boundary=done and stop,executed_blocks=self.executed_blocks)

from guarded_response import GuardedResponseConsumer
class ShortConsumer(GuardedResponseConsumer):
    def select(self,*args,**kwargs):
        result=super().select(*args,**kwargs)
        if result['selected'] is not None:
            entry=next(e for e in self.entries if e['name']==result['selected'])
            result['scope']=entry['scope']
        return result
