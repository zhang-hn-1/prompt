"""Risk-only state engine for a proposed long-term entry gate. No learned weights.
Current/recent10s source risk is protected. No label or GT enters this interface.
The mock all-admit score must reproduce B; no risk absence becomes observed free.
"""
import numpy as np
class EntryState:
    def __init__(self,shape=(400,400)):
        self.shape=tuple(shape);self.last=np.full(shape,-np.inf);self.persistent=np.full(shape,-np.inf);self.epochs=np.zeros(shape,np.int64);self.frame=-1;self.source_time=-np.inf;self.now=-np.inf;self.last_cells=None;self.last_scores=None
    def update(self,frame,source_time,now,cells,scores,decision_frame,decision_time):
        # Metadata guards precede ALL payload access and mutation.
        if isinstance(frame,bool) or not isinstance(frame,(int,np.integer)) or frame<0 or isinstance(decision_frame,bool) or not isinstance(decision_frame,(int,np.integer)) or decision_frame!=frame or not np.isfinite([source_time,now,decision_time]).all() or source_time>now or decision_time!=source_time or frame<self.frame or source_time<self.source_time or now<self.now or (frame==self.frame and source_time!=self.source_time) or (frame>self.frame and source_time<=self.source_time):raise ValueError('causal immutable capture and decision clocks required')
        cells=np.asarray(cells);scores=np.asarray(scores,float)
        if cells.ndim!=2 or cells.shape[1]!=2 or not np.issubdtype(cells.dtype,np.integer) or scores.shape!=(len(cells),) or np.any(np.isfinite(scores)&((scores<0)|(scores>1))) or np.any(np.isinf(scores)) or np.any(cells<0) or np.any(cells>=np.array(self.shape)):raise ValueError('current in-bounds unique cells and probability/NaN required')
        if len(np.unique(cells,axis=0))!=len(cells):raise ValueError('capture cells unique; stereo eyes not independent trials')
        if frame==self.frame:
            if not np.array_equal(cells,self.last_cells) or not np.array_equal(scores,self.last_scores,equal_nan=True):raise ValueError('same exposure payload/decision changed')
        else:
            ix=tuple(cells.T);self.epochs[ix]+=1;self.last[ix]=source_time
            accept=(self.epochs[ix]>=2)&np.isfinite(scores)&(scores>=.95)
            if accept.any():self.persistent[tuple(cells[accept].T)]=source_time
            self.frame=int(frame);self.source_time=float(source_time);self.last_cells=cells.copy();self.last_scores=scores.copy()
        self.now=float(now);recent=np.isfinite(self.last)&(now-self.last<=10.);active=recent|np.isfinite(self.persistent)
        labels=np.full(self.shape,-1,np.int8);labels[active]=1
        clock=np.where(np.isfinite(self.persistent),np.maximum(self.last,self.persistent),self.last);clock[~active]=-np.inf
        return labels,clock

def interface_checks():
    cell=np.array([[1,1]],int)
    outputs={}
    for name,score in [('all_admit',1.),('none_admit',0.),('unknown',np.nan)]:
        g=EntryState((4,4));g.update(0,0.,0.,cell,[score],0,0.);g.update(1,.5,.5,cell,[score],1,.5)
        recent,_=g.update(1,.5,.9,cell,[score],1,.5);assert recent[1,1]==1 and g.epochs[1,1]==2
        labels,_=g.update(2,11.,11.,np.empty((0,2),int),[],2,11.)
        assert labels[1,1]==(1 if score==1 else -1) and not np.any(labels==0)
        outputs[name]=True
    g=EntryState((4,4));before=g.last.copy()
    class Poison:
        def __array__(self,*args,**kwargs):raise AssertionError('future payload accessed')
    try:g.update(1,2.,1.,Poison(),Poison(),1,2.)
    except ValueError:pass
    else:raise AssertionError('future accepted')
    assert np.array_equal(before,g.last)
    return dict(**outputs,current_and_recent10s_protected=True,duplicate_capture_not_independent=True,future_before_payload=True,unknown_never_free=True,fixtures_not_learned_predictions=True)
