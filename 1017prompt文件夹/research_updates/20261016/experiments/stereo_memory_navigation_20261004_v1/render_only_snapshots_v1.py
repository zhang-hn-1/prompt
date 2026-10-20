"""Frozen-clock/state radiance diagnostic only, no physics callback."""
import copy
import numpy as np

def record_render_only(render,clock,snapshot,capture,count=24):
 if type(count) is not int or not 1<=count<=24:raise ValueError('bounded 1..24 render snapshots')
 before=copy.deepcopy(snapshot());time=copy.deepcopy(clock());rows=[]
 for i in range(count):
  render();after=snapshot();assert clock()==time and len(after)==len(before) and all(np.array_equal(a,b) for a,b in zip(before,after)),'render changed physical state or clock'
  rows.append(copy.deepcopy(capture()))
 return rows
