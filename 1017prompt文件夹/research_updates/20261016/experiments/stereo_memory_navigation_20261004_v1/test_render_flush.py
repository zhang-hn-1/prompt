"""Breaks caught: trusting the timestamp while an annotator still holds an old frame."""
import numpy as np
import pytest
import render_flush as f

class Pipeline:
    def __init__(self):self.calls=0;self.content='old';self.state=np.array([0.])
    def render(self):
        self.calls+=1
        if self.calls>=2:self.content='current'
    def snapshot(self):return [self.state.copy()]

def test_current_clock_still_flushes_queued_old_content():
    p=Pipeline()
    f.fixed_render_flush(p.render,lambda:1.,p.snapshot)
    assert p.content=='current'

def test_flush_cannot_advance_physical_state():
    p=Pipeline()
    def bad():p.render();p.state+=.1
    with pytest.raises(RuntimeError,match='state'):f.fixed_render_flush(bad,lambda:1.,p.snapshot)

def test_flush_cannot_advance_clock():
    p=Pipeline()
    with pytest.raises(RuntimeError,match='clock'):f.fixed_render_flush(p.render,lambda:p.calls,p.snapshot)
