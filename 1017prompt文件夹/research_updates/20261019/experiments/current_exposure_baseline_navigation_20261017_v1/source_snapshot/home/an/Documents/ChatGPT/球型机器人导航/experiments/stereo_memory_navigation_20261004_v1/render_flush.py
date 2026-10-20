"""Fixed render-only barrier. No depth/GT-driven loop or future physical inputs.

Live known-plane audit showed the first current-timestamp annotator data still
held old geometry. Three unconditional renders cover update plus confirmation.
It is an acquisition correction; physics time and state must be invariant.
"""
import numpy as np

def fixed_render_flush(render,clock,snapshot):
    initial_clock=clock();initial=[np.asarray(x).copy() for x in snapshot()]
    for _ in range(3):
        render()
        if clock()!=initial_clock:raise RuntimeError('render advanced physics clock')
        after=snapshot()
        if len(initial)!=len(after) or not all(np.array_equal(a,b) for a,b in zip(initial,after)):
            raise RuntimeError('render changed physical state')
