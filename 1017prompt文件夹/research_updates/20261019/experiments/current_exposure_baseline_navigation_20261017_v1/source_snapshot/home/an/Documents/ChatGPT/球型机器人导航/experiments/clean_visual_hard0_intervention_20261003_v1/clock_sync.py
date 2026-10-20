"""Require an image timestamp matching the current physics state; no stepping."""
import math

def ensure_current_frames(cameras,raw_time_s,render):
    t=float(raw_time_s)
    if not math.isfinite(t):raise ValueError('finite physics clock required')
    for attempt in range(4):
        frames=[c.get_current_frame() for c in cameras]
        ages=[t-float(f['rendering_time']) for f in frames]
        if all(math.isfinite(age) and abs(age)<=1e-5 for age in ages):return frames
        if any(not math.isfinite(age) or age < -1e-5 for age in ages):raise RuntimeError('image is invalid or from a future physics state')
        if attempt<3:render()
    raise RuntimeError('current rendered images unavailable after 3 non-physics renders')
