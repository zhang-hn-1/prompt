"""Support derived from complete records; raw-record authenticity handled separately."""
import math,struct
TEMPLATES={'stop':(0.,0.),'forward':(.3,0.),'left':(.3,.1),'right':(.3,-.1)}
def f32(v):return struct.unpack('<f',struct.pack('<f',float(v)))[0]
def derive_support(records):
 cells=set()
 for r in records:
  if r['scorable'] is not True:continue
  program=r['program'];v=r['estimated_velocity']
  if len(program)!=25 or len(v)!=2 or not all(math.isfinite(x) for x in v):continue
  speed=math.hypot(*v)
  state='low' if speed<=.05 else ('moving' if speed<=.5 else None)
  if state is None:continue
  for name,pair in TEMPLATES.items():
   if all(tuple(f32(x) for x in p)==tuple(f32(x) for x in pair) for p in program):cells.add((name,state))
 return dict(programs=sorted({a for a,b in cells}),state_bins=sorted({b for a,b in cells}),support_cells=[list(c) for c in sorted(cells)],support_from='complete_scorable_only')
