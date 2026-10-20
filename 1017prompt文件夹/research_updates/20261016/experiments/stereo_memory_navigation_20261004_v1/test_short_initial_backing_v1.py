from types import SimpleNamespace as S
import numpy as np
from short_initial_backing_v1 import initial_reverse_edges,prefer_shorter_initial_backing

def plan(dirs):return S(status='success',waypoints=tuple(S(drive_direction=d) for d in dirs))
def test_initial_edges_ignore_terminal_waypoint_and_do_not_count_later_reverse():
 assert initial_reverse_edges(plan(['reverse','forward','reverse','forward']))==1
 assert initial_reverse_edges(plan(['forward','reverse','reverse']))==0
 assert initial_reverse_edges(plan(['reverse','reverse','reverse']))==2

def test_only_strictly_less_initial_backing_can_replace_original():
 b=plan(['reverse']*5+['forward']*3);a=plan(['reverse','forward','forward'])
 assert prefer_shorter_initial_backing(b,a) is a
 assert prefer_shorter_initial_backing(b,None) is b
 assert prefer_shorter_initial_backing(b,plan(['reverse']*5+['forward'])) is b
 assert prefer_shorter_initial_backing(plan(['forward','forward']),a).waypoints[0].drive_direction=='forward'
