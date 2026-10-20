from local_terminal_capture_context_v1 import LocalTerminalCapture

def test_five_low_speed_and_true_goal_not_cursor_or_ever_qualified():
 c=LocalTerminalCapture()
 for i in range(4):assert not c.observe(i*.1,[0,0],[0,0],[.1,0],1)['qualified_now']
 assert c.observe(.4,[0,0],[0,0],[.1,0],1)['qualified_now']
 assert not c.observe(.5,[0,0],[.06,0],[.1,0],1)['qualified_now']
 assert not c.observe(.6,[0,0],[0,0],[1,0],1)['qualified_now']

def test_new_reference_or_missing_sample_resets_count():
 c=LocalTerminalCapture()
 for i in range(5):c.observe(i*.1,[0,0],[0,0],[.1,0],1)
 assert c.observe(.5,[0,0],[0,0],[.1,0],2)['count']==1
 assert c.observe(.7,[0,0],[0,0],[.1,0],2)['count']==1
