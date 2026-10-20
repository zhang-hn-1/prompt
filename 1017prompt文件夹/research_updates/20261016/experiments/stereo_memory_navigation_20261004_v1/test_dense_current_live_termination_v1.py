import pytest

def test_initial_handoff_abort_is_explicit_complete_stop_not_navigation_timeout():
 from dense_current_live_termination_v1 import validate_termination
 identity={'front_view_acquisition':{'failed':True,'handed_off':False}}
 assert validate_termination({'termination':'REJECT_INITIAL_VIEW_HANDOFF_TIMEOUT','time_s':20.}, {'outcome':'early_stop','scored_end_s':20.},identity,200)=='initial_view_handoff_reject'
 with pytest.raises(ValueError):validate_termination({'termination':'REJECT_INITIAL_VIEW_HANDOFF_TIMEOUT','time_s':20.},{'outcome':'early_stop','scored_end_s':20.},{'front_view_acquisition':{'failed':False,'handed_off':False}},200)

def test_runtime_interruption_or_early_stop_never_counted_as_navigation_completion():
 from dense_current_live_termination_v1 import validate_termination
 with pytest.raises(ValueError):validate_termination({'termination':'timeout35','time_s':6.5},{'outcome':'early_stop','scored_end_s':6.5},{},65)
