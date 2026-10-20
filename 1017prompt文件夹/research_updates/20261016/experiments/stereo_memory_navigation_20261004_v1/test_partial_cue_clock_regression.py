import pytest
import partial_cue_source_v2 as m

def test_real_runtime_float32_clock_reaches_native_payload_contract():
 # Exact measured failing source epoch, no renderer or scientific labels.
 p={'time_s':4.499999899417162,'frame':45,'frame_width':640}
 # None RGB intentionally stops after clocks; identity check must accept.
 with pytest.raises(TypeError):m.PartialCueSetSource(.2).update(p,None,None,None,False,p['time_s'],45,None,None,None)

def test_real_one_frame_mismatch_still_rejected_before_payload():
 p={'time_s':4.499999899417162,'frame':44,'frame_width':640}
 with pytest.raises(ValueError):m.PartialCueSetSource(.2).update(p,None,None,None,False,p['time_s'],45,None,None,None)
