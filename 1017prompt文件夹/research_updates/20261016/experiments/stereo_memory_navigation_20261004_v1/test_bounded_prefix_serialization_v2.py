import pytest

def test_json_roundtrip_preserves_values_but_not_container_identity():
 from assess_bounded_prefix_e1_v2 import serialized_equal
 assert serialized_equal({'requests':[(.5,0.)]},{'requests':[[.5,0.]]})
 assert not serialized_equal({'requests':[(.5,0.)]},{'requests':[[.5,.01]]})
