import json
from types import SimpleNamespace
import log_before_close as m

def test_mapping_is_written_before_native_close_can_terminate(tmp_path):
 assert hasattr(m,'wrap_runtime'), 'pre-close persistence missing'
 instance=SimpleNamespace(mapping_records=[dict(active=True)])
 class Native:
  def close(self,return_code=0):
   assert json.loads((tmp_path/'mapping_case.json').read_text())==instance.mapping_records
 cls=m.wrap_runtime(Native,lambda:instance,tmp_path);runtime=cls();runtime.bank_episode_id='case';runtime.close()
