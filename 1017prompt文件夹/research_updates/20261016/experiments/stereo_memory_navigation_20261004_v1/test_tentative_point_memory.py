import copy,numpy as np,pytest
from tentative_point_memory import TentativePointMemory

def packet(t=0.,frame=0):
 return dict(time_s=t,frame=frame,world=np.array([[1.,2.,3.]]),query_pixel_id=np.array([2]),partner_pixel_id=np.array([3]),input_sha256={"rgb":"a"*64,"pose":"b"*64})

def test_provenance_current_past_expiry_immutable_and_unknown():
 m=TentativePointMemory();p=packet();m.update(p,0);p["world"][:]=99;s=m.at(0);assert s["current_count"]==1 and s["historical_count"]==0
 assert s["records"][0]["world"].tolist()==[[1.,2.,3.]]
 s["records"][0]["world"][:]=98;assert m.at(1)["historical_count"]==1
 assert m.at(10)["historical_count"]==1 and m.at(10.1)["historical_count"]==0
 assert not m.at(1)["certified_free"] and not m.at(1)["source_admitted"]

def test_bad_time_before_payload_and_duplicate_no_state_change():
 class Trap(dict):
  def __getitem__(self,k):
   if k not in ("time_s","frame"):raise AssertionError("future payload read")
   return super().__getitem__(k)
 m=TentativePointMemory()
 with pytest.raises(ValueError):m.update(Trap(time_s=2,frame=1),1)
 with pytest.raises(ValueError):m.update(Trap(time_s=np.nan,frame=1),1)
 m.update(packet(),0)
 with pytest.raises(ValueError):m.update(packet(),0)
 assert m.at(0)["current_count"]==1

def test_whole_prefix_truncation_future_replacement():
 seq=[packet(i,i) for i in range(4)]
 def run(values):
  m=TentativePointMemory();result=[]
  for p in values:m.update(p,p["time_s"]);result.append((m.at(p["time_s"])["current_count"],m.at(p["time_s"])["historical_count"]))
  return result
 baseline=run(seq);altered=copy.deepcopy(seq);altered[3]["world"][:]=1e10
 assert baseline[:3]==run(altered)[:3]==run(seq[:3])

def test_invalid_provenance_or_geometry_rejected():
 m=TentativePointMemory();p=packet();p["world"][0,0]=np.nan
 with pytest.raises(ValueError):m.update(p,0)
 p=packet();p["input_sha256"]={"rgb":"wrong"}
 with pytest.raises(ValueError):m.update(p,0)
