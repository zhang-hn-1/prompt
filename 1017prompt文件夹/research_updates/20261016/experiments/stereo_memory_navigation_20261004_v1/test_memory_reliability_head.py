import numpy as np,torch,pytest
from memory_reliability_head import ReliabilityHead

def test_deterministic_finite_probabilities_and_safe_state_roundtrip(tmp_path):
 torch.manual_seed(7);a=ReliabilityHead(np.zeros(21),np.ones(21));x=np.arange(63,dtype=float).reshape(3,21)/63;p=a.predict(x);assert p.shape==(3,) and ((p>=0)&(p<=1)).all();f=tmp_path/'weights.pt';torch.save(a.state_dict(),f);b=ReliabilityHead(np.zeros(21),np.ones(21));b.load_state_dict(torch.load(f,weights_only=True,map_location='cpu'));np.testing.assert_equal(p,b.predict(x))
@pytest.mark.parametrize('x',[np.zeros((3,22)),np.ones((2,21))*np.nan,{'features':np.zeros((1,21)),'future_GT':None}])
def test_only_finite_21feature_matrix(x):
 a=ReliabilityHead(np.zeros(21),np.ones(21))
 with pytest.raises(ValueError):a.predict(x)
