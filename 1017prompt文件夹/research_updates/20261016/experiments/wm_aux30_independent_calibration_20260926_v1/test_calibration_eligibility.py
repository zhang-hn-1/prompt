import pytest
from calibration_eligibility import check_eligibility
ERROR='V2 formal collection has zero high-angle auxiliary samples (|auxiliary| > 16.1 deg)'
ID='easy_2_cal_speed_histories'
SHA='d2cd085da5bd0300239534e20de3ff4b826415b3f5615d884a26d8ef557bbacd'
def test_exact_known_coverage_gap_is_separate_from_formal_pass():
 assert check_eligibility(ID,'calibration',SHA,[ERROR]) == {'calibration_eligible':True,'formal_schema_passed':False,'coverage_exception':True}
@pytest.mark.parametrize('name,group,digest,errors',[
 ('another','calibration',SHA,[ERROR]),(ID,'fresh',SHA,[ERROR]),(ID,'calibration','changed',[ERROR]),
 (ID,'calibration',SHA,[ERROR,'missing image']), (ID,'calibration',SHA,['nonfinite data']),
])
def test_never_waives_other_data_or_failures(name,group,digest,errors):
 with pytest.raises(AssertionError):check_eligibility(name,group,digest,errors)
def test_unmodified_formal_pass():
 assert check_eligibility('ordinary','fresh','hash',[]) == {'calibration_eligible':True,'formal_schema_passed':True,'coverage_exception':False}
