"""Narrow calibration-only amendment; never changes the formal V2 audit."""
COVERAGE_ERROR = 'V2 formal collection has zero high-angle auxiliary samples (|auxiliary| > 16.1 deg)'
SEQUENCE = 'easy_2_cal_speed_histories'
REQUEST_SHA = 'd2cd085da5bd0300239534e20de3ff4b826415b3f5615d884a26d8ef557bbacd'
def check_eligibility(name, group, request_sha, errors):
    exception = bool(errors)
    if exception:
        assert (name, group, request_sha, errors) == (SEQUENCE, 'calibration', REQUEST_SHA, [COVERAGE_ERROR]), 'Unapproved audit failure'
    return {'calibration_eligible': True, 'formal_schema_passed': not exception, 'coverage_exception': exception}
