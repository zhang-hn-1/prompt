import importlib
import numpy as np
import pytest

def metric():return importlib.import_module('dense_adapter_metrics')

def test_missing_predictions_stay_in_full_denominator():
    e=np.r_[np.zeros(90),np.full(10,np.inf)];s=metric().source_scores(e)
    assert s['n']==100 and s['correct']==90 and s['retained']==90 and s['precision']==1
    assert s['complete_correct_fraction']==.9 and s['gate_pass']

def test_retained_wrong_predictions_cannot_claim_precision_pass():
    s=metric().source_scores(np.r_[np.zeros(90),np.ones(10)])
    assert s['complete_correct_fraction']==.9 and s['precision']==.9 and not s['gate_pass']

def test_no_outputs_has_null_precision_and_p90():
    s=metric().source_scores(np.full(100,np.inf))
    assert s['precision'] is None and s['finite_p90_m'] is None and not s['gate_pass']

def test_small_or_nan_denominator_not_admitted():
    assert not metric().source_scores(np.zeros(99))['gate_pass']
    s=metric().source_scores(np.r_[np.zeros(90),np.full(10,np.nan)])
    assert s['correct']==90 and s['retained']==90
