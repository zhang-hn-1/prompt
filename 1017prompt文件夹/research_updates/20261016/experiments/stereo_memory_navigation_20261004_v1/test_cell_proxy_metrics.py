import numpy as np
from cell_proxy_metrics import confusion

def test_empty_predictions_are_undefined_not_perfect():
 d=confusion(np.array([False,False]),np.array([True,False]))
 assert d['precision'] is None and d['recall']==0 and d['FN']==1

def test_no_positive_labels_recall_is_not_a_pass():
 d=confusion(np.array([True,False]),np.array([False,False]))
 assert d['recall'] is None and d['precision']==0 and d['FP']==1

def test_confusion_retains_all_error_counts():
 d=confusion(np.array([True,True,False,False]),np.array([True,False,True,False]))
 assert [d[k] for k in ['TP','FP','FN','TN']]==[1,1,1,1]
 assert d['precision']==d['recall']==.5
