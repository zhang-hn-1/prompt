"""One frozen class/score adapter; labels are MODEL hypotheses, never measured depth."""
import numpy as np

def hypothesis_from_rotated(labels,confidence,mask,positive_ids):
    labels=np.asarray(labels);confidence=np.asarray(confidence,float);mask=np.asarray(mask,bool)
    if labels.ndim!=2 or confidence.shape!=labels.shape or not np.isfinite(confidence).all() or (confidence<0).any() or (confidence>1).any():raise ValueError('finite per-pixel labels/confidence')
    native_labels=np.rot90(labels,-1).copy();native_confidence=np.rot90(confidence,-1).copy()
    if native_labels.shape!=mask.shape:raise ValueError('native mask shape must match inverse-rotated image')
    selected=np.isin(native_labels,positive_ids)&(native_confidence>=.9)&~mask
    return dict(label_native=native_labels,confidence_native=native_confidence,structural_hypothesis=selected,role='unresolved_structural_model_hypothesis',observed_depth=False,certified_free=False)
