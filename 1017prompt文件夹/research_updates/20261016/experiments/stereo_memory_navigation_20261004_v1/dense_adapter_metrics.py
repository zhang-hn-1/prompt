"""All-query frozen source gates; no performance rescue from empty outputs."""
import numpy as np

def source_scores(errors):
 e=np.asarray(errors,float);n=e.size;finite=np.isfinite(e);retained=int(finite.sum());correct=int((finite&(e<=.1)).sum());fraction=None if n==0 else correct/n;precision=None if not retained else correct/retained;p90=None if not retained else float(np.percentile(e[finite],90))
 return dict(n=n,correct=correct,retained=retained,complete_correct_fraction=fraction,precision=precision,finite_p90_m=p90,coverage=None if not n else retained/n,adequate=n>=100,gate_pass=bool(n>=100 and fraction>=.9 and precision is not None and precision>=.99 and p90 is not None and p90<=.1))
