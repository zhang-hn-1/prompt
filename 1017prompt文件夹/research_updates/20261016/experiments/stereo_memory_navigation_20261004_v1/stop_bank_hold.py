"""One causal request-source intervention, with no physical-state reset."""
import numpy as np

def hold_bank_schedule(past_requests):
    prefix=np.asarray(past_requests,np.float32)
    if prefix.shape!=(75,10) or not np.isfinite(prefix).all():
        raise ValueError('exactly75 completed request blocks required; no future requests')
    commands=np.zeros((135,10),np.float32)
    commands[:75]=prefix
    commands[75:,1::2]=prefix[-1,1::2]
    return commands
