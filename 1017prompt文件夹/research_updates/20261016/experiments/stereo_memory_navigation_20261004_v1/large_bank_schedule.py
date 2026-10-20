"""Exactly two targeted open-plane turn-stop measurements; no internal reset."""
import numpy as np

def schedule(name):
    if name not in ('positive','negative'):raise ValueError(name)
    commands=np.zeros((180,5,2),np.float32)
    commands[60:120,:,0]=.3
    commands[60:120,:,1]=(1 if name=='positive' else -1)*np.pi/6
    return commands.reshape(180,10)
