import numpy as np
def schedule():
    commands=np.zeros((320,5,2),np.float32)
    commands[60:260,:,0]=-.3
    return commands.reshape(320,10)
