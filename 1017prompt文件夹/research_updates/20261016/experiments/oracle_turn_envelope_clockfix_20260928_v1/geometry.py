import numpy as np

def relocate(xy,source_yaw,target_xy,target_yaw):
    angle=target_yaw-source_yaw
    r=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    xy=np.asarray(xy)
    return (xy-xy[0])@r.T+np.asarray(target_xy)
