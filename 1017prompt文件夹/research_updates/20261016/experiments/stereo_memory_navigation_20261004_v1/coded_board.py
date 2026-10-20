"""RGB-only coded-square measurement; source geometry is a separate offline interface."""
import cv2
import numpy as np

def decode_patch(patch,codes):
    image=cv2.resize(np.asarray(patch,np.uint8),(140,140),interpolation=cv2.INTER_LINEAR)
    values=np.array([[np.median(image[r*20+6:r*20+14,c*20+6:c*20+14]) for c in range(7)] for r in range(7)],np.uint8)
    if int(values.max())-int(values.min())<40:return None
    threshold,_=cv2.threshold(values,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    bits=(values>threshold).astype(np.uint8)
    border=np.r_[bits[0],bits[-1],bits[1:-1,0],bits[1:-1,-1]]
    if np.count_nonzero(border)>2:return None
    scores=[]
    for k in range(4):
        distances=np.count_nonzero(np.asarray(codes)!=np.rot90(bits[1:-1,1:-1],k),axis=(1,2))
        scores.extend((int(d),i,k) for i,d in enumerate(distances))
    scores.sort()
    if scores[0][0]>1 or scores[1][0]-scores[0][0]<3:return None
    return dict(id=scores[0][1],rotate_ccw=scores[0][2],hamming=scores[0][0])

def detect_tags(rgb,mask,codes):
    gray=cv2.cvtColor(np.asarray(rgb),cv2.COLOR_RGB2GRAY)
    binary=cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,31,7)
    contours,_=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    found={}
    for contour in contours:
        if cv2.contourArea(contour)<400:continue
        q=cv2.approxPolyDP(contour,.035*cv2.arcLength(contour,True),True)
        if len(q)!=4 or not cv2.isContourConvex(q):continue
        q=q.reshape(4,2).astype(np.float32)
        centre=q.mean(0);q=q[np.argsort(np.arctan2(q[:,1]-centre[1],q[:,0]-centre[0]))]
        q=np.roll(q,-np.argmin(q.sum(1)),axis=0)
        if np.min(np.linalg.norm(q-np.roll(q,1,axis=0),axis=1))<20:continue
        support=np.zeros(gray.shape,np.uint8);cv2.fillConvexPoly(support,q.astype(int),1)
        if np.any(np.asarray(mask,bool)&(support!=0)):continue
        dest=np.array([[0,0],[139,0],[139,139],[0,139]],np.float32)
        patch=cv2.warpPerspective(gray,cv2.getPerspectiveTransform(q,dest),(140,140))
        decoded=decode_patch(patch,codes)
        if decoded is None:continue
        refined=cv2.cornerSubPix(gray,q.reshape(-1,1,2),(3,3),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.01)).reshape(4,2)
        canonical=np.roll(refined,-decoded['rotate_ccw'],axis=0)
        item=dict(**decoded,corners=canonical,area=float(cv2.contourArea(contour)))
        old=found.get(decoded['id'])
        if old is None or item['area']>old['area']:found[decoded['id']]=item
    return [found[i] for i in sorted(found)]

def world_corners(centre,side):
    p=np.asarray(centre,float);s=side/2
    return p+np.array([[0,-s,s],[0,s,s],[0,s,-s],[0,-s,-s]])

def project_points(points,pose,focal,cx,cy):
    pose=np.asarray(pose,float);rotation=pose[:3,:3]@np.diag([1.,-1.,-1.])
    p=np.linalg.solve(rotation,(np.asarray(points)-pose[:3,3]).T).T
    with np.errstate(divide='ignore',invalid='ignore'):
        xy=p[:,:2]/p[:,2,None]*focal+np.array([cx,cy])
    return xy,p[:,2]

def tag_cell_corners(centre,row,col):
    p=np.asarray(centre,float)+np.array([0,.14-.04*col,.14-.04*row])
    return p+np.array([[0,0,0],[0,-.04,0],[0,-.04,-.04],[0,0,-.04]])
