"""Necessary current-eye evidence classification; never a physical/free-space certificate."""
import copy
import numpy as np

def qualify_packet(packet,left_rgb,right_rgb,left_CAD,right_CAD,rgb_time_s,now,frame):
 t=float(packet['time_s']);rt=float(rgb_time_s);n=float(now);pf=packet['frame']
 if not np.isfinite([t,rt,n]).all() or t!=rt or t!=n or not isinstance(frame,(int,np.integer)) or frame<0 or pf!=frame or abs(t-frame*.1)>1e-5:raise ValueError('same current exposure clocks/frame before payload')
 images=[np.asarray(left_rgb),np.asarray(right_rgb)];masks=[np.asarray(left_CAD),np.asarray(right_CAD)]
 if any(a.shape!=(480,640,3) or a.dtype!=np.uint8 for a in images) or any(a.shape!=(480,640) or a.dtype!=np.bool_ for a in masks) or packet['frame_width']!=640:raise ValueError('native current RGB and boolean CAD required')
 w=np.asarray(packet['world']);sig=np.asarray(packet['sigma_m']);ids=[np.asarray(packet[k]) for k in ['query_pixel_id','partner_pixel_id']]
 if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all() or sig.shape!=(len(w),) or not np.isfinite(sig).all() or np.any(sig<0):raise ValueError('finite provisional points/sigma required')
 if any(a.shape!=(len(w),) or not np.issubdtype(a.dtype,np.integer) or np.any((a<0)|(a>=480*640)) for a in ids):raise ValueError('native pixel IDs required')
 supports=[];reasons=[]
 for a,mask,ix in zip(images,masks,ids):
  y,x=ix//640,ix%640;black=np.all(a[y,x]==0,axis=1);occluded=mask[y,x];supports.append(~black&~occluded);reasons.append(np.where(occluded,'CAD_occluded',np.where(black,'black_RGB_unknown','nonblack_RGB_candidate')))
 eligible=supports[0]&supports[1]
 def select(which,role):
  p=copy.deepcopy(dict(packet))
  for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:p[k]=np.asarray(packet[k])[which].copy()
  p.update(certified_free=False,source_model_admitted=False,measured_geometry_admitted=False,evidence_role=role)
  return p
 return dict(eligible=eligible,left_reason=reasons[0],right_reason=reasons[1],eligible_packet=select(eligible,'image_supported_provisional'),unknown_packet=select(~eligible,'UNKNOWN_unsupported'),raw_packet_preserved=True,certified_free=False,source_model_admitted=False,measured_geometry_admitted=False,nonblack_not_surface_certificate=True)
