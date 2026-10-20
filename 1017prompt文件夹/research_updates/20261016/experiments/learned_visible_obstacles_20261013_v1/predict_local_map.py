"""Diagnostic inference only: rejected development checkpoint cannot rank commands."""
from pathlib import Path
import argparse,json,hashlib,cv2,numpy as np,torch
from model import VisibleMapNet,tentative
D=Path(__file__).resolve().parent

def predict(left_rgb,right_rgb,gravity,time_s):
 for a in [left_rgb,right_rgb]:
  if a.shape!=(480,640,3) or a.dtype!=np.uint8:raise ValueError('paired RGB must be uint8 480x640x3')
 g=np.asarray(gravity,np.float32)
 if g.shape!=(3,) or not np.isfinite(g).all() or abs(np.linalg.norm(g)-1)>.001 or not np.isfinite(time_s):raise ValueError('finite current unit gravity/time')
 x=np.concatenate([cv2.resize(a,(160,120),interpolation=cv2.INTER_AREA) for a in [left_rgb,right_rgb]],2).transpose(2,0,1);m=VisibleMapNet();m.load_state_dict(torch.load(D/'model_final.pt',map_location='cpu',weights_only=True)['state_dict']);m.eval()
 with torch.no_grad():labels,p=tentative(m(torch.from_numpy(x[None]).float()/255,torch.from_numpy(g[None])))
 return dict(labels=labels[0].numpy(),probabilities=p[0].numpy(),time_s=float(time_s),resolution_m=.1,local_origin_forward_lateral_m=np.array([0.,-3.]),certified_free=False,allow_command_consumer=False,admission_state='rejected_development_only',height_bins_m=np.array([[-.03,.03],[.05,.5],[.5,1],[1,1.5]]),height_bin_class_ids=np.array([1,2,3,4]),class_names=np.array(["unknown","visible_ground_not_free","obstacle_005_05","obstacle_05_1","obstacle_1_15"]),checkpoint_sha256=hashlib.sha256((D/"model_final.pt").read_bytes()).hexdigest())
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('input');ap.add_argument('output');a=ap.parse_args();v=np.load(a.input)
 if set(v.files)!={'left_rgb','right_rgb','gravity','time_s'}:raise ValueError('input whitelist: only current pairedRGB/gravity/time')
 out=predict(v['left_rgb'],v['right_rgb'],v['gravity'],float(v['time_s']));np.savez_compressed(a.output,**out);print(json.dumps(dict(status='diagnostic_prediction_saved',navigation_admitted=False)))
