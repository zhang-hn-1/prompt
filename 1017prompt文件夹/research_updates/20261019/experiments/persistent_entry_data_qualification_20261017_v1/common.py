from pathlib import Path
import json,hashlib,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1]
NAMES=('front_left','front_right','rear_left','rear_right');PARTNER=(1,0,3,2)
FEATURES=['log_current_points','current_eye_fraction','log_lifetime_capture_count','recent_time_span_over10','depth_mean_over3','depth_std_over3','estimated_height_mean_over1p5','estimated_height_std_over1p5','body_range_mean_over3','abs_disparity_mean_over448','abs_disparity_std_over448','gray_mean_over255','gray_std_over255','gradient_mean_over255','paired_RGB_MAD_over255','CAD_distance_over640','current_XY_scatter_over0p1','recent_height_mean_std_over1p5','recent_body_displacement_over3','recent_depth_mean_std_over3','recent_eye_fraction','recent_gray_mean_std_over255','previous_capture_age_over10','current_pair_fraction']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return json.loads(Path(p).read_text())
def write(p,r):Path(p).write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
