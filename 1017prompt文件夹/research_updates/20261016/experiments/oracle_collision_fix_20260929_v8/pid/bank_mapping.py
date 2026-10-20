"""Small measured steady-response inverse, not a dynamic world model."""
import math
import numpy as np

class BankMap:
    def __init__(self,payload):
        self.directions=payload['directions']
        for direction,levels in self.directions.items():
            if direction not in ('forward','reverse'):raise ValueError('unknown direction')
            for level in ('low','high'):
                row=levels[level];b=np.asarray(row['bank_rad']);k=np.asarray(row['curvature_body_1_m'])
                if b.ndim!=1 or len(b)<3 or b.shape!=k.shape or not np.isfinite(b).all() or not np.isfinite(k).all():raise ValueError('invalid calibration')
                if not np.all(np.diff(b)>0) or not np.all(np.diff(k)<0):raise ValueError('nonmonotone calibration; do not repair by sorting curvature')
                if not math.isfinite(row['speed_m_s']) or row['speed_m_s']<=0:raise ValueError('invalid speed')
            if levels['low']['speed_m_s']>=levels['high']['speed_m_s']:raise ValueError('unordered speed endpoints')
    def query(self,main,speed,curvature):
        if not all(math.isfinite(x) for x in (main,speed,curvature)) or main==0 or speed<0:raise ValueError('invalid current query')
        direction='forward' if main>0 else 'reverse';levels=self.directions[direction]
        lo,hi=levels['low']['speed_m_s'],levels['high']['speed_m_s'];weight=float(np.clip((speed-lo)/(hi-lo),0,1));bank=0.;clamped=False
        for key,w in [('low',1-weight),('high',weight)]:
            row=levels[key];k=row['curvature_body_1_m'];b=row['bank_rad']
            bank+=w*float(np.interp(curvature,k[::-1],b[::-1]))
            if w>1e-12:clamped|=curvature<min(k) or curvature>max(k)
        return dict(bank_rad=bank,encoded_aux_rad=bank/.75,curvature_clamped=bool(clamped),speed_clamped=bool(speed<lo or speed>hi),speed_weight=weight,direction=direction,measured_speed_m_s=float(speed),requested_curvature_body_1_m=float(curvature))

def make_policy(base_class,model):
    class MappedBankOracle(base_class):
        policy_name='oracle_calibrated_bank_pid_experimental'
        def reset(self,context):
            super().reset(context);self.mapping_records=[]
        def _request_action(self,main_velocity,auxiliary_angle,*,source):
            record={'source':source,'active':False,'main_request_rad_s':float(main_velocity),'original_aux_rad':float(auxiliary_angle)}
            if source=='tracking' and self._tracking_audit.get('valid',False):
                speed=float(self._tracking_audit['world_xy_speed_m_s'])
                try:choice=model.query(main_velocity,speed,self._desired_curvature_1_m)
                except KeyError:record['fallback']='unsupported_drive_direction'
                else:
                    correction=float(self._tracking_audit['yaw_rate_feedback_auxiliary_rad'])
                    auxiliary_angle=choice['encoded_aux_rad']+correction
                    record.update(choice);record.update(active=True,outer_yaw_correction_rad=correction,new_aux_rad=float(auxiliary_angle),bank_target_after_outer_correction_rad=float(.75*np.clip(auxiliary_angle,self.protocol.minimum[1],self.protocol.maximum[1])))
                    self._auxiliary_target_angle_rad=float(auxiliary_angle)
            self.mapping_records.append(record)
            self._tracking_audit['calibrated_bank_mapping']=record.copy()
            return super()._request_action(main_velocity,auxiliary_angle,source=source)
    return MappedBankOracle
