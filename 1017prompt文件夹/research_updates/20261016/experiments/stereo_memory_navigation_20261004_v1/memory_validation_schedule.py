"""Two predeclared sensor-collection commands; current clock only."""
from retreat_observation_schedule import request as retreat_request

def request(current_time_s,kind):
 if kind not in ('far','near'):raise ValueError('predeclared collection kind required')
 a=retreat_request(current_time_s)
 if kind=='far':a[::2]*=-1
 return a
