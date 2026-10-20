def choose(choices,eligible,candidate):
 if not eligible:return 'brake'
 labels=['turn','hold'] if candidate else ['turn']
 safe=[k for k in labels if k in choices and choices[k]['clearance_m']>0]
 return min(safe,key=lambda k:choices[k]['goal_distance_m']) if safe else 'brake'
