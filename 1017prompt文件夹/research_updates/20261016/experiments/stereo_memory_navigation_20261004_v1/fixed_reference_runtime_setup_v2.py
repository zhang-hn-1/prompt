"""Required open-plane runtime geometry setup; same path for both arms, markers hidden."""
def prepare_fixed_runtime(runtime,frozen,episode):
 runtime.set_reference_visualization_path([[w['x_m'],w['y_m']] for w in frozen['waypoints']])
 return runtime.reset(episode)
