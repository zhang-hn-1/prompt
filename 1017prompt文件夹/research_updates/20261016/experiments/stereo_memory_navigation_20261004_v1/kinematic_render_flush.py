"""Refresh measured articulation FK before render on CPU and CUDA worlds.

Isaac's installed SimulationContext.render refreshes FK only on CUDA worlds.
The CPU benchmark steps physics without render; its camera parent otherwise
renders the preceding control block pose. This updates execution caches only.
"""
from render_flush import fixed_render_flush

def kinematic_render_flush(world,clock,snapshot):
    view=world.physics_sim_view
    if view is None:raise RuntimeError('physics view required for current articulation FK')
    def render():
        view.update_articulations_kinematic()
        world.render()
    fixed_render_flush(render,clock,snapshot)
