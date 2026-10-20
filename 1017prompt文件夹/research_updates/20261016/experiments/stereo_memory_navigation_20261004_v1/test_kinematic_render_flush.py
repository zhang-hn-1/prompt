"""CPU-world rendering must refresh articulation FK without advancing physics."""
import pytest
from types import SimpleNamespace
from kinematic_render_flush import kinematic_render_flush

def test_cpu_render_uses_current_articulation_pose():
    state=[2.];cached=[1.];frames=[];updates=[]
    def update():cached[:]=state[:];updates.append(1)
    world=SimpleNamespace(physics_sim_view=SimpleNamespace(update_articulations_kinematic=update),render=lambda:frames.append(cached[0]))
    kinematic_render_flush(world,lambda:(2.,),lambda:[state[:]])
    assert frames==[2.,2.,2.] and len(updates)==3 and state==[2.]

def test_refresh_that_changes_physics_is_rejected():
    state=[2.]
    world=SimpleNamespace(physics_sim_view=SimpleNamespace(update_articulations_kinematic=lambda:state.__setitem__(0,3.)),render=lambda:None)
    with pytest.raises(RuntimeError,match='physical state'):kinematic_render_flush(world,lambda:(2.,),lambda:[state[:]])

def test_missing_physics_view_cannot_claim_refresh():
    world=SimpleNamespace(physics_sim_view=None,render=lambda:None)
    with pytest.raises(RuntimeError):kinematic_render_flush(world,lambda:(2.,),lambda:[[2.]])
