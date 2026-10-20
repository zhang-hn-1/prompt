"""Breaks caught: unseen volume certification and ignoring stopping displacement."""
import numpy as np
import primitive_planner as p

def grid(value=0):
    return np.full((50,50),value,np.int8),np.array([-2.,-2.]),.1

def test_unknown_support_cannot_become_a_certified_candidate():
    labels,origin,res=grid(-1)
    result=p.assess_support(np.array([[0.,0.],[.3,0.]]),labels,origin,res,.6)
    assert not result['support_passed'] and result['unknown_cells']>0
    assert result['full_3d_certified'] is False

def test_stopping_tail_obstacle_invalidates_motion():
    labels,origin,res=grid();labels[30,20]=1  # x=1m
    # Propulsion segment ends at .2m; coasting then reaches .5m.
    first=p.assess_support(np.array([[0.,0.],[.2,0.]]),labels,origin,res,.6)
    full=p.assess_support(np.array([[0.,0.],[.2,0.],[.5,0.]]),labels,origin,res,.6)
    assert first['support_passed'] and not full['support_passed']
    assert full['occupied_cells']>0

def test_support_and_body_collision_certificate_are_distinct():
    labels,origin,res=grid()
    result=p.assess_support(np.array([[0.,0.],[.3,0.]]),labels,origin,res,.6)
    assert result['support_passed']
    assert not result['full_3d_certified']

def test_out_of_map_is_unknown():
    labels,origin,res=grid()
    result=p.assess_support(np.array([[3.,3.]]),labels,origin,res,.6)
    assert not result['support_passed'] and result['unknown_cells']>0
