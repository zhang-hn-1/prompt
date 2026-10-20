"""Existing distance-only behavior as callable RED."""
from direction_root_planning import plan_kinodynamic_path
def nominal_edge_cost(length,speed,maximum_speed):return float(length)
