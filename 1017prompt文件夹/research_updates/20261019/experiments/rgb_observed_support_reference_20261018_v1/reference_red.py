"""Actual RGB cost + float64 threepath reference, NOT OpenCV reproduction."""
import numpy as np

def build_cost(left,right,disparities):
    raise NotImplementedError('BT RGB costs not implemented')

def aggregate(cost,support):
    raise NotImplementedError('batched reference aggregation not implemented')

def decode(total,disparities):
    raise NotImplementedError('integer uniqueness reference not implemented')
