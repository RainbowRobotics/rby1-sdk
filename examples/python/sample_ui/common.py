import time
import numpy as np
import math as m
import google.protobuf.timestamp_pb2 as timestamp_pb2

GREEN_CODE = '#57965C'
RED_CODE = '#C94F4F'

CM_STATE_STRING = ["Unknown", "Idle", "Enabled", "MinorFault", "MajorFault"]


def get_current_time_in_timestamp():
    ts = timestamp_pb2.Timestamp()
    ts.FromNanoseconds(time.monotonic_ns())
    return ts
  
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])