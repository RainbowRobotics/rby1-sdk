import time
import google.protobuf.timestamp_pb2 as timestamp_pb2

GREEN_CODE = '#57965C'
RED_CODE = '#C94F4F'

CM_STATE_STRING = ["Unknown", "Idle", "Enabled", "MinorFault", "MajorFault"]


def get_current_time_in_timestamp():
    ts = timestamp_pb2.Timestamp()
    ts.FromNanoseconds(time.monotonic_ns())
    return ts
