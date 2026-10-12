"""Timestamp-based bounded buffering; callback arrival order is irrelevant."""
import math
import numpy as np
from .geometry import _timestamp_ns

class TimestampBuffer:
    def __init__(self, capacity=180):
        self.capacity = capacity
        self.samples = []

    def add(self, message):
        timestamp = _timestamp_ns(message)
        if timestamp is None:
            raise ValueError('Missing ROS timestamp')
        self.samples.append((timestamp, message))
        self.samples.sort(key=lambda item: item[0])
        self.samples = self.samples[-self.capacity:]

    def nearest(self, timestamp, tolerance_ns=200_000_000):
        if not self.samples:
            return None
        stamp, msg = min(self.samples, key=lambda item: abs(item[0]-timestamp))
        return msg if abs(stamp-timestamp) <= tolerance_ns else None

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def image_array(msg):
    """Honor ROS image row padding and byte order; return BGR or float32 meters."""
    if msg.encoding not in ('rgb8', 'bgr8', '32FC1'):
        raise ValueError(f'Unsupported image encoding {msg.encoding}')
    depth = msg.encoding == '32FC1'
    dtype = np.dtype('>f4' if msg.is_bigendian else '<f4') if depth else np.dtype('u1')
    channels = 1 if depth else 3
    row_bytes = msg.width * channels * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) < msg.height * msg.step:
        raise ValueError('Truncated image or invalid row stride')
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = raw[:, :row_bytes].copy().view(dtype)
    if depth:
        return pixels.reshape(msg.height, msg.width).astype(np.float32)
    pixels = pixels.reshape(msg.height, msg.width, 3)
    return pixels[:, :, ::-1].copy() if msg.encoding == 'rgb8' else pixels
