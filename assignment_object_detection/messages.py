"""ROS 2 CDR wire types. Fixed covariance arrays must NOT be sequences."""
from __future__ import annotations
from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import array, sequence, uint8, uint32, int32, float64

@dataclass
class Time(IdlStruct):
    sec: int32
    nanosec: uint32

@dataclass
class Header(IdlStruct):
    stamp: Time
    frame_id: str

@dataclass
class Image(IdlStruct):
    header: Header
    height: uint32
    width: uint32
    encoding: str
    is_bigendian: uint8
    step: uint32
    data: sequence[uint8]

@dataclass
class Vector3(IdlStruct):
    x: float64
    y: float64
    z: float64

@dataclass
class Quaternion(IdlStruct):
    x: float64
    y: float64
    z: float64
    w: float64

@dataclass
class Pose(IdlStruct):
    position: Vector3
    orientation: Quaternion

@dataclass
class PoseWithCovariance(IdlStruct):
    pose: Pose
    covariance: array[float64, 36]

@dataclass
class Twist(IdlStruct):
    linear: Vector3
    angular: Vector3

@dataclass
class TwistWithCovariance(IdlStruct):
    twist: Twist
    covariance: array[float64, 36]

@dataclass
class Odometry(IdlStruct):
    header: Header
    child_frame_id: str
    pose: PoseWithCovariance
    twist: TwistWithCovariance

@dataclass
class Transform(IdlStruct):
    translation: Vector3
    rotation: Quaternion

@dataclass
class TransformStamped(IdlStruct):
    header: Header
    child_frame_id: str
    transform: Transform

@dataclass
class TFMessage(IdlStruct):
    transforms: sequence[TransformStamped]

# pycdr2's metaclass predates Python 3.14's lazy annotations. Postponed
# annotations preserve the fields; resolve them before pycdr2 builds codecs.
import typing
for _type in (Time,Header,Image,Vector3,Quaternion,Pose,PoseWithCovariance,Twist,
              TwistWithCovariance,Odometry,Transform,TransformStamped,TFMessage):
    _type.__annotations__ = typing.get_type_hints(_type, include_extras=True)
