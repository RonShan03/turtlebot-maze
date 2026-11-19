"""Pure algorithms, independent of ROS. T_parent_child maps child points to parent."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(position, quaternion):
    T=np.eye(4); T[:3,:3]=Rotation.from_quat(quaternion).as_matrix(); T[:3,3]=position
    return T


def planar(T):
    yaw=math.atan2(T[1,0],T[0,0]); c,s=math.cos(yaw),math.sin(yaw)
    result=np.eye(4); result[:2,:2]=[[c,-s],[s,c]]; result[:2,3]=T[:2,3]
    return result


class LocalizationBridge:
    def __init__(self): self.alignment=None

    def update(self, T_slam_camera, T_base_camera, T_odom_base):
        if self.alignment is None:
            self.alignment = T_odom_base @ T_base_camera @ np.linalg.inv(T_slam_camera)
        T_map_base = self.alignment @ T_slam_camera @ np.linalg.inv(T_base_camera)
        return planar(T_map_base @ np.linalg.inv(T_odom_base)), planar(T_map_base)


def bresenham(x0,y0,x1,y1):
    dx,dy=abs(x1-x0),-abs(y1-y0); sx=1 if x0<x1 else -1; sy=1 if y0<y1 else -1
    err=dx+dy
    while True:
        yield x0,y0
        if x0==x1 and y0==y1: break
        e=2*err
        if e>=dy: err+=dy; x0+=sx
        if e<=dx: err+=dx; y0+=sy


class OccupancyMapper:
    """Ray insertion only. Never performs scan matching or estimates robot pose."""
    def __init__(self, resolution=.05, size=40.):
        self.resolution=resolution; self.width=int(round(size/resolution)); self.origin=-size/2
        self.log_odds=np.zeros((self.width,self.width),np.float32)
        self.observed=np.zeros((self.width,self.width),bool)

    def cell(self, x,y):
        return int(math.floor((x-self.origin)/self.resolution)),int(math.floor((y-self.origin)/self.resolution))

    def inside(self,x,y): return 0<=x<self.width and 0<=y<self.width

    def insert(self,T_map_scan,ranges,angle_min,angle_increment,range_min,range_max):
        x0,y0=self.cell(*T_map_scan[:2,3])
        if not self.inside(x0,y0): return
        for i,r in enumerate(ranges):
            if np.isnan(r) or r<range_min or r==float('-inf'): continue
            hit=np.isfinite(r) and r<range_max
            distance=min(float(r),range_max)
            a=angle_min+i*angle_increment
            endpoint=T_map_scan @ np.array([distance*math.cos(a),distance*math.sin(a),0.,1.])
            x1,y1=self.cell(*endpoint[:2]); cells=list(bresenham(x0,y0,x1,y1))
            for x,y in cells[:-1]:
                if self.inside(x,y): self.log_odds[y,x]-=.4; self.observed[y,x]=True
            if self.inside(x1,y1):
                self.log_odds[y1,x1]+=.85 if hit else -.4; self.observed[y1,x1]=True
        np.clip(self.log_odds,-4,4,out=self.log_odds)

    def grid(self):
        values=np.rint(100/(1+np.exp(-self.log_odds))).astype(np.int8)
        values[~self.observed]=-1
        return values
