"""Prepare assignment-local sensor asset; do not edit the earlier assignment model."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

project = Path(__file__).resolve().parents[2]
repo = project.parents[1]
ET.register_namespace("xacro","http://www.ros.org/wiki/xacro")
tree = ET.parse(repo / "tb_worlds/urdf/gz_waffle.sdf.xacro")
root = tree.getroot()
link = root.find(".//link[@name='camera_link']")
link.find("pose").text = "0.07 0 0.16 0 0 0"
link.find("pose").set("relative_to", "base_link")
sensor = link.find("sensor")
sensor.find("pose").text = "0 0 0 0 0 0"
sensor.find("gz_frame_id").text = "assignment3_camera_optical"
sensor.find("update_rate").text = "15"
sensor.find("camera/image/width").text = "640"
sensor.find("camera/image/height").text = "480"
sensor.find("camera/lens/projection/tx").text = "0"
output = project / "task3_ros2_nav2/generated/robot.sdf.xacro"
output.parent.mkdir(parents=True,exist_ok=True)
tree.write(output,encoding="unicode",xml_declaration=True)
print(output)
params=yaml.safe_load((repo / 'tb_worlds/configs/nav2_params.yaml').read_text())
params.pop('amcl',None)
params.pop('map_server',None)
def sim_time(value):
    if isinstance(value,dict):
        if 'ros__parameters' in value: value['ros__parameters']['use_sim_time']=True
        for item in value.values(): sim_time(item)
sim_time(params)
controller=params['controller_server']['ros__parameters']
controller['controller_frequency']=10.
controller['costmap_update_timeout']=2.
controller['FollowPath']={
    'plugin':'dwb_core::DWBLocalPlanner','min_vel_x':0.,'min_vel_y':0.,'max_vel_x':.18,'max_vel_y':0.,
    'max_vel_theta':.6,'min_speed_xy':0.,'max_speed_xy':.18,'min_speed_theta':0.,'acc_lim_x':.5,
    'acc_lim_y':0.,'acc_lim_theta':1.,'decel_lim_x':-.5,'decel_lim_y':0.,'decel_lim_theta':-1.,
    'vx_samples':12,'vy_samples':1,'vtheta_samples':15,'sim_time':1.5,'linear_granularity':.05,
    'angular_granularity':.025,'transform_tolerance':1.,'xy_goal_tolerance':.15,'trans_stopped_velocity':.02,
    'short_circuit_trajectory_evaluation':True,'stateful':True,
    'critics':['RotateToGoal','Oscillation','BaseObstacle','GoalAlign','PathAlign','PathDist','GoalDist'],
    'BaseObstacle.scale':.02,'PathAlign.scale':32.,'PathAlign.forward_point_distance':.1,
    'GoalAlign.scale':24.,'GoalAlign.forward_point_distance':.1,'PathDist.scale':32.,'GoalDist.scale':24.,
    'RotateToGoal.scale':32.,'RotateToGoal.slowing_factor':5.,'RotateToGoal.lookahead_time':-1.}
controller['general_goal_checker']['xy_goal_tolerance']=.15
params['global_costmap']['global_costmap']['ros__parameters']['static_layer']['subscribe_to_updates']=True
params['planner_server']['ros__parameters']['GridBased']['allow_unknown']=True
for kind in ('local_costmap','global_costmap'):
    params[kind][kind]['ros__parameters']['transform_tolerance']=1.
dest=project/'task3_ros2_nav2/ros2_ws/src/ds685_vslam_nav/config/nav2.yaml'
dest.parent.mkdir(parents=True,exist_ok=True)
dest.write_text(yaml.safe_dump(params,sort_keys=False))
print(dest)
