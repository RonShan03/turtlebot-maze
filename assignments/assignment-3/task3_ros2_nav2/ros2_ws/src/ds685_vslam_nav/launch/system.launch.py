import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('ds685_vslam_nav')
    tb=get_package_share_directory('tb_worlds')
    nav=get_package_share_directory('nav2_bringup')
    root='/work/assignments/assignment-3'
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(tb,'launch','tb_world.launch.py')),
        launch_arguments={'headless':'true','robot_sdf':root+'/task3_ros2_nav2/generated/robot.sdf.xacro',
                          'world':os.path.join(tb,'worlds','sim_house_enhanced.sdf.xacro'),
                          'texture_base':os.path.join(tb,'worlds','textures')}.items())
    mount=Node(package='tf2_ros',executable='static_transform_publisher',name='assignment3_camera_mount',
               arguments=['--x','0.07','--y','0','--z','0.16','--frame-id','base_link','--child-frame-id','assignment3_camera'])
    optical=Node(package='tf2_ros',executable='static_transform_publisher',name='assignment3_optical_rotation',
                 arguments=['--roll','-1.5707963267948966','--pitch','0','--yaw','-1.5707963267948966',
                            '--frame-id','assignment3_camera','--child-frame-id','assignment3_camera_optical'])
    info=Node(package='ros_gz_bridge',executable='parameter_bridge',name='assignment3_camera_info',
              arguments=['/camera/realsense_d435i/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
              remappings=[('/camera/realsense_d435i/camera_info','/camera/color/camera_info')],parameters=[{'use_sim_time':True}])
    adapter=Node(package='ds685_vslam_nav',executable='camera_adapter',parameters=[{'use_sim_time':True}])
    wrapper=ExecuteProcess(cmd=['bash','-lc',
        'source /opt/ros/jazzy/setup.bash; source /opt/vslam/ros_ws/install/setup.bash; '
        'for i in $(seq 1 180); do test -s '+root+'/data/sim-camera.yaml && break; sleep 1; done; '
        'mkdir -p '+root+'/results/ros2/live; '
        'exec ros2 run stella_vslam_ros run_slam -v /opt/vslam/orb_vocab.fbow -c '+root+'/data/sim-camera.yaml '
        '--viewer none --eval-log-dir '+root+'/results/ros2/live --map-db-out '+root+'/data/simulation-map.msg '
        '--ros-args -p use_sim_time:=true -p publish_tf:=false -p use_exact_time:=true '
        '-p camera_frame:=assignment3_camera -r camera/color/image_raw:=/vslam/color/image_raw '
        '-r camera/depth/image_raw:=/vslam/depth/image_raw'],output='screen',sigterm_timeout='30')
    pose=Node(package='ds685_vslam_nav',executable='pose_bridge',parameters=[{'use_sim_time':True}])
    mapper=Node(package='ds685_vslam_nav',executable='occupancy_mapper',parameters=[{'use_sim_time':True}])
    navigation=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav,'launch','navigation_launch.py')),
        launch_arguments={'use_sim_time':'true','autostart':'true','use_composition':'False',
                          'params_file':os.path.join(share,'config','nav2.yaml')}.items())
    return LaunchDescription([sim,mount,optical,info,adapter,wrapper,pose,mapper,TimerAction(period=12.,actions=[navigation])])
