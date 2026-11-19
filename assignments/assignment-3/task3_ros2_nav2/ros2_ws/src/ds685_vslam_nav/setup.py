from glob import glob
from setuptools import setup
setup(name='ds685_vslam_nav', version='0.1.0', packages=['ds685_vslam_nav'],
      data_files=[('share/ament_index/resource_index/packages',['resource/ds685_vslam_nav']),
                  ('share/ds685_vslam_nav',['package.xml']),
                  ('share/ds685_vslam_nav/launch',glob('launch/*.launch.py')),
                  ('share/ds685_vslam_nav/config',glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts':[
          'pose_bridge = ds685_vslam_nav.nodes:pose_bridge_main',
          'occupancy_mapper = ds685_vslam_nav.nodes:mapper_main',
          'camera_adapter = ds685_vslam_nav.camera_adapter:main',
          'smoke_test = ds685_vslam_nav.smoke:main',
          'navigate = ds685_vslam_nav.navigate:main']})
