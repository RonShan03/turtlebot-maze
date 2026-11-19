"""Jazzy ament already registers this library hook; remove upstream's duplicate.

Guarded patch for ROS wrapper 186f623dd1c24ee83678f8e1da593801f450bbb3.
"""
from pathlib import Path
import sys
path=Path(sys.argv[1])
source=path.read_text()
old='''if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()
'''
if source.count(old)!=1:
    raise RuntimeError('upstream library hook changed; inspect before applying compatibility patch')
path.write_text(source.replace(old,''))
header=path.parent/'src/stella_vslam_ros.h'
text=header.read_text()
old_header='#include <cv_bridge/cv_bridge.h>'
if text.count(old_header)!=1: raise RuntimeError('upstream cv_bridge include changed; inspect before patching')
header.write_text(text.replace(old_header,'#include <cv_bridge/cv_bridge.hpp>'))
