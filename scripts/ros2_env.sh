#!/usr/bin/env bash
set -eo pipefail

# The workspace is an isolated install on /mnt/c. Add the package prefix
# explicitly so ros2 can discover its ament index and executable from WSL.
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
workspace=/mnt/c/source/IsaacDemo/ros2_ws
package_prefix="$workspace/install/isaac_drywall_demo"
export AMENT_PREFIX_PATH="$package_prefix:/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PYTHONPATH="$workspace/build/isaac_drywall_demo:$package_prefix/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$package_prefix/lib/isaac_drywall_demo:$PATH"
