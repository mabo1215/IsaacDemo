#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

apt-get update
apt-get install -y --no-install-recommends \
  locales software-properties-common curl ca-certificates gnupg lsb-release \
  build-essential python3-rosdep2

locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
add-apt-repository -y universe

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | \
    sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -n 1)"
  test -n "$version"
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.jammy_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
fi

# Ubuntu Jammy's preinstalled ROS 1 helper packages own the same Python files
# as ROS 2's -modules packages. Remove only those three obsolete packages
# before apt resolves the ROS 2 desktop transaction.
for old_package in python3-catkin-pkg python3-rospkg python3-rosdistro; do
  if dpkg-query -W -f='${Status}' "$old_package" 2>/dev/null | grep -q 'install ok installed'; then
    dpkg --remove --force-depends "$old_package" || true
  fi
done
apt-get -f install -y

apt-get update
apt-get install -y --no-install-recommends \
  ros-humble-desktop ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
  ros-humble-xacro ros-humble-tf2-ros ros-humble-vision-msgs \
  python3-colcon-common-extensions

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update

workspace='/mnt/c/source/IsaacDemo/ros2_ws'
mkdir -p "$workspace/src"
if ! grep -q '/opt/ros/humble/setup.bash' /root/.bashrc 2>/dev/null; then
  printf '\nsource /opt/ros/humble/setup.bash\n' >> /root/.bashrc
fi
if [ -d "$workspace/src/isaac_drywall_demo" ]; then
  source /opt/ros/humble/setup.bash
  cd "$workspace"
  rosdep install --from-paths src --ignore-src --rosdistro humble -y
  colcon build --symlink-install
fi

echo 'ROS 2 Humble installation completed.'
