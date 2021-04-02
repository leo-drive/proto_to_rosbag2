# proto_to_rosbag2
Converts the temporary proto_rosbag2 file to rosbag2 file.

## Build Instructions
```bash

# Some info will be taken from https://docs.ros.org/en/rolling/Installation/Linux-Development-Setup.html

# These are rolling dependencies, more recent instructions will be in the website above
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
  
# install some pip packages needed for testing
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

# Here we create the workspace and download the Rolling repos in it
mkdir -p ~/rosbag_converter_proto_ws/src
cd ~/rosbag_converter_proto_ws

wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
vcs import src < ros2.repos

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro rolling -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# Now this package should be cloned in it
cd ~/rosbag_converter_proto_ws/src
git clone https://github.com/leo-drive/proto_to_rosbag2.git
cd ~/rosbag_converter_proto_ws

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-up-to proto_to_rosbag2
```

## Run Instructions
```bash
Edit this file with your directories:
~/rosbag_converter_proto_ws/src/proto_to_rosbag2/params/test_param.yaml
# defaults:
#    path_in_ros2_serialized_binary: "/home/mfc/bags/038/038.proto_rosbag2" # This is the output of ros1bridge_ws's package.
#    path_out_ros2_bag: "/home/mfc/bags/038/038" # Last word here becomes the folder that contains the 038.db3 and metadata.yaml, 
#    So be careful when naming it. It will remove this directory before writing.

cd ~/rosbag_converter_proto_ws
source ~/rosbag_converter_proto_ws/install/setup.bash
ros2 run proto_to_rosbag2 proto_to_rosbag2_node --ros-args --params-file ~/rosbag_converter_proto_ws/src/proto_to_rosbag2/params/test_param.yaml

# Now final structure should look like this:
mfc@mfc-SG13:~/bags/038$ tree
.
├── 038
│   ├── 038_0.db3
│   └── metadata.yaml
├── 038.bag
└── 038.proto_rosbag2

2 directories, 6 files

# and you can play the ros2 bag file like:
ros2 bag play ~/bags/038/038/038_0.db3
```