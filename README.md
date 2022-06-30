# proto_to_rosbag2
Converts the temporary proto_rosbag2 file to rosbag2 file.

# IMPORTANT
In order to get `.proto_rosbag2` file from a rosbag1 file, follow instructions at: https://github.com/leo-drive/ros1_bridge

## Build Instructions
```bash
mkdir -p ~/rosbag_converter_proto_ws/src
cd ~/rosbag_converter_proto_ws/src
git clone https://github.com/leo-drive/proto_to_rosbag2.git
cd ~/rosbag_converter_proto_ws

source /opt/ros/galactic/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Run Instructions
```bash
Edit this file with your directories:
~/rosbag_converter_proto_ws/src/proto_to_rosbag2/params/test_param.yaml
# defaults:
    path_in_ros2_serialized_binary: "/home/mfc/bags/038/038.proto_rosbag2" # This is the output of ros1bridge_ws's package. 
    # The saved file names will be with ".proto_rosbag2_000, .proto_rosbag2_001, ..." extensions. With max 1GB file sizes.
    # But you don't need to worry about the numbers in the end.
    
    path_out_ros2_bag: "/home/mfc/bags/038/038" # Last word here becomes the folder that contains the 038.db3 and metadata.yaml, 
    # So be careful when naming it. It will remove this directory before writing.
```
```bash
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
