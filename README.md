# gnss_multipath_plugin
GPS Multipath Plugin in Gazebo

Install:
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/kpant14/gnss_multipath_plugin.git
cd ../
colcon build
source ./install/setup.bash
ros2 launch gnss_multipath_plugin hong_kong.launch.py
```

Make sure to put this repo in the src/ directory of a catkin workspace

ros2_ws/src/gnss_multipath_plugin/
