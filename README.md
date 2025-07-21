# Updated gnss_multipath_plugin
GPS Multipath Plugin in Gazebo Harmonic

Depends on libpredict: https://github.com/la1k/libpredict in the branch Updated2Ubuntu24
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
```
Clone the libpredict library into the workspace as it is a dependency 
```
git clone -b Updated2Ubuntu24 https://github.com/Darkjrcy/libpredict.git
```
This plugin is an updated version of the original gnss_multipath_plugin, modified to work within the new Gazebo Harmonic simulation software. It amintains the same multipath offset logic of the original plugin for Gazebo Classic, but includes several improvements like greater number of satellites, maximum visible satallites selection and compatibility of the plugin with Gazebo Sim verions. To copy the repository use the next commands. 
```
git clone -b Migration2GazeboSim https://github.com/Darkjrcy/gnss_multipath_plugin.git
cd ../
colcon build
```
Source the environment before executing the launch file
```
source ./install/setup.bash
```
