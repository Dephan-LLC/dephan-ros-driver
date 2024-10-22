# ROS driver for DEPHAN LLC LiDars

## ROS Distro
It is a package for [***ros:noetic***](https://hub.docker.com/_/ros) ROS distro. 

## Building && running 
1. `git clone https://github.com/Dephan-LLC/dephan-ros-driver.git`
2. `cd dephan-ros-driver`
3. `catkin_make`
4. `source devel/setup.bash` 

For testing node please run:
1. In terminal 1: `roscore`
2. In terminal 2: `rosrun mech_lidar_driver mech_driver`
3. In terminal 3: `rostopic echo point_cloud2_data`
