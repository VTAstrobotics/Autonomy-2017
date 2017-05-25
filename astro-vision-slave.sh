#! /bin/sh

export ROS_HOSTNAME="10.0.1.35"
export ROS_MASTER_URI="http://10.0.1.30:11311"
. /home/astrobotics/ros_code/devel/setup.sh
#roslaunch autonomous_control imu_launch.launch
roslaunch autonomous_control autonomous.launch

