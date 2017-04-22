#include <lidar_receive.h>
#include <ros/ros.h>
#include <robot_msgs/PointArray.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <string.h>
#include "bldc_interface.h"
#include "comm_uart.h"
#include "bldc_interface_uart.h"
#include <unistd.h> // for usleep

int main(int argc, char **argv){
	ros::init(argc, argv, "lidar_receive");
	ros::NodeHandle nh;
	lidar_receive::LidarReceive LR(nh);
	while(ros::ok()){
		LR.receiveCB();
		ros::spinOnce();
		ros::Rate rate(5);
		rate.sleep();
		LR.constructMap();
	}
}