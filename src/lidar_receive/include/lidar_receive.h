#ifndef LIDAR_RECEIVE_H
#define LIDAR_RECEIVE_H

#include <ros/ros.h>
#include <robot_msgs/PointArray.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <string.h>
#include "bldc_interface.h"
#include "comm_uart.h"
#include "bldc_interface_uart.h"
#include <unistd.h> // for usleep

namespace lidar_receive{
	class LidarReceive{
	public:
		LidarReceive(ros::NodeHandle& nh);
		void getPoint(const geometry_msgs::Point& subby);
		void constructMap();
		void receiveCB();
		
	private:
		float oldVal[3];
		float newVal[3];
		int mapCounter;
		float mapArray[3][49];
		ros::Subscriber lidarSub;
	};
}
#endif
	