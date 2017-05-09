#ifndef LIDAR_RECEIVE_H
#define LIDAR_RECEIVE_H

#include <ros/ros.h>
//#include <robot_msgs/PointArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>

namespace lidar_receive{
	class LidarReceive{
	public:
		LidarReceive(ros::NodeHandle& nh);
		void getPoint(const geometry_msgs::Point& subby);
		void constructMap();
		void receiveCB();
		void startMapCB();
		
	private:
		float oldVal[3];
		float newVal[3];
		int mapCounter;
		float mapArray[3][49];
		ros::Subscriber lidarSub;
		ros::Publisher lidarSweep;
	};
}
#endif
	
