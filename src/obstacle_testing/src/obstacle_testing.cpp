#include <ros/ros.h>
#include <obstacle_testing/obstacle_testing.h>
#include <boost/foreach.hpp>
#include <XmlRpcException.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <std_msgs/Int8MultiArray.h>

//int arrayZero[10] = {};

namespace obstacle_testing{

	ObstacleTesting::ObstacleTesting(ros::NodeHandle& nh) : obsArray{}
	{
		stripeSub = nh.subscribe("stripe_obs_array", 1, &ObstacleTesting::stripeCB,this);

		// obsArray = {};
	}

	void ObstacleTesting::stripeCB(const std_msgs::Int8MultiArray& obj){
		for (int jj = 0; jj<10; jj = jj+1){
			obsArray[jj]=obj.data[jj];	
		}
	}
	void ObstacleTesting::primary(){

		ROS_INFO_STREAM("Current Array is" <<  obsArray );
	}
}