#ifndef QUAT_2_EULER_H
#define QUAT_2_EULER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace quat2euler{

class Quat2Euler{
public:
	Quat2Euler(ros::NodeHandle& nh);
	void convert(const sensor_msgs::Imu& imu);
private:
	ros::Subscriber sub;
	ros::Publisher convert_pub;
};
}
#endif