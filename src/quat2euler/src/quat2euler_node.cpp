#include <quat2euler/quat2euler.h>
#include <ros/ros.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "quat2euler");
	ros::NodeHandle nh;
	quat2euler::Quat2Euler q2e(nh);
	ros::spin();
}