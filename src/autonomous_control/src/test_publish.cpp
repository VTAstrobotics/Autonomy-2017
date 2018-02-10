#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;


	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
       	srand(time(0));

	ros::Rate rate(2);
	while(ros::ok()) {
	geometry_msgs::Twist rand_msg;
	rand_msg.linear.x = double(rand())/double(RAND_MAX);
	rand_msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;

	pub.publish(rand_msg);
	ROS_INFO_STREAM("Sending random velocity command:"<<"linear-"<<rand_msg.linear.x<<"angular="<<rand_msg.angular.z);
	rate.sleep();
	}
}
