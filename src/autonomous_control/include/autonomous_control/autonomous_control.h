#ifndef AUTONOMOUS_CONTROL_H
#define AUTONOMOUS_CONTROL_H

#include <ros/ros.h>
#include <apriltags_ros/MetaPose.h>
#include <robot_msgs/Autonomy.h>

namespace autonomous_control{
	float posX, posY, posZ, oX, oY, oZ, oW, pX, pY;
	robot_msgs::Autonomy motor_command;
	class AutonomousControl{
	public:
		AutonomousControl(ros::NodeHandle& nh);
		void tag_seen(const apriltags_ros::MetaPose& pose);
		void primary();
	private:
		ros::Subscriber sub;
		ros::Publisher pub;
		ros::Publisher motor_command_;
	};
}
#endif