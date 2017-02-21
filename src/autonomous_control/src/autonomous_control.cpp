#include <ros/ros.h>
#include <autonomous_control/autonomous_control.h>
#include <boost/foreach.hpp>
#include <XmlRpcException.h>

namespace autonomous_control{

	AutonomousControl::AutonomousControl(ros::NodeHandle& nh)
	{
		pub = nh.advertise<apriltags_ros::MetaPose>("/tag_check",1);
		motor_command_ = nh.advertise<robot_msgs::Autonomy>("/motor_command",1);

		sub = nh.subscribe("filteredCamData", 1, &AutonomousControl::tag_seen,this);

		motor_command.leftRatio = 0.0;
		motor_command.rightRatio = 0.0;
		motor_command.digCmd = 0.0;
		motor_command.dumpCmd = 0.0;

		posX = 0.0;
		posY = 0.0;
		posZ = 0.0;
		oX = 0.0;
		oY = 0.0;
		oZ = 0.0;
		oW = 0.0;
		pX = 0.0;
		pY = 0.0;
	}
	void AutonomousControl::tag_seen(const apriltags_ros::MetaPose& pose){
		
		posX = pose.pose.position.x;
		posY = pose.pose.position.y;
		posZ = pose.pose.position.z;
		oX = pose.pose.orientation.x;
		oY = pose.pose.orientation.y;
		oZ = pose.pose.orientation.z;
		oW = pose.pose.orientation.w;
		pX = pose.px;
		pY = pose.py;
	}
	void AutonomousControl::primary(){
		if(oW != 1.0){
			motor_command.leftRatio = 0.5;
			motor_command.rightRatio = -0.5;
			motor_command_.publish(motor_command);
		}
		else{
			motor_command.leftRatio = 0.0;
			motor_command.rightRatio = 0.0;
			motor_command_.publish(motor_command);
			ROS_INFO_STREAM("I see it");
		}

	}
}