#ifndef AUTONOMOUS_CONTROL_H
#define AUTONOMOUS_CONTROL_H

#include <ros/ros.h>
#include <apriltags_ros/MetaPose.h>
#include <robot_msgs/Autonomy.h>
#include <robot_msgs/Atest.h>
#include <sensor_msgs/Imu.h>

namespace autonomous_control{
	
	class AutonomousControl{
	public:
		AutonomousControl(ros::NodeHandle& nh);
		void tag_seen(const apriltags_ros::MetaPose& pose);
		void getImu(const sensor_msgs::Imu& imu);
		void primary();
		void halt();
	private:
		float posX, posY, posZ, oX, oY, oZ, oW, pX, pY, imuX, imuY, imuZ, imuW, targetAng, prevZ, newZ, tempZ, imuForward;
		bool detected, turn, faceForward;
		robot_msgs::Autonomy motor_command;
		typedef enum{FindBeacon, Orient90, DriveToCenter, Orient180, DriveToMine, Halt} machineState;
		machineState state;
		int LorR, numRot;
		ros::Subscriber camSub;
		ros::Subscriber imuSub;
		ros::Publisher pub;
		ros::Publisher motor_command_;

		void LOrR();
		void target90R(float desired);
		void target90L(float desired);
		void target180(float desired);
		void updateIMU();
	};
}
#endif
