#ifndef AUTONOMOUS_CONTROL_H
#define AUTONOMOUS_CONTROL_H

#include <ros/ros.h>
#include <apriltags_ros/MetaPose.h>
#include <robot_msgs/Autonomy.h>
#include <robot_msgs/Atest.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

namespace autonomous_control{
	
	class AutonomousControl{
	public:
		AutonomousControl(ros::NodeHandle& nh);
		void tag_seen(const apriltags_ros::MetaPose& pose);
		void getImu(const sensor_msgs::Imu& imu);
		void synced(const std_msgs::Bool& val);
		void primary();
		void halt();
	private:
		float posX, posY, posZ, oX, oY, oZ, oW, pX, pY, imuX, imuY, imuZ, imuW, targetAng, prevZ, newZ, tempZ, imuForward;
		float forwardRatio, backwardRatio, brake;
		bool detected, turn, faceForward;
		robot_msgs::Autonomy motor_command;
		typedef enum{FindBeacon, Orient90, DriveToCenter, Orient180, DriveToMine, Halt} machineState;
		machineState state;
		int LorR, numRot;
		ros::Subscriber camSub;
		ros::Subscriber imuSub;
		ros::Subscriber syncSub;
		ros::Publisher pub;
		ros::Publisher motor_command_;
		ros::Publisher cali_command_;
		ros::Publisher scan_command_;
		ros::Publisher nuc_sync_;
		std_msgs::Empty empty;
		std_msgs::Bool cali;
		std_msgs::Bool nuc_sync;

		void LOrR();
		void target90R(float desired);
		void target90L(float desired);
		void target180(float desired);
		void updateIMU();
	};
}
#endif
