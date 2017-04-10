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
		void primary();
		void halt();
		void updateTag();
	private:
		float posX, posY, posZ, oX, oY, oZ, oW, pX, pY, imuX, imuY, imuZ, imuW, targetAng, prevZ, newZ, tempZ, imuForward, oZStore;
		float forwardRatio, backwardRatio, brake, obsFieldStart;
		bool detected, turn, faceForward, moveComplete, waiting, waitComplete;
		robot_msgs::Autonomy motor_command;
		typedef enum{FindBeacon, Orient90, DriveToCenter, Orient180, DriveToObsField, Halt, Wait, ScanField} machineState;
		machineState state;
		int LorR, numRot, count;
		ros::Subscriber camSub;
		ros::Subscriber imuSub;
		ros::Subscriber syncSub;
		ros::Publisher pub;
		ros::Publisher motor_command_;
		ros::Publisher cali_command_;
		ros::Publisher scan_command_;
		ros::Publisher mappingSignal;
		std_msgs::Empty empty;
		std_msgs::Bool cali;
		std_msgs::Bool mapPub;
		void LOrR();
		void target90R(float desired);
		void target90L(float desired);
		void target180(float desired);
		void updateIMU();
		void hold(int waitTime);
	};
}
#endif
