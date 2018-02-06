#ifndef AUTONOMOUS_CONTROL_H
#define AUTONOMOUS_CONTROL_H

#include <ros/ros.h>
#include <apriltags_ros/MetaPose.h>
#include <robot_msgs/Autonomy.h>
#include <robot_msgs/MotorFeedback.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>

namespace autonomous_control{

	class AutonomousControl{
	public:
		AutonomousControl(ros::NodeHandle& nh);
		void tag_seen(const apriltags_ros::MetaPose& pose);
		void getImu(const sensor_msgs::Imu& imu);
		//void getLidar(const std_msgs::Lidar& lidar);
		void getStripe(const std_msgs::Int8MultiArray& stripe);
		void Idleing(const std_msgs::Bool& cmd);
		void Feedback(const robot_msgs::MotorFeedback& mf);
		void IR0(const std_msgs::Int8& val);
		void IR1(const std_msgs::Int8& val);
		void primary();
		void halt();
		void updateTag();
	private:
		float posX, posY, posZ, oX, oY, oZ, oW, pX, pY, imuX, imuY, imuZ, imuW, lidarX, lidarY, lidarZ, lidarW, targetAng, prevZ, newZ, tempZ, imuForward, oZStore;
		float forwardRatio, backwardRatio, brake, obsFieldStart, drumForward, drumReverse;
		float drumRPM, leftRPM, rightRPM, liftPos, liftLowerLimit, liftUpperLimit;
		bool detected, turn, faceForward, moveComplete, waitComplete, go, startup, angleTargeted,inObsField;
		robot_msgs::Autonomy motor_command;
		robot_msgs::Status status;
		typedef enum{sensorIntialization, Orient90, DriveToCenter, Orient180, DriveToObsField, DriveToMine, Mining, DepositPrep, Deposit, ReturnToObs, ReturnToBin,
			DeadMan, DumpPrep, Dump, DumpFinish, TravelPrep, Halt, Wait, Idle, Prep, Orient180imu} machineState;
		machineState state;
		machineState prevState;
		int LorR, numRot, count, cycleCount, ir0, ir1;
		ros::Subscriber camSub;
		ros::Subscriber imuSub;
		ros::Subscriber syncSub;
		ros::Subscriber lidarSweep;
		ros::Subscriber stripeArray;
		ros::Subscriber idle;
		ros::Subscriber feedback;
		//ros::Subscriber ir0sub;
		//ros::Subscriber ir1sub;
		ros::Publisher pub;
		ros::Publisher motor_command_;
		ros::Publisher status_command_;
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
		void updateLidar();

		void hold(int waitTime);
	};
}
#endif
