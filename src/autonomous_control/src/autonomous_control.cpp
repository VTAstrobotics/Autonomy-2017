#include <ros/ros.h>
#include <autonomous_control/autonomous_control.h>
#include <boost/foreach.hpp>
#include <XmlRpcException.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

namespace autonomous_control{

	AutonomousControl::AutonomousControl(ros::NodeHandle& nh)
	{
		pub = nh.advertise<apriltags_ros::MetaPose>("/tag_check",1);
		motor_command_ = nh.advertise<robot_msgs::Autonomy>("/motor_command",1);

		camSub = nh.subscribe("filteredCamData", 1, &AutonomousControl::tag_seen,this);
		imuSub = nh.subscribe("convertedImu", 1, &AutonomousControl::getImu,this);

		ROSCONSOLE_AUTOINIT;
		log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
		my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
		ROS_DEBUG_ONCE("Starting Autonomous Control");	

		/*motor_command.leftRatio = 0.0;
		motor_command.rightRatio = 0.0;
		motor_command.digCmd = 0.0;
		motor_command.dumpCmd = 0.0;*/

		posX = 0.0;
		posY = 0.0;
		posZ = 0.0;
		oX = 0.0;
		oY = 0.0;
		oZ = 0.0;
		oW = 0.0;
		pX = 0.0;
		pY = 0.0;
		detected=false;
		imuX=0.0;
		imuY=0.0;
		imuZ=0.0;
		imuW=0.0;
		state = FindBeacon;
		LorR = 0;
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
		detected = pose.detected;
	}

	void AutonomousControl::getImu(const sensor_msgs::Imu& imu){
		imuX=imu.orientation.x;
		imuY=imu.orientation.y;
		imuZ=imu.orientation.z;
		imuW=imu.orientation.w;
	}

	void AutonomousControl::primary(){
		switch(state){
			case FindBeacon:
				if(oW != 1.0){
					motor_command.leftRatio=150;
					motor_command.rightRatio=150;
					ROS_DEBUG_ONCE("Looking for Target");
				}
				else{
					halt();
					LOrR();
					turn = true;
					state=Orient90;
				}
			break;

			case Orient90:
				
				switch (LorR) {
					case 0:
						state=Orient180;
					break;
					
					case -1:
						if ( oZ < 270 - 3 && oZ > 90) {
							target90L(270 - oZ);
							if(oZ > targetAng){
								motor_command.rightRatio = 30;
								motor_command.leftRatio = 30;
							}
						}
						else if ( oZ > 270 + 3) {
							target90R(oZ - 180);
							if(oZ < targetAng){
								motor_command.rightRatio = 150;
								motor_command.leftRatio = 150;
							}
						}
						else if (oZ < 90) {
							target90R(90 - oZ);
							if(oZ < targetAng){
								motor_command.rightRatio = 150;
								motor_command.leftRatio = 150;
							}
						}
						state = DriveToCenter;
						ROS_DEBUG_ONCE("I'm driving to the center");
					break;
					
					case 1:
						if ( oZ < 270 && oZ > 90 + 3) {
							target90R(oZ - 90);
							if(oZ < targetAng){
								motor_command.rightRatio = 150;
								motor_command.leftRatio = 150;
							}
						}
						else if ( oZ > 270) {
							target90L(oZ - 180);
							if(oZ > targetAng){
								motor_command.rightRatio = 30;
								motor_command.leftRatio = 30;
							}
						}
						else if (oZ < 90 - 3) {
							target90L(90 - oZ);
							if(oZ > targetAng){
								motor_command.rightRatio = 30;
								motor_command.leftRatio = 30;
							}
						}
						state = DriveToCenter;
					break;
				}
			break;

			case DriveToCenter:
				halt();
				state=Orient180;
			break;

			case Orient180:
				halt();
				state=DriveToMine;
			break;

			case DriveToMine:
				halt();
				state=Halt;
			break;

			case Halt:
				halt();
			break;
			
			default:
				halt();
			break;
		}

		motor_command_.publish(motor_command);
		/*if(!detected){
			motor_command.leftRatio = 150;
			motor_command.rightRatio = 150;
			//motor_command.turn=true;
			motor_command_.publish(motor_command);
			ROS_INFO_STREAM("Turning");
		}
		else{
			motor_command.leftRatio = 90;
			motor_command.rightRatio = 90;
			//motor_command.turn=false;
			motor_command_.publish(motor_command);
			ROS_INFO_STREAM("I see it");
		}*/
		//oW = 0.0;
	}

	void AutonomousControl::halt(){
		motor_command.leftRatio=90;
		motor_command.rightRatio=90;
	}

	void AutonomousControl::LOrR(){
		if (posX < -0.5) {     //left
			LorR = -1;
			ROS_DEBUG_ONCE("I'm on the left");
		}
		else if (posX > 0.5) { //right
			LorR = 1;
			ROS_DEBUG_ONCE("I'm on the right");
		}
		else {          //center
			LorR = 0;
			ROS_DEBUG_ONCE("I'm already at the center");
		}
	}

	void AutonomousControl::target90R(float desired){
		if(turn){
			targetAng = imuZ - desired;
		}
		turn = false;
	}

	void AutonomousControl::target90L(float desired){
		if(turn){
			targetAng = imuZ + desired;
		}
		turn = false;
	}

	void AutonomousControl::target180(float desired){

	}
}