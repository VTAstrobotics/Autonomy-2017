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
		motor_command_ = nh.advertise<robot_msgs::Autonomy>("/robot/autonomy",1);
		cali_command_ = nh.advertise<std_msgs::Bool>("cali_command",1);
		mappingSignal = nh.advertise<std_msgs::Bool>("startMapping",1);
		camSub = nh.subscribe("filteredCamData", 1, &AutonomousControl::tag_seen,this);
		imuSub = nh.subscribe("imu/converted", 1, &AutonomousControl::getImu,this);

		ROSCONSOLE_AUTOINIT;
		log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
		my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
		

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
		obsFieldStart = 1.5;
		detected=false;
		imuX=0.0;
		imuY=0.0;
		imuZ=0.0;
		imuW=0.0;
		prevZ=0.0;
		state = FindBeacon;
		LorR = 0;
		numRot = 0;
		imuForward = 0;
		faceForward = false;  // bool used to check whether bot is facing forward

		forwardRatio = 0.2; 
		backwardRatio = -0.2; 
		brake = 0.0;

		waiting = false;
		waitComplete = false;

		mapPub.data = false;

		ros::Duration(3.0).sleep();
		ROS_DEBUG_ONCE("Starting Autonomous Control");	

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
		cali.data=true;
		//ROS_INFO_STREAM("Field Position is x = " << posX << ",  y = " << posY);
	}

	void AutonomousControl::getImu(const sensor_msgs::Imu& imu){
		imuX=imu.orientation.x;
		imuY=imu.orientation.y;
		imuZ=imu.orientation.z;
		imuW=imu.orientation.w;		
	}

	void AutonomousControl::primary(){
		updateIMU();
		switch(state){
			case FindBeacon:
				if(oW != 1.0){
					motor_command.leftRatio=forwardRatio;
					motor_command.rightRatio=backwardRatio;
					cali_command_.publish(cali);
					cali.data=false;
					ROS_DEBUG_ONCE("Looking for Target");
					ROS_DEBUG_ONCE("Calibrating lidar");
				}
				else{
					//ros::Duration(2.0).sleep(); // sleep for two seconds
					imuForward = imuZ + 180 - oZ;  //Calculate forward IMU Angle
					LOrR();
					turn = true;
					state=Wait;
					ROS_DEBUG_ONCE("Found Target");
					halt();
					break;
				}
			break;

			case Wait:
				hold(20);
				if (waiting && waitComplete){
					updateIMU();
					updateTag();
					state = Orient90;
					tempZ = imuZ;  //Store IMU Angle at halt
					ROS_DEBUG_STREAM("Tag angle " << oZ);
					ROS_DEBUG_STREAM("IMU angle " << imuZ);
				}
			break;

			if (moveComplete == false) {
				case Orient90:				
					switch (LorR) {
						case 0:
							state=Orient180;
						break;
					
						case -1:
							if ( oZStore < 270 - 3 && oZStore > 90) { //This works
								//Right hand side, facing quadrant 1 or 2
								target90L(270 - oZStore);
								ROS_DEBUG_ONCE("Turning Left 1");
								if(newZ < targetAng){
									motor_command.rightRatio = forwardRatio;
									motor_command.leftRatio = backwardRatio;
								}
								else{
									halt();
									state = DriveToCenter;
									moveComplete = true;
								}
							}
							else if ( oZStore > 270 + 3) { //will likely never happen
								// Right Hand side, facing quadrant 3
								target90R(oZStore - 180);
								ROS_DEBUG_ONCE("Turning Right 2");
								if(newZ > targetAng){
									motor_command.rightRatio = backwardRatio;
									motor_command.leftRatio = forwardRatio;
								}
								else{
									halt();
									state = DriveToCenter;
									moveComplete = true;
								}
							}
							else if (oZStore < 90) { //This works 4/3
								ROS_DEBUG_ONCE("Turning Right 3");
								// Right Hand Side, Facing Quadrant 4
								target90R(90 + oZStore);
								if(newZ > targetAng){
									motor_command.rightRatio = backwardRatio;
									motor_command.leftRatio = forwardRatio;
								}
								else{
									halt();
									state = DriveToCenter;
									moveComplete = true;
								}
							}
						break;
					
						case 1:						
							if ( oZStore < 270 && oZStore > 90 + 3) { //this works
								// Left hand side, facing Quadrant 1 or 2
								target90R(oZStore - 90);
								ROS_DEBUG_ONCE("Turning Right 4");
								if(newZ > targetAng){
									motor_command.rightRatio = backwardRatio;
									motor_command.leftRatio = forwardRatio;
								}
								else{
									halt();
									state = DriveToCenter;
									moveComplete = true;
								}
							}	
							else if ( oZStore> 270) {
								// Left Hand Side, Facing Quadrant 3
								target90L(450 - oZStore);
								ROS_DEBUG_ONCE("Turning Left 5");
								if(newZ < targetAng){
									motor_command.rightRatio = forwardRatio;
									motor_command.leftRatio = backwardRatio;
								}
								else{
									halt();
									state = DriveToCenter;
									moveComplete = true;
								}
							}
							else if (oZStore < 90 - 3) {
								// Left Hand Side, Facing Quadrant 4
								target90L(90 - oZStore);
								ROS_DEBUG_ONCE("Turning Left 6");
								if(newZ < targetAng){
									motor_command.rightRatio = forwardRatio;
									motor_command.leftRatio = backwardRatio;
								}
								else{
									halt();
									state = DriveToCenter;
									moveComplete = true;
								}
							}
						break;
						}
					}
				break;

			case DriveToCenter:
				ROS_DEBUG_ONCE("I'm driving to the center");
				if(posX < -0.2 || posX > 0.2){
					motor_command.rightRatio=forwardRatio;
					motor_command.leftRatio=forwardRatio;
				}
				else{
					halt();
					state=Orient180;
				}
			break;

			case Orient180:
				if(oZ < 177){
					target180(180-oZ);
					if(newZ < targetAng){
					ROS_DEBUG_ONCE("Turning Left to Face Forward");
					motor_command.rightRatio = forwardRatio;
					motor_command.leftRatio = backwardRatio;
					}
					else{
					halt();
					state=DriveToObsField;
					}
				}
				else if(oZ > 183) {
					target180(180-oZ);
					if(newZ > targetAng){
					ROS_DEBUG_ONCE("Turning Right to Face Forward");	
					motor_command.rightRatio = backwardRatio;
					motor_command.leftRatio = forwardRatio;
					}
					else{
					halt();
					state=DriveToObsField;
					}
				}
			break;

			case DriveToObsField:
				ROS_DEBUG_ONCE("Driving to Start of Obstacle Field");
				if(posY < obsFieldStart){
					motor_command.rightRatio=forwardRatio;
					motor_command.leftRatio=forwardRatio;
				}
				else{
					scan_command_.publish(empty);
					halt();
					state=ScanField;
				}
			break;



			case ScanField:
				// shit the bed 
				// git schwifty

				mapPub.data = true;			// Tells the mapping Arduino to start Scanning the Field
				//mappingSignal.publish(mapPub);
				hold(60);
				state = Halt;
				// mapPub.data = false;
				// mappingSignal.publish(mapPub);// Tells the mapping Arduino to stop scanning
			break;




			case Halt:
				ROS_DEBUG_ONCE("HALT!");
				halt();
			break;
			


			default:
				halt();
			break;
		}
		mappingSignal.publish(mapPub);  // Mapping Signal for Arduino
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
		ROS_DEBUG_ONCE("Halt Command Called");
		motor_command.leftRatio=brake;
		motor_command.rightRatio=brake;
		motor_command_.publish(motor_command);
	}

	void AutonomousControl::LOrR(){
		if (posX < -0.25) {     //right
			LorR = -1;
			ROS_DEBUG_ONCE("I'm on the right (negative x)");
		}
		else if (posX > 0.25) { //left
			LorR = 1;
			ROS_DEBUG_ONCE("I'm on the left (positive x)");
		}
		else {          //center
			LorR = 0;
			ROS_DEBUG_ONCE("I'm already at the center");
		}
	}

	void AutonomousControl::target90R(float desired){
		if(turn){
			targetAng = newZ - desired;
			ROS_DEBUG_STREAM("Target angle " << targetAng);
		}
		updateIMU();
		
		turn = false;
	}

	void AutonomousControl::target90L(float desired){
		if(turn){
			targetAng = newZ + desired;
			ROS_DEBUG_STREAM("Target angle " << targetAng);
		}
				updateIMU();

		turn = false;
	}

	void AutonomousControl::updateIMU(){
		prevZ = tempZ -360*numRot;
		tempZ = imuZ;
		


		if ((prevZ >= 270.00 && prevZ <= 360.00)&&(tempZ>=0.0 && tempZ <=90.0)) {
			numRot = numRot+1;
			newZ = tempZ + 360*numRot;
			ROS_DEBUG_ONCE("IMU positive turn over");
		}
		else if ((prevZ <=90.0 && prevZ>=0.0)&&(tempZ<=360 && tempZ >=270)) {
			numRot = numRot -1;
			newZ = tempZ + 360*numRot;
			ROS_DEBUG_ONCE("IMU negative turn over");
		}
		else{
			newZ = tempZ + 360*numRot;			
		}
		//ROS_INFO_STREAM("Updated IMU Angle is " << newZ);
	}

	void AutonomousControl::updateTag(){
		oZStore = oZ;
		ROS_DEBUG_STREAM("Storing Orientation of " << oZStore);
		moveComplete = false;
	}                    

	void AutonomousControl::target180(float desired){
		updateIMU();
		if(!faceForward){
			targetAng = newZ + desired;
			ROS_DEBUG_STREAM("Turning Forward by " << targetAng);
		}
		faceForward = true;
		
	}


	void AutonomousControl::hold(int waitTime){
		ROS_INFO_STREAM("waitTime " << waitTime);
		ROS_INFO_STREAM("count " << count);
		if(count < waitTime){
			count++;
		}
		else{
			waiting = true;
			waitComplete = true;
		}
	}
}
	