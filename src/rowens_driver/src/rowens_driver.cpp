#include <ros/ros.h>
#include <rowens_driver/rowens_driver.h>
/*#include "motor.h"
#include "motortypes.h"*/

namespace rowens_driver{

	RowensDriver::RowensDriver(ros::NodeHandle& nh){
		autoSub = nh.subscribe("/robot/autonomy",1, &RowensDriver::cmd_recieved, this);

		leftRatio = 0.0;
		rightRatio = 0.0;
		digCmd = 0.0;
		dumpCmd = 0.0;

		/*
		Motor::init();
		Motor leftDrive(LEFTDRIVE);
		Motor rightDrive(RIGHTDRIVE);
		*/
	}

	void RowensDriver::cmd_recieved(const robot_msgs::Autonomy& cmd){
		leftRatio = cmd.leftRatio;
		rightRatio = cmd.rightRatio;
		digCmd = cmd.digCmd;
		dumpCmd = cmd.dumpCmd;
	}

	void RowensDriver::primary(){
		/*
		leftDrive.set_Speed(leftRatio);
		rightDrive.set_Speed(rightRatio);
		*/
	}
}