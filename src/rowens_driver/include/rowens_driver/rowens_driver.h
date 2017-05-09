#ifndef ROWENS_DRIVER_H
#define ROWENS_DRIVER_H

#include <ros/ros.h>
#include <robot_msgs/Autonomy.h>
#include "bldc.h"
#include "motortypes.h"

namespace rowens_driver{

	BLDC leftDrive(LEFTDRIVE, Alien_4260);
	BLDC rightDrive(RIGHTDRIVE, Alien_4260);

	class RowensDriver{
	public:
		RowensDriver(ros::NodeHandle& nh);
		void cmd_recieved(const robot_msgs::Autonomy& cmd);
		void primary();
	private:
		float leftRatio, rightRatio, digCmd, dumpCmd;
		ros::Subscriber autoSub;
		
	};
}
#endif
