#ifndef ROWENS_DRIVER_H
#define ROWENS_DRIVER_H

#include <ros/ros.h>
#include <robot_msgs/Autonomy.h>

namespace rowens_driver{

	class RowensDriver{
	public:
		RowensDriver(ros::NodeHandle& nh);
		void cmd_recieved(const robot_msgs::Autonomy& cmd);
	private:
		float leftRatio, rightRatio, digCmd, dumpCmd;
		ros::Subscriber autoSub;

		primary();
	};
}
#endif