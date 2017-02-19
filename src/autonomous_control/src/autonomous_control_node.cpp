#include <autonomous_control/autonomous_control.h>
#include <ros/ros.h>
#include <stdlib.h>


int main(int argc, char **argv){
	ros::init(argc, argv, "autonomous_control");
	ros::NodeHandle nh;
	autonomous_control::AutonomousControl ATC(nh);
	while(ros::ok()){
		ros::spinOnce();
		ATC.primary();
	}
	
}