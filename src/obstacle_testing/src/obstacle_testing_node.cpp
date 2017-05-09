#include <obstacle_testing/obstacle_testing.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Int8MultiArray.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "autonomous_control");
	ros::NodeHandle nh;
	obstacle_testing::ObstacleTesting OBT(nh);
	while(ros::ok()){
		ros::spinOnce();
		ros::Rate rate(5);
		rate.sleep();
		
	}
	
}
