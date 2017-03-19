#include <rowens_driver/rowens_driver.h>
#include <ros/ros.h>
#include <stdlib.h>


int main(int argc, char **argv){
	ros::init(argc, argv, "rowens_driver");
	ros::NodeHandle nh;
	rowens_driver::RowensDriver RD(nh);
	while(ros::ok()){
		ros::spinOnce();
		ros::Rate rate(5);
		rate.sleep();
		RD.primary();
	}
	
}