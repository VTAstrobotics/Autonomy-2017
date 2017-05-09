#include <lidar_receive.h>
#include <ros/ros.h>
#include <robot_msgs/PointArray.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "lidar_receive");
	ros::NodeHandle nh;
	lidar_receive::LidarReceive LR(nh);
	while(ros::ok()){
		LR.receiveCB();
		LR.startMapCB();
		ros::spinOnce();
		ros::Rate rate(5);
		rate.sleep();
		LR.constructMap();
	}
}