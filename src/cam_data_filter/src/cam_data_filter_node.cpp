#include <cam_data_filter/cam_data_filter.h>
#include <ros/ros.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "cam_data_filter");
	ros::NodeHandle nh;
	cam_data_filter::CamDataFilter CDF(nh);
	ros::spin();
}