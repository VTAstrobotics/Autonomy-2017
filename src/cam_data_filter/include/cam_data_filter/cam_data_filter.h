#ifndef CAM_DATA_FILTER_H
#define CAM_DATA_FILTER_H

#include <ros/ros.h>
#include <apriltags_ros/MetaPose.h>

namespace cam_data_filter{

	class CamDataFilter{
	public:
		CamDataFilter(ros::NodeHandle& nh);
		void lConvert(const apriltags_ros::MetaPose& pose);
		void rConvert(const apriltags_ros::MetaPose& pose);
		void bConvert(const apriltags_ros::MetaPose& pose);
	private:
		ros::Subscriber L;
		ros::Subscriber R;
		ros::Subscriber B;
		ros::Publisher pub;
	};
}
#endif