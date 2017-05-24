#ifndef CAM_DATA_FILTER_H
#define CAM_DATA_FILTER_H

#include <ros/ros.h>
#include <apriltags_ros/MetaPose.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

namespace cam_data_filter{

	class CamDataFilter{
	public:
		CamDataFilter(ros::NodeHandle& nh);
		void lConvert(const apriltags_ros::AprilTagDetectionArray& array);
		void rConvert(const apriltags_ros::AprilTagDetectionArray& array);
		void bConvert(const apriltags_ros::AprilTagDetectionArray& array);
	private:
		ros::Subscriber L;
		ros::Subscriber R;
		ros::Subscriber B;
		ros::Publisher pub;
		apriltags_ros::MetaPose zeroTag;
		float oneOffset, twoOffset, threeOffset, fourOffset;
	};
}
#endif