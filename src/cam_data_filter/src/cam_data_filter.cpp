#include <ros/ros.h>
#include <cam_data_filter/cam_data_filter.h>
#include <boost/foreach.hpp>
#include <XmlRpcException.h>


namespace cam_data_filter{
	CamDataFilter::CamDataFilter(ros::NodeHandle& nh)
	{
		pub = nh.advertise<apriltags_ros::MetaPose>("/filteredCamData",1);

		L = nh.subscribe("left/cam_pose", 1, &CamDataFilter::lConvert,this);
		R = nh.subscribe("right/cam_pose", 1, &CamDataFilter::rConvert,this);
		B = nh.subscribe("back/cam_pose", 1, &CamDataFilter::bConvert,this);
	}
	void CamDataFilter::lConvert(const apriltags_ros::MetaPose& pose){
		apriltags_ros::MetaPose pose2 = pose;
		pose2.pose.orientation.z +=(M_PI/2);
		pose2.pose.orientation.x=pose2.pose.orientation.x*180/M_PI;
		pose2.pose.orientation.y=pose2.pose.orientation.y*180/M_PI;
		pose2.pose.orientation.z=pose2.pose.orientation.z*180/M_PI;
		if(pose2.pose.orientation.z < 0)
		{
			pose2.pose.orientation.z+=360;
		}
		pub.publish(pose2);
	}

	void CamDataFilter::rConvert(const apriltags_ros::MetaPose& pose){
		apriltags_ros::MetaPose pose2 = pose;
		pose2.pose.orientation.z -=(M_PI/2);
		pose2.pose.orientation.x=pose2.pose.orientation.x*180/M_PI;
		pose2.pose.orientation.y=pose2.pose.orientation.y*180/M_PI;
		pose2.pose.orientation.z=pose2.pose.orientation.z*180/M_PI;
		if(pose2.pose.orientation.z < 0)
		{
			pose2.pose.orientation.z+=360;
		}
		pub.publish(pose2);
	}

	void CamDataFilter::bConvert(const apriltags_ros::MetaPose& pose){
		apriltags_ros::MetaPose pose2 = pose;
		pose2.pose.orientation.x=pose2.pose.orientation.x*180/M_PI;
		pose2.pose.orientation.y=pose2.pose.orientation.y*180/M_PI;
		pose2.pose.orientation.z=pose2.pose.orientation.z*180/M_PI;
		if(pose2.pose.orientation.z < 0)
		{
			pose2.pose.orientation.z+=360;
		}
		pub.publish(pose2);
	}
}