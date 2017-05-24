#include <ros/ros.h>
#include <cam_data_filter/cam_data_filter.h>
#include <boost/foreach.hpp>
#include <XmlRpcException.h>


namespace cam_data_filter{
	CamDataFilter::CamDataFilter(ros::NodeHandle& nh)
	{
		pub = nh.advertise<apriltags_ros::MetaPose>("/filteredCamData",1);

		L = nh.subscribe("left/tag_detections", 1, &CamDataFilter::lConvert,this);
		R = nh.subscribe("right/tag_detections", 1, &CamDataFilter::rConvert,this);
		B = nh.subscribe("back/tag_detections", 1, &CamDataFilter::bConvert,this);

		oneOffset = 0.335;
		twoOffset = 0.195;
		threeOffset = -0.195;
		fourOffset = -0.335;
	}
	void CamDataFilter::lConvert(const apriltags_ros::AprilTagDetectionArray& array){
		if(array.detections.size() > 0){
			int id = array.detections[0].id;
			bool found = false;

			apriltags_ros::MetaPose pose2;
			pose2.pose = array.detections[0].pose.pose;
			pose2.pose.orientation.z +=(M_PI/2);
			pose2.pose.orientation.x=pose2.pose.orientation.x*180/M_PI;
			pose2.pose.orientation.y=pose2.pose.orientation.y*180/M_PI;
			pose2.pose.orientation.z=pose2.pose.orientation.z*180/M_PI;
			if(pose2.pose.orientation.z < 0)
			{
				pose2.pose.orientation.z+=360;
			}
			switch(id){
				/*
				1 +0.335
				2 +18.5
				3 -19.5
				4 -33.5
				*/
				case 0:
					//data is qualtiy nothing to do
				break;

				case 1:
					pose2.pose.position.x += oneOffset;
				break;

				case 2:
					pose2.pose.position.x += twoOffset;
				break;

				case 3:
					pose2.pose.position.x += threeOffset;
				break;

				case 4:
					pose2.pose.position.x += fourOffset;
				break;

			}
			/*while(int count = 0; count < 5 && !found; count++){
				if(array.detections[count].id == 0){
					zeroTag.pose=array.detections[count].pose;
					0Pub.publish(zeroTag);
					found = true;
				}

			}*/
			pub.publish(pose2);
		}
	}

	void CamDataFilter::rConvert(const apriltags_ros::AprilTagDetectionArray& array){
		if(array.detections.size()>0){
			int id = array.detections[0].id;
			bool found = false;
			
			apriltags_ros::MetaPose pose2;
			pose2.pose = array.detections[0].pose.pose;
			pose2.pose.orientation.z -=(M_PI/2);
			pose2.pose.orientation.x=pose2.pose.orientation.x*180/M_PI;
			pose2.pose.orientation.y=pose2.pose.orientation.y*180/M_PI;
			pose2.pose.orientation.z=pose2.pose.orientation.z*180/M_PI;
			if(pose2.pose.orientation.z < 0)
			{
				pose2.pose.orientation.z+=360;
			}
			switch(id){
				/*
				1 +0.335
				2 +18.5
				3 -19.5
				4 -33.5
				*/
				case 0:
					//data is qualtiy nothing to do
				break;

				case 1:
					pose2.pose.position.x += oneOffset;
				break;

				case 2:
					pose2.pose.position.x += twoOffset;
				break;

				case 3:
					pose2.pose.position.x += threeOffset;
				break;

				case 4:
					pose2.pose.position.x += fourOffset;
				break;

			}
			/*while(int count = 0; count < 5 && !found; count++){
				if(array.detections[count].id == 0){
					zeroTag.pose=array.detections[count].pose;
					0Pub.publish(zeroTag);
					found = true;
				}

			}*/
			pub.publish(pose2);
		}
	}

	void CamDataFilter::bConvert(const apriltags_ros::AprilTagDetectionArray& array){
		if(array.detections.size()>0){
			int id = array.detections[0].id;
			bool found = false;

			apriltags_ros::MetaPose pose2;
			pose2.pose = array.detections[0].pose.pose;
			pose2.pose.orientation.x=pose2.pose.orientation.x*180/M_PI;
			pose2.pose.orientation.y=pose2.pose.orientation.y*180/M_PI;
			pose2.pose.orientation.z=pose2.pose.orientation.z*180/M_PI;
			if(pose2.pose.orientation.z < 0)
			{
				pose2.pose.orientation.z+=360;
			}
			switch(id){
				/*
				1 +0.335
				2 +18.5
				3 -19.5
				4 -33.5
				*/
				case 0:
					//data is qualtiy nothing to do
				break;

				case 1:
					pose2.pose.position.x += oneOffset;
				break;

				case 2:
					pose2.pose.position.x += twoOffset;
				break;

				case 3:
					pose2.pose.position.x += threeOffset;
				break;

				case 4:
					pose2.pose.position.x += fourOffset;
				break;

			}
			/*while(int count = 0; count < 5 && !found; count++){
				if(array.detections[count].id == 0){
					zeroTag.pose=array.detections[count].pose;
					0Pub.publish(zeroTag);
					found = true;
				}

			}*/
			pub.publish(pose2);
		}
	}
}