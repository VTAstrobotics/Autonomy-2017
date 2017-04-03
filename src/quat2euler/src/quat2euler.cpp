#include <quat2euler/quat2euler.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <XmlRpcException.h>

namespace quat2euler{
	Quat2Euler::Quat2Euler(ros::NodeHandle& nh)
	{
		convert_pub = nh.advertise<sensor_msgs::Imu>("imu/converted",1);
		sub = nh.subscribe("/imu/data_madgwick",1,&Quat2Euler::convert,this);
	}
	void Quat2Euler::convert(const sensor_msgs::Imu& imu){
		sensor_msgs::Imu imu2 = imu;
		float x = imu2.orientation.x;
		float y = imu2.orientation.y;
		float z = imu2.orientation.z;
		float w = imu2.orientation.w;

		double ysqr = y * y;

		// roll (x-axis rotation)
		double t0 = +2.0 * (w * x + y * z);
		double t1 = +1.0 - 2.0 * (x * x + ysqr);
		float roll = std::atan2(t0, t1);

		// pitch (y-axis rotation)
		double t2 = +2.0f * (w * y - z * x);
		t2 = t2 > 1.0 ? 1.0 : t2;
		t2 = t2 < -1.0 ? -1.0 : t2;
		float pitch = std::asin(t2);

		// yaw (z-axis rotation)
		double t3 = +2.0 * (w * z + x * y);
		double t4 = +1.0 - 2.0 * (ysqr + z * z);  
		float yaw = std::atan2(t3, t4);

		if(yaw < 0){
			yaw+=(2*M_PI);
		}

		imu2.orientation.x = (roll*180/M_PI);
		imu2.orientation.y = (pitch*180/M_PI);
		imu2.orientation.z = (yaw*180/M_PI);
		imu2.orientation.w = 0.0;

		convert_pub.publish(imu2);
	}

}
