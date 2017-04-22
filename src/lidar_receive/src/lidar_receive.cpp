#include <ros/ros.h>
#include <robot_msgs/PointArray.h>
#include <lidar_receive.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <string.h>
#include "bldc_interface.h"
#include "comm_uart.h"
#include "bldc_interface_uart.h"
#include <unistd.h> // for usleep

namespace lidar_receive{

	//Setup Function
	LidarReceive::LidarReceive(ros::NodeHandle& nh)
	{
		//Subscribing to "/lidar_obs_array" topic
		lidarSub = nh.subscribe("/lidar_obs_array", 1, &LidarReceive::getPoint,this);
		// Initialize serial interface
		comm_uart_init((char*)"/dev/ttyACM1");

		oldVal[0] = 0.0;// old value received from Lidar
		newVal[0] = 0.0;// newest value receive from Lidar
		oldVal[1] = 0.0;// old value received from Lidar
		newVal[1] = 0.0;// newest value receive from Lidar
		oldVal[2] = 0.0;// old value received from Lidar
		newVal[2] = 0.0;// newest value receive from Lidar
		mapCounter = 0;		   // Counter to track point in Array
	}


	// Callback Function to Lidar Message
	void LidarReceive::getPoint(const geometry_msgs::Point& subby){
		newVal[0] = subby.x;
		newVal[1] = subby.y;
		newVal[2] = subby.z;
	}

	void LidarReceive::receiveCB(){
		receive_packet();
		usleep(10000);
    	bldc_interface_uart_run_timer();
    	// Print lidar1 array
    	printf("\r\n");
  		printf("[ ");
  		for(int i = 0; i < 50; i++)
    	printf("%.2f ", lidar1[i]);
  		printf("]");
  		printf("\r\n");

  		// Print lidar2 array
    	printf("\r\n");
  		printf("[ ");
  		for(int i = 0; i < 50; i++)
    	printf("%.2f ", lidar2[i]);
  		printf("]");
  		printf("\r\n");

  		// Print lidar3 array
    	printf("\r\n");
  		printf("[ ");
  		for(int i = 0; i < 50; i++)
    	printf("%.2f ", lidar3[i]);
  		printf("]");
  		printf("\r\n");
	}

	// Function to construct map
	void LidarReceive::constructMap(){
		// if the value isn't new, do nothing
		if (mapCounter==49){
			//publish if array is full
		}
		else if (newVal[0] == oldVal[0])
		{

		}
		// otherwise, insert the value into the map array and update the "oldVal"
		else
		{
			mapArray[0][mapCounter] = newVal[0];
			mapArray[1][mapCounter] = newVal[1];
			mapArray[2][mapCounter] = newVal[2];
			oldVal[0] = newVal[0];
			oldVal[1] = newVal[1];
			oldVal[2] = newVal[2];
			mapCounter = mapCounter++;
		}
	}
}
