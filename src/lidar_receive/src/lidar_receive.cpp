#include <ros/ros.h>
//#include <robot_msgs/PointArray.h>
#include <lidar_receive.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <string.h>
#include "arduino_interface.h"
#include "comm_uart.h"
#include "arduino_interface_uart.h"
#include <unistd.h> // for usleep

// Static variables.
// Used for serial rx callback to indicate
// that arrays have been successfully received
static bool lidar1_received = false;
static bool lidar2_received = false;
static bool lidar3_received = false;

// Initialize lidar arrays
// defined in arduino_interface.h
float lidar1[50] = {};
float lidar2[50] = {};
float lidar3[50] = {};



namespace lidar_receive{

// Serial Rx callback functions.
// These functions will be invoked automatically
// upon receiving the corresponding data.
void arduino_val_lidar1_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar1[i]);
  printf("]");
  printf("\r\n");
  lidar1_received = true;
}

void arduino_val_lidar2_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar2[i]);
  printf("]");
  printf("\r\n");
  lidar2_received = true;
}

void arduino_val_lidar3_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar3[i]);
  printf("]");
  printf("\r\n");
  lidar3_received = true;
}

	//Setup Function
	LidarReceive::LidarReceive(ros::NodeHandle& nh)
	{
		//Subscribing to "/lidar_obs_array" topic
		lidarSub = nh.subscribe("/lidar_obs_array", 1, &LidarReceive::getPoint,this);
		lidarSweep = nh.advertise<std_msgs::Empty>("lidarSweep",1);
		// Initialize serial interface
		comm_uart_init((char*)"/dev/ttyUART1");
		// Give arduino interface a callback function to handle received values
  		arduino_interface_set_rx_lidar1_func(arduino_val_lidar1_received);
  		arduino_interface_set_rx_lidar2_func(arduino_val_lidar2_received);
  		arduino_interface_set_rx_lidar3_func(arduino_val_lidar3_received);

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
		receive_packet(); // Receive data. Packet handler will automatically
						  // invoke the correct callback functions.
		usleep(10000); // Can adjust. Experiment.
    	arduino_interface_uart_run_timer(); // reset packet handler
	}

	void LidarReceive::startMapCB(){
		// if (startMapCommand)
		arduino_interface_start_map(); // Tell the arduino to begin mapping
		usleep(10000);
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
