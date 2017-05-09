// Main program to test UART communication from Linux to Arduino
// Last modified on 4/23/2017 by: Ryan Owens
#include <stdio.h>
#include <string.h>
#include "arduino_interface.h"
#include "comm_uart.h"
#include "arduino_interface_uart.h"
#include <unistd.h> // for usleep

// Initialize lidar arrays
// defined in arduino_interface.h
float lidar1[50] = {};
float lidar2[50] = {};
float lidar3[50] = {};

void arduino_val_lidar1_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar1[i]);
  printf("]");
  printf("\r\n");
}

void arduino_val_lidar2_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar2[i]);
  printf("]");
  printf("\r\n");
}

void arduino_val_lidar3_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar3[i]);
  printf("]");
  printf("\r\n");
}

int main(void) {

	// variables
	int command = 0;
	bool loop = true;
	
	// For the serial interface
	comm_uart_init("/dev/ttyACM1");
	// Give arduino interface a callback function to handle received values
  arduino_interface_set_rx_lidar1_func(arduino_val_lidar1_received);
  arduino_interface_set_rx_lidar2_func(arduino_val_lidar2_received);
  arduino_interface_set_rx_lidar3_func(arduino_val_lidar3_received);
	
	// Main loop 
	while(loop) {
    arduino_interface_start_map();
		receive_packet();
		usleep(10000);
    arduino_interface_uart_run_timer();
	}
	comm_uart_close();
}
