// Main program to test UART communication from BBB to VESC
// Last modified on 3/7/2017 by: Ryan Owens
#include <stdio.h>
#include <string.h>
#include "bldc_interface.h"
#include "comm_uart.h"
#include "bldc_interface_uart.h"
#include <unistd.h> // for usleep

void bldc_val_lidar1_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar1[i]);
  printf("]");
  printf("\r\n");
}

void bldc_val_lidar2_received(void) {
  printf("\r\n");
  printf("[ ");
  for(int i = 0; i < 50; i++)
    printf("%.2f ", lidar2[i]);
  printf("]");
  printf("\r\n");
}

void bldc_val_lidar3_received(void) {
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
	// Give bldc interface a callback function to handle received values
  bldc_interface_set_rx_lidar1_func(bldc_val_lidar1_received);
  bldc_interface_set_rx_lidar2_func(bldc_val_lidar2_received);
  bldc_interface_set_rx_lidar3_func(bldc_val_lidar3_received);
	
	// Main loop 
	while(loop) {
		receive_packet();
		usleep(10000);
    bldc_interface_uart_run_timer();
	}
	comm_uart_close();
}
