/*
	Copyright 2017 Ryan Owens

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * arduino_interface.c
 */
#include "arduino_interface.h"
#include "buffer.h"
#include <string.h>

// Private variables
static unsigned char send_buffer[256];

// Private variables for received data
static watchdog_values values;

// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len);

// Function pointers
static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static void(*forward_func)(unsigned char *data, unsigned int len) = 0;

// Function pointers for received data
static void(*rx_watchdog_func)(watchdog_values *values) = 0;
static void(*rx_lidar1_func)(void) = 0;
static void(*rx_lidar2_func)(void) = 0;
static void(*rx_lidar3_func)(void) = 0;

/**
 * Initialize arduino_interface.
 *
 * @param func
 * A function to be used when sending packets. Null (0) means that no packets will be sent.
 */
void arduino_interface_init(void(*func)(unsigned char *data, unsigned int len)) {
	send_func = func;
}

/**
 * Provide a function to forward received data to instead of processing it and calling handlers.
 * This will also prevent data from being sent.
 *
 * @param func
 * The forward function. Null (0) to disable forwarding.
 */
void arduino_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len)) {
	forward_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void arduino_interface_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void arduino_interface_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	if (forward_func) {
		forward_func(data, len);
		return;
	}

	int32_t ind = 0;
	int i = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_GET_LIDAR1:
		ind = 0;
			for (i = 0; i < 50; i++){
					lidar1[i] = buffer_get_float32(data, 10000.0, &ind);
				}
			
		if (rx_lidar1_func) {
			rx_lidar1_func();
		}
		break;
			
	case COMM_GET_LIDAR2:
		ind = 0;
			for (i = 0; i < 50; i++){
					lidar2[i] = buffer_get_float32(data, 10000.0, &ind);
				}
			
		if (rx_lidar2_func) {
			rx_lidar2_func();
		}
		break;
			
	case COMM_GET_LIDAR3:
		ind = 0;
			for (i = 0; i < 50; i++){
					lidar3[i] = buffer_get_float32(data, 10000.0, &ind);
				}
			
		if (rx_lidar3_func) {
			rx_lidar3_func();
		}
		break;

	case COMM_GET_WATCHDOG:
		ind = 0;
		values.batVoltage = buffer_get_float32(data, 10000.0, &ind);
		values.stateOfCharge = buffer_get_float32(data, 10000.0, &ind);
		values.cell1Temp = buffer_get_float32(data, 10000.0, &ind);
		values.cell2Temp = buffer_get_float32(data, 10000.0, &ind);
		values.dcConverterTemp = buffer_get_float32(data, 10000.0, &ind);
		values.current = buffer_get_float32(data, 10000.0, &ind);
		values.cpuTemp = buffer_get_float32(data, 10000.0, &ind);
		values.ssrTemp = buffer_get_float32(data, 10000.0, &ind);
		values.boxTemp = buffer_get_float32(data, 10000.0, &ind);
		values.vcc = buffer_get_float32(data, 10000.0, &ind);
		values.timeFromLastStop = buffer_get_float32(data, 10000.0, &ind);
		values.totalKWH = buffer_get_float32(data, 10000.0, &ind);

		if (rx_watchdog_func) {
			rx_watchdog_func(&values);
		}
		break;

	default:
		break;
	}
}

/**
 * Function pointer setters. When data that is requested with the get functions
 * is received, the corresponding function pointer will be called with the
 * received data.
 *
 * @param func
 * A function to be called when the corresponding data is received.
 */

void arduino_interface_set_rx_watchdog_func(void(*func)(watchdog_values *values)) {
	rx_watchdog_func = func;
}

void arduino_interface_set_rx_lidar1_func(void(*func)(void)) {
	rx_lidar1_func = func;
}

void arduino_interface_set_rx_lidar2_func(void(*func)(void)) {
	rx_lidar2_func = func;
}

void arduino_interface_set_rx_lidar3_func(void(*func)(void)) {
	rx_lidar3_func = func;
}

// Setters
void arduino_interface_start_map(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_START_MAP;
	send_packet_no_fwd(send_buffer, send_index);
}

// Getters
void arduino_interface_get_values(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_WATCHDOG;
	send_packet_no_fwd(send_buffer, send_index);
}

// Other functions

// Helpers

// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len) {
	if (!forward_func) {
		arduino_interface_send_packet(data, len);
	}
}

