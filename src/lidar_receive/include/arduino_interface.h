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
*
*/

#ifndef ARDUINO_INTERFACE_H_
#define ARDUINO_INTERFACE_H_

#include "datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

// Global variables
extern float lidar1[50];
extern float lidar2[50];
extern float lidar3[50];
	
// interface functions
void arduino_interface_init(void(*func)(unsigned char *data, unsigned int len));
void arduino_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len));
void arduino_interface_send_packet(unsigned char *data, unsigned int len);
void arduino_interface_process_packet(unsigned char *data, unsigned int len);

// Function pointer setters
void arduino_interface_set_rx_watchdog_func(void(*func)(watchdog_values *values));
void arduino_interface_set_rx_lidar1_func(void(*func)(void));
void arduino_interface_set_rx_lidar2_func(void(*func)(void));
void arduino_interface_set_rx_lidar3_func(void(*func)(void));

// Setters
void arduino_interface_start_map(void);

// Getters

// Other functions

// Helpers

#ifdef __cplusplus
}
#endif
#endif /* arduino_INTERFACE_H_ */
