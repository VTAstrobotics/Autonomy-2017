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
 * arduino_interface_uart.h
 *
 *  Created on: 23 Apr 2017
 *      Author: Ryan
 */

#ifndef ARDUINO_INTERFACE_UART_H_
#define ARDUINO_INTERFACE_UART_H_

#ifdef __cplusplus
extern "C" {
#endif
	
// Includes
#include "packet.h" // For the MAX_PACKET_LEN define

// Functions
void arduino_interface_uart_init(void(*func)(unsigned char *data, unsigned int len));
void arduino_interface_uart_process_byte(unsigned char b);
void arduino_interface_uart_run_timer(void);

#ifdef __cplusplus
}
#endif
#endif /* arduino_INTERFACE_UART_H_ */
