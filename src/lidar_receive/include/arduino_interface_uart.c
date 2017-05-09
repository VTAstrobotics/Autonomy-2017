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
 * arduino_interface_uart.c
 *
 *  Created on: 23 Apr 2017
 *      Author: Ryan
 */

#include "arduino_interface_uart.h"
#include "arduino_interface.h"

// Settings
#define PACKET_HANDLER			0

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_arduino_interface(unsigned char *data, unsigned int len);

/**
 * Initialize the UART arduino interface and provide a function to be used for
 * sending packets.
 *
 * @param func
 * Function provided for sending packets.
 */
void arduino_interface_uart_init(void(*func)(unsigned char *data, unsigned int len)) {
	// Initialize packet handler
	packet_init(func, process_packet, PACKET_HANDLER);

	// Initialize the arduino interface and provide a send function
	arduino_interface_init(send_packet_arduino_interface);
}

/**
 * Process one byte received on the UART. Once a full packet is received the
 * corresponding callback will be called by arduino_interface.
 *
 * @param b
 * The byte received on the UART to process.
 */
void arduino_interface_uart_process_byte(unsigned char b) {
	packet_process_byte(b, PACKET_HANDLER);
}

/**
 * Call this function at around 1 khz to reset the state of the packet
 * interface after a timeout in case data is lost.
 */
void arduino_interface_uart_run_timer(void) {
	packet_timerfunc();
}

/**
 * Callback for the packet handled for when a whole packet is received,
 * assembled and checked.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
static void process_packet(unsigned char *data, unsigned int len) {
	// Let arduino_interface process the packet.
	arduino_interface_process_packet(data, len);
}

/**
 * Callback that arduino_interface uses to send packets.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
static void send_packet_arduino_interface(unsigned char *data, unsigned int len) {
	// Pass the packet to the packet handler to add checksum, length, start and stop bytes.
	packet_send_packet(data, len, PACKET_HANDLER);
}


