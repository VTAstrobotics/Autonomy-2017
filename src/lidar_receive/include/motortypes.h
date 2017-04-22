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
 * motortypes.h
 *
 * This file defines and declares the data types needed to allow
 * multiples VESCs with motors.h/cpp. Data included is
 * CAN ID, Min and Max values for scaling Digital/Analog signals, 
 * Brake Current, and Braking enable.
 *
 *  Created on: 19 mar 2017
 *      Author: Ryan
 */
#ifndef MOTORTYPES_H
#define MOTORTYPES_H

// Define CAN ID map here.
// ID = -1 for the VESC with UART connection
enum VescID {VESC1 = -1, VESC2 = 1};

// Struct to store motor specific parameters
struct Motor_Config {
	bool enable_brake; // Enable/Disable braking
	int Max_Erpm; // Maximum RPM of Motor
	int Min_Erpm; // Minimum RPM of motor
	float Max_Amps; // Maximum Current of Motor
	float Min_Amps; // Minimum Current of Motor
	float Max_Duty; // Maximum Duty cycle of Motor
	float Min_Duty; // Minimum Duty cycle of Motor
	float Brake_Current; // Brake current to use for motor
	// For Analog/Digital Inputs
	float Scale_Max; // Maximum input value
	float Scale_Min; // Minimum input value
};

// Motor parameter definitions
// Change and define new motors as needed
const Motor_Config motor1 = {
	true, // enable_brake;
	30000, // Max_Erpm
	900, // Min_Erpm
	50, // Max_Amps
	1, // Min_Amps
	0.95, // Max_Duty
	0.05, // Min_Duty
	3, // Brake_Current
	1.0, // Scale_Max
	-1.0 // Scale_Min
};

const Motor_Config motor2 = {
	true, // enable_brake;
	35000, // Max_Erpm
	900, // Min_Erpm
	50, // Max_Amps
	1, // Min_Amps
	0.95, // Max_Duty
	0.05, // Min_Duty
	3, // Brake_Current
	1.0, // Scale_Max
	-1.0 // Scale_Min
};
#endif