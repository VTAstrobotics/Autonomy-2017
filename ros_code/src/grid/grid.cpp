/*
	This is a coordinate plane system for the rover

	The grid, during testing will be dynamically allocated
	based on some level of precision declared in the beginning
	lines of code. 

	FIELD WIDTH : 3.78m
	FIELD HEIGHT: 7.38m

	LINEAR MARGIN	: tbd
	LATERAL MARGIN	: tbd
	
	**ALL UNITS ARE IN METERS**
	**IMPERIAL IS FOR SCRUBS **
	this is to change the file blah blah


*/

#include <math.h>
#include <stdio.h>

const float FIELD_WIDTH = 3.78;
const float FIELD_HEIGHT= 7.38;
const float LINEAR_MARGIN = 0.2;	//safe space ahead and aft of the rover
const float LATERAL_MARGIN= 0.3;	//safe space on either side of the middle of the rover
const float PRECISION	= 0.01;	//minimum linear precision i.e: to be accurate to one square centimeter, set to 0.01.

int main(int argc, char** argv) {

	//Find out how many bits we need for a node
	float nodes = (int)(FIELD_WIDTH * FIELD_HEIGHT * (1.0/PRECISION)*(1.0/PRECISION));
	int bits = log2(nodes);
	printf("nodes = %f \n bits = %d\n", nodes, bits);

}
