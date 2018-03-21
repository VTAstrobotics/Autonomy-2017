#include "PathFinder.h"
#include "AutonomyMap.h"

void PathFinder::runPath(){
	// code that will run the best path
}

/*
constructor 
*/
PathFinder::PathFinder(){
	// creates a temporary path that
	// the best path also references 
	tempPath = new Path;
	bestPath = tempPath;

	// create objects
	Map map;
	Robot robot;
	// assign the objects
	this->map = map;
	this->robot = robot; 
}

/*
finds the best path for the robot to take given the position of the robot
on the field. Once the best case is established, the robot executes the best path 
*/
void PathFinder::autonomyAlgorithm(){
	//prime the loop
	int miningCol = 0;
	tempPath->createPath(miningCol);
	miningCol++;

	// more mining paths to find 
	while(miningCol < COL){
		tempPath = new Path; // create a new path object
		tempPath->createPath(miningCol);

		// get the two distances and compare
		int tempDistance = tempPath->getPathDistance();
		int bestDistance = bestPath->getPathDistance();

		// change the best path if the tempDistance is shorter
		if(tempDistance < bestDistance){
			bestPath = tempPath;
		}

		// go to the next miningCol
		miningCol++;
	}
	
	// run the best Path
	runPath();
}

void PathFinder::runPath(){
	// algorothm to run path
}