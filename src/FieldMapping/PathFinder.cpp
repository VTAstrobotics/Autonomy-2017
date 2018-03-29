#include "AutonomyMap.h"
#include "PathFinder.h"

/*
constructor that initializes the private member variables
and sets the pointers to nullptr
*/
PathFinder::PathFinder(){
  this->robot = Robot robot;
  this->map = Map map(thisRobot);

  bestPath = nullptr;
  tempPath = nullptr;
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


/*
iterates through the moves in the Path that bestPath
points to. Uses the ROS motor Node to send the directions
to the robot
*/
void PathFinder::runPath(){
  // get the path and create move pointer
  vector<Move> path = bestPath->getPath();
  Move lastMove = nullptr;
  Move thisMove = nullptr;

  // run through each move
  for(int movement = 0; movement < bestPath->size(); movement++){
    // get the move and direction, update last move for comparison
    lastMove = thisMove;
    thisMove = &(path[movement]); // get the address of the move

    Direction moveDirection = thisMove->getDirection();
    // determine which way to move motors
    switch(moveDirection){
      // determine what the last case was relative to this case
      // and make a proper decision on rotation of robot and
      // movement direction
      case RIGHT:
        break;
      case LEFT:
        break;
      case TOWARDS_MINE:
        break;
      case TOWARDS_BIN:
        break;
      case NOT_DETERMINED: default:
        break;
    }
  }
}
