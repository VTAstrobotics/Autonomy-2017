#include "AutonomyMap.h"
#include "PathFinder.h"

/*
constructor that initializes the private member variables
and sets the pointers to nullptr
*/
<<<<<<< HEAD
PathFinder::PathFinder(){
  robot = Robot robot1;
  map = Map map(robot1);

  bestPath = nullptr;
  tempPath = nullptr;
=======
// Note: robot is automatically initialized, since it has an no-argument constructor
//       robot will be initialized before map, since it was declared before map in the header file
PathFinder::PathFinder() : map(robot), bestPath(nullptr), tempPath(nullptr) {
>>>>>>> cae3900c763b4f91a1c64b3b7431ae43663c7fb0
}

/*
finds the best path for the robot to take given the position of the robot
on the field. Once the best case is established, the robot executes the best path
*/
void PathFinder::autonomyAlgorithm(){
	//prime the loop
	int miningCol = 0;
	tempPath->createPath(miningCol, map, robot);
	bestPath = tempPath; // get the memory location for the first path
	miningCol++;

	// more mining paths to find
	while(miningCol < COLS){
		tempPath = new Path; // create a new path object
		tempPath->createPath(miningCol, map, robot);

		// get the two distances and compare
		int tempDistance = tempPath->getPathDistance();
		int bestDistance = bestPath->getPathDistance();

		// change the best path if the tempDistance is shorter
		if(tempDistance < bestDistance){
			bestPath = tempPath;
		}
    else{ // delete the memory
      delete tempPath;
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

  // create a move object and set the robot towards bin
  // c++ note: variables are initialized automatically when declared, no need to use =
  //           the = operator is actually a copy operator in c++
  Move lastMove;
  lastMove.setDirection(TOWARDS_BIN);
  Move thisMove;

  // run through each move
  for(int movement = 0; movement < bestPath->getPath().size(); movement++){
    // get the move and direction, update last move for comparison
    thisMove = path[movement]; // get the move object

    Direction moveDirection = thisMove.getDirection();

    // orient in the correct direction and move forward
    switch(moveDirection){
      // determine what the last case was relative to this case
      // and make a proper decision on rotation of robot and
      // movement direction
      case RIGHT:
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveForward();
        break;
      case LEFT:
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveForward();
        break;
      case TOWARDS_MINE:
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveForward();
        break;
      case TOWARDS_BIN: // will face towards the mine but go in reverse
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveBackward();
        break;
      case NOT_DETERMINED: default:
        // some error handling case
        break;
    }
    // reprime the last move
    lastMove = thisMove;
  }
}

/*
Used to turn the robot to the correct position for movement
*/
void PathFinder::turnToDirection(Direction oldDirection, Direction newDirection){
  // check for each direction
  if(oldDirection == LEFT){
    switch(newDirection){
      case RIGHT:
        turnRight();
        turnRight();
        break;
      case TOWARDS_BIN:
        turnLeft();
        break;
      case TOWARDS_MINE:
        turnRight();
        break;
      case LEFT: default:
        break; // already in the correct position
    } // end of switch
  }
  if(oldDirection == RIGHT){
    switch(newDirection){
      case TOWARDS_MINE: case TOWARDS_BIN:
        turnLeft();
        break;
      case LEFT:
        turnLeft();
        turnLeft();
        break;
      case RIGHT: default:
        break; // in the correct position
    }
  }
  if(oldDirection == TOWARDS_BIN){ // actually facing the mining area
    switch(newDirection){
      case RIGHT:
        turnRight();
        break;
      case LEFT:
        turnLeft();
        break;
      case TOWARDS_BIN: case TOWARDS_MINE: default:
        break; // in the correct position
    }
  }
  if(oldDirection == TOWARDS_MINE){
    switch(newDirection){
      case RIGHT:
        turnRight();
        break;
      case LEFT:
        turnLeft();
        break;
      case TOWARDS_BIN: case TOWARDS_MINE: default:
        break; // in the correct position
    }
  }
} // end of turn to position

/*
moves the robot froward one square distnace
*/
void PathFinder::moveForward(){
  //contains the code to move forard from the ROS topic
  motor_command.rightRatio = forwardRatio;
  motor_command.leftRatio = forwardRatio;
}

/*
Moves the robot backward. This should only be called
on a TOWARDS_BIN state
*/
void PathFinder::moveBackward(){
  motor_command.rightRatio = backwardRatio;
  motor_command.leftRatio = backwardRatio;
}

/*
Turns the robot to the right 90 degrees
*/
void PathFinder::turnRight(){
  motor_command.rightRatio = backwardRatio;
  motor_command.leftRatio = forwardRatio;
}
/*
Turns the robot to the left 90 degrees
*/
void PathFinder::turnLeft(){
  motor_command.rightRatio = forwardRatio;
  motor_command.leftRatio = backwardRatio;
}

/*
Runs the path backwards. This will iterate through the best path in reverse
and then create a path that is opposite the movements of the best path
*/
void PathFinder::runBackwards(){
  // create an iterator that starts at the end of the vector and moves forward
  std::vector<Move>::reverse_iterator rit = bestPath->getPath().rbegin();
  Path *reversePath = new Path;

  // iterates through the vector backwards
  for(; rit != bestPath->getPath().rend(); rit--){
    Move thisMove; // create the move to be placed into the new vector

    // set the move to the opposite of what
    switch(rit->getDirection()){
      case RIGHT:
        thisMove.setDirection(LEFT);
        break;
      case LEFT:
        thisMove.setDirection(RIGHT);
        break;
      case TOWARDS_BIN:
        thisMove.setDirection(TOWARDS_MINE);
        break;
      case TOWARDS_MINE:
        thisMove.setDirection(TOWARDS_BIN);
        break;
      default:
        break;
    }
    // add the move to the reverse path
    reversePath->addMove(thisMove);
  }

  // get the path and create move pointer
  vector<Move> path = reversePath->getPath();

  // create a move object and set the robot towards bin
  Move lastMove;
  lastMove.setDirection(TOWARDS_BIN);
  Move thisMove;

  // run through each move
  for(int movement = 0; movement < bestPath->getPath().size(); movement++){
    // get the move and direction, update last move for comparison
    thisMove = path[movement]; // get the move object

    Direction moveDirection = thisMove.getDirection();

    // orient in the correct direction and move forward
    switch(moveDirection){
      // determine what the last case was relative to this case
      // and make a proper decision on rotation of robot and
      // movement direction
      case RIGHT:
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveForward();
        break;
      case LEFT:
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveForward();
        break;
      case TOWARDS_MINE:
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveForward();
        break;
      case TOWARDS_BIN: // will face towards the mine but go in reverse
        turnToDirection(lastMove.getDirection(), moveDirection);
        moveBackward();
        break;
      case NOT_DETERMINED: default:
        // some error handling case
        break;
    }
    // reprime the last move
    lastMove = thisMove;
  }

 // delete the path from memory
 delete reversePath;
}
