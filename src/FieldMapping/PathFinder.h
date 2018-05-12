<<<<<<< HEAD
=======
#include "AutonomyMap.h"
#include "Path.h"
#include "Move.h"
#include "Map.h"
#include "Robot.h"

>>>>>>> cae3900c763b4f91a1c64b3b7431ae43663c7fb0
#ifndef PATHFINDER_H
#define PATHFINDER_H
#include "AutonomyMap.h"
#include "Map.h"
#include "Path.h"
#include "Robot.h"
#include "Move.h"

/*
class that ties in all functionality of the robot, map and various paths to be
used in the path planning algorithm
*/
class PathFinder{
private:
  Robot robot; // our robot
  Map map; // the map of the field
  Path *bestPath; // represent the most optimal path to take
  Path *tempPath; // temporary path that we are creating
  void runPath(); // this will get the robot to run the best path
<<<<<<< HEAD
  void runBackwards(); // this will run the best path backwards
=======
  void runBackwards();
>>>>>>> cae3900c763b4f91a1c64b3b7431ae43663c7fb0
  void moveForward(); // will move the robot forward .8 meters
  void moveBackward(); // will move teh robot backward .8 meters
  void turnRight(); // turns the robot 90 degrees to the right
  void turnLeft(); // turns the robot 90 degrees to the left
  void turnToDirection(Direction dir1, Direction dir2); // turns the robot to the correct Direction

public:
  PathFinder(); // constructor

  // acts as the main method to find the path
  void autonomyAlgorithm(); // creates the best path and then runsPath
};

#endif
