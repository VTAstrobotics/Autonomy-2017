#include "AutonomyMap.h"
#ifndef PATHFINDER_H
#define PATHFINDER_H

/*
class that ties in all functionality of the robot, map and various paths to be
used in the path planning algorithm
*/
class PathFinder{
private:
  Robot robot; // our robot
  Map map; // the map of the field
  Path bestPath; // represent the most optimal path to take

public:
  PathFinder(); // constructor

  // acts as the main method
  void findBestPath();

  // accessor methods
  Path getBestPath();
  Map getMap();
  Robot getRobot();

};

#endif
