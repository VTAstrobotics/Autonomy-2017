#include "AutonomyMap.h"
#ifndef MOVE_H
#define MOVE_H

/*
This class will define the next move of the robot. We will
gain access to the map by inheriting from the Map class
*/
class Move{
private:
  Direction direction; // direction of the move
  double distance;
public:
  Move(); // constructor

  // accessor methods
  void setDirection(Direction datWay);
  void setDistance(double dist);

  // mutator methods
  Direction getDirection();
  double getDistance();

};


#endif
