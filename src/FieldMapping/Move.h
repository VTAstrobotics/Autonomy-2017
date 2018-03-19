#include "AutonomyMap.h"
#ifndef MOVE_H
#define MOVE_H

/*
This class will define the next move of the robot. It will give a direction for
the move and will give
*/
class Move{
private:
  Direction direction; // direction of the move
  double distance; // distance traveled in meters
public:
  Move(); // constructor for a move object

  // accessor methods
  void setDirection(Direction datWay);
  void setDistance(double dist);

  // mutator methods
  Direction getDirection();
  double getDistance();

};


#endif
