#include "AutonomyMap.h"
#ifndef MOVE_H
#define MOVE_H

/*
This class will define the next move of the robot. It will give a direction for
the move. All movements are the same distance.
*/
class Move{
private:
  Direction direction; // direction of the move
public:
  Move(); // constructor for a move object

  // mutator methods
  void setDirection(Direction datWay);

  // accessor methods
  Direction getDirection();
};


#endif
