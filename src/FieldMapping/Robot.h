#include "AutonomyMap.h"
#ifndef ROBOT_H
#define ROBOT_H

class Robot{
private:
  // position of the robot
  int xPos;
  int yPos;
  bool inMiningField;

public:
  Robot(); // constructor to create Robot
  // accessor methods
  int getRow(); // represents the row
  int getCol();

  // mutator methods
  void setRow(int xPos);
  void setCol(int yPos);

};

#endif
