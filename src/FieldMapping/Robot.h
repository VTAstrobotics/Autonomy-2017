#include "AutonomyMap.h"
#ifndef ROBOT_H
#define ROBOT_H

class Robot{
private:
  // position of the robot
  int xPos;
  int yPos;
  const char SYMBOL = '$'; // money team baby

public:
  Robot(); // constructor to create Robot
  // accessor methods
  int getRow(); // represents the row
  int getCol(); //
  char getSymbol();

  // mutator methods
  void setRow(int xPos);
  void setCol(int yPos);

}

#endif
