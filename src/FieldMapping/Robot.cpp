#include "Robot.h"

Robot::Robot(){
  // set the x and y posisition to default value 10
  this->xPos = 2;
  this->yPos = 2;
}

/**
returns the xPos of the robot
*/
int Robot::getRow(){
  return this->xPos;
}

/**
returns the yPos of the robot
*/
int Robot::getCol(){
  return this->yPos;
}


/*
sets the xPosition of the robot to the newXPos
*/
void Robot::setRow(int newXPos){
  this->xPos = newXPos;
}

/*
sets the yPos of the robot to the newYPos
*/
void Robot::setCol(int newYPos){
  this->yPos = newYPos;
}
