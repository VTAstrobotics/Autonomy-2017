#include "Robot.h"

Robot::Robot(){
  // set the x and y posisition to default value 10
  this->xPos = 10;
  this->yPos = 10;
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

/**
returns the symbol of the robot
*/
char Robot::getSymbol(){
  return this->SYMBOL;
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
