#include "Move.h"

Move::Move() : direction(NOT_DETERMINED){}

/*
sets the direction of the move to the param direction
@param-datWay is the direction the robot will move in
*/
void Move::setDirection(Direction datWay){
  this->direction = datWay;
}

/*
gets the direction of the Move
@return a Direction enum representing the move
*/
Direction Move::getDirection(){
  return this->direction;
}
