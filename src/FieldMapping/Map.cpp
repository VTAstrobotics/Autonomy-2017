#include "Map.h"

// creates constructor for the class
Map::Map(){
  // create the robot
  this->robot();

  // initialize the array to all spaces
  for(int row = 0; index < ROWS; row++){
    for(int col = 0; col < COLS; col++){
      map[row][col] = ' '; // set initially to a space
    }
  }

  // place the robot on the Map
  map[robot.getX()][robot.getY()] = robot.getSymbol();
  
  // read in 3D cloud data and place the obstacles accordingly
}
