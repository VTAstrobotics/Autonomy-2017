#include "Map.h"

// creates constructor for the class
Map::Map(Robot& robot){
  // initialize the array to all spaces
  for(int row = 0; row < ROWS; row++){
    for(int col = 0; col < COLS; col++){
      map[row][col] = ' '; // set initially to a space
    }
  }

  // create the map, here I have the array as a vector so this needs to be changed
  // based on how you read it in
  for(int index = 0; index < arrayMsg.length(); index++){
    // this obstacle is greater than our allowed height
    if(arrayMsg[index] > 20){
       // defines an obstacle and places it in the correct orientation on map
      map[index % 4][index / 4] = this->obstacle;
    }
  }

  // get starting point of Robot to be used with path planning
  startRow = robot.getRow();
  startCol = robot.getCol();

  // read in 3D cloud data and place the obstacles accordingly
}

/*
returns the character at the specified col and row
*/
char Map::charAt(int row, int col){
  return this->map[row][col];
}

/*
sets the square peice of the map to the piece argument
*/
void Map::setSquare(int row, int col, char piece){
  this->map[row][col] = piece;
}
