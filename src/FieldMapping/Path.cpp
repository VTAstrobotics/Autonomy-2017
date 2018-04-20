#include "Path.h"

/*
initializes the path to default values
*/
Path::Path(){}

/*
returns the distance of the path which is determineed by teh number of moves in the
vector
@return int representing the distance
*/
int Path::getPathDistance(){
  thisPath.shrink_to_fit();   // removes the unecessary space
  return thisPath.size();
}

/*
calculates the hypotenuse from the potential position to
the index in the mining column
*/
double calcDistance(int robotRow, int robotCol, int mineCol, Map map){
  // check to see if the move is valid
  if(robotRow < 0 || robotRow > 9 || robotCol < 0 || robotCol > 4 ){
    return 10000000; // this is not a valid position so make it a large number
  }
  // check to see if the move has an obstacle
  char charAtIndex = map.charAt(robotRow, robotCol);
  if(charAtIndex == 'X'){
    return 10000000; // this is not a valid position so make it a large number
  }
  // calculate the difference betweent eh two points
  int squareRowDiff = pow(robotRow - MINING_ROW, 2.0);
  int squareColDiff = pow(robotCol - mineCol, 2.0);
  int addDiff = squareColDiff + squareRowDiff;
  return sqrt(addDiff);
}

/*
adds a move to the path
@param: aMove the move to be added
*/
void Path::addMove(Move aMove){
  thisPath.push_back(aMove);
}

/*
sizes the vector to the smallest size and then returns the
path filled with moves

@return a vector object filled with Move objects
*/
vector<Move> Path::getPath(){
  this->thisPath.shrink_to_fit();   // removes the unecessary space
  return this->thisPath;
}

/*
Creats the path towards a specific miningCol on the field.

@param-robot: used to get the positon of the robot on the Map
@param-map: map containing the obstacle field. Used to detect spaces with obstacles
@param-miningCol: the miningCol of the field to go towards
*/
void Path::createPath(int miningCol, Map map, Robot robot){
  // prime the variables used in the loop
  bool inMiningField = false; // used to see if the robot is in the mining field
  int robotCol = robot.getCol();
  int robotRow = robot.getRow();

  while(!inMiningField){ // test
    // create a newMove and add it to the path
    Move *newMove = new Move;
    Direction nextDirection = nextMove(robotRow, robotCol, miningCol, map);
    newMove->setDirection(nextDirection);
    addMove((*newMove)); // adds the move to the vector

    //reprime
    if(robotCol == miningCol){ // we're where we need to be here
      inMiningField = true;
    }
  }
}

/*
generates the next move to be added to the path. Returns the Direction of the moves
that should be taken and updates the robot row and robot col based on the direction taken

@param-robotRow: current row of the robot. This will get updated if moved towards mine or bin
@param-robotCol: current col of the robot. Thiw iwll get updated if moved left or right.
@param-miningCol: the miningCol that we are trying to reach
@param-map: map of the obstacle field. This will be used to see if an obstacle is in the position
@return a Direction enum which will be set to the direction of the nextMove
*/
Direction Path::nextMove(int &robotRow, int &robotCol, int miningCol, Map map){
  double towardsBin = calcDistance(robotRow-1, robotCol, miningCol, map);
  double towardsMine = calcDistance(robotRow+1, robotCol, miningCol, map);
  double towardsRight = calcDistance(robotRow, robotCol-1, miningCol, map);
  double towardsLeft = calcDistance(robotRow+1, robotCol+1, miningCol, map);

  // assume we will always go towards the mine
  if(towardsMine >= towardsBin && towardsMine >= towardsLeft && towardsBin >= towardsRight){
    robotRow++;
    return TOWARDS_MINE; // Direction enum to be returned
  }
  else if(towardsRight > towardsMine && towardsRight >= towardsLeft && towardsRight >= towardsBin){
    robotCol--;
    return RIGHT; // Direction enum
  }
  else if(towardsLeft > towardsMine && towardsLeft > towardsRight && towardsLeft >= towardsBin){
    robotCol++;
    return LEFT; // Direction enum
  }
  else{
    robotRow--;
    return TOWARDS_BIN;
  }

}
