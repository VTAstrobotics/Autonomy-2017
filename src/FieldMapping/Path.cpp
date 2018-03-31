#include "AutonomyMap.h"
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
  if(robotRow < 0 || robotRow > 9 robotCol < 0 || robotCol > 4 ){
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
Creates the path to be used
*/
void Path::createPath(int miningCol, Map map, Robot robot){
  // prime the variables used in the loop
  boolean inMiningField = false; // used to see if the robot is in the mining field
  int robotCol = robot.getCol();
  int robotRow = robot.getRow();

  while(!inMiningField){ // test
    // create a newMove and add it to the path
    Move *newMove = new Move;
    Direction nextDirection = nextMove(robotRow, robotCol, miningCol, map);
    newMove->setDirection(nextDirection);
    addMove((*newMove)); // adds the move to the vector

    //reprime
    if(robotCol == miningCol){
      inMiningField = true;
    }
  }
}

/*
generates the next move to be added to the path. Returns the Direction of the moves
that should be taken and updates the robot row and robot col based on the direction taken

*/
Direction Path::nextMove(int &robotRow, int &robotCol, int miningCol, Map map){
  int towrdsBin = calcDistance(robotRow-1, robotCol, miningCol, map);
  int towardsMine = calcDistance(robotRow+1, robotCol, miningCol, map);
  int towardsRight = calcDistance(robotRow, robotCol-1, miningCol, map);
  int towardsLeft = calcDistance(robotRow+1, robotCol+1, miningCol, map);

  // assume we will always
  if(towardsMine >= towardsBin && towardsMine >= towardsLeft && towardsBin >= towardsRight){
    return TOWARDS_MINE; // Direction enum to be returned
  }
  else if(towardsRight > towardsMine && towardsRight >= towardsLeft && towardsRight >= towardsBin){
    return RIGHT; // Direction enum
  }
  else if(towardsLeft > towardsMine && towardsLeft > towardsRight && towardsLeft >= towrdsBin){
    return LEFT; // Direction enum
  }
  else{
    return TOWARDS_BIN;
  }

}
