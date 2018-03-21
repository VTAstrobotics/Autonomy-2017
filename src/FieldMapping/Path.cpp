#include "AutonomyMap.h"
#include "Path.h"

// adds a move to the vector
void Path::addMove(Move move){
  thisPath.pushBack(move);
}

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
double calcDistance(int robotRow, int robotCol, int mineCol){
  // calculate the difference betweent eh two points
  int squareRowDiff = pow(robotRow - miningRow, 2.0);
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
